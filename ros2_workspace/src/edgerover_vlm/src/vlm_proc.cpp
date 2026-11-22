// vlm_proc: Qwen3-VL 常驻服务
// 数据面: FrameRing 共享内存取最新帧；控制面: ZeroMQ REP 收问题、回答案
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <exception>
#include <memory>
#include <unistd.h>

#include <vector>
#include <time.h>
#include <zmq.h>
#include <opencv2/imgproc.hpp>

#include "edgerover_ipc/frame_ring.hpp"
#include "rkllm.h"
#include "image_enc.h"

using namespace std::chrono;
using edgerover::ipc::ReadResult;

static volatile std::sig_atomic_t g_stop = 0;
static void on_sig(int) {g_stop = 1;}

static long ms_since(steady_clock::time_point t)
{
  return duration_cast<milliseconds>(steady_clock::now() - t).count();
}

static uint64_t now_mono_ns()
{
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

// 一次生成的结果，通过 rkllm_run 的 userdata 传给回调
struct Gen
{
  std::string text;
  steady_clock::time_point t0;
  long ttft_ms = -1;   // 首字延迟
  int tokens = 0;
  bool err = false;
};

static int on_result(RKLLMResult * r, void * ud, LLMCallState s)
{
  auto * g = static_cast<Gen *>(ud);
  if (!g) {return 0;}
  if (s == RKLLM_RUN_NORMAL && r && r->text) {
    if (g->ttft_ms < 0) {g->ttft_ms = ms_since(g->t0);}
    g->text += r->text;
    g->tokens++;
  } else if (s == RKLLM_RUN_ERROR) {
    g->err = true;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr,
      "usage: vlm_proc <vision.rknn> <llm.rkllm> [max_new_tokens=128] [ctx=4096] [npu_cores=3]"
      " [endpoint=ipc:///tmp/edgerover_vlm]\n");
    return 1;
  }
  const int max_new = argc > 3 ? std::atoi(argv[3]) : 64;   //最多生成的 token 数
  const int ctx_len = argc > 4 ? std::atoi(argv[4]) : 4096;
  const int cores   = argc > 5 ? std::atoi(argv[5]) : 3;   // VLM 忙时 yolo 已降到 5fps，core0 基本空闲，编码器用满 3 核
  const char * endpoint = argc > 6 ? argv[6] : "ipc:///tmp/edgerover_vlm";
  const uint64_t kMaxFrameAgeNs = 500ull * 1000000ull;  // 帧超过 0.5s 视为相机已停

  // Ctrl+C：不加 SA_RESTART，让 zmq_msg_recv 返回 EINTR 从而退出循环
  struct sigaction sa; std::memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_sig;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // 1. LLM
  RKLLMParam p = rkllm_createDefaultParam();
  p.model_path = argv[2];
  p.top_k = 1;
  p.max_new_tokens = max_new;

  p.extend_param.enabled_cpus_num = 4;
  p.extend_param.enabled_cpus_mask = 0xF0;   // CPU4-7 = A76 大核

  p.max_context_len = ctx_len;
  p.skip_special_token = true;
  p.img_start = "<|vision_start|>";
  p.img_end = "<|vision_end|>";
  p.img_content = "<|image_pad|>";
  LLMHandle llm = nullptr;
  if (rkllm_init(&llm, &p, on_result) != 0) {std::fprintf(stderr, "rkllm_init failed\n"); return 1;}

  // 2. 视觉编码器
  rknn_app_context_t enc;
  std::memset(&enc, 0, sizeof(enc));
  if (init_imgenc(argv[1], &enc, cores) != 0) {rkllm_destroy(llm); return 1;}
  const int W = enc.model_width, H = enc.model_height;
  std::vector<float> embed((size_t)enc.model_image_token * enc.model_embed_size * enc.io_num.n_output);

  // 3. FrameRing：camera_proc 可能还没建好共享内存，每秒重试一次
  std::unique_ptr<edgerover::ipc::FrameRingReader> rp;
  while (!g_stop && !rp) {
    try {
      rp = std::make_unique<edgerover::ipc::FrameRingReader>("/edgerover_frames");
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[vlm] waiting camera: %s\n", e.what());
      sleep(1);
    }
  }
  if (!rp) {release_imgenc(&enc); rkllm_destroy(llm); return 0;}   // 等待期间收到 SIGTERM
  auto & reader = *rp;
  std::vector<uint8_t> buf(reader.slot_size());

  RKLLMInferParam ip;
  std::memset(&ip, 0, sizeof(ip));
  ip.mode = RKLLM_INFER_GENERATE;
  ip.keep_history = 0;

  // 4. ZeroMQ REP
  void * zctx = zmq_ctx_new();
  void * sock = zmq_socket(zctx, ZMQ_REP);
  if (zmq_bind(sock, endpoint) != 0) {
    std::fprintf(stderr, "zmq_bind %s failed: %s\n", endpoint, zmq_strerror(errno));
    return 1;
  }

  // 资源仲裁：回答期间广播 busy，yolo_proc 订阅后降帧，把 DDR 带宽让给 VLM
  void * state = zmq_socket(zctx, ZMQ_PUB);
  if (zmq_bind(state, "ipc:///tmp/edgerover_state") != 0) {
    std::fprintf(stderr, "zmq_bind state failed: %s\n", zmq_strerror(errno));
    return 1;
  }
  auto pub_state = [&](const char * s) {zmq_send(state, s, std::strlen(s), ZMQ_DONTWAIT);};

  std::fprintf(stderr, "[vlm] ready, encoder %dx%d, max_new=%d, listening on %s\n", W, H, max_new, endpoint);

  // 处理一个问题，返回 "OK ...统计...\n回答" 或 "ERR 原因"
  auto answer = [&](const std::string & q) -> std::string {
      edgerover::ipc::FrameInfo info;
      ReadResult rr = ReadResult::kNoNewFrame;
      auto t_req = steady_clock::now();
      while ((rr = reader.read_latest(buf.data(), buf.size(), info, 0)) != ReadResult::kOk &&
        ms_since(t_req) < 1000) {}
      if (rr != ReadResult::kOk) {return "ERR no frame";}
      const uint64_t age_ns = now_mono_ns() - info.timestamp_ns;
      if (age_ns > kMaxFrameAgeNs) {return "ERR stale frame (camera_proc stopped?)";}

      cv::Mat src(info.height, info.width, CV_8UC3, buf.data()), img;
      cv::resize(src, img, cv::Size(W, H), 0, 0, cv::INTER_LINEAR);

      auto t_enc = steady_clock::now();
      if (run_imgenc(&enc, img.data, embed.data()) != 0) {return "ERR encoder";}
      const long enc_ms = ms_since(t_enc);

      std::string prompt = "<image>" + q + "\n请用一两句话简短回答。";
      RKLLMInput in;
      std::memset(&in, 0, sizeof(in));
      in.input_type = RKLLM_INPUT_MULTIMODAL;
      in.role = "user";
      in.multimodal_input.prompt = const_cast<char *>(prompt.c_str());
      in.multimodal_input.image_embed = embed.data();
      in.multimodal_input.n_image_tokens = enc.model_image_token;
      in.multimodal_input.n_image = 1;
      in.multimodal_input.image_width = W;
      in.multimodal_input.image_height = H;

      Gen g;
      g.t0 = steady_clock::now();
      rkllm_run(llm, &in, &ip, &g);
      const long llm_ms = ms_since(g.t0);
      if (g.err) {return "ERR llm";}

      const double gen_s = (llm_ms - g.ttft_ms) / 1000.0;
      char stat[256];
      std::snprintf(stat, sizeof(stat),
        "OK frame=%lu age=%lums enc=%ldms ttft=%ldms tokens=%d tps=%.1f total=%ldms\n",
        (unsigned long)info.frame_id, (unsigned long)(age_ns / 1000000), enc_ms, g.ttft_ms,
        g.tokens, gen_s > 0 ? g.tokens / gen_s : 0.0, ms_since(t_req));
      std::fprintf(stderr, "[vlm] %s", stat);
      return std::string(stat) + g.text;
    };

  while (!g_stop) {
    zmq_msg_t m;
    zmq_msg_init(&m);
    const int n = zmq_msg_recv(&m, sock, 0);
    if (n < 0) {
      zmq_msg_close(&m);
      if (errno == EINTR) {continue;}
      break;
    }
    std::string q(static_cast<char *>(zmq_msg_data(&m)), n);
    zmq_msg_close(&m);
    std::fprintf(stderr, "[vlm] Q: %s\n", q.c_str());

    pub_state("vlm busy");
    const std::string reply = answer(q);
    pub_state("vlm idle");

    zmq_send(sock, reply.data(), reply.size(), 0);  // REP 必须对每个请求都回一条
  }

  std::fprintf(stderr, "[vlm] shutting down\n");
  zmq_close(sock);
  zmq_close(state);
  zmq_ctx_term(zctx);
  release_imgenc(&enc);
  rkllm_destroy(llm);
  return 0;
}