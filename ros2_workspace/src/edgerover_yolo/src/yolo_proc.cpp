// YOLOv5 检测进程：FrameRing(640x640 RGB) -> NPU(CORE_0，把 CORE_1/2 留给 VLM) -> ZMQ PUB(JSON)
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>
#include <unistd.h>
#include <zmq.h>
#include "rknn_api.h"
#include "edgerover_ipc/frame_ring.hpp"

using namespace edgerover::ipc;

static volatile sig_atomic_t g_stop = 0;
static int64_t now_ns() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000000000LL + t.tv_nsec; }
static inline float sigm(float x) { return 1.f / (1.f + expf(-x)); }

constexpr int kIn = 640, kCls = 80, kElem = 85, kMaxDet = 100;
constexpr int64_t kBusyPeriodNs = 200000000LL, kBusyTimeoutNs = 60000000000LL;   // VLM 忙时 5fps；超 60s 视为 VLM 挂了
constexpr float kObjThr = 0.25f, kNmsThr = 0.45f;
static const float kAnchors[3][6] = {{10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};
static const int kStrides[3] = {8, 16, 32};

struct Det { float x1, y1, x2, y2; int cls; float score; int track_id = 0; };

static float iou(const Det& a, const Det& b) {
  float iw = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
  float ih = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);
  if (iw <= 0 || ih <= 0) return 0.f;
  float inter = iw * ih;
  float u = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return u > 0 ? inter / u : 0.f;
}

// 原 C 版的 static g_smoother 写在头文件里（每个 .c 一份拷贝），这里改成对象成员
class Smoother {
  struct Track { Det d; int id; int miss; bool used; };
  std::vector<Track> tr_;
  int next_ = 1;
  static constexpr float kAlpha = 0.8f, kMatch = 0.4f;
  static constexpr int kMaxMiss = 3, kMaxTracks = 32;
 public:
  void update(std::vector<Det>& dets) {
    for (auto& t : tr_) t.used = false;
    for (auto& d : dets) {
      int best = -1; float bi = kMatch;
      for (size_t i = 0; i < tr_.size(); i++) {
        if (tr_[i].used || tr_[i].d.cls != d.cls) continue;
        float v = iou(tr_[i].d, d);
        if (v > bi) { bi = v; best = (int)i; }
      }
      if (best >= 0) {
        Track& t = tr_[best];
        t.d.x1 = kAlpha * d.x1 + (1 - kAlpha) * t.d.x1;   // EMA 平滑框
        t.d.y1 = kAlpha * d.y1 + (1 - kAlpha) * t.d.y1;
        t.d.x2 = kAlpha * d.x2 + (1 - kAlpha) * t.d.x2;
        t.d.y2 = kAlpha * d.y2 + (1 - kAlpha) * t.d.y2;
        t.d.score = d.score; t.miss = 0; t.used = true;
        d = t.d; d.track_id = t.id;
      } else if ((int)tr_.size() < kMaxTracks) {
        d.track_id = next_;
        tr_.push_back({d, next_++, 0, true});
      }
    }
    for (auto& t : tr_) if (!t.used) t.miss++;
    tr_.erase(std::remove_if(tr_.begin(), tr_.end(), [](const Track& t) { return t.miss > kMaxMiss; }), tr_.end());
  }
};

class Yolo {
 public:
  bool init(const char* path) {
    if (rknn_init(&ctx_, (void*)path, 0, 0, nullptr) < 0) { fprintf(stderr, "rknn_init failed\n"); return false; }
    rknn_set_core_mask(ctx_, RKNN_NPU_CORE_0);   // 只用 core0
    rknn_input_output_num io{};
    rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));
    if (io.n_output != 3) { fprintf(stderr, "expect 3 outputs, got %u\n", io.n_output); return false; }

    in_attr_.index = 0;
    rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in_attr_, sizeof(in_attr_));
    // 关键：声明输入为 UINT8 + NHWC，这样帧环里的 RGB 可以直接写进 input_mem，不用转换
    in_attr_.type = RKNN_TENSOR_UINT8;
    in_attr_.fmt = RKNN_TENSOR_NHWC;
    in_mem_ = rknn_create_mem(ctx_, kIn * kIn * 3);
    if (!in_mem_ || rknn_set_io_mem(ctx_, in_mem_, &in_attr_) < 0) return false;

    for (int i = 0; i < 3; i++) {
      out_attr_[i].index = i;
      rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr_[i], sizeof(rknn_tensor_attr));
      if (out_attr_[i].type != RKNN_TENSOR_INT8 || out_attr_[i].n_dims != 4 || out_attr_[i].dims[1] != 3 * kElem) {
        fprintf(stderr, "output %d: need INT8 1x255xHxW (type=%d dims=%d,%d,%d,%d)\n", i, out_attr_[i].type,
                out_attr_[i].dims[0], out_attr_[i].dims[1], out_attr_[i].dims[2], out_attr_[i].dims[3]);
        return false;
      }
      out_mem_[i] = rknn_create_mem(ctx_, out_attr_[i].size_with_stride);
      if (!out_mem_[i] || rknn_set_io_mem(ctx_, out_mem_[i], &out_attr_[i]) < 0) return false;
    }
    return true;
  }

  uint8_t* input_buf() { return (uint8_t*)in_mem_->virt_addr; }
  size_t input_size() const { return kIn * kIn * 3; }

  // 返回 640x640 坐标系下的框
  bool infer(std::vector<Det>& out) {
    rknn_mem_sync(ctx_, in_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);
    if (rknn_run(ctx_, nullptr) < 0) return false;
    std::vector<Det> cand;
    for (int h = 0; h < 3; h++) {
      rknn_mem_sync(ctx_, out_mem_[h], RKNN_MEMORY_SYNC_FROM_DEVICE);
      decode(h, cand);
    }
    nms(cand, out);
    return true;
  }

  ~Yolo() {
    for (auto* m : out_mem_) if (m) rknn_destroy_mem(ctx_, m);
    if (in_mem_) rknn_destroy_mem(ctx_, in_mem_);
    if (ctx_) rknn_destroy(ctx_);
  }

 private:
  void decode(int h, std::vector<Det>& cand) {
    const rknn_tensor_attr& at = out_attr_[h];
    const int gh = at.dims[2], gw = at.dims[3];
    const int ws = at.w_stride ? at.w_stride : gw;          // NCHW 行跨度
    const int8_t* p = (const int8_t*)out_mem_[h]->virt_addr;
    const float scale = at.scale; const int zp = at.zp;
    const int stride = kStrides[h];
    // zoo 导出的 head 已含 sigmoid，输出就是概率；阈值直接换算到 int8 域，绝大多数格子一次整数比较就跳过
    const int obj_q = (int)ceilf(kObjThr / scale + zp);
    const int plane = gh * ws;

    for (int a = 0; a < 3; a++) {
      const int8_t* base = p + a * kElem * plane;
      for (int gy = 0; gy < gh; gy++) {
        for (int gx = 0; gx < gw; gx++) {
          const int off = gy * ws + gx;
          int oq = base[4 * plane + off];
          if (oq < obj_q) continue;
          int best = 0, bq = base[5 * plane + off];
          for (int c = 1; c < kCls; c++) { int q = base[(5 + c) * plane + off]; if (q > bq) { bq = q; best = c; } }
          float score = ((oq - zp) * scale) * ((bq - zp) * scale);
          if (score < kObjThr) continue;
          float tx = (base[off] - zp) * scale, ty = (base[plane + off] - zp) * scale;
          float tw = (base[2 * plane + off] - zp) * scale, th = (base[3 * plane + off] - zp) * scale;
          float cx = (tx * 2.f - 0.5f + gx) * stride;
          float cy = (ty * 2.f - 0.5f + gy) * stride;
          float sw = tw * 2.f, sh = th * 2.f;
          float bw = sw * sw * kAnchors[h][a * 2], bh = sh * sh * kAnchors[h][a * 2 + 1];
          cand.push_back({cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2, best, score});
        }
      }
    }
  }

  static void nms(std::vector<Det>& cand, std::vector<Det>& out) {
    std::sort(cand.begin(), cand.end(), [](const Det& a, const Det& b) { return a.score > b.score; });
    if (cand.size() > 300) cand.resize(300);
    out.clear();
    for (const auto& d : cand) {
      bool keep = true;
      for (const auto& k : out)
        if (k.cls == d.cls && iou(k, d) > kNmsThr) { keep = false; break; }   // 按类别抑制
      if (keep) { out.push_back(d); if ((int)out.size() >= kMaxDet) break; }
    }
  }

  rknn_context ctx_ = 0;
  rknn_tensor_attr in_attr_{};
  rknn_tensor_attr out_attr_[3]{};
  rknn_tensor_mem* in_mem_ = nullptr;
  rknn_tensor_mem* out_mem_[3]{};
};

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: yolo_proc <model.rknn> [endpoint=ipc:///tmp/edgerover_det]\n"); return 1; }
  const char* endpoint = argc > 2 ? argv[2] : "ipc:///tmp/edgerover_det";
  signal(SIGINT, [](int) { g_stop = 1; });
  signal(SIGTERM, [](int) { g_stop = 1; });

  Yolo yolo;
  if (!yolo.init(argv[1])) return 2;

  // 等 camera_proc 创建共享内存
  FrameRingReader* reader = nullptr;
  while (!g_stop && !reader) {
    try { reader = new FrameRingReader("/edgerover_frames"); }
    catch (const std::exception& e) { fprintf(stderr, "waiting camera: %s\n", e.what()); sleep(1); }
  }
  if (!reader) return 0;

  void* zctx = zmq_ctx_new();
  void* pub = zmq_socket(zctx, ZMQ_PUB);
  int hwm = 2; zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));   // 慢订阅者只会丢旧消息，不会拖住检测
  if (zmq_bind(pub, endpoint) != 0) { perror("zmq_bind"); return 3; }

  // 订阅 VLM 状态；connect 在对方 bind 之前也能成功，启动顺序无所谓
  void* sub = zmq_socket(zctx, ZMQ_SUB);
  zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(sub, "ipc:///tmp/edgerover_state");
  bool vlm_busy = false;
  int64_t busy_since = 0, last_infer = 0;

  Smoother smoother;
  std::vector<Det> dets;
  FrameInfo info;
  uint64_t last = 0;
  int64_t t_stat = now_ns(); int n_stat = 0; double infer_sum = 0;
  std::string js;

  while (!g_stop) {

    // 非阻塞取完所有状态消息，只保留最新的
    char sb[32]; int sn;
    while ((sn = zmq_recv(sub, sb, sizeof(sb) - 1, ZMQ_DONTWAIT)) > 0) {
      sb[std::min(sn, (int)sizeof(sb) - 1)] = 0;
      bool b = strcmp(sb, "vlm busy") == 0;
      if (b != vlm_busy) fprintf(stderr, "[yolo] %s -> %s\n", sb, b ? "throttle 5fps" : "full speed");
      vlm_busy = b;
      if (b) busy_since = now_ns();
    }
    if (vlm_busy && now_ns() - busy_since > kBusyTimeoutNs) { vlm_busy = false; fprintf(stderr, "[yolo] busy timeout, full speed\n"); }
    if (vlm_busy && now_ns() - last_infer < kBusyPeriodNs) { usleep(5000); continue; }

    // 直接把帧读进 NPU 输入内存（一次拷贝）
    auto r = reader->read_latest(yolo.input_buf(), yolo.input_size(), info, last);
    if (r == ReadResult::kNoNewFrame) { usleep(2000); continue; }
    if (r == ReadResult::kTorn) continue;
    if (r == ReadResult::kBufferTooSmall) { fprintf(stderr, "frame bigger than 640x640x3\n"); break; }
    last = info.frame_id;

    int64_t t0 = now_ns();
    last_infer = t0;

    if (!yolo.infer(dets)) { fprintf(stderr, "infer failed\n"); break; }
    double infer_ms = (now_ns() - t0) / 1e6;

    // 640 坐标 -> 原图坐标：减 pad，除 scale，夹到原图范围
    const float sw = (float)info.src_width, sh = (float)info.src_height;
    for (auto& d : dets) {
      d.x1 = std::min(std::max((d.x1 - info.lb_pad_x) / info.lb_scale, 0.f), sw);
      d.x2 = std::min(std::max((d.x2 - info.lb_pad_x) / info.lb_scale, 0.f), sw);
      d.y1 = std::min(std::max((d.y1 - info.lb_pad_y) / info.lb_scale, 0.f), sh);
      d.y2 = std::min(std::max((d.y2 - info.lb_pad_y) / info.lb_scale, 0.f), sh);
    }
    smoother.update(dets);

    char head[256];
    snprintf(head, sizeof(head), "{\"frame_id\":%llu,\"ts_ns\":%lld,\"age_ms\":%.1f,\"infer_ms\":%.1f,\"w\":%u,\"h\":%u,\"dets\":[",
             (unsigned long long)info.frame_id, (long long)info.timestamp_ns, (now_ns() - info.timestamp_ns) / 1e6,
             infer_ms, info.src_width, info.src_height);
    js = head;
    for (size_t i = 0; i < dets.size(); i++) {
      char b[160];
      snprintf(b, sizeof(b), "%s{\"id\":%d,\"cls\":%d,\"score\":%.2f,\"box\":[%.0f,%.0f,%.0f,%.0f]}",
               i ? "," : "", dets[i].track_id, dets[i].cls, dets[i].score, dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);
      js += b;
    }
    js += "]}";
    zmq_send(pub, js.data(), js.size(), ZMQ_DONTWAIT);

    n_stat++; infer_sum += infer_ms;
    if (now_ns() - t_stat > 2000000000LL) {
      double dt = (now_ns() - t_stat) / 1e9;
      fprintf(stderr, "[yolo] %.1f fps, infer avg %.1f ms, dets=%zu\n", n_stat / dt, infer_sum / n_stat, dets.size());
      t_stat = now_ns(); n_stat = 0; infer_sum = 0;
    }
  }
  zmq_close(pub); 
  zmq_close(sub);
  zmq_ctx_term(zctx);

  delete reader;
  return 0;
}