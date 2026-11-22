// supervisor: 按 camera -> yolo -> vlm 拉起子进程，监控退出与卡死，指数退避重启
// 用法: supervisor [install_dir=$HOME/edge_rover/ros2_workspace/install] [model_dir=$HOME/edge_rover/model]
#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>
#include <zmq.h>
#include "edgerover_ipc/frame_ring.hpp"

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

struct Child {
  std::string name;
  std::vector<std::string> argv;
  pid_t pid = -1;
  int fails = 0;                  // 连续失败次数，决定退避时间
  Clock::time_point started{}, next_start{}, last_beat{};
  std::chrono::seconds stall_limit{5};   // 超过这么久没有心跳就判卡死
  bool running() const { return pid > 0; }
};

static void spawn(Child& c) {
  pid_t p = fork();
  if (p == 0) {
    std::vector<char*> av;
    for (auto& s : c.argv) av.push_back(const_cast<char*>(s.c_str()));
    av.push_back(nullptr);
    execv(av[0], av.data());
    perror("execv");
    _exit(127);
  }
  c.pid = p;
  c.started = c.last_beat = Clock::now();
  fprintf(stderr, "[sup] start %s pid=%d\n", c.name.c_str(), (int)p);
}

// SIGTERM，最多等 5s，不退再 SIGKILL
static void stop_child(Child& c) {
  if (!c.running()) return;
  kill(c.pid, SIGTERM);
  for (int i = 0; i < 50; i++) {
    int st;
    if (waitpid(c.pid, &st, WNOHANG) == c.pid) { c.pid = -1; return; }
    std::this_thread::sleep_for(100ms);
  }
  fprintf(stderr, "[sup] %s ignored SIGTERM, SIGKILL\n", c.name.c_str());
  kill(c.pid, SIGKILL);
  waitpid(c.pid, nullptr, 0);
  c.pid = -1;
}

// 登记一次失败并安排退避重启：1,2,4,8,16,30s；稳定运行超过 60s 则清零
static void schedule_restart(Child& c, const char* why) {
  if (Clock::now() - c.started > 60s) c.fails = 0;
  int delay = std::min(30, 1 << std::min(c.fails, 5));
  c.fails++;
  c.next_start = Clock::now() + std::chrono::seconds(delay);
  fprintf(stderr, "[sup] %s %s, restart in %ds (fails=%d)\n", c.name.c_str(), why, delay, c.fails);
}

// ===== SECTION: heartbeat =====
// camera: 共享内存里的 latest_frame 持续不增长 -> 卡死
// yolo  : det PUB 持续没有消息 -> 卡死
// vlm   : 空闲时没输出是正常的，只看 busy 状态持续过久（回答一次约 7s，给 90s 余量）
using edgerover::ipc::FrameRingReader;

static std::unique_ptr<FrameRingReader> g_ring;
static uint64_t g_ring_last_id = 0;
static void* g_zctx = nullptr;
static void* g_det_sub = nullptr;
static void* g_state_sub = nullptr;
static bool g_vlm_busy = false;
static Clock::time_point g_vlm_busy_since{};
constexpr auto kVlmBusyLimit = 90s;

static void* make_sub(const char* ep) {
  void* s = zmq_socket(g_zctx, ZMQ_SUB);
  zmq_setsockopt(s, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(s, ep);   // 对端还没 bind 也没关系，会自动重连
  return s;
}

// 非阻塞取空所有消息，返回条数；最后一条内容（截断到 cap-1）写进 last
static int drain(void* sub, char* last, int cap) {
  int cnt = 0, n;
  char tmp[64];
  while ((n = zmq_recv(sub, tmp, sizeof(tmp), ZMQ_DONTWAIT)) >= 0) {
    cnt++;
    int m = std::min(n, (int)sizeof(tmp));
    m = std::min(m, cap - 1);
    memcpy(last, tmp, m);
    last[m] = 0;
  }
  return cnt;
}

// 子进程停掉后清理对应的监控状态
static void on_child_down(Child& c) {
  if (c.name == "camera") { g_ring.reset(); g_ring_last_id = 0; }   // 下次重新映射共享内存
  if (c.name == "vlm") g_vlm_busy = false;
}

// 返回 false 表示判定卡死
static bool check_heartbeat(Child& c) {
  const auto now = Clock::now();
  char last[32] = {0};

  if (c.name == "camera") {
    if (!g_ring) {
      try { g_ring = std::make_unique<FrameRingReader>("/edgerover_frames"); g_ring_last_id = 0; }
      catch (const std::exception&) {}   // 共享内存还没建好，靠启动宽限期兜底
    }
    if (g_ring) {
      uint64_t id = g_ring->latest_frame_id();
      if (id != g_ring_last_id) { g_ring_last_id = id; c.last_beat = now; }
    }
  } else if (c.name == "yolo") {
    if (drain(g_det_sub, last, sizeof(last)) > 0) c.last_beat = now;
  } else if (c.name == "vlm") {
    if (drain(g_state_sub, last, sizeof(last)) > 0) {
      bool b = strcmp(last, "vlm busy") == 0;
      if (b && !g_vlm_busy) g_vlm_busy_since = now;
      g_vlm_busy = b;
    }
    return !(g_vlm_busy && now - g_vlm_busy_since > kVlmBusyLimit);
  }

  if (c.stall_limit.count() == 0) return true;
  return now - c.last_beat <= c.stall_limit;   // last_beat 在 spawn 时初始化，启动期自带宽限
}

int main(int argc, char** argv) {
  const char* home = getenv("HOME") ? getenv("HOME") : "/root";
  std::string inst = argc > 1 ? argv[1] : std::string(home) + "/edge_rover/ros2_workspace/install";
  std::string mdl = argc > 2 ? argv[2] : std::string(home) + "/edge_rover/model";

  struct sigaction sa{};
  sa.sa_handler = on_sig;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  g_zctx = zmq_ctx_new();
  g_det_sub = make_sub("ipc:///tmp/edgerover_det");
  g_state_sub = make_sub("ipc:///tmp/edgerover_state");

  std::vector<Child> kids(3);
  kids[0].name = "camera";
  kids[0].argv = {inst + "/edgerover_perception/lib/edgerover_perception/camera_proc"};
  kids[1].name = "yolo";
  kids[1].argv = {inst + "/edgerover_yolo/lib/edgerover_yolo/yolo_proc", mdl + "/yolov5s_rk3588.rknn"};
  kids[2].name = "vlm";
  kids[2].argv = {inst + "/edgerover_vlm/lib/edgerover_vlm/vlm_proc",
                  mdl + "/qwen3vl/qwen3-vl-2b_vision_rk3588.rknn",
                  mdl + "/qwen3vl/qwen3-vl-2b-instruct_w8a8_rk3588.rkllm"};

  kids[0].stall_limit = 8s;   // 4K 相机启动 + 出第一帧留足时间
  kids[1].stall_limit = 10s;  // rknn_init + 等 camera 创建共享内存
  kids[2].stall_limit = 0s;   // vlm 忙时长时间无输出属正常，另行处理

  for (auto& k : kids) { spawn(k); std::this_thread::sleep_for(1s); }

  while (!g_stop) {
    for (size_t i = 0; i < kids.size(); i++) {
      Child& k = kids[i];
      if (k.running()) {
        int st;
        const char* why = nullptr;
        if (waitpid(k.pid, &st, WNOHANG) == k.pid) { k.pid = -1; why = "exited"; }
        else if (!check_heartbeat(k)) { stop_child(k); why = "stalled"; }

        if (why) {
          schedule_restart(k, why);
          on_child_down(k);
          if (i == 0) {   // camera 重启：yolo/vlm 手里的 reader 和 last_id 作废，必须联动重启
            for (size_t j = 1; j < kids.size(); j++) {
              stop_child(kids[j]);
              on_child_down(kids[j]);
              kids[j].fails = 0;
              kids[j].next_start = k.next_start + std::chrono::seconds(2 * (int)j);
              fprintf(stderr, "[sup] camera down -> restart %s after camera\n", kids[j].name.c_str());
            }
          }
        }
      } else if (Clock::now() >= k.next_start) {
        spawn(k);
      }
    }
    std::this_thread::sleep_for(200ms);
  }

  fprintf(stderr, "[sup] shutting down\n");
  for (auto it = kids.rbegin(); it != kids.rend(); ++it) stop_child(*it);   // 反序停：vlm -> yolo -> camera
  zmq_close(g_det_sub);
  zmq_close(g_state_sub);
  zmq_ctx_term(g_zctx);
  return 0;
}
