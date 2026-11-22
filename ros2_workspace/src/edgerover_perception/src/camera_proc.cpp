#include <chrono>
#include <csignal>
#include <cstdio>
#include <vector>

#include "edgerover_ipc/frame_ring.hpp"

// 你的驱动是 C 代码：用 extern "C" 包起来，告诉 C++ 编译器按 C 的方式链接这些函数
extern "C" {
#include "common.h"
#include "dev_Camera.h"
#include "rga_helper.h"
}

using namespace std::chrono;
static volatile std::sig_atomic_t g_running = 1;

static int64_t now_ns()
{
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

int main()
{
  std::signal(SIGINT, [](int) {g_running = 0;});
  std::signal(SIGTERM, [](int) {g_running = 0;});  // systemd 停止服务时发送的是 SIGTERM

  constexpr uint32_t kOutW = 640, kOutH = 640, kOutSize = kOutW * kOutH * 3;

  str_CameraCtx cam{};
  if (Camera_init(&cam) < 0) {
    std::fprintf(stderr, "[camera] Camera_init failed (%s)\n", VIDEO_DEVICE);
    return 1;
  }

  edgerover::ipc::FrameRingWriter writer("/edgerover_frames", 4, kOutSize);
  std::vector<uint8_t> rgb(kOutSize);
  std::printf("[camera] %s %dx%d -> FrameRing 640x640 RGB\n", VIDEO_DEVICE, CAMERA_WIDTH, CAMERA_HEIGHT);

  uint64_t frames = 0, rga_fail = 0;
  double rga_ms_sum = 0;
  auto report_at = steady_clock::now() + seconds(5);

  while (g_running) {
    uint32_t idx = 0;
    const int ret = Camera_Dequeue_Frame(&cam, &idx);
    if (ret == -2) {continue;}   // 暂时没有帧（和你原来代码里的约定一致）
    if (ret < 0) {break;}
    const int64_t ts = now_ns(); // 取帧时刻作为时间戳

    float scale = 1.f, pad_x = 0.f, pad_y = 0.f;
    const int64_t t0 = now_ns();
    const int r = rga_nv12_to_letterbox_rgb(
      cam.buffers[idx].dmabuf_fd, CAMERA_WIDTH, CAMERA_HEIGHT,
      rgb.data(), kOutW, kOutH, &scale, &pad_x, &pad_y);
    rga_ms_sum += (now_ns() - t0) / 1e6;

    // RGA 用完后立刻把 buffer 还给驱动（不管成功失败都要还，否则驱动的 buffer 会被耗尽）
    Camera_Queue_Buffer(&cam, idx);
    if (r < 0) {++rga_fail; continue;}

    edgerover::ipc::FrameInfo info;
    info.timestamp_ns = ts;
    info.width = kOutW;
    info.height = kOutH;
    info.format = 0;             // 约定 0 = RGB888
    info.data_size = kOutSize;
    info.src_width = CAMERA_WIDTH;
    info.src_height = CAMERA_HEIGHT;
    info.lb_scale = scale;
    info.lb_pad_x = pad_x;
    info.lb_pad_y = pad_y;
    writer.write(rgb.data(), info);
    ++frames;

    if (steady_clock::now() >= report_at) {
      std::printf("[camera] fps=%.1f rga_avg=%.2fms rga_fail=%lu\n",
        frames / 5.0, frames ? rga_ms_sum / frames : 0.0, rga_fail);
      frames = rga_fail = 0;
      rga_ms_sum = 0;
      report_at += seconds(5);
    }
  }

  Camera_Cleanup(&cam);
  std::printf("[camera] exit\n");
  return 0;
}