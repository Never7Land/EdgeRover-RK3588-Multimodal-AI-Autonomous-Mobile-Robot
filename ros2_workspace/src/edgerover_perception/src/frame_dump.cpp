#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "edgerover_ipc/frame_ring.hpp"

using edgerover::ipc::ReadResult;

int main(int argc, char ** argv)
{
  const std::string out = argc > 1 ? argv[1] : "/tmp/frame.jpg";
  edgerover::ipc::FrameRingReader reader("/edgerover_frames");
  std::vector<uint8_t> buf(reader.slot_size());
  edgerover::ipc::FrameInfo info;

  // 最多尝试 2 秒，直到读到一帧完整的数据
  for (int i = 0; i < 2000; ++i) {
    if (reader.read_latest(buf.data(), buf.size(), info, 0) == ReadResult::kOk) {
      cv::Mat rgb(info.height, info.width, CV_8UC3, buf.data());
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);   // OpenCV 保存图片要求 BGR 顺序
      cv::imwrite(out, bgr);
      std::printf("saved frame %lu -> %s (src %ux%u, scale=%.4f, pad=%.0f,%.0f)\n",
        info.frame_id, out.c_str(), info.src_width, info.src_height,
        info.lb_scale, info.lb_pad_x, info.lb_pad_y);
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::fprintf(stderr, "no frame received\n");
  return 1;
}