
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>
#include <vector>
#include <unistd.h>

#include "edgerover_ipc/frame_ring.hpp"

using namespace std::chrono;
using edgerover::ipc::ReadResult;

// 控制程序运行状态，Ctrl+C 时将其设置为 0
static volatile std::sig_atomic_t g_running = 1;

// 共享内存读端主函数

int main(int argc, char ** argv)
{
  // 注册 SIGINT 信号处理函数，按下 Ctrl+C 后通知程序退出
  std::signal(SIGINT, [](int) {g_running = 0;});

  // 参数：每帧模拟的处理耗时，单位毫秒（例如 50 模拟一个比较慢的 YOLO）
  const int work_ms = argc > 1 ? std::atoi(argv[1]) : 0;

  // 写端可能还没启动：每 0.5 秒重试一次
  std::unique_ptr<edgerover::ipc::FrameRingReader> reader;
  while (g_running && !reader) 
  {
    try 
    {
      // 创建共享内存读端对象，连接写端创建的共享内存
      reader = std::make_unique<edgerover::ipc::FrameRingReader>("/edgerover_frames");
    } 
    catch (const std::exception & e) 
    {
      // 如果共享内存还不存在，打印等待信息并稍后重试
      std::printf("[reader] waiting for writer: %s\n", e.what());
      std::this_thread::sleep_for(milliseconds(500));
    }
  }

  // 如果读端创建失败或程序收到退出信号，则结束程序
  if (!reader) 
  {
    return 0;
}

  // 创建图像缓冲区，大小与共享内存中单个槽位的容量一致
  std::vector<uint8_t> buf(reader->slot_size());

  // 记录上一次读取的帧 ID，以及各项统计数据
  uint64_t last_id = 0, ok = 0, skipped = 0, torn = 0, corrupt = 0;

  // 累计读取延迟，单位毫秒
  double latency_sum_ms = 0;

  // 设置第一次统计时间：启动后 1 秒
  auto report_at = steady_clock::now() + seconds(1);

  // 持续读取共享内存中的最新图像帧
  while (g_running) 
  {
    // 用于保存当前读取帧的元数据信息
    edgerover::ipc::FrameInfo info;

    // 从环形缓冲区读取最新一帧

    const ReadResult r = reader->read_latest(buf.data(),// 图像数据存放位置
                                             buf.size(),// 缓冲区容量
                                             info,      // 接收帧的元数据
                                             last_id);  // 上一次成功读取的帧 ID

    if (r == ReadResult::kOk) 
    {
        // 校验：整帧必须是同一个值；只要有一个字节不对，就是 bug
        // 写端使用 frame_id % 256 填充整帧，因此读端可以用相同规则校验
        const uint8_t expect = static_cast<uint8_t>(info.frame_id % 256);

        // 遍历当前帧的所有有效数据，检查每个字节是否符合预期
        for (uint32_t i = 0; i < info.data_size; ++i) 
        {
            if (buf[i] != expect) 
            {
            // 发现数据不一致，说明当前帧可能发生了数据损坏
            ++corrupt;
            break;
            }
        }

        // 根据帧 ID 计算两次成功读取之间跳过了多少帧
        // 例如上次读取 ID=10，本次读取 ID=13，则跳过了 11、12 两帧
        if (last_id != 0) 
        {
            skipped += info.frame_id - last_id - 1;
        }

        // 更新上一次读取的帧 ID
        last_id = info.frame_id;

        // 成功读取帧的数量加 1
        ++ok;

        // 获取当前单调时钟时间，转换成纳秒
        const int64_t now = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();

        // 计算从写端写入时间戳到读端读取到数据的时间差
        latency_sum_ms += (now - info.timestamp_ns) / 1e6;

        // 模拟图像处理耗时，例如模拟 YOLO 推理需要 50ms
        if (work_ms > 0) 
        {
            std::this_thread::sleep_for(milliseconds(work_ms));
        }
    } 
    else if (r == ReadResult::kTorn) 
    {
      // 检测到撕裂读取，说明读取期间数据可能被写端修改
      // 当前帧作废，马上重试
      ++torn;
    } 
    else 
    {
      // 没有新帧可读，休息 1ms，避免循环空转占满 CPU
      std::this_thread::sleep_for(milliseconds(1));
    }

    // 每隔 1 秒输出一次统计信息
    if (steady_clock::now() >= report_at) 
    {
        std::printf(
            "[reader pid=%d] fps=%lu skipped=%lu torn=%lu CORRUPT=%lu avg_latency=%.2fms\n",
            getpid(), ok, skipped, torn, corrupt, ok ? latency_sum_ms / ok : 0.0);

        // 清零本次统计数据，开始下一秒的统计
        ok = skipped = torn = 0;
        latency_sum_ms = 0;

        // 将下一次统计时间向后推迟 1 秒
        report_at += seconds(1);
    }
  }

  // 收到退出信号后结束程序
  return 0;
}
