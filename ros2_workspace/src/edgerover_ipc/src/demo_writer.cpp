
#include <chrono>
#include <csignal>
#include <cstdio>   
#include <cstdlib> 
#include <cstring>  
#include <thread>   
#include <vector>   

#include "edgerover_ipc/frame_ring.hpp"

using namespace std::chrono;

// 全局运行标志： 1 程序继续运行，0 程序准备退出。

static volatile std::sig_atomic_t g_running = 1;

/** 
 * @brief 主函数
 * 
 * 模拟摄像头图像采集端，持续生成图像数据，并通过共享内存环形缓冲区发送给其他进程。
 * 
*/

int main(int argc, char ** argv)
{
    // 注册 SIGINT 信号处理函数。
    // 用户按下 Ctrl+C 时，系统发送 SIGINT，将运行标志设置为 0。
    std::signal(SIGINT, [](int) {g_running = 0;});

    // 获取命令行指定的帧率，如果没有指定参数，默认使用 30 FPS。
    const int fps = argc > 1 ? std::atoi(argv[1]) : 30;

    // 640 × 640 × 3 = 1,228,800 字节。
    // YOLO 常用 640 × 640 输入尺寸
    constexpr uint32_t kW = 640, kH = 640, kSize = kW * kH * 3;

    // 创建共享内存写端对象，共享内存名称：/edgerover_frames
    // 环形缓冲区槽位数量 4 ，每个槽位的数据容量：kSize 字节
    edgerover::ipc::FrameRingWriter writer("/edgerover_frames", 4, kSize);

    // 创建图像缓冲区，用于存放当前准备发送的图像数据。
    // vector 会自动管理内存，大小为一帧图像的数据量。
    std::vector<uint8_t> frame(kSize);

    // 打印共享内存配置，方便确认写端启动情况。
    std::printf("[writer] /edgerover_frames: 4 slots x %u bytes, %d fps\n", kSize, fps);

    // 获取当前单调时钟时间，作为第一次发送的时间基准。
    // steady_clock 不受系统时间调整影响，适合计算时间间隔。
    auto next = steady_clock::now();

    // 记录已经写入共享内存的帧数量。
    uint64_t count = 0;

    // 主循环：持续生成图像帧并写入共享内存，直到收到 Ctrl+C。
    while (g_running) 
    {

        // 将整帧图像填充为相同的字节值。
        // 每一帧使用不同的数值：(count + 1) % 256。
        //
        // 例如：
        // 第 1 帧：所有字节都是 1
        // 第 2 帧：所有字节都是 2
        // 第 3 帧：所有字节都是 3
        //
        // 这种方式不是真实图像，只是用于测试共享内存通信。
        // 读端可以逐字节检查，如果一帧中出现不同数值，就说明可能读取到了不同帧的数据，即发生了数据撕裂。
        std::memset(frame.data(), static_cast<int>((count + 1) % 256), kSize);

        // 创建当前帧的元信息结构体：用于描述图像的时间戳、宽度、高度和数据大小。
        edgerover::ipc::FrameInfo info;

        // 获取当前本帧生成的时间戳，并转换为纳秒。
        info.timestamp_ns = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();

        // 设置图像宽度。
        info.width = kW;        // 设置图像宽度。

        info.height = kH;       // 设置图像高度。

        info.data_size = kSize; // 设置当前帧实际数据大小。

        // 将图像数据和元信息写入共享内存环形缓冲区。write()返回写入的帧编号，并更新 count。
        count = writer.write(frame.data(), info);

        // 每写入 5 秒左右的帧数，打印一次运行状态。
        if (count % (fps * 5) == 0) 
        {
            std::printf("[writer] frame %lu\n", count);
        }

        // 计算下一帧应该发送的时间。1000000 / fps：每帧对应的微秒数。
        next += microseconds(1000000 / fps);

        // 休眠到指定时间点，再开始下一轮循环。
        // 相比每次 sleep 固定时长，这种方式以统一时间基准控制帧率，可以减少循环处理耗时造成的累计时间漂移。
        std::this_thread::sleep_until(next);
    }

    // 收到 Ctrl+C 后退出循环，打印最终发送的帧数。
    std::printf("[writer] exit after %lu frames\n", count);

    // 返回 0，表示程序正常结束。
    return 0;
}
