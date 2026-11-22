#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

//在 edgerover 这个命名空间里面，再创建一个 ipc 命名空间
namespace edgerover::ipc
{

    // 一帧的元信息
    // 花括号初始化会进行窄化转换检查
    struct FrameInfo
    {
        uint64_t frame_id{0};       // 帧号，从 1 开始递增（0 表示"还没有帧"）
        int64_t  timestamp_ns{0};   // 采集时间：CLOCK_MONOTONIC，同一台机器上的进程之间可以直接比较
        uint32_t width{0};
        uint32_t height{0};
        uint32_t format{0};         // 像素格式，由使用方自己约定（比如 0=RGB888，1=NV12）
        uint32_t data_size{0};      // 实际数据的字节数

        //letterbox 参数：YOLO 用它把 640x640 上的检测框还原到原图坐标
        uint32_t src_width{0};     // 原图宽度（3840）
        uint32_t src_height{0};    // 原图高度（2160）
        float lb_scale{1.0f};      // 缩放比例
        float lb_pad_x{0.0f};      // 左右填充的像素数
        float lb_pad_y{0.0f};      // 上下填充的像素数
    };

    // 枚举类（enum class） 
    // enum class 的成员受到作用域保护，必须通过 ReadResult:: 访问成员
    enum class ReadResult
    {
        kOk,              // 成功读到一帧完整的新数据
        kNoNewFrame,      // 没有比上次更新的帧
        kTorn,            // 读取过程中被写端覆盖了，这一帧作废（正常现象，下次再读即可）
        kBufferTooSmall,  // 调用者提供的缓冲区放不下这一帧
    };

    namespace detail
    {
        constexpr uint32_t kMagic = 0x45524652;  // 魔数 "ERFR"，用来确认这块内存确实是 FrameRing
        constexpr uint32_t kVersion = 2;

        // 共享内存里不能放 std::string / std::vector / 指针：
        // 它们保存的是本进程的地址，到了另一个进程里就是无效地址。只能放固定大小的普通数据。

        struct alignas(64) ShmHeader
        {
            std::atomic<uint32_t> magic;        // 最后写入，读端看到正确的魔数，就说明初始化已经完成
            uint32_t version;
            uint32_t slot_count;
            uint32_t slot_size;                 // 每个槽位的数据区大小
            std::atomic<uint64_t> latest_frame;  // 最新写完的帧号
        };

        // alignas(64) 要求 SlotHeader 结构体按照 64 字节对齐
        // 为共享内存数据布局和缓存访问效率考虑

        struct alignas(64) SlotHeader
        {
            // atomic<uint64_t>：表示一个具有原子操作特性的 uint64_t 类型变量

            std::atomic<uint64_t> seq;     // seqlock：奇数=正在写，偶数=已写完

            FrameInfo info;
            // 后面紧跟着 slot_size 字节的数据区
        };

        // 跨进程使用的原子变量必须是"无锁"的，否则它内部会用进程私有的锁，跨进程就失效了
        // static_assert(条件, "错误提示"); 条件不满足，编译直接失败，不让程序带着不满足的条件运行

        static_assert(std::atomic<uint64_t>::is_always_lock_free, "need lock-free 64-bit atomics");
        static_assert(std::atomic<uint32_t>::is_always_lock_free, "need lock-free 32-bit atomics");
        
    }  // namespace detail

    // 写端：整个系统里只能有一个（相机进程）
    class FrameRingWriter
    {
        public:
            FrameRingWriter(const std::string & name, uint32_t slot_count, uint32_t slot_size);
            ~FrameRingWriter();
            FrameRingWriter(const FrameRingWriter &) = delete;
            FrameRingWriter & operator=(const FrameRingWriter &) = delete;

            // 写入一帧，返回分配的帧号；info.frame_id 会被忽略
            uint64_t write(const void * data, const FrameInfo & info);

        private:
            detail::SlotHeader * slot(uint64_t index) const;

            std::string name_;
            int fd_{-1};
            uint8_t * base_{nullptr};
            size_t total_size_{0};
            size_t slot_stride_{0};
            detail::ShmHeader * header_{nullptr};
            uint64_t next_id_{0};
    };

    // 读端：可以有任意多个（YOLO、VLM、录像……）
    class FrameRingReader
    {
        public:
            // 如果写端还没创建，或者版本不匹配，会抛出 std::runtime_error
            explicit FrameRingReader(const std::string & name);
            ~FrameRingReader();
            FrameRingReader(const FrameRingReader &) = delete;
            FrameRingReader & operator=(const FrameRingReader &) = delete;

            // 读取最新一帧；只有帧号比 last_seen_frame_id 更大时才会读取
            ReadResult read_latest(void * out_buffer, 
                                    size_t buffer_size, 
                                    FrameInfo & out_info,
                                    uint64_t last_seen_frame_id) const;

            uint32_t slot_size() const {return header_->slot_size;}
            // 供 supervisor 做心跳：写端每写完一帧，这个值就会增长
            uint64_t latest_frame_id() const {return header_->latest_frame.load(std::memory_order_acquire);}

        private:

            const detail::SlotHeader * slot(uint64_t index) const;

            int fd_{-1};
            const uint8_t * base_{nullptr};
            size_t total_size_{0};
            size_t slot_stride_{0};
            const detail::ShmHeader * header_{nullptr};
    };

}  // namespace edgerover::ipc