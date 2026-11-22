#include <fcntl.h>       // shm_open、O_CREAT、O_RDWR 等文件操作标志
#include <sys/mman.h>    // mmap、munmap、shm_open、shm_unlink
#include <sys/stat.h>    // fstat、struct stat
#include <unistd.h>      // ftruncate、close

#include <cerrno>        // errno：系统调用错误码
#include <cstring>       // strerror、memcpy
#include <new>           // placement new
#include <stdexcept>     // runtime_error、invalid_argument

#include "edgerover_ipc/frame_ring.hpp"

namespace edgerover::ipc
{
    /*
     * @brief 内部辅助函数
     * 命匿名命名空间：属于 edgerover::ipc 模块的一部分
     */
    namespace
    {
        // 将 value 向上对齐到 align 的整数倍。
        size_t round_up(size_t value, size_t align) 
        {
            return (value + align - 1) / align * align;
        }

        // 每个槽位 = 槽位头 + 数据区，向上对齐到 64 字节（一条 CPU 缓存行），避免相邻槽位互相干扰
        size_t calc_stride(uint32_t slot_size)
        {
            return round_up(sizeof(detail::SlotHeader) + slot_size, 64);
        }

        // [[noreturn]] 属性说明符：告诉编译器，这个函数永远不会正常返回到调用它的地方
        [[noreturn]] void throw_errno(const std::string & what)
        {
            //执行 throw 语句抛出异常，程序的控制流会直接中断并向上层抛出
            throw std::runtime_error(what + ": " + std::strerror(errno));
        }
    }  // namespace

    // ============================ 写端 ============================

    /** 
     * @brief 构造函数：创建并初始化共享内存环形缓冲区。
     * 
     * @param name          共享内存名称；
     * @param slot_count    槽位数量
     * @param slot_size     每个槽位的数据容量。
     *
     * @return 
     */

    FrameRingWriter::FrameRingWriter(const std::string & name, 
                                     uint32_t slot_count, 
                                     uint32_t slot_size): 
                                     name_(name),
                                     slot_stride_(calc_stride(slot_size)) 
    {
        // 至少需要两个槽位，形成环形缓冲区
        if (slot_count < 2) 
        {
            throw std::invalid_argument("slot_count must be >= 2");
        }

        // 删除上次崩溃后可能留下的同名共享内存，然后重新创建
        // 对象转换成字符串指针
        shm_unlink(name_.c_str());

        // 创建共享内存对象，O_RDWR 允许读写，0666 权限
        fd_ = shm_open(name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd_ < 0) 
        {
            throw_errno("shm_open " + name_);
        }

        // 计算共享内存总大小：头部大小 + 槽位步长 × 槽位数量
        total_size_ = sizeof(detail::ShmHeader) + slot_stride_ * slot_count;

        // 设置共享内存对象的实际大小。
        if (ftruncate(fd_, static_cast<off_t>(total_size_)) != 0) 
        {
            throw_errno("ftruncate");
        }

        // 将共享内存映射到当前进程的虚拟地址空间。
        // MAP_SHARED ：其他映射同一共享内存的进程也能看到修改。
        void * ptr = mmap(nullptr, total_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr == MAP_FAILED) 
        {
            throw_errno("mmap");
        }

        // 保存共享内存映射区域的起始地址。
        base_ = static_cast<uint8_t *>(ptr);

        // 定位放置 new placement new：在共享内存这块已有的地址上构造对象（不另外申请内存）
        // 初始化共享内存头部信息。
        header_ = new (base_) detail::ShmHeader{};

        // 写入共享内存
        header_->version    = detail::kVersion; //版本
        header_->slot_count = slot_count;       //槽位数量
        header_->slot_size  = slot_size;        //槽位容量

        // 初始化最新帧编号为 0，表示当前还没有写入图像。
        header_->latest_frame.store(0, std::memory_order_relaxed);

        // 逐个初始化环形缓冲区中的槽位。
        for (uint32_t i = 0; i < slot_count; ++i) 
        {
            // 根据头部大小和槽位步长，计算当前槽位的内存地址并构造对象。
            // auto 自动推断出这个变量的实际数据类型
            auto * s = new (base_ + sizeof(detail::ShmHeader) + i * slot_stride_) detail::SlotHeader{};

            // 初始化当前槽位的序列号。
            s->seq.store(0, std::memory_order_relaxed);
        }

        // 最后写入魔数（release）：通知读端共享内存已经初始化完成。
        // 配合读端 acquire 操作，保证前面的初始化结果对读端可见。
        header_->magic.store(detail::kMagic, std::memory_order_release);
    }

    /** 
     * @brief 析构函数
     * 
     * 解除内存映射、关闭文件描述符，并删除共享内存名称。
     */

    FrameRingWriter::~FrameRingWriter()
    {
        if (base_) 
        {
            munmap(base_, total_size_);
        }
        if (fd_ >= 0) 
        {
            close(fd_);
        }

        shm_unlink(name_.c_str());  // 写端退出时删除共享内存
    }

    /**
     * @brief 根据槽位编号获取对应的槽位头部地址。
     * 
     * @param index         槽位编号；返回值：对应槽位的
     *
     * @return  对应槽位的 SlotHeader 指针。
    */
    detail::SlotHeader * FrameRingWriter::slot(uint64_t index) const
    {
        return reinterpret_cast<detail::SlotHeader *>(base_ + sizeof(detail::ShmHeader) + index * slot_stride_);
    }

    /**
     * @brief 根据槽位编号获取对应的槽位头部地址，向环形缓冲区写入一帧图像
     * 
     * @param index         槽位编号；返回值：对应槽位的
     *
     * @return  对应槽位的 SlotHeader 指针。
    */
    uint64_t FrameRingWriter::write(const void * data, const FrameInfo & info)
    {
        // 检查图像数据大小是否超过单个槽位的容量。
        if (info.data_size > header_->slot_size) 
        {
            throw std::invalid_argument("frame larger than slot_size");
        }

        // 生成新的帧编号，并通过取模运算确定本次使用哪个槽位。
        const uint64_t id = ++next_id_;
        detail::SlotHeader * s = slot(id % header_->slot_count);

        // Seqlock 第 1 步：将序列号变成奇数，表示当前槽位正在写入。
        const uint64_t seq = s->seq.load(std::memory_order_relaxed);
        s->seq.store(seq + 1, std::memory_order_relaxed);

        // 内存屏障：约束后续数据写入的执行顺序。保证读端先看到"奇数"，然后才看到下面写入的新数据
        std::atomic_thread_fence(std::memory_order_release);

        // 第 2 步：写入元信息和图像数据
        s->info = info;

        // 将当前帧编号保存到帧元信息中。
        s->info.frame_id = id;

        // 将实际图像数据复制到当前槽位的头部之后。
        std::memcpy(reinterpret_cast<uint8_t *>(s) + sizeof(detail::SlotHeader), data, info.data_size);

        // 第 3 步：seq 变成偶数（release），表示当前槽位写入完成；数据一定先于这个偶数被读端看到
         // release 保证前面的数据写入先于完成状态发布。
        s->seq.store(seq + 2, std::memory_order_release);

        // 第 4 步：更新"最新帧号"，读端据此找到这一帧
        header_->latest_frame.store(id, std::memory_order_release);

        return id;
    }

    // ============================ 读端 ============================

    /**
     * @brief 打开写端创建的共享内存，并建立只读映射。
     * 
     * @param name 写端使用的共享内存名称
    */
    FrameRingReader::FrameRingReader(const std::string & name)
    {
        // 只读打开共享内存：O_RDONLY 表示只允许读取
        fd_ = shm_open(name.c_str(), O_RDONLY, 0);
        if (fd_ < 0) 
        {
            throw_errno("shm_open " + name + " (writer not running?)");
        }

        // 获取共享内存对象的实际大小。
        struct stat st{};
        if (fstat(fd_, &st) != 0) 
        {
            throw_errno("fstat");
        }

        total_size_ = static_cast<size_t>(st.st_size);

        // 检查共享内存大小是否至少能够容纳头部信息。
        if (total_size_ < sizeof(detail::ShmHeader)) 
        {
            throw std::runtime_error("shared memory too small (writer still initializing?)");
        }

        // 将共享内存映射到当前进程，PROT_READ 表示只读访问。
        void * ptr = mmap(nullptr, total_size_, PROT_READ, MAP_SHARED, fd_, 0);
        if (ptr == MAP_FAILED) 
        {
            throw_errno("mmap");
        }

        base_   = static_cast<const uint8_t *>(ptr);
        header_ = reinterpret_cast<const detail::ShmHeader *>(base_);

        // 检查魔数，确认写端已经完成共享内存初始化。
        // acquire 与写端的 release 配合，建立初始化数据的可见性。
        if (header_->magic.load(std::memory_order_acquire) != detail::kMagic) 
        {
            throw std::runtime_error("bad magic (writer still initializing?)");
        }

        // 检查共享内存版本，避免读写双方使用不同的数据结构定义。
        if (header_->version != detail::kVersion) 
        {
            throw std::runtime_error("FrameRing version mismatch");
        }

        // 根据头部记录的槽位容量，计算每个槽位的实际步长。
        slot_stride_ = calc_stride(header_->slot_size);

        // 检查共享内存总大小是否与槽位数量和步长匹配。
        if (total_size_ < sizeof(detail::ShmHeader) + slot_stride_ * header_->slot_count) 
        {
            throw std::runtime_error("shared memory size mismatch");
        }
    }

    /** 
     * @brief 析构函数：解除共享内存映射并关闭文件描述符
     * 
     * 读端只负责释放自己的资源，不负责删除共享内存名称。
     */
    FrameRingReader::~FrameRingReader()
    {
        if (base_) 
        {
            munmap(const_cast<uint8_t *>(base_), total_size_);
        }
        if (fd_ >= 0) 
        {
            close(fd_);
        }
    }

    /** 
     * @brief 根据槽位编号获取对应的只读槽位地址
     * 
     * @param index 槽位编号；
     * 
     * 返回值：对应槽位的常量指针
    */
    const detail::SlotHeader * FrameRingReader::slot(uint64_t index) const
    {
        return reinterpret_cast<const detail::SlotHeader *>(base_ + sizeof(detail::ShmHeader) + index * slot_stride_);
    }
    
    /** 
     * @brief 读取最新一帧图像，并通过 Seqlock 检查读取期间是否发生数据覆盖
     * 
     * @param out_buffer            接收图像数据的缓冲区
     * @param buffer_size           接收缓冲区的容量
     * @param out_info              输出读取到的帧信息
     * @param last_seen_frame_id    调用方上一次成功读取的帧编号
     * 
     * 返回值：读取状态，包括成功、无新帧、数据冲突和缓冲区不足
    */
    ReadResult FrameRingReader::read_latest(void * out_buffer, size_t buffer_size, FrameInfo & out_info,uint64_t last_seen_frame_id) const
    {
        // 获取写端发布的最新帧编号
        const uint64_t id = header_->latest_frame.load(std::memory_order_acquire);

        // 没有新帧，或者最新帧已经被当前读端读取过。
        if (id == 0 || id <= last_seen_frame_id) 
        {
            return ReadResult::kNoNewFrame;
        }

        // 根据最新帧编号找到对应的槽位。
        const detail::SlotHeader * s = slot(id % header_->slot_count);

        // Seqlock 读端第 1 步：读取序列号。
        // 奇数表示写端正在修改这个槽位，此时放弃读取。
        const uint64_t seq1 = s->seq.load(std::memory_order_acquire);
        if (seq1 & 1U) 
        {
            return ReadResult::kTorn;
        }

        // 第 2 步：复制帧元信息。
        // 如果写端在读取期间修改槽位，后续会通过序列号检查发现。
        FrameInfo info;
        std::memcpy(&info, &s->info, sizeof(info));

        // 检查槽位中的帧编号是否与目标帧一致。
        // 不一致说明槽位可能已经被更新的帧占用。
        if (info.frame_id != id) 
        {
            return ReadResult::kTorn;  // 槽位已经被更新的帧占用了
        }

        // 检查接收缓冲区是否足够大。
        if (info.data_size > buffer_size) 
        {
            return ReadResult::kBufferTooSmall;
        }

        // 将槽位中的图像数据复制到读端自己的缓冲区。
        std::memcpy(out_buffer, reinterpret_cast<const uint8_t *>(s) + sizeof(detail::SlotHeader),info.data_size);

        // 第 3 步：内存屏障之后再次读取序列号。
        // 如果前后序列号不同，说明读取过程中槽位发生了变化，数据作废。
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t seq2 = s->seq.load(std::memory_order_relaxed);
        if (seq1 != seq2) 
        {
            return ReadResult::kTorn;
        }

        // 所有检查通过，将帧信息输出给调用方。
        out_info = info;

        return ReadResult::kOk;
    }

}  // namespace edgerover::ipc