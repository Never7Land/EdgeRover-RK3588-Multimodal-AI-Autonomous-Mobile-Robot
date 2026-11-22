/*
 * camera.c
 *
 * V4L2 摄像头模块相关实现
 */

#include "dev_Camera.h"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/ioctl.h>



/************************************************************
 *                    Camera_Open_and_Querycap
 *------------------------------------------------------------
 * 
 * 功能：打开摄像头设备，并查询驱动能力。
 *
 ************************************************************/
static int Camera_Open_and_Querycap(int *fd, struct v4l2_capability *cap)
{
    *fd = open(VIDEO_DEVICE, O_RDWR);
    if (*fd < 0)
    {
        perror("open /dev/video0");
        return -1;
    }

    memset(cap, 0, sizeof(*cap));

    if (xioctl(*fd, VIDIOC_QUERYCAP, cap) < 0)  //VIDIOC_QUERYCAP 询驱动能力，确认是视频捕获设备
    {
        perror("VIDIOC_QUERYCAP");
        close(*fd);
        return -1;
    }

    printf("V4L2 device opened successfully.\n");
    printf("\n===== V4L2 Device =====\n");
    printf("Driver : %s\n", cap->driver);   //驱动程序名称
    printf("Card   : %s\n", cap->card);     //设备/板卡名称

    return 0;
}


/************************************************************
 *                    Camera_Set_Format
 *------------------------------------------------------------
 *
 * 功能：设置摄像头采集格式 -> NV12 多平面
 *
 ************************************************************/
static int Camera_Set_Format(int fd)
{
    struct v4l2_format fmt; //采集格式

    memset(&fmt, 0, sizeof(fmt));   //防止垃圾数据影响底层驱动校验

    fmt.type                   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;    //数据流类型为：多平面视频捕获
    fmt.fmt.pix_mp.width       = CAMERA_WIDTH;                          //设置图像宽度 (像素)
    fmt.fmt.pix_mp.height      = CAMERA_HEIGHT;                         //设置图像高度 (像素)
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;                     //像素格式为 NV12
    fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;                       //扫描方式：逐行扫描

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) // VIDIOC_S_FMT 设置格式
    {
        perror("VIDIOC_S_FMT");
        return -1;
    }

    printf("\n===== Camera Format =====\n");
    printf("Width  : %u\n", fmt.fmt.pix_mp.width);      // 生效的图像宽度
    printf("Height : %u\n", fmt.fmt.pix_mp.height);     // 生效的图像高度
    printf("Format : %c%c%c%c\n",                       // 将 32 位 FourCC 编码解析为 4 个 ASCII 字符输出
           fmt.fmt.pix_mp.pixelformat & 0xff,
           (fmt.fmt.pix_mp.pixelformat >> 8) & 0xff,
           (fmt.fmt.pix_mp.pixelformat >> 16) & 0xff,
           (fmt.fmt.pix_mp.pixelformat >> 24) & 0xff);
    printf("Planes : %u\n", fmt.fmt.pix_mp.num_planes); // 驱动分配的物理内存平面数量

    return 0;
}


/************************************************************
 *              Camera_Request_and_Map_Buffers
 *------------------------------------------------------------
 *
 * 功能：请求 V4L2 内核缓冲区，并将其映射到用户空间
 *
 ************************************************************/
static int Camera_Request_and_Map_Buffers(struct str_CameraCtx *ctx)
{
    struct v4l2_requestbuffers reqbuf;              //V4L2-框架：向内核驱动申请/释放视频帧缓冲区（Video Buffers）的数据结构

    struct v4l2_buffer buf;                         //V4L2-流式I/O机制：和交换[单个视频帧缓冲区]信息

    struct v4l2_plane  planes[VIDEO_MAX_PLANES];    //V4L2-多平面：描述单帧图像中某一个独立物理/逻辑内存平面

    memset(&reqbuf, 0, sizeof(reqbuf));

    reqbuf.count  = BUFFER_COUNT;                       // 期望申请的帧缓冲区数量 (4个)
    reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; // 捕获模式：多平面 (MPLANE)
    reqbuf.memory = V4L2_MEMORY_MMAP;                   // 内存管理模式：内存映射 (MMAP)

    if (xioctl(ctx->fd, VIDIOC_REQBUFS, &reqbuf) < 0)  //VIDIOC_REQBUFS 申请 V4L2 Buffer
    {
        perror("VIDIOC_REQBUFS");
        return -1;
    }

    ctx->buffer_count = reqbuf.count;   //分配的缓冲区数量
    printf("\nV4L2 buffers allocated: %u\n", reqbuf.count);

    // 循环查询每个缓冲区的信息
    // 使用 mmap 将内核空间内存映射至用户空间

    for (unsigned int i = 0; i < reqbuf.count; i++)
    {
        ctx->buffers[i].dmabuf_fd = -1;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  // 数据流类型
        buf.memory   = V4L2_MEMORY_MMAP;                    //内存类型
        buf.index    = i;                                   //当前要查询的缓冲区索引 (0 ~ reqbuf.count - 1)
        buf.m.planes = planes;                              //绑定多平面信息接收数组
        buf.length   = VIDEO_MAX_PLANES;                    //告诉内核 planes 数组的最大容量

        // QUERYBUF 查询特定缓冲区在内核中的内存参数
        if (xioctl(ctx->fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }

        //记录 Plane 0 的物理总字节大小（以连续存储或首平面为代表）
        ctx->buffers[i].length = planes[0].length;

        //执行系统调用 mmap，将驱动导出的 Plane 0 内存物理偏移映射为用户空间可直接读写的虚拟地址指针

        ctx->buffers[i].start  = mmap( NULL,                    // 映射地址由内核自动选择
                                       planes[0].length,        // 需要映射的内存长度
                                       PROT_READ | PROT_WRITE,  // 映射区保护权限：可读、可写
                                       MAP_SHARED,              // 共享映射：对该内存的修改会直接反映到内核设备中
                                       ctx->fd,                 // 相机设备节点文件描述符
                                       planes[0].m.mem_offset); // 传给 mmap 的底层物理/逻辑偏移量 (Cookie)

        //校验 mmap 映射是否成功
        if (ctx->buffers[i].start == MAP_FAILED)
        {
            perror("mmap V4L2");
            ctx->buffers[i].start = NULL;
            return -1;
        }

        printf("V4L2 buffer %u: %p, %zu bytes\n",
               i,
               ctx->buffers[i].start,       //虚拟内存起始地址
               ctx->buffers[i].length);     //虚拟内存起始大小


        //----------

        struct v4l2_exportbuffer expbuf;

        memset(&expbuf, 0, sizeof(expbuf));

        expbuf.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        expbuf.index = i;
        expbuf.plane = 0;
        expbuf.flags = O_CLOEXEC;

        if (xioctl(ctx->fd, VIDIOC_EXPBUF, &expbuf) < 0)
        {
            perror("VIDIOC_EXPBUF");
            return -1;
        }

        ctx->buffers[i].dmabuf_fd = expbuf.fd;

        printf("V4L2 buffer %u: dma_fd=%d\n",i,ctx->buffers[i].dmabuf_fd);

        //----------
    }

    return 0;
}


/************************************************************
 *              Camera_Enqueue_All
 *------------------------------------------------------------
 *
 * 功能：将所有缓冲区入队，为 STREAMON 做准备
 * 
 * 把应用程序申请到的物理内存地址“报备”并“挂载”到驱动的内部等待队列（Incoming Queue）中
 * 驱动会把这些地址配置给 DMA 控制器
 *
 ************************************************************/
static int Camera_Enqueue_All(struct str_CameraCtx *ctx)
{
    struct v4l2_buffer buf;                             // V4L2 流式 I/O 机制：用于向驱动递交缓冲区信息的控制数据结构
    struct v4l2_plane  planes[VIDEO_MAX_PLANES];        // V4L2 多平面机制：存储单帧图像各平面的属性信息

    /* 遍历所有由 VIDIOC_REQBUFS 分配的缓冲区，逐个执行 QBUF 入队操作 */
    for (unsigned int i = 0; i < ctx->buffer_count; i++)
    {
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  // 数据流类型：多平面捕获
        buf.memory   = V4L2_MEMORY_MMAP;                    // 内存管理模式：MMAP 映射
        buf.index    = i;                                   // 要压入队列的缓冲区索引号
        buf.m.planes = planes;                              // 绑定多平面数据结构指针
        buf.length   = VIDEO_MAX_PLANES;                    // 指定 planes 数组可用的最大空间

        /* 用 VIDIOC_QBUF 将当前索引为 i 的空闲缓冲区转交给内核驱动（交出控制权） */
        if (xioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) 
        {
            perror("VIDIOC_QBUF");
            return -1;
        }
    }

    return 0;
}

/************************************************************
 *              Camera_Stream_On
 *------------------------------------------------------------
 *
 * 功能：启动视频流
 *
 ************************************************************/
static int Camera_Stream_On(struct str_CameraCtx *ctx)
{
    //设置要开启的数据流类型：多平面视频捕获 (Multi-Planar Video Capture)
    ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    //发送 VIDIOC_STREAMON 命令给内核，参数为数据流类型的指针
    if (xioctl(ctx->fd, VIDIOC_STREAMON, &ctx->buf_type) < 0)
    {
        perror("VIDIOC_STREAMON");
        return -1;
    }

    return 0;   // 视频数据流成功开启
}

/************************************************************
 *                      Camera_init
 *------------------------------------------------------------
 *
 * 功能：Camera 初始化
 *
 ************************************************************/
int Camera_init(struct str_CameraCtx *ctx)
{
    struct v4l2_capability cap;

    memset(ctx, 0, sizeof(*ctx));

    if (Camera_Open_and_Querycap(&ctx->fd, &cap) < 0)   // 打开摄像头设备，并查询驱动能力
        return -1;

    if (Camera_Set_Format(ctx->fd) < 0)                 // 置摄像头采集格式 -> NV12 多平面
        goto error_close;

    if (Camera_Request_and_Map_Buffers(ctx) < 0)        //请求 V4L2 内核缓冲区，并将其映射到用户空间
        goto error_unmap;

    if (Camera_Enqueue_All(ctx) < 0)                    //将所有缓冲区入队，为 STREAMON 做准备
        goto error_unmap;

    if (Camera_Stream_On(ctx) < 0)                      //启动视频流
        goto error_unmap;

    return 0;

error_unmap:
    Camera_Cleanup(ctx);
    return -1;

error_close:
    close(ctx->fd);
    ctx->fd = -1;
    return -1;
}

/************************************************************
 *                      Camera_Dequeue_Frame
 *------------------------------------------------------------
 *
 * 功能：从摄像头取出一帧已经填充好的图像数据
 *
 ************************************************************/
int Camera_Dequeue_Frame(struct str_CameraCtx *ctx, 
                         uint32_t *out_index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    if (ctx == NULL || out_index == NULL)
    {
        return -1;
    }

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = VIDEO_MAX_PLANES;

    //从内核的视频输入队列中 “取出” 一帧已经填充好图像数据的缓冲区（Dequeue Buffer）

    if (xioctl(ctx->fd, VIDIOC_DQBUF, &buf) < 0)
    {
        if (errno == EINTR)
        {
            return -2;  /* 被信号中断，上层可选择重试 */
        }

        perror("VIDIOC_DQBUF");
        return -1;
    }

    if (buf.index >= ctx->buffer_count)
    {
        fprintf(stderr,
                "Invalid V4L2 buffer index: %u\n",
                buf.index);
        return -1;
    }

    // 这里的 buf 和 planes 只在执行 VIDIOC_DQBUF 时使用。
    // 函数返回前，把稳定的 buf.index 复制给上层
    *out_index = buf.index;

    return 0;
}


/************************************************************
 *                      Camera_Queue_Buffer
 *------------------------------------------------------------
 *
 * 功能：将处理完毕的缓冲区重新入队，供内核再次填充
 *
 ************************************************************/
int Camera_Queue_Buffer(struct str_CameraCtx *ctx, 
                        uint32_t index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    if (ctx == NULL || index >= ctx->buffer_count)
    {
        return -1;
    }

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.index    = index;
    buf.m.planes = planes;
    buf.length   = VIDEO_MAX_PLANES;

    if (xioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0)
    {
        perror("VIDIOC_QBUF");
        return -1;
    }

    return 0;
}

/************************************************************
 *                      Camera_Cleanup
 *------------------------------------------------------------
 *
 * 功能：停止视频流并释放所有 V4L2 资源
 *
 ************************************************************/
void Camera_Cleanup(struct str_CameraCtx *ctx)
{
    if (ctx->fd < 0)
        return;

    /* 停止视频流 */
    if (ctx->buf_type != 0)
    {
        xioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->buf_type);
    }

    /* 解除 mmap 映射 */
    for (unsigned int i = 0; i < ctx->buffer_count; i++)
    {
        if (ctx->buffers[i].start != NULL)
        {
            munmap(ctx->buffers[i].start, ctx->buffers[i].length);

            if (ctx->buffers[i].dmabuf_fd >= 0)
            {
                close(ctx->buffers[i].dmabuf_fd);
                ctx->buffers[i].dmabuf_fd = -1;
            }
        }
    }

    close(ctx->fd);
    ctx->fd = -1;
}
