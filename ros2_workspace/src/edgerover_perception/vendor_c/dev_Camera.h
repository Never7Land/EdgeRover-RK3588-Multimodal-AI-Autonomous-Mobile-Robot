/*
 * dev_Camera.h
 *
 * V4L2 摄像头模块接口
 *
 * 本模块负责摄像头的打开、格式设置、缓冲区管理、视频流启停以及帧采集。
 * 上层代码只需要调用 Camera_init() / camera_capture_frame() / Camera_Cleanup()
 * 即可完成一帧图像的获取，无需关心 V4L2 的复杂 ioctl 细节。
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <stddef.h>

#include "common.h"
#include <linux/videodev2.h>

/* ============================================================
 * 数据结构
 * ============================================================ */

/**
 * @brief 描述一个通过 mmap 映射到用户空间的 V4L2 缓冲区。
 */
struct v4l2_buffer_data
{
    void  *start;   /* 用户空间映射起始地址 */
    size_t length;  /* 缓冲区长度（字节） */
    int dmabuf_fd;
};

/**
 * @brief 摄像头上下文，保存打开后的所有状态。
 *
 * 后续所有 camera_* 接口都需要传入该结构体指针。
 */
struct str_CameraCtx
{
    int                         fd;                     /* V4L2 设备文件描述符 */
    struct v4l2_buffer_data     buffers[BUFFER_COUNT];  /* 映射后的缓冲区数组   */
    unsigned int                buffer_count;           /* 实际分配的缓冲区数量 */
    enum v4l2_buf_type          buf_type;               /* 缓冲区类型（多平面） */
};


/* ============================================================
 * 接口函数
 * ============================================================ */

/**
 * @brief 初始化摄像头。
 *
 * 依次完成：打开设备、查询能力、设置 NV12 多平面格式、请求内核缓冲区、
 * mmap 映射、入队，最后启动视频流 (STREAMON)。
 *
 * @param ctx   输出参数，成功后被填充为可用上下文
 * @return      成功返回 0；失败返回 -1
 */
int Camera_init(struct str_CameraCtx *ctx);

/**
 * @brief 从摄像头取出一帧已经填充好的图像数据。
 *
 * 调用者处理完该帧后，需调用 Camera_Queue_Buffer() 将其归还，
 * 否则内核可用缓冲区将耗尽，导致无法继续采集。
 *
 * @param ctx       已初始化的摄像头上下文
 * @param out_buf   输出参数，返回该帧对应的 V4L2 buffer 描述符
 * @return          成功返回 0；失败返回 -1
 */
int Camera_Dequeue_Frame(struct str_CameraCtx *ctx, uint32_t *out_index);

/**
 * @brief 将处理完毕的缓冲区重新入队，供内核再次填充。
 *
 * @param ctx   已初始化的摄像头上下文
 * @param buf   待归还的缓冲区描述符
 * @return      成功返回 0；失败返回 -1
 */
int Camera_Queue_Buffer(struct str_CameraCtx *ctx, uint32_t index);

/**
 * @brief 停止视频流并释放所有 V4L2 资源。
 *
 * @param ctx   已初始化的摄像头上下文
 */
void Camera_Cleanup(struct str_CameraCtx *ctx);

#endif /* CAMERA_H */
