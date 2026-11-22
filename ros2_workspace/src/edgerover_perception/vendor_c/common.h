/*
 * dev_Camera.h
 *
 * 通用头文件
 *
 * 本文件集中存放整个工程都会用到的宏定义、结构体声明和工具函数原型。
 * 避免每个模块都重复定义相同的常量，同时统一错误处理风格。
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 * 设备节点与图像参数
 * ============================================================ */

#define VIDEO_DEVICE    "/dev/video44"      /* V4L2 摄像头设备节点 */
#define DRM_DEVICE      "/dev/dri/card0"    /* DRM 显示设备节点 */

#define CAMERA_WIDTH    (3840)              /* IMX415 分辨率 */
#define CAMERA_HEIGHT   (2160)              

#define BUFFER_COUNT    (4)                 /* V4L2 请求的内核缓冲区数量 */



/**
 * @brief 对 ioctl 进行包装，自动处理被信号中断的情况 (EINTR)。
 *
 * @param fd      文件描述符
 * @param request ioctl 请求码
 * @param arg     请求参数
 * @return        ioctl 的返回值；失败时返回 -1 并设置 errno
 */
int xioctl(int fd, unsigned long request, void *arg);

/**
 * @brief 获取单调时钟，返回自某个未指定起点以来的毫秒数。
 *
 * 常用于计算帧率 (FPS) 或单次处理耗时。
 *
 * @return 毫秒数
 */
double get_time_ms(void);

#endif /* COMMON_H */
