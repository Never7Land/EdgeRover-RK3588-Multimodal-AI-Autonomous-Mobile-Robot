#ifndef RGA_HELPER_H
#define RGA_HELPER_H

#include <stddef.h>

/*
 * V4L2 NV12:
 *   输入 2592x1944
 *
 * RGA:
 *   缩放到 640x480
 *
 * 输出仍然是 NV12
 */

int rga_resize_nv12(
    int src_fd,
    int src_width,
    int src_height,
    void *dst,
    int dst_width,
    int dst_height
);

int rga_nv12_to_rgb888(
    int src_fd,
    int src_width,
    int src_height,
    void *dst_rgb,
    int dst_width,
    int dst_height
);

int rga_nv12_to_letterbox_rgb(
    int src_fd,
    int src_width,       // 2592
    int src_height,      // 1944
    void *dst_vir,       // 直接传入 RKNN 的输入内存地址 (640x640 RGB888)
    int dst_width,       // 640
    int dst_height,      // 640
    float *out_ratio,
    float *out_dw,
    float *out_dh
);

#endif