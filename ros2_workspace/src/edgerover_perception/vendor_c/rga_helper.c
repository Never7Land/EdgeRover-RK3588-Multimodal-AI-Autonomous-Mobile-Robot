#include "rga_helper.h"

#include <stdio.h>

#include "im2d.h"
#include "rga.h"

#include <stdio.h>
#include <rga/RgaApi.h>
#include <rga/im2d.h>

#include <math.h>

// 升级版：输入 NV12 (FD方式)，直接输出 RGB888 / BGR888 (虚拟地址方式)
int rga_nv12_to_rgb888(
    int src_fd,
    int src_width,
    int src_height,
    void *dst_rgb,
    int dst_width,
    int dst_height
)
{
    rga_info_t src_info;
    rga_info_t dst_info;

    memset(&src_info, 0, sizeof(rga_info_t));
    memset(&dst_info, 0, sizeof(rga_info_t));

    // 1. 源图像：2592x1944, NV12 格式
    src_info.fd = src_fd;
    src_info.mmuFlag = 1;
    rga_set_rect(&src_info.rect, 0, 0, src_width, src_height, src_width, src_height, RK_FORMAT_YCbCr_420_SP);

    // 2. 目标图像：640x480, RGB888 格式 (如果需要 BGR，改为 RK_FORMAT_BGR_888)
    dst_info.virAddr = dst_rgb;
    dst_info.mmuFlag = 1;
    rga_set_rect(&dst_info.rect, 0, 0, dst_width, dst_height, dst_width, dst_height, RK_FORMAT_RGB_888);

    // 3. 执行硬件缩放 + 格式转换
    int ret = c_RkRgaBlit(&src_info, &dst_info, NULL);
    if (ret < 0) {
        printf("[RGA] c_RkRgaBlit (NV12->RGB888) failed! ret=%d\n", ret);
        return -1;
    }

    return 0;
}


int rga_resize_nv12(
    int src_fd,
    int src_width,
    int src_height,
    void *dst,
    int dst_width,
    int dst_height
)
{
    rga_info_t src_info;
    rga_info_t dst_info;

    memset(&src_info, 0, sizeof(rga_info_t));
    memset(&dst_info, 0, sizeof(rga_info_t));

    // 1. 设置源图像属性 (2592x1944 FD 方式)
    src_info.fd = src_fd;
    src_info.mmuFlag = 1; // 开启 MMU
    rga_set_rect(&src_info.rect, 0, 0, src_width, src_height, src_width, src_height, RK_FORMAT_YCbCr_420_SP);

    // 2. 设置目标图像属性 (640x480 VirtualAddr 方式)
    dst_info.virAddr = dst;
    dst_info.mmuFlag = 1; // 开启 MMU
    rga_set_rect(&dst_info.rect, 0, 0, dst_width, dst_height, dst_width, dst_height, RK_FORMAT_YCbCr_420_SP);

    // 3. 调用底层经典 RGA API 执行 Blit/Resize
    int ret = c_RkRgaBlit(&src_info, &dst_info, NULL);
    if (ret < 0) {
        printf("[RGA] c_RkRgaBlit failed! ret=%d\n", ret);
        return -1;
    }

    return 0;
}

/************************************************************
 *            rga_nv12_to_letterbox_rgb
 *-----------------------------------------------------------
 * 功能：利用 RGA 硬件一步到位完成：
 *       2592x1944 (NV12) -> 等比例缩放 + 色彩转换 + 居中补黑边 -> 640x640 (RGB888)
 ************************************************************/
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
) {
    if (src_fd < 0 || dst_vir == NULL) return -1;

    // 1. 动态计算 Letterbox 缩放比例与上下/左右偏移量（供后处理还原坐标）
    float scale = fminf((float)dst_width / src_width, (float)dst_height / src_height);
    int target_w = (int)roundf(src_width * scale);
    int target_h = (int)roundf(src_height * scale);

    int pad_x = (dst_width - target_w) / 2;
    int pad_y = (dst_height - target_h) / 2;

    if (out_ratio) *out_ratio = scale;
    if (out_dw)    *out_dw = (float)pad_x;
    if (out_dh)    *out_dh = (float)pad_y;

    // 2. 将目标 640x640 RGB 画布先清零（自动填充黑边）
    memset(dst_vir, 0, dst_width * dst_height * 3);

    // 3. 配置源图像属性 (2592x1944 NV12, DMA-BUF 方式)
    rga_info_t src_info;
    memset(&src_info, 0, sizeof(rga_info_t));
    src_info.fd = src_fd;
    src_info.mmuFlag = 1;
    rga_set_rect(&src_info.rect, 0, 0, src_width, src_height, src_width, src_height, RK_FORMAT_YCbCr_420_SP);

    // 4. 配置目标图像属性 (居中绘制到 640x640 RGB888 画布)
    rga_info_t dst_info;
    memset(&dst_info, 0, sizeof(rga_info_t));
    dst_info.virAddr = dst_vir;
    dst_info.mmuFlag = 1;
    // 【硬件关键点】：源图会被 RGA 硬件自动缩放到 (target_w, target_h)，并贴在画布的 (pad_x, pad_y) 位置
    rga_set_rect(&dst_info.rect, pad_x, pad_y, target_w, target_h, dst_width, dst_height, RK_FORMAT_RGB_888);

    // 5. 调用底层 RGA 硬件加速 Blit
    int ret = c_RkRgaBlit(&src_info, &dst_info, NULL);
    if (ret < 0) {
        printf("[RGA] c_RkRgaBlit failed! ret=%d\n", ret);
        return -1;
    }

    return 0;
}