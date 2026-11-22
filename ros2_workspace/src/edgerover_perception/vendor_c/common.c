/*
 * common.c
 *
 * 通用工具函数实现
 */

#include "dev_Camera.h"

#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>

/************************************************************
 *                    xioctl
 *------------------------------------------------------------
 * 功能：工具函数
 * 
 *************************************************************/
/**
 * @brief 对 ioctl 的包装，自动处理被信号中断的情况 (EINTR)。
 *
 * @param fd      文件描述符
 * @param request ioctl 请求码
 * @param arg     请求参数
 * @return        ioctl 的返回值；失败时返回 -1 并设置 errno
 */
int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;

    do
    {
        ret = ioctl(fd, request, arg);
    }
    while (ret == -1 && errno == EINTR);

    return ret;
}


/************************************************************
 *                    get_time_ms
 *------------------------------------------------------------
 * 功能：获取时间
 *
 ************************************************************/
double get_time_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}
