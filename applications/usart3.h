//
// Created by 23391 on 2024/7/31.
//
#include <rtthread.h>

#ifndef RTTHREAD_USART3_H
#define RTTHREAD_USART3_H

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size);

static void serial_thread_entry(void *parameter);

int uart3_entry_point();

#endif //RTTHREAD_USART3_H
