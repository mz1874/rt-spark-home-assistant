/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-06     Supperthomas first version
 * 2023-12-03     Meco Man     support nano version
 */

#include <rtthread.h>
#include "wifi_connection.h"
#include "stdio.h"
#include "usart3.h"
#include "mqtt_client.h"
#include "aht_10.h"
#include "ap3216c.h"

rt_mq_t mq = RT_NULL;
rt_mq_t lux_mq = RT_NULL;

int main(void)
{

    mq = rt_mq_create("mq", PACKAGE_SIZE,
                      5, RT_IPC_FLAG_FIFO);

    lux_mq = rt_mq_create("lux_mq", LUX_PACKAGE_SIZE,
                      5, RT_IPC_FLAG_FIFO);

    wifi_connection(NULL);
    //启动串口3线程
    uart3_entry_point();
    //启动MQTT
    ka_mqtt();
    //启动AHT10
    rt_thread_t thread = rt_thread_create("aht_10", AHT_test_entry, RT_NULL, 2048, 24, 10);
    rt_thread_startup(thread);

    rt_thread_t thread3 = rt_thread_create("lux", task_entry, RT_NULL, 2048, 24, 10);
    rt_thread_startup(thread3);
    return 0;
}
