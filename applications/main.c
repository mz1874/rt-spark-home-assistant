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

#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include "wifi_connection.h"
#include "stdio.h"
#include <rtdevice.h>
#include "usart3.h"
#define GPIO_LED_B    GET_PIN(F, 11)
#define GPIO_LED_R    GET_PIN(F, 12)

int main(void)
{
    wifi_connection(NULL);
    rt_pin_mode(GPIO_LED_R, PIN_MODE_OUTPUT);
    //启动串口3线程
    uart3_entry_point();
    return 0;
}
