#include <rtthread.h>
#include "wifi_connection.h"
#include "stdio.h"
#include "usart3.h"
#include "mqtt_client.h"
#include "aht_10.h"
#include "ap3216c.h"
#include "drv_matrix_led.h"
rt_mq_t mq = RT_NULL;
rt_mq_t lux_mq = RT_NULL;

int main(void)
{

    mq = rt_mq_create("mq_humidity_temperature", PACKAGE_SIZE,
                      5, RT_IPC_FLAG_FIFO);

    lux_mq = rt_mq_create("mq_lux", LUX_PACKAGE_SIZE,
                      5, RT_IPC_FLAG_FIFO);

    wifi_connection(NULL);
    //启动串口3线程
    uart3_entry_point();
    //启动MQTT
    ka_mqtt();
    //启动AHT10
    rt_thread_t thread = rt_thread_create("aht_10_task", AHT_test_entry, RT_NULL, 2048, 24, 10);
    rt_thread_startup(thread);
    //启动光照传感器
    rt_thread_t thread3 = rt_thread_create("lux_task", task_entry, RT_NULL, 2048, 24, 10);
    rt_thread_startup(thread3);

    return 0;
}
