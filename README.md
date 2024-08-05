# 星火一号 - Home Assistant

**简介** 
>本DIY项目旨在于集成星火一号上的板载外设到Home Assistant，使其用户可以在手机端或者网页上实时的查看室内的湿度，温度，光照强度和控制灯光等。同时
> 可以在HA上实现自动化操作。 比如当光照强度大于50Lux的时候自动关闭或者打开灯光。

演示视频 : <a href="https://www.bilibili.com/video/BV1PyYFeSEWV/?spm_id_from=333.999.0.0&vd_source=fa81ee8dac5a78e9ccb692c6642f6fe2">点击跳转</a>

## 主要程序解析


**依赖模块**

| 模块名称        | 说明                            |
|-------------|-------------------------------|
| RW007       | （软件包）提供WIFI连接                 |
| AHT10       | （软件包）湿度和温度传感器                 |
| 安信可VC01     | 离线语音模块，串口输出                   |
| Kawali MQTT | （软件包）MQTT 客户端，提供基于WIFI的MQTT连接 |
| CJSON       | （软件包）C语言实现的JSON解析             |
| AP3216C     | （软件包）提供距离以及光照强度的检测            |
| RT-Thread   | RTOS                          |


**核心文件说明**


| 模块名称        | 说明                          |
|-------------|-----------------------------|
| [aht_10.c](applications%2Faht_10.c)       | 负责AHT10 湿温度传感器的温度采集（生产者）    |
| [mqtt_client.c](applications%2Fmqtt_client.c)      | 负责MQTT的连接和消息的发布和订阅（消费者和生产者） |
| [usart3.c](applications%2Fusart3.c)     | 负责接收和解析VC01的串口数据包 Uart3     |
| [wifi_connection.c](applications%2Fwifi_connection.c) | 负责wifi的连接                   |
| [drv_matrix_led.c](board%2Fports%2Fled_matrix%2Fdrv_matrix_led.c)     | 板载支持包，用于LED矩阵的点亮            |
| [ap3216c.c](packages%2Fap3216c-latest%2Fap3216c.c)     | 用于距离和光照强度的检测                |


**实现步骤**

1- 创建湿温度和光照强度的消息队列，用于MQTT消费者（发布消息）和 生产者（湿温度和光照强度传感器）的通讯


2- 主线程中，启动WIFI的连接（RW007）

` wifi_connection(NULL);`

**[wifi_connection.c](applications%2Fwifi_connection.c)**
```c

#include "wifi_connection.h"
#include <msh.h>

#define WIFI_SSID "ImmortalWrt"             /*WIFI 名称*/
#define WIFI_PASSWORD "mazha1997"           /*WIFI 密码*/

void wifi_connection(void *args)
{
    char wifi_scan_command[] = "wifi scan";
    char wifi_connection_command[] = "wifi join " WIFI_SSID " " WIFI_PASSWORD;
    msh_exec(wifi_scan_command, rt_strlen(wifi_scan_command));
    msh_exec(wifi_connection_command, rt_strlen(wifi_connection_command));
}

```

3- 启动串口3的线程，等待VC01发送数据。若收到数据，调用drv_matrix_led.h 的函数，点亮或者熄灭LED灯

`uart3_entry_point();`

```c
#include <string.h>
#include "usart3.h"
#include "drv_matrix_led.h"

#define SAMPLE_UART_NAME       "uart3"

/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
static rt_device_t serial;

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size) {
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

static void serial_thread_entry(void *parameter) {
    char ch;
    while (1) {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial, -1, &ch, 1) != 1) {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        /* 打印接收到的字节 */
        rt_kprintf("Received: %x\n", ch);
        /* 读取到的数据通过串口错位输出 */
        if (ch == 1) {  // 对应 '\001'
            turn_on_red_led();
        } else if (ch == 2) {  // 对应 '\002'
            // 蓝色
            turn_on_blue_led();
        } else if (ch == 3) {  // 对应 '\003'
            // 绿色
            turn_on_green_led();
        } else if (ch == 4) {  // 对应 '\004'
            led_matrix_rst();
        } else if (ch == 5) {  // 对应 '\005'
            // 开启水泵
        } else if (ch == 6) {  // 对应 '\006'
            // 关闭水泵
        }
        ch = ch + 1;
        rt_device_write(serial, 0, &ch, 1);
    }
}

int uart3_entry_point() {
    rt_err_t ret = RT_EOK;
    char uart_name[8] = SAMPLE_UART_NAME;
    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial) {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }
    rt_kprintf("fond %s serial!\n", uart_name);
    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 发送字符串 */
    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    } else {
        ret = RT_ERROR;
    }
    return ret;
}

MSH_CMD_EXPORT(uart3_entry_point, uart device dma sample);
```

4- 启动MQTT服务器

` ka_mqtt();`

```C
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mqttclient.h"
#include <msh.h>
#include "cJSON.h"
#include "drv_matrix_led.h"
#ifndef KAWAII_MQTT_HOST
#define KAWAII_MQTT_HOST               "jiejie01.top"
#endif
#ifndef KAWAII_MQTT_PORT
#define KAWAII_MQTT_PORT               "1883"
#endif
#ifndef KAWAII_MQTT_CLIENTID
#define KAWAII_MQTT_CLIENTID           "rtthread001"
#endif
#ifndef KAWAII_MQTT_USERNAME
#define KAWAII_MQTT_USERNAME           "rt-thread"
#endif
#ifndef KAWAII_MQTT_PASSWORD
#define KAWAII_MQTT_PASSWORD           "rt-thread"
#endif
#ifndef KAWAII_MQTT_SUBTOPIC
#define KAWAII_MQTT_SUBTOPIC           "rtt-sub"
#endif
#ifndef KAWAII_MQTT_PUBTOPIC
#define KAWAII_MQTT_PUBTOPIC           "rtt-pub"
#endif

mqtt_client_t *client = NULL;

rt_sem_t sem_mqtt_connection = RT_NULL;

extern rt_mq_t mq;
extern rt_mq_t lux_mq;
extern rt_mq_t rgb_mq;

static void sub_topic_handle1(void *client, message_data_t *msg) {
    (void) client;

    RGBColor_TypeDef rgb;
    rgb.R = 255;
    rgb.G = 255;
    rgb.B = 255;


    cJSON *json = cJSON_Parse(msg->message->payload);

    //获取开关状态
    cJSON *name = cJSON_GetObjectItem(json, "state");



    // 获取 color 对象
    cJSON *color = cJSON_GetObjectItem(json, "color");
    if (color != NULL) {
        // 获取 r, g, b 属性
        cJSON *r = cJSON_GetObjectItem(color, "r");
        cJSON *g = cJSON_GetObjectItem(color, "g");
        cJSON *b = cJSON_GetObjectItem(color, "b");
        // 检查 r, g, b 属性是否存在并打印其值
        if (r != NULL && cJSON_IsNumber(r)) {
            rgb.R = r->valueint;
            rt_kprintf("r: %d\n", r->valueint);
        } else {
            rt_kprintf("The attribute 'r' does not exist or is not a number.\n");
        }

        if (g != NULL && cJSON_IsNumber(g)) {
            rgb.G = g->valueint;
            rt_kprintf("g: %d\n", g->valueint);
        } else {
            rt_kprintf("The attribute 'g' does not exist or is not a number.\n");
        }
        if (b != NULL && cJSON_IsNumber(b)) {
            rgb.B = b->valueint;
            rt_kprintf("b: %d\n", b->valueint);
        } else {
            rt_kprintf("The attribute 'b' does not exist or is not a number.\n");
        }
    }
    cJSON_Delete(json);
    turn_on_led(rgb,  name->valuestring);
}


static void sub_topic_handle2(void *client, message_data_t *msg) {
    (void) client;
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
    KAWAII_MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name,
                      (char *) msg->message->payload);
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
}


static void mqtt_connection(void *parameter) {
    client = NULL;

    rt_thread_delay(6000);

    mqtt_log_init();

    client = mqtt_lease();

    mqtt_set_host(client, KAWAII_MQTT_HOST);
    mqtt_set_port(client, KAWAII_MQTT_PORT);
    mqtt_set_user_name(client, KAWAII_MQTT_USERNAME);
    mqtt_set_password(client, KAWAII_MQTT_PASSWORD);
    mqtt_set_client_id(client, KAWAII_MQTT_CLIENTID);
    mqtt_set_clean_session(client, 1);

    KAWAII_MQTT_LOG_I("The ID of the Kawaii client is: %s \r\n", KAWAII_MQTT_CLIENTID);

    int index = mqtt_connect(client);
    if (index == KAWAII_MQTT_CONNECT_FAILED_ERROR) {
        //尝试重连一次
        rt_kprintf("connection failed ! re-connecting.....\r\n");
        index = mqtt_connect(client);
        if (index == KAWAII_MQTT_CONNECT_FAILED_ERROR) {
            char command[] = "reboot";
            rt_kprintf("connection failed ! rebooting.....\r\n");
            msh_exec(command, rt_strlen(command));
        }
    }
    rt_sem_release(sem_mqtt_connection);
    mqtt_subscribe(client, "home/rgb1/set", QOS0, sub_topic_handle1);
    mqtt_subscribe(client, "home/bedroom/switch1/set", QOS0, sub_topic_handle2);
}

static void temperature_humidity_publish(void *parameter) {
    char payload[PACKAGE_SIZE];
    while (1) {
        if (rt_mq_recv(mq, &payload, sizeof(payload), 1000) > 0) {
            mqtt_message_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.qos = QOS1;
            // msg.payload 直接指向接收到的 payload
            msg.payload = (void *) payload;

            mqtt_publish(client, "office/sensor1", &msg);
        }
    }
}


static void lux_publish_task(void *parameter) {
    char payload[LUX_PACKAGE_SIZE]; // Buffer to hold the string representation of the receive value
    while (1) {
        if (rt_mq_recv(lux_mq, &payload, sizeof(payload), 1000) > 0) {
            mqtt_message_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.qos = QOS1;
            // msg.payload 直接指向接收到的 payload
            msg.payload = (void *) payload;
            mqtt_publish(client, "hah", &msg);
        }
    }
}


int ka_mqtt(void) {
    rt_thread_t tid_mqtt, second_publish, lux_publish;
    sem_mqtt_connection = rt_sem_create("mqtt_connection", 1, RT_IPC_FLAG_FIFO);
    rt_sem_take(sem_mqtt_connection, RT_WAITING_FOREVER);
    tid_mqtt = rt_thread_create("mqtt_connection_task", mqtt_connection, RT_NULL, 2048, 17, 10);
    if (tid_mqtt == RT_NULL) {
        return -RT_ERROR;
    }
    rt_thread_startup(tid_mqtt);
    rt_sem_take(sem_mqtt_connection, RT_WAITING_FOREVER);
    second_publish = rt_thread_create("temp_hum_publish_task", temperature_humidity_publish, RT_NULL, 2048, 17, 10);
    if (second_publish == RT_NULL) {
        return -RT_ERROR;
    }
    rt_thread_startup(second_publish);
    rt_sem_release(sem_mqtt_connection);
    lux_publish = rt_thread_create("lux_publish_task", lux_publish_task, RT_NULL, 2048, 17, 10);
    if (lux_publish == RT_NULL) {
        return -RT_ERROR;
    }
    rt_thread_startup(lux_publish);
    return RT_EOK;
}

MSH_CMD_EXPORT(ka_mqtt, Kawaii MQTT client test program);

```

5- 启动湿温度Task和 光照强度的Task

**启动湿温度Task**

`
rt_thread_t thread = rt_thread_create("aht_10_task", AHT_test_entry, RT_NULL, 2048, 24, 10);
rt_thread_startup(thread);
`

```c
#include "aht_10.h"
#include "mqttclient.h"

//I2C_BUS设备指针，用于等会寻找与记录AHT挂载的I2C总线
struct rt_i2c_bus_device *i2c_bus = RT_NULL;
extern rt_mailbox_t mailbox;

extern rt_mq_t mq;
//AHT命令的空参数
rt_uint8_t Parm_Null[2] = {0, 0};



//写命令（主机向从机传输数据）
rt_err_t write_reg(struct rt_i2c_bus_device *Device, rt_uint8_t reg, rt_uint8_t *data) {
    //代写入的数据
    //数组大小为3的原因：buf[0]--命令（即上面的AHT_CALIBRATION_CMD、AHT_NORMAL_CMD、AHT_GET_DATA_CMD
    //                  buf[1]/buf[2]为命令后跟的参数，AHT有些命令后面需要加上参数，具体可查看数据手册
    rt_uint8_t buf[3];

    //记录数组大小
    rt_uint8_t buf_size;

    //I2C传输的数据结构体
    struct rt_i2c_msg msgs;

    buf[0] = reg;
    if (data != RT_NULL) {
        buf[1] = data[0];
        buf[2] = data[1];
        buf_size = 3;
    } else {
        buf_size = 1;
    }

    msgs.addr = AHT_ADDR;   //消息要发送的地址：即AHT地址
    msgs.flags = RT_I2C_WR; //消息的标志位：读还是写，是否需要忽视ACK回应，是否需要发送停止位，是否需要发送开始位(用于拼接数据使用)...
    msgs.buf = buf;         //消息的缓冲区：待发送/接收的数组
    msgs.len = buf_size;    //消息的缓冲区大小：待发送/接收的数组的大小

    if (rt_i2c_transfer(Device, &msgs, 1) == 1) {
        return RT_EOK;
    } else {
        return RT_ERROR;
    }
}

//读数据（从机向主机返回数据）
rt_err_t read_reg(struct rt_i2c_bus_device *Device, rt_uint8_t len, rt_uint8_t *buf) {
    struct rt_i2c_msg msgs;

    msgs.addr = AHT_ADDR;       //消息要发送的地址：即AHT地址
    msgs.flags = RT_I2C_RD;     //消息的标志位：读还是写，是否需要忽视ACK回应，是否需要发送停止位，是否需要发送开始位(用于拼接数据使用)...
    msgs.buf = buf;             //消息的缓冲区：待发送/接收的数组
    msgs.len = len;             //消息的缓冲区大小：待发送/接收的数组的大小

    //传输函数，上面有介绍
    if (rt_i2c_transfer(Device, &msgs, 1) == 1) {
        return RT_EOK;
    } else {
        return RT_ERROR;
    }
}

//读取AHT的温湿度数据
void read_temp_humi(float *Temp_Data, float *Humi_Data) {
    //根据数据手册我们可以看到要读取一次数据需要使用到的数组大小为6
    rt_uint8_t Data[6];


    write_reg(i2c_bus, AHT_GET_DATA_CMD, Parm_Null);      //发送一个读取命令，让AHT进行一次数据采集
    rt_thread_mdelay(500);                          //等待采集
    read_reg(i2c_bus, 6, Data);                     //读取数据

    //根据数据手册进行数据处理
    *Humi_Data = (Data[1] << 12 | Data[2] << 4 | (Data[3] & 0xf0) >> 4) * 100.0 / (1 << 20);
    *Temp_Data = ((Data[3] & 0x0f) << 16 | Data[4] << 8 | Data[5]) * 200.0 / (1 << 20) - 50;
}

//AHT进行初始化
void AHT_Init(const char *name) {
    //寻找AHT的总线设备
    i2c_bus = rt_i2c_bus_device_find(name);
    if (i2c_bus == RT_NULL) {
        rt_kprintf("Can't Find I2C_BUS Device");    //找不到总线设备
    } else {
        write_reg(i2c_bus, AHT_NORMAL_CMD, Parm_Null);    //设置为正常工作模式
        rt_thread_mdelay(400);

        rt_uint8_t Temp[2];     //AHT_CALIBRATION_CMD需要的参数
        Temp[0] = 0x08;
        Temp[1] = 0x00;
        write_reg(i2c_bus, AHT_CALIBRATION_CMD, Temp);
        rt_thread_mdelay(400);
    }

}

void AHT_test_entry(void *args) {
    float humidity, temperature;
    char payload[PACKAGE_SIZE];
    AHT_Init(AHT_I2C_BUS_NAME);     //进行设备初始化
    while (1) {
        read_temp_humi(&temperature, &humidity);    //读取数据
        snprintf(payload, sizeof(payload), "{\"humidity\":%.1f,\"temperature\":%.1f}", humidity, temperature);
        rt_mq_send(mq, payload, sizeof(payload));
        rt_thread_mdelay(1000);
    }

}
```

**启动光照传感器**

`
//启动光照传感器
rt_thread_t thread3 = rt_thread_create("lux_task", task_entry, RT_NULL, 2048, 24, 10);
rt_thread_startup(thread3);
`

```c
/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-20     Ernest Chen  the first version
 */

#include "rt-thread/include/rtthread.h"
#include "rt-thread/include/rthw.h"
#include "rtdevice.h"
#include "finsh.h"
#include "stdio.h"
#include <string.h>
#include "ap3216c.h"

#ifdef PKG_USING_AP3216C

#define DBG_ENABLE
#define DBG_SECTION_NAME "ap3216c"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include "rt-thread/include/rtdbg.h"

//System Register
#define AP3216C_SYS_CONFIGURATION_REG 0x00 //default
#define AP3216C_SYS_INT_STATUS_REG 0x01
#define AP3216C_SYS_INT_CLEAR_MANNER_REG 0x02
#define AP3216C_IR_DATA_L_REG 0x0A
#define AP3216C_IR_DATA_H_REG 0x0B
#define AP3216C_ALS_DATA_L_REG 0x0C
#define AP3216C_ALS_DATA_H_REG 0x0D
#define AP3216C_PS_DATA_L_REG 0x0E
#define AP3216C_PS_DATA_H_REG 0x0F

//ALS Register
#define AP3216C_ALS_CONFIGURATION_REG 0x10 //range 5:4,persist 3:0
#define AP3216C_ALS_CALIBRATION_REG 0x19
#define AP3216C_ALS_THRESHOLD_LOW_L_REG 0x1A  //bit 7:0
#define AP3216C_ALS_THRESHOLD_LOW_H_REG 0x1B  //bit 15:8
#define AP3216C_ALS_THRESHOLD_HIGH_L_REG 0x1C //bit 7:0
#define AP3216C_ALS_THRESHOLD_HIGH_H_REG 0x1D //bit 15:8

//PS Register
#define AP3216C_PS_CONFIGURATION_REG 0x20
#define AP3216C_PS_LED_DRIVER_REG 0x21
#define AP3216C_PS_INT_FORM_REG 0x22
#define AP3216C_PS_MEAN_TIME_REG 0x23
#define AP3216C_PS_LED_WAITING_TIME_REG 0x24
#define AP3216C_PS_CALIBRATION_L_REG 0x28
#define AP3216C_PS_CALIBRATION_H_REG 0x29
#define AP3216C_PS_THRESHOLD_LOW_L_REG 0x2A  //bit 1:0
#define AP3216C_PS_THRESHOLD_LOW_H_REG 0x2B  //bit 9:2
#define AP3216C_PS_THRESHOLD_HIGH_L_REG 0x2C //bit 1:0
#define AP3216C_PS_THRESHOLD_HIGH_H_REG 0x2D //bit 9:2

#define AP3216C_ADDR 0x1e /*0x3c=0x1e<<1*/


extern rt_mq_t lux_mq;

static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t data)
{
    struct rt_i2c_msg msgs;
    rt_uint8_t temp[2];

    temp[0] = reg;
    temp[1] = data;

    msgs.addr = AP3216C_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = temp;
    msgs.len = 2;

    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("Writing command error");
        return -RT_ERROR;
    }
}

static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = AP3216C_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = AP3216C_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if (rt_i2c_transfer(bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("Reading command error");
        return -RT_ERROR;
    }
}

rt_err_t ap3216c_reset_sensor(ap3216c_device_t dev)
{
    if (dev == RT_NULL)
    {
        return -RT_EINVAL;
    }

    write_reg(dev->i2c, AP3216C_SYS_CONFIGURATION_REG, AP3216C_MODE_SW_RESET); //reset
    rt_thread_mdelay(15); /* need to wait at least 10ms */
    return RT_EOK;
}

/**
 * This function is convenient to getting data except including high and low data for this sensor.
 * note:after reading lower register first,reading higher add one.
 */
static rt_uint32_t read_low_and_high(ap3216c_device_t dev, rt_uint8_t reg, rt_uint8_t len)
{
    rt_uint32_t data;
    rt_uint8_t buf = 0;

    read_regs(dev->i2c, reg, len, &buf); //low
    data = buf;
    read_regs(dev->i2c, reg + 1, len, &buf); //high
    data = data + (buf << len * 8);

    return data;
}

#ifdef AP3216C_USING_HW_INT

/**
 * This function is only used to set threshold without filtering times
 *
 * @param dev the name of ap3216c device
 * @param cmd first register , and other cmd count by it.
 * @param threshold threshold and filtering times of als threshold
 */
static void set_threshold(ap3216c_device_t dev, ap3216c_cmd_t cmd, ap3216c_threshold_t threshold)
{
    ap3216c_set_param(dev, cmd, (threshold.min & 0xff));
    ap3216c_set_param(dev, (ap3216c_cmd_t)(cmd + 1), (threshold.min >> 8));
    ap3216c_set_param(dev, (ap3216c_cmd_t)(cmd + 2), (threshold.max & 0xff));
    ap3216c_set_param(dev, (ap3216c_cmd_t)(cmd + 3), threshold.max >> 8);
}

static void ap3216c_hw_interrupt(void *args)
{
    ap3216c_device_t dev = (ap3216c_device_t)args;

    if (dev->als_int_cb)
    {
        dev->als_int_cb(dev->als_int_cb);
    }
    if (dev->ps_int_cb)
    {
        dev->ps_int_cb(dev->ps_int_cb);
    }
}

static void ap3216c_int_init(ap3216c_device_t dev)
{
    RT_ASSERT(dev);

    rt_pin_mode(AP3216C_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(AP3216C_INT_PIN, PIN_IRQ_MODE_FALLING, ap3216c_hw_interrupt, (void *)dev);
    rt_pin_irq_enable(AP3216C_INT_PIN, PIN_IRQ_ENABLE);
}

/**
 * This function registers als interrupt with callback function
 *
 * @param dev the name of ap3216c device
 * @param enabled enable or disenable als interrupt
 * @param threshold threshold and filtering times of als threshold
 *
 * @param int_cb callback funtion is defined by user.
 */
void ap3216c_int_als_cb(ap3216c_device_t dev, rt_bool_t enabled, ap3216c_threshold_t threshold, ap3216c_int_cb int_cb)
{
    RT_ASSERT(dev);

    if (enabled)
    {
        dev->als_int_cb = int_cb;
        set_threshold(dev, AP3216C_ALS_LOW_THRESHOLD_L, threshold);
    }
    else
    {
        dev->als_int_cb = RT_NULL;
    }
}

/**
 * This function registers ps interrupt with callback function
 *
 * @param dev the name of ap3216c device
 * @param enabled enable or disenable ps interrupt
 * @param threshold threshold and filtering times of ps threshold
 *
 * @param int_cb callback funtion is defined by user.
 */
void ap3216c_int_ps_cb(ap3216c_device_t dev, rt_bool_t enabled, ap3216c_threshold_t threshold, ap3216c_int_cb int_cb)
{
    RT_ASSERT(dev);

    if (enabled)
    {
        dev->ps_int_cb = int_cb;
        set_threshold(dev, AP3216C_PS_LOW_THRESHOLD_L, threshold);
    }
    else
    {
        dev->ps_int_cb = RT_NULL;
    }
}

#endif /* AP3216C_USING_HW_INT */

/**
 * This function initializes ap3216c registered device driver
 *
 * @param dev the name of ap3216c device
 *
 * @return the ap3216c device.
 */
ap3216c_device_t ap3216c_init(const char *i2c_bus_name)
{
    ap3216c_device_t dev;

    RT_ASSERT(i2c_bus_name);

    dev = rt_calloc(1, sizeof(struct ap3216c_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for ap3216c device on '%s' ", i2c_bus_name);
        rt_free(dev);

        return RT_NULL;
    }

    dev->i2c = rt_i2c_bus_device_find(i2c_bus_name);

    if (dev->i2c == RT_NULL)
    {
        LOG_E("Can't find ap3216c device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->lock = rt_mutex_create("mutex_ap3216c", RT_IPC_FLAG_FIFO);
    if (dev->lock == RT_NULL)
    {
        LOG_E("Can't create mutex for ap3216c device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    /* reset ap3216c */
    ap3216c_reset_sensor(dev);


    ap3216c_set_param(dev, AP3216C_SYSTEM_MODE, AP3216C_MODE_ALS_AND_PS);

#ifdef AP3216C_USING_HW_INT
    /* init interrupt mode	*/
    ap3216c_int_init(dev);
#endif /* AP3216C_USING_HW_INT */

    return dev;
}

/**
 * This function releases memory and deletes mutex lock
 *
 * @param dev the pointer of device driver structure
 */
void ap3216c_deinit(ap3216c_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->lock);

    rt_free(dev);
}

/**
 * This function reads temperature by ap3216c sensor measurement
 *
 * @param dev the pointer of device driver structure
 *
 * @return the ambient light converted to float data.
 */
float ap3216c_read_ambient_light(ap3216c_device_t dev)
{
    float brightness = 0.0; // default error data
    rt_err_t result;
    rt_uint32_t read_data;
    rt_uint8_t temp;

    RT_ASSERT(dev);

    result = rt_mutex_take(dev->lock, RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        read_data = read_low_and_high(dev, AP3216C_ALS_DATA_L_REG, 1);

        ap3216c_get_param(dev, AP3216C_ALS_RANGE, &temp);
        if (temp == AP3216C_ALS_RANGE_20661)
        {
            brightness = 0.35 * read_data; //sensor ambient light converse to reality
        }
        else if (temp == AP3216C_ALS_RANGE_5162)
        {
            brightness = 0.0788 * read_data; //sensor ambient light converse to reality
        }
        else if (temp == AP3216C_ALS_RANGE_1291)
        {
            brightness = 0.0197 * read_data; //sensor ambient light converse to reality
        }
        else if (temp == AP3216C_ALS_RANGE_323)
        {
            brightness = 0.0049 * read_data; //sensor ambient light converse to reality
        }
        else
        {
            LOG_E("Failed to get range of ap3216c");
        }
    }
    else
    {
        LOG_E("Failed to reading ambient light");
    }
    rt_mutex_release(dev->lock);

    return brightness;
}

/**
 * This function reads temperature by ap3216c sensor measurement
 *
 * @param dev the pointer of device driver structure
 *
 * @return the proximity data.
 */
uint16_t ap3216c_read_ps_data(ap3216c_device_t dev)
{
    rt_uint16_t proximity = 0;
    rt_err_t result;

    RT_ASSERT(dev);

    result = rt_mutex_take(dev->lock, RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        rt_uint32_t read_data;
        read_data = read_low_and_high(dev, AP3216C_PS_DATA_L_REG, 1); //read two data

        if (1 == ((read_data >> 6) & 0x01 || (read_data >> 14) & 0x01))
        {
            LOG_I("The data of PS is invalid for high intensive IR light ");
        }

        proximity = (read_data & 0x000f) + (((read_data >> 8) & 0x3f) << 4); //sensor proximity converse to reality
    }
    else
    {
        LOG_E("Failed to reading ps data ");
    }
    rt_mutex_release(dev->lock);

    return proximity;
}

/**
 * This function sets parameter of ap3216c sensor
 *
 * @param dev the pointer of device driver structure
 * @param cmd the parameter cmd of device
 * @param value for setting value in cmd register
 *
 * @return the setting parameter status,RT_EOK reprensents setting successfully.
 */
rt_err_t ap3216c_set_param(ap3216c_device_t dev, ap3216c_cmd_t cmd, rt_uint8_t value)
{
    RT_ASSERT(dev);

    switch (cmd)
    {
    case AP3216C_SYSTEM_MODE:
    {
        if (value > AP3216C_MODE_ALS_AND_PS_ONCE)
        {
            LOG_E("Setting system mode parameter is wrong !");
            return -RT_ERROR;
        }
        /* default 000,power down */
        write_reg(dev->i2c, AP3216C_SYS_CONFIGURATION_REG, value);

        break;
    }
    case AP3216C_INT_PARAM:
    {
        if (value > AP3216C_ALS_CLEAR_MANNER_BY_SOFTWARE)
        {
            LOG_E("Setting int parameter is wrong !");
            return -RT_ERROR;
        }
        write_reg(dev->i2c, AP3216C_SYS_INT_CLEAR_MANNER_REG, value);

        break;
    }

    case AP3216C_ALS_RANGE:
    {
        rt_uint8_t args;

        if (!(value == AP3216C_ALS_RANGE_20661 || value == AP3216C_ALS_RANGE_5162 || value == AP3216C_ALS_RANGE_1291 || value == AP3216C_ALS_RANGE_323))
        {
            LOG_E("Setting als dynamic range is wrong, please refer als_range");
            return -RT_ERROR;
        }
        read_regs(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, 1, &args);
        args &= 0xcf;
        args |= value << 4;
        write_reg(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, args);

        break;
    }
    case AP3216C_ALS_PERSIST:
    {
        rt_uint8_t args = 0;

        if (value > 0x0f)
        {
            LOG_E("Setting als persist overflows ");
            return -RT_ERROR;
        }
        read_regs(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, 1, &args);
        args &= 0xf0;
        args |= value;
        write_reg(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, args);

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_L:
    {
        write_reg(dev->i2c, AP3216C_ALS_THRESHOLD_LOW_L_REG, value);

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_H:
    {
        write_reg(dev->i2c, AP3216C_ALS_THRESHOLD_LOW_H_REG, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_L:
    {
        write_reg(dev->i2c, AP3216C_ALS_THRESHOLD_HIGH_L_REG, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_H:
    {
        write_reg(dev->i2c, AP3216C_ALS_THRESHOLD_HIGH_H_REG, value);

        break;
    }
    case AP3216C_PS_GAIN:
    {
        rt_uint8_t args = 0;

        if (value > 0x3)
        {
            LOG_E("Setting ps again overflows ");
            return -RT_ERROR;
        }
        read_regs(dev->i2c, AP3216C_PS_CONFIGURATION_REG, 1, &args);
        args &= 0xf3;
        args |= value;
        write_reg(dev->i2c, AP3216C_PS_CONFIGURATION_REG, args);

        break;
    }
    case AP3216C_PS_PERSIST:
    {
        rt_uint8_t args = 0;

        if (value > 0x3)
        {
            LOG_E("Setting ps persist overflows ");
            return -RT_ERROR;
        }
        read_regs(dev->i2c, AP3216C_PS_CONFIGURATION_REG, 1, &args);
        args &= 0xfc;
        args |= value;
        write_reg(dev->i2c, AP3216C_PS_CONFIGURATION_REG, args);

        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_L:
    {
        if (value > 0x3)
        {
            LOG_E("Setting ps low threshold of low bit is wrong !");
            return -RT_ERROR;
        }
        write_reg(dev->i2c, AP3216C_PS_THRESHOLD_LOW_L_REG, value);

        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_H:
    {
        write_reg(dev->i2c, AP3216C_PS_THRESHOLD_LOW_H_REG, value);

        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_L:
    {
        if (value > 0x3)
        {
            LOG_E("Setting ps high threshold of low bit is wrong !");
            return -RT_ERROR;
        }
        write_reg(dev->i2c, AP3216C_PS_THRESHOLD_HIGH_L_REG, value);

        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_H:
    {
        write_reg(dev->i2c, AP3216C_PS_THRESHOLD_HIGH_H_REG, value);

        break;
    }

    default:
    {
        return -RT_ERROR;
    }
    }

    return RT_EOK;
}

/**
 * This function gets parameter of ap3216c sensor
 *
 * @param dev the pointer of device driver structure
 * @param cmd the parameter cmd of device
 * @param value to get value in cmd register
 *
 * @return the getting parameter status,RT_EOK reprensents getting successfully.
 */
rt_err_t ap3216c_get_param(ap3216c_device_t dev, ap3216c_cmd_t cmd, rt_uint8_t *value)
{
    RT_ASSERT(dev);

    switch (cmd)
    {
    case AP3216C_SYSTEM_MODE:
    {
        read_regs(dev->i2c, AP3216C_SYS_CONFIGURATION_REG, 1, value);

        if (*value > AP3216C_MODE_ALS_AND_PS_ONCE)
        {
            LOG_E("Getting system mode parameter is wrong !");
            return -RT_ERROR;
        }
        break;
    }
    case AP3216C_INT_PARAM:
    {
        read_regs(dev->i2c, AP3216C_SYS_INT_CLEAR_MANNER_REG, 1, value);

        if (*value > AP3216C_ALS_CLEAR_MANNER_BY_SOFTWARE)
        {
            LOG_E("Getting int parameter is wrong !");
            return -RT_ERROR;
        }
        break;
    }
    case AP3216C_ALS_RANGE:
    {
        rt_uint8_t temp;

        read_regs(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, 1, value);
        temp = (*value & 0xff) >> 4;

        if (!(temp == AP3216C_ALS_RANGE_20661 || temp == AP3216C_ALS_RANGE_5162 || temp == AP3216C_ALS_RANGE_1291 || temp == AP3216C_ALS_RANGE_323))
        {
            LOG_E("Getting als dynamic range is wrong, please refer als_range");
            return -RT_ERROR;
        }

        *value = temp;

        break;
    }
    case AP3216C_ALS_PERSIST:
    {
        rt_uint8_t temp;

        read_regs(dev->i2c, AP3216C_ALS_CONFIGURATION_REG, 1, value);
        temp = *value & 0x0f;

        if (temp > 0x0f)
        {
            LOG_E("Getting als persist is wrong, please refer als_range");
            return -RT_ERROR;
        }
        *value = temp;

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_L:
    {
        read_regs(dev->i2c, AP3216C_ALS_THRESHOLD_LOW_L_REG, 1, value);

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_H:
    {
        read_regs(dev->i2c, AP3216C_ALS_THRESHOLD_LOW_H_REG, 1, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_L:
    {
        read_regs(dev->i2c, AP3216C_ALS_THRESHOLD_HIGH_L_REG, 1, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_H:
    {
        read_regs(dev->i2c, AP3216C_ALS_THRESHOLD_HIGH_H_REG, 1, value);

        break;
    }
    case AP3216C_PS_GAIN:
    {
        rt_uint8_t temp;

        read_regs(dev->i2c, AP3216C_PS_CONFIGURATION_REG, 1, &temp);

        *value = (temp & 0xc) >> 2;

        break;
    }
    case AP3216C_PS_PERSIST:
    {
        rt_uint8_t temp;

        read_regs(dev->i2c, AP3216C_PS_CONFIGURATION_REG, 1, &temp);

        *value = temp & 0x3;

        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_L:
    {
        read_regs(dev->i2c, AP3216C_PS_THRESHOLD_LOW_L_REG, 1, value);
        if ((*value & 0xff) > 0x3)
        {
            LOG_E("Getting ps low threshold of low bit is wrong !");
            return -RT_ERROR;
        }
        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_H:
    {
        read_regs(dev->i2c, AP3216C_PS_THRESHOLD_LOW_H_REG, 1, value);
        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_L:
    {
        read_regs(dev->i2c, AP3216C_PS_THRESHOLD_HIGH_L_REG, 1, value);

        if ((*value & 0xff) > 3)
        {
            LOG_E("Getting ps high threshold of low bit is wrong !");
            return -RT_ERROR;
        }
        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_H:
    {
        read_regs(dev->i2c, AP3216C_PS_THRESHOLD_HIGH_H_REG, 1, value);

        break;
    }

    default:
    {
        return -RT_ERROR;
    }
    }

    return RT_EOK;
}

void ap3216c(int argc, char *argv[])
{
    static ap3216c_device_t dev = RT_NULL;

    if (argc > 1)
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (argc > 2)
            {
                /* initialize the sensor when first probe */
                if (!dev || strcmp(dev->i2c->parent.parent.name, argv[2]))
                {
                    /* deinit the old device */
                    if (dev)
                    {
                        ap3216c_deinit(dev);
                    }
                    dev = ap3216c_init(argv[2]);
                }
            }
            else
            {
                rt_kprintf("ap3216c probe <dev_name>   - probe sensor by given name\n");
            }
        }
        else if (!strcmp(argv[1], "read"))
        {
            if (dev)
            {
                rt_uint16_t ps_data;
                float brightness;

                /* read the sensor */
                ps_data = ap3216c_read_ps_data(dev);
//                if (ps_data == 0)
//                {
//                    rt_kprintf("object is not proximity of sensor \n");
//                }
//                else
//                {
//                    rt_kprintf("ap3216c read current ps data      : %d\n", ps_data);
//                }
                char payload[LUX_PACKAGE_SIZE];
                brightness = ap3216c_read_ambient_light(dev);
//                rt_kprintf("ap3216c measure current brightness: %d.%d(lux) \n", (int)brightness, ((int)(10 * brightness) % 10));
                snprintf(payload, sizeof(payload), "{\"psData\":%d,\"brightness\":%.1f}", ps_data, brightness);
                rt_mq_send(lux_mq,payload ,sizeof(payload));
            }
            else
            {
                rt_kprintf("Please using 'ap3216c probe <dev_name>' first\n");
            }
        }
        else
        {
            rt_kprintf("Unknown command. Please enter 'ap3216c' for help\n");
        }
    }
    else
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("ap3216c probe <dev_name>   - probe sensor by given name\n");
        rt_kprintf("ap3216c read               - read sensor ap3216c data\n");
    }
}


void task_entry(void * args)
{
    char *argv[] = {
            "ap3216c",
            "probe",
            "i2c2",
            NULL
    };

    char *argv_read[] = {
            "ap3216c",
            "read",
            NULL
    };
    ap3216c(3, argv);
    while(1)
    {
        ap3216c(2, argv_read);
        rt_thread_delay(200);
    }
}
MSH_CMD_EXPORT(ap3216c, ap3216c sensor function);

#endif /* PKG_USING_AP3216C */

```

## Home assistant 配置

**1- 配置湿温度传感器实体**

>docker exec -it [容器ID] /bin/bash

使用 vi 编辑 configuration.yaml. 在MQTT的下级节点增加。 如下则为订阅office/sensor1 主题，解析主题消息的temperature 作为温度，解析humidity
作为湿度显示在Home Assistant中

```c
sensor:
    - name: "Temperature"
      unique_id: temperature
      state_topic: "office/sensor1"
      suggested_display_precision: 1
      unit_of_measurement: "..C"
      value_template: "{{ value_json.temperature }}"
    - name: "Humidity"
      unique_id: humidity
      state_topic: "office/sensor1"
      unit_of_measurement: "%"
      value_template: "{{ value_json.humidity }}"
```

重载Home assistant 配置
![reload-config.jpg](ass%2Freload-config.jpg)

在配置-> 集成 -> 实体。 在实体目录下可以找到，命名为Temperature 和 Humidity
![humidity-temperature.jpg](ass%2Fhumidity-temperature.jpg)

在dashboard中可以按照下图顺序配置对应的湿度和温度实体。（其他添加卡片的方式和这个步骤一致。后面不再赘述）
![steps.jpg](ass%2Fsteps.jpg)

**2 - 配置光照和距离传感器实体**

使用 vi 编辑 configuration.yaml. 在MQTT的下级节点的sensor下增加。

```c
- name: "Distance"
      unique_id: distance
      state_topic: "hah"
      suggested_display_precision: 1
      unit_of_measurement: "cm"
      value_template: "{{ value_json.psData }}"
    - name: "Lux"
      unique_id: lux
      state_topic: "hah"
      unit_of_measurement: "Lux"
      value_template: "{{ value_json.brightness }}"

```

3 - 配置RGB灯

使用 vi 编辑 configuration.yaml. 在MQTT的下级节点增加light节点。

```c
light:
    - schema: json
      unique_id: light
      optimistic: true
      name: mqtt_json_light_1
      state_topic: "home/rgb122"
      command_topic: "home/rgb1/set"
      brightness: true
      supported_color_modes: ["rgb"]
```


## MQTT 配置

MQTT需要新建以下的Topic

1 - `home/rgb1/set`

2 - `hah`

3 - `office/sensor1`