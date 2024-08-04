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
