#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mqttclient.h"

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

static void sub_topic_handle1(void* client, message_data_t* msg)
{
    (void) client;
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
    KAWAII_MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name, (char*)msg->message->payload);
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
}


static int mqtt_publish_handle1(mqtt_client_t *client)
{
    mqtt_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.qos = QOS0;
    msg.payload = (void *)"this is a kawaii mqtt test ...";

    return mqtt_publish(client, KAWAII_MQTT_PUBTOPIC, &msg);
}

static void kawaii_mqtt_demo(void *parameter)
{
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

    KAWAII_MQTT_LOG_I("The ID of the Kawaii client is: %s ", KAWAII_MQTT_CLIENTID);

    mqtt_connect(client);

    rt_sem_release(sem_mqtt_connection);
    mqtt_subscribe(client, KAWAII_MQTT_SUBTOPIC, QOS0, sub_topic_handle1);
    
    while (1) {
        mqtt_publish_handle1(client);
                               
        mqtt_sleep_ms(4 * 1000);
    }
}


static int mqtt_publish_handle2(mqtt_client_t *client)
{
    mqtt_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.qos = QOS0;
    msg.payload = (void *)"this is a kawaii mqtt test ...";
    return mqtt_publish(client, "/aht10/test", &msg);
}


static void publish2(void *parameter) {
    while (1) {
        mqtt_publish_handle2(client);
        mqtt_sleep_ms(4 * 1000);
    }
}

int ka_mqtt(void)
{
    rt_thread_t tid_mqtt, second_publish;
    sem_mqtt_connection = rt_sem_create("mqtt_connection", 1, RT_IPC_FLAG_FIFO);
    rt_sem_take(sem_mqtt_connection,RT_WAITING_FOREVER);
    tid_mqtt = rt_thread_create("kawaii_demo", kawaii_mqtt_demo, RT_NULL, 2048, 17, 10);
    if (tid_mqtt == RT_NULL) {
        return -RT_ERROR;
    }
    rt_thread_startup(tid_mqtt);
    rt_sem_take(sem_mqtt_connection,RT_WAITING_FOREVER);
    second_publish = rt_thread_create("second_publish", publish2, RT_NULL, 2048, 17, 10);
    if (second_publish == RT_NULL) {
        return -RT_ERROR;
    }
    rt_thread_startup(second_publish);
    rt_sem_release(sem_mqtt_connection);

    return RT_EOK;
}

MSH_CMD_EXPORT(ka_mqtt, Kawaii MQTT client test program);
