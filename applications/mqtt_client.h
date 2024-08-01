#ifndef __MQTT_CLIENT__
#define __MQTT_CLIENT__
#include "mqttclient.h"

static void sub_topic_handle1(void* client, message_data_t* msg);

static int mqtt_publish_handle1(mqtt_client_t *client);

static void kawaii_mqtt_demo(void *parameter);

static int mqtt_publish_handle2(mqtt_client_t *client);

static void publish2(void *parameter);

int ka_mqtt(void);

#endif