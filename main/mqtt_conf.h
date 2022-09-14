#ifndef MQTT_CONF_H
#define MQTT_CONF_H
#include<stdint.h>
#include "mqtt_client.h"

#define CONF_MQTT_SERVER "159.89.104.215"
#define CONF_MQTT_USER "dogcam"
#define CONF_MQTT_PASS "134679test"
#define INFO_TOPIC "/dogcam/info"
#define COMMAND_TOPIC "/dogcam/command"
#define ALARM_TOPIC "/dogcam/alarm"

int mqtt_data_processing(char* data);
void mqtt_command_respond(esp_mqtt_client_handle_t client,uint8_t command_id, bool status);
void mqtt_alarm(esp_mqtt_client_handle_t client,uint8_t class_id, uint8_t score);

enum mqtt_command {
    LIGHT_ON = 1 ,
    LIGHT_OFF ,
    START_STREAM ,
    STOP_STREAM ,
    VIBRO_ON ,
    VIBRO_OFF ,
    PLAY_COMMAND_1 ,
    PLAY_COMMAND_2 ,
    PLAY_COMMAND_3 ,
    PLAY_COMMAND_4 ,
    PLAY_COMMAND_5 ,
};



#endif