#ifndef DOG_CAM_LOG_H
#define DOG_CAM_LOG_H
#include <string.h>
#include<stdint.h>
#include "mqtt_client.h"
#include "freertos/event_groups.h"

void dogcam_log(EventBits_t dogcam_bits,uint8_t class_id, uint8_t score);
void dogcam_mqtt_client_init(esp_mqtt_client_handle_t client);
#endif