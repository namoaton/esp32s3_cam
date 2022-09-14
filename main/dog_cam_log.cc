#include "dog_cam_log.h"
#include "mqtt_conf.h"
#include "sdcard_mount.h"
#include "esp_log.h"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG_DOG_CAM = "DOG CAM";
esp_mqtt_client_handle_t dogcam_mqtt_client = NULL;

void dogcam_mqtt_client_init(esp_mqtt_client_handle_t client)
{
    dogcam_mqtt_client = client;
    ESP_LOGI(TAG_DOG_CAM, "dogcam_mqtt_client %p  client %p", dogcam_mqtt_client, client);
}

void dogcam_log(EventBits_t dogcam_bits, uint8_t class_id, uint8_t score)
{
    if (dogcam_bits & WIFI_CONNECTED_BIT)
    {
        // write to mqtt
        ESP_LOGI(TAG_DOG_CAM, "dogcam_mqtt_client %p  class-ID %d score %d",
                 dogcam_mqtt_client, class_id, score);
        mqtt_alarm(dogcam_mqtt_client, class_id, score);
    }

    if (dogcam_bits & WIFI_FAIL_BIT)
    {
        // write to sdcard
        write_to_log_file(class_id, score);
    }
}