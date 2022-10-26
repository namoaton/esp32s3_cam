#include "device_info.h"
#include "esp_log.h"

const char *DEVICE_TAG = "DOG CAMERA";

void print_device_info(Device_info* dev){
    ESP_LOGI(DEVICE_TAG, "\nDevice info: \n==========\nSSID: %s\nPASS: %s\n==========\n",
            dev->ssid,dev->ssid_pass);
     ESP_LOGI(DEVICE_TAG, "\nOutdoor temperature is %.2f\n",
            dev->temp);

}
void set_temp_device_info(Device_info*dev, float temperature){
    // ESP_LOGI(DEVICE_TAG, "Found address 0X%2X %s",addr, buff);
}