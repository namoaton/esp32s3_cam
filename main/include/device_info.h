#ifndef DEVICEINFO_H
#define DEVICEINFO_H
#include<stdint.h>

struct device_info_s{
      char ssid[40];
      char ssid_pass[40];
      float temp;
      uint8_t user_led;
      uint8_t ir_led;
      uint8_t ultrasound;
      uint8_t vibro;
};

typedef struct device_info_s Device_info;

void print_device_info(Device_info* dev);
void set_temp_device_info(Device_info*dev, float temperature);

#endif