#include "PCT2075.h"
#include "i2c_utils_component.h"
#include "esp_log.h"

esp_err_t pct2075_read_temp(float *temperature)
{
    uint8_t reg_addr = PCT2075_REG_TEMP;
    uint8_t temp_data[2];

    esp_err_t ret = i2c_read(PCT2075_ADDR, &reg_addr, 1, temp_data, sizeof(temp_data));

    ESP_LOGI("I2C READ TEMP", "[0]=0%2X, [1]=0%2X]", temp_data[0], temp_data[1]);
    int8_t sign = 1;
    if (temp_data[0] >> 7)
    {
        temp_data[0] = temp_data[0] & 0x7f;
    }
    uint16_t convert_temp = (temp_data[0] << 8) + temp_data[1];
    convert_temp = convert_temp >> 5;
    *temperature = sign * (float)(convert_temp * 0.125);
    return ret;
};
esp_err_t pct2075_turn_on()
{
    uint8_t reg_addr = PCT2075_REG_CONF;
    uint8_t power_on = 0x00;
    esp_err_t ret = i2c_write(PCT2075_ADDR, &reg, &power_on, 1);
}

esp_err_t pct2075_turn_off()
{
    uint8_t reg_addr = PCT2075_REG_CONF;
    uint8_t power_off = 0x01;
    esp_err_t ret = i2c_write(PCT2075_ADDR, &reg, &power_off, 1);
}