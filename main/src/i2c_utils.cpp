
#include <string.h>
#include  "driver/i2c.h"
#include "hal/i2c_types.h"
#include "esp_log.h"
#include "i2c_utils_component.h"

const char *I2C_UTILS_TAG = "I2C UTILS";
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        // .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_turn_on_periph(){
    int ret;
    uint8_t write_buf[2] = {0x01, 0xff};


    ret = i2c_master_write_to_device(I2C_MASTER_NUM, PCA9554_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

static esp_err_t i2c_PCA9554_set_all_pin_output(){
    int ret;
    uint8_t write_buf[2] = {0x03, 0x00};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, PCA9554_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

static esp_err_t i2c_PCA9554_read_output(){
    int ret;
    uint8_t data[2] ;
    uint8_t len = 1;
    uint8_t reg_addr  = 0x01 ;
    ret =  i2c_master_write_read_device(I2C_MASTER_NUM,PCA9554_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
        
    return ret;
}

static void i2c_info(uint8_t addr){
   char buff[100];
   memset(buff, 0, sizeof(buff));
   switch (addr)
   {
   case PCA9554_ADDR:
    sprintf(buff,"%s","PCA9554" );
    i2c_PCA9554_read_output();
    break;
   
   default:
    break;
   }
   ESP_LOGI(I2C_UTILS_TAG, "Found address 0X%2X %s",addr, buff);

}

esp_err_t i2c_scan(void){
    int ret;

    i2c_PCA9554_set_all_pin_output();
    i2c_turn_on_periph();
    i2c_info(PCA9554_ADDR);
    vTaskDelay(500 / portTICK_RATE_MS);
    for(int addr = 0; addr<128; addr++){
        // ESP_LOGI(I2C_UTILS_TAG, "Looking for address 0X%2X",addr);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    
        if (ret == ESP_OK)
        {
            // printf("Found device at: 0x%2x\n", addr);
            i2c_info(addr);
        }
    }
    return ret;
}


 esp_err_t i2c_read(uint8_t device_address,
                    const uint8_t *write_buffer ,
                    size_t write_size,
                    uint8_t *read_buffer, 
                    size_t read_size) 
                    {
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                     device_address, 
                     write_buffer, 
                     write_size, 
                     read_buffer, 
                     read_size, 
                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
 }