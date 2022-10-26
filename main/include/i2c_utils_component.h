
#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include <esp_err.h>



#define I2C_MASTER_SCL_IO           5      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

#define PCA9554_ADDR                        0x27
#define _ADDR                               0x23

esp_err_t i2c_master_init();
esp_err_t i2c_scan();
esp_err_t i2c_write(uint8_t sensor_addr,uint8_t reg,uint8_t* data, uint8_t len);
esp_err_t i2c_read(uint8_t device_address,const uint8_t *write_buffer ,size_t write_size,uint8_t *read_buffer, 
                     size_t read_size);

#endif
