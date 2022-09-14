#ifndef SDCARD_MOUNT_H
#define SDCARD_MOUNT_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>

#include <mbedtls/base64.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#define MOUNT_POINT "/sdcard"

gpio_num_t SD_CMD = GPIO_NUM_38;
gpio_num_t SD_CLK = GPIO_NUM_39;
gpio_num_t SD_D0 = GPIO_NUM_40;

sdmmc_card_t *card;

static const char *TAG_SDCARD = "SD CARD";

void sd_card_init()
{
  esp_err_t ret;

  // Options for mounting the filesystem.
  // If format_if_mount_failed is set to true, SD card will be partitioned and
  // formatted in case when mounting fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
      .format_if_mount_failed = true,
#else
      .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
      .max_files = 5,
      .allocation_unit_size = 16 * 1024};
  // sdmmc_card_t *card;
  const char mount_point[] = MOUNT_POINT;
  ESP_LOGI(TAG_SDCARD, "Initializing SD card");

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.

  ESP_LOGI(TAG_SDCARD, "Using SDMMC peripheral");
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  // Set bus width to use:
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
  slot_config.width = 4;
#else
  slot_config.width = 1;
#endif

  // On chips where the GPIOs used for SD card can be configured, set them in
  // the slot_config structure:
// #ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
//   slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
//   slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
//   slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
// #ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
//   slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
//   slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
//   slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
// #endif // CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
// #endif // CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
  slot_config.width = 1;
  slot_config.clk = SD_CLK;
  slot_config.cmd = SD_CMD;
  slot_config.d0 = SD_D0;
  // Enable internal pullups on enabled pins. The internal pullups
  // are insufficient however, please make sure 10k external pullups are
  // connected on the bus. This is for debug / example purpose only.
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  ESP_LOGI(TAG_SDCARD, "Mounting filesystem");
  // gpio_pulldown_dis((gpio_num_t)13);
  // gpio_pullup_en((gpio_num_t)13);
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      ESP_LOGE(TAG_SDCARD, "Failed to mount filesystem. "
                           "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
    }
    else
    {
      ESP_LOGE(TAG_SDCARD, "Failed to initialize the card (%s). "
                           "Make sure SD card lines have pull-up resistors in place.",
               esp_err_to_name(ret));
    }
    return;
  }
  ESP_LOGI(TAG_SDCARD, "Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, card);
}

void sd_card_unmount()
{
  esp_vfs_fat_sdmmc_unmount();
}

int8_t read_config_file(char *ssid, char *ssid_pass)

{
  // First create a file.
  // const char *file_hello = MOUNT_POINT"/wif_conf.txt";

  // ESP_LOGI(TAG_SDCARD, "Opening file %s", file_hello);
  // FILE *fh = fopen(file_hello, "w");
  // if (fh == NULL) {
  //       ESP_LOGE(TAG_SDCARD, "Failed to open file for writing");
  //     return;

  // }
  // fprintf(fh, "Hello Max!\n");
  // fclose(fh);
  // ESP_LOGI(TAG_SDCARD, "File written");

  const char *wifi_conf_txt = MOUNT_POINT "/wif_conf.txt";

  // Open renamed file for reading
  ESP_LOGI(TAG_SDCARD, "Reading file %s", wifi_conf_txt);
  FILE *f = fopen(wifi_conf_txt, "r");
  if (f == NULL)
  {
    ESP_LOGE(TAG_SDCARD, "Failed to open file for reading");
    return -1;
  }

  // Read a line from file
  char line[150];
  fgets(line, sizeof(line), f);
  fclose(f);

  sscanf(line, "%s %s", ssid, ssid_pass);
  ESP_LOGI(TAG_SDCARD, "Read conf from file: SSID = '%s' SSID_PASS = '%s'", ssid, ssid_pass);
  // char *pos = strchr(line, '\n');
  // if (pos) {
  //     *pos = '\0';
  // }
  return 0;
}

void write_to_config_file(char *ssid, char *ssid_pass)
{
  const char *wifi_conf_txt = MOUNT_POINT "/wif_conf.txt";
  // todo: check ssid length an password length, check if not double exising config

  ESP_LOGI(TAG_SDCARD, "Opening file %s", wifi_conf_txt);
  FILE *fh = fopen(wifi_conf_txt, "w");
  if (fh == NULL)
  {
    ESP_LOGE(TAG_SDCARD, "Failed to open file for writing");
    return;
  }
  fprintf(fh, "%s %s", ssid, ssid_pass);
  fclose(fh);
  ESP_LOGI(TAG_SDCARD, "File written");
}

void write_to_log_file(uint8_t class_id, uint8_t score)
{
  const char *log_file_path = MOUNT_POINT "/log.txt";
  FILE *log_file = fopen(log_file_path, "w");
  if (log_file == NULL)
  {
    ESP_LOGE(TAG_SDCARD, "Failed to open file for writing");
    return;
  }
  fclose(log_file);
  // todo: add time
  log_file = fopen(log_file_path, "a");
  fprintf(log_file, "%d : %d \n\r", class_id, score);
  fclose(log_file);
  ESP_LOGI(TAG_SDCARD, "Log data written");
}

void write_photo_to_sd( uint8_t *_jpg_buf, size_t _jpg_buf_len )
{
    if(_jpg_buf_len > 0){
    int64_t timestamp = esp_timer_get_time();
    char *pic_name = (char*)malloc(17 + sizeof(int64_t));
    
    sprintf(pic_name,MOUNT_POINT "/pic_%lli.jpg", timestamp);
    ESP_LOGI(TAG_SDCARD, "Opening file %s", pic_name);
    
    FILE *file = fopen(pic_name, "w");
    if (file != NULL){
      fwrite(_jpg_buf, 1, _jpg_buf_len, file);
      ESP_LOGI(TAG_SDCARD, "File saved: %s", pic_name);
    }
    else{
      ESP_LOGE(TAG_SDCARD, "Could not open file %s", pic_name);
    }
    fclose(file);
    free(pic_name);
    free(_jpg_buf);
  }
 
}

#endif