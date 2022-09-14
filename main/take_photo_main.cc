#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "main_functions.h"

// wifi credentials
//  #include "wifi_manager.h"
//  #include "http_app.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include "button.h"
#include "dl_lib_matrix3d.h"
#include "fb_gfx.h"
#include "sdcard_mount.h"

#include "driver/gpio.h"
#include "include/app_button.hpp"
#include "include/app_lcd.hpp"
// #define BOARD_ESP32CAM_AITHINKER 1
#define CONFIG_CAMERA_MODULE_ESP_S3_EYE 1
// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

#ifdef CONFIG_CAMERA_MODULE_ESP_S3_EYE
#define CAMERA_MODULE_NAME "ESP-S3-EYE"
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13
#define XCLK_GPIO_NUM 15

#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#endif

EventBits_t bits;
static const char *TAG_0 = "TAKE_PICTURE";

gpio_num_t LED_PIN = GPIO_NUM_4;
gpio_num_t VIBRO_PIN = GPIO_NUM_2;
gpio_num_t POWER_ON_PIN = GPIO_NUM_16;
gpio_num_t CAM_POWER_PIN = GPIO_NUM_32;
dl_matrix3du_t *image_matrix = NULL;

TaskHandle_t tf_task_handle = NULL;
 QueueHandle_t xQueueFrame_2 = xQueueCreate(2, sizeof(camera_fb_t *));
static esp_err_t init_camera();

void led_pin_init()
{
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE; // disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT;                            // set as output mode
  io_conf.pin_bit_mask = (1ULL << LED_PIN);                   // bit mask of the pins that you want to set,
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;               // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                   // disable pull-up mode
  esp_err_t error = gpio_config(&io_conf);                    // configure GPIO with the given settings
  if (error != ESP_OK)
  {
    printf("error configuring outputs \n");
  }
}
void vibro_pin_init()
{
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE; // disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT;                            // set as output mode
  io_conf.pin_bit_mask = (1ULL << VIBRO_PIN);                 // bit mask of the pins that you want to set,
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;                // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                   // disable pull-up mode
  esp_err_t error = gpio_config(&io_conf);                    // configure GPIO with the given settings
  if (error != ESP_OK)
  {
    printf("error configuring outputs \n");
  }
}
void power_on_pin_init()
{
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE; // disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT;                            // set as output mode
  io_conf.pin_bit_mask = (1ULL << POWER_ON_PIN);              // bit mask of the pins that you want to set,
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;               // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                   // disable pull-up mode
  esp_err_t error = gpio_config(&io_conf);                    // configure GPIO with the given settings
  if (error != ESP_OK)
  {
    printf("error configuring outputs \n");
  }
}

/*void start_stream()
{

  // stop  tensorflow task
  stop_tf_task();
  vTaskDelay(300 / portTICK_RATE_MS);
  // reinit camera for big resolution and color
  esp_err_t res = esp_camera_deinit();
  if (res != ESP_OK)
  {
    ESP_LOGE(TAG_0, "cam error %s", esp_err_to_name(res));
  }
  gpio_set_level(CAM_POWER_PIN, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(CAM_POWER_PIN, 1);
  init_camera();
  camera_fb_t *fb = NULL;

  fb = esp_camera_fb_get();
  if (fb == NULL)
  {
    ESP_LOGE(TAG_MQTT, "start stream failed");
    mqtt_command_respond(mqtt_client, START_STREAM, false);
    return;
  }
  ESP_LOGI(TAG_MQTT, "cam init  width - %d px, height - %d px", fb->width, fb->height);
  // image_matrix = dl_matrix3du_alloc(1, 96,96,3);//fb->width, fb->height, 3);
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  // reinit http hadler
  start_webserver();
  ESP_LOGI(TAG_MQTT, "start stream");
  mqtt_command_respond(mqtt_client, START_STREAM, true);
}*/

static camera_config_t camera_config = {};
// };

static esp_err_t init_camera()
{
#ifdef BOARD_ESP32CAM_AITHINKER
  camera_config.pin_pwdn = CAM_PIN_PWDN;
  camera_config.pin_reset = CAM_PIN_RESET;
  camera_config.pin_xclk = CAM_PIN_XCLK;
  camera_config.pin_sscb_sda = CAM_PIN_SIOD;
  camera_config.pin_sscb_scl = CAM_PIN_SIOC;

  camera_config.pin_d7 = CAM_PIN_D7;
  camera_config.pin_d6 = CAM_PIN_D6;
  camera_config.pin_d5 = CAM_PIN_D5;
  camera_config.pin_d4 = CAM_PIN_D4;
  camera_config.pin_d3 = CAM_PIN_D3;
  camera_config.pin_d2 = CAM_PIN_D2;
  camera_config.pin_d1 = CAM_PIN_D1;
  camera_config.pin_d0 = CAM_PIN_D0;
  camera_config.pin_vsync = CAM_PIN_VSYNC;
  camera_config.pin_href = CAM_PIN_HREF;
  camera_config.pin_pclk = CAM_PIN_PCLK;

  // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  camera_config.xclk_freq_hz = 10000000;
  camera_config.ledc_timer = LEDC_TIMER_0;
  camera_config.ledc_channel = LEDC_CHANNEL_0;

  camera_config.pixel_format = PIXFORMAT_GRAYSCALE; // PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
  camera_config.frame_size = FRAMESIZE_CIF;         // FRAMESIZE_SVGA,   // FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  camera_config.jpeg_quality = 12; // 0-63 lower number means higher quality
  camera_config.fb_count = 2;      // if more than one, i2s runs in continuous mode. Use only with JPEG
  camera_config.fb_location = CAMERA_FB_IN_PSRAM;
  camera_config.grab_mode = CAMERA_GRAB_LATEST;
#endif

#ifdef CONFIG_CAMERA_MODULE_ESP_S3_EYE
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer = LEDC_TIMER_0;
  camera_config.pin_d0 = Y2_GPIO_NUM;
  camera_config.pin_d1 = Y3_GPIO_NUM;
  camera_config.pin_d2 = Y4_GPIO_NUM;
  camera_config.pin_d3 = Y5_GPIO_NUM;
  camera_config.pin_d4 = Y6_GPIO_NUM;
  camera_config.pin_d5 = Y7_GPIO_NUM;
  camera_config.pin_d6 = Y8_GPIO_NUM;
  camera_config.pin_d7 = Y9_GPIO_NUM;
  camera_config.pin_xclk = XCLK_GPIO_NUM;
  camera_config.pin_pclk = PCLK_GPIO_NUM;
  camera_config.pin_vsync = VSYNC_GPIO_NUM;
  camera_config.pin_href = HREF_GPIO_NUM;
  camera_config.pin_sscb_sda = SIOD_GPIO_NUM;
  camera_config.pin_sscb_scl = SIOC_GPIO_NUM;
  camera_config.pin_pwdn = PWDN_GPIO_NUM;
  camera_config.pin_reset = -1; // RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 10000000;
  camera_config.pixel_format = PIXFORMAT_GRAYSCALE;
  camera_config.frame_size = FRAMESIZE_CIF;
  camera_config.jpeg_quality = 10;
  camera_config.fb_count = 2;
  camera_config.fb_location = CAMERA_FB_IN_PSRAM;
  camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#endif
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG_0, "Camera Init Failed");
    return err;
  }

  return ESP_OK;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char *str)
{
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

/*static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      res = ESP_FAIL;
    }
    else
    {
      if (fb->width > 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            ESP_LOGI(TAG_0, "JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }


esp_camera_fb_return(pic);

   //  ESP_LOGI(TAG_0, "JPEG length %d",_jpg_buf_len);
    // dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (image_matrix != NULL)
    {
      fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
      // HERE print some text
      if (person_score > 210)
      {
        rgb_print(image_matrix, 0x000000FF, "Person");
      }
      bool jpeg_converted = fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
      if (jpeg_converted == false)
      {
        ESP_LOGI(TAG_0, "JPEG convert failed");
      }
      // dl_matrix3du_free(image_matrix);
    }


  return res;
}*/

void monitoring_task(void *pvParameter)
{
  for (;;)
  {
    ESP_LOGI("Memory task", "free heap: %d", esp_get_free_heap_size());
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void camera_task(void *pvParameter)
{
  for (;;)
  {
    camera_fb_t *fb = NULL;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    fb = esp_camera_fb_get();
    if (!fb)
    {

      ESP_LOGE(TAG_0, "Camera capture failed");
    }
    if (fb->format != PIXFORMAT_JPEG)
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);

      if (!jpeg_converted)
      {
        ESP_LOGI(TAG_0, "JPEG compression failed");
      }
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    write_photo_to_sd(_jpg_buf, _jpg_buf_len);

    esp_camera_fb_return(fb);
    ESP_LOGI("Camera task", "free heap: %d", esp_get_free_heap_size());
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
void get_and_and_save_image()
{
  esp_err_t ret = ESP_OK;
  camera_fb_t *fb = NULL;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  fb = esp_camera_fb_get();
  if (!fb)
  {

    ESP_LOGE(TAG_0, "Camera capture failed");
  }
  if (fb->format != PIXFORMAT_JPEG)
  {
    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);

    if (!jpeg_converted)
    {
      ESP_LOGI(TAG_0, "JPEG compression failed");
    }
  }
  else
  {
    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;
  }
  
  xQueueSend(xQueueFrame_2,&fb, portMAX_DELAY);
  write_photo_to_sd(_jpg_buf, _jpg_buf_len);

  esp_camera_fb_return(fb);
}
extern "C" void app_main()
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  // init_interrupt();
  // sdcard init and get wifi credentials
  char ssid[40];
  char ssid_pass[40];
  memset(ssid, 0, sizeof(ssid));
  memset(ssid_pass, 0, sizeof(ssid_pass));
  // xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);
  sd_card_init();

  // int8_t read_conf_result = read_config_file(ssid, ssid_pass);

  vTaskDelay(2000 / portTICK_RATE_MS);
  // sd_card_unmount();

  if (ESP_OK != init_camera())
  {
    return;
  }

  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  // image_matrix = dl_matrix3du_alloc(1, 96,96,3);//fb->width, fb->height, 3);
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!fb)
  {

    ESP_LOGE(TAG_0, "Camera capture failed");
    return;
  }

  AppButton *key = new AppButton();
  esp_camera_fb_return(fb);
 

  AppLCD *lcd = new AppLCD(key, xQueueFrame_2);
  key->attach(lcd);
  lcd->run();
  key->run();
  // xTaskCreatePinnedToCore(&camera_task, "camera_task", 2048, NULL, 1, NULL, 1);

  while (1)
  {
    vTaskDelay(500 / portTICK_RATE_MS);
    if (key->pressed)
    {
      ESP_LOGE("BUTTON", "Pressed %d", key->pressed);
      get_and_and_save_image();
      key->pressed = BUTTON_IDLE;
    }
  }
}