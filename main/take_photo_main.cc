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
#include "http_servers.h"

#include "driver/gpio.h"
#include "include/app_button.hpp"
#include "include/app_lcd.hpp"

#include "device_info.h"
#include "i2c_utils_component.h"
#include "PCT2075.h"
#include <dirent.h>
#define MOUNT_POINT "/sdcard"
/* Max length a file path can have on storage */
#define FILE_PATH_MAX 1024

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE (200 * 1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* Scratch buffer size */
#define SCRATCH_BUFSIZE 8192

// #define BOARD_ESP32CAM_AITHINKER 1
// #define CONFIG_CAMERA_MODULE_ESP_S3_EYE 1
#define CONFIG_CAMERA_MODULE_ESP_S3_DOG_CAM 1
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

#ifdef CONFIG_CAMERA_MODULE_ESP_S3_DOG_CAM
#define CAMERA_MODULE_NAME "ESP-S3-DOG_CAM"
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 1

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

static const char *TAG_FIND_VALUE = "FIND VALUE";
static const char *TAG_SDCARD = "SD CARD";
static const char *TAG_HTML = "HTML AP";
static const char *TAG_WIFI = "WIFI ";
static const char *TAG_0 = "TAKE_PICTURE";
const char *redirect = "<head><meta http-equiv=\"refresh\" content=\"0;url=/\"></head>";
static esp_err_t root_get_handler(httpd_req_t *req);
EventBits_t bits;
static int s_retry_num = 0;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

gpio_num_t LED_PIN = GPIO_NUM_4;
gpio_num_t VIBRO_PIN = GPIO_NUM_2;
gpio_num_t POWER_ON_PIN = GPIO_NUM_16;
gpio_num_t CAM_POWER_PIN = GPIO_NUM_32;
dl_matrix3du_t *image_matrix = NULL;
camera_fb_t *fb = NULL;
TaskHandle_t tf_task_handle = NULL;
QueueHandle_t xQueueFrame_2 = xQueueCreate(2, 9216 * 2);
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
  camera_config.frame_size = FRAMESIZE_96X96;
  camera_config.jpeg_quality = 10;
  camera_config.fb_count = 2;
  camera_config.fb_location = CAMERA_FB_IN_PSRAM;
  camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#endif
#ifdef CONFIG_CAMERA_MODULE_ESP_S3_DOG_CAM
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
  camera_config.pin_reset =  RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 10000000;
  camera_config.pixel_format = PIXFORMAT_GRAYSCALE;
  camera_config.frame_size = FRAMESIZE_96X96;
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
    // camera_fb_t *fb = NULL;
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
  // camera_fb_t *fb = NULL;
  uint16_t *frame = (uint16_t *)heap_caps_malloc((96 * 96) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  fb = esp_camera_fb_get();
  if (!fb)
  {

    ESP_LOGE(TAG_0, "Camera capture failed");
  }
  int counter = 0;
  for (int16_t i = 0; i < 9216; i++)
  {
    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;

    uint8_t red_mask = 0xf8;
    uint8_t green_mask = 0xfc;
    uint8_t blue_mask = 0xf8;

    uint8_t red_shift = 8;
    uint8_t green_shift = 3;
    uint8_t blue_shift_left = 3;

    red = fb->buf[i] & red_mask;
    green = fb->buf[i] & green_mask;
    blue = fb->buf[i]; //&blue_mask;

    red = red << red_shift;
    green = green << green_shift;
    blue = blue >> blue_shift_left;

    frame[i] = red + green + blue;
    counter++;
    // if(counter%30 == 0){
    //   ESP_LOGI("convert rgb565","%X",frame[i]);
    // }
    // frame[i] = red+2047;
  }
  xQueueSend(xQueueFrame_2, frame, portMAX_DELAY);
  vTaskDelay(100 / portTICK_RATE_MS);
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
  heap_caps_free(frame);
}

static esp_err_t Text2Html(httpd_req_t *req, char *filename)
{
  ESP_LOGI(TAG_SDCARD, "Reading %s", filename);
  FILE *fhtml = fopen(filename, "r");
  if (fhtml == NULL)
  {
    ESP_LOGE(TAG_SDCARD, "fopen fail. [%s]", filename);
    return ESP_FAIL;
  }
  else
  {
    char line[128];
    while (fgets(line, sizeof(line), fhtml) != NULL)
    {
      size_t linelen = strlen(line);
      // remove EOL (CR or LF)
      for (int i = linelen; i > 0; i--)
      {
        if (line[i - 1] == 0x0a)
        {
          line[i - 1] = 0;
        }
        else if (line[i - 1] == 0x0d)
        {
          line[i - 1] = 0;
        }
        else
        {
          break;
        }
      }
      ESP_LOGD(TAG_SDCARD, "line=[%s]", line);
      if (strlen(line) == 0)
        continue;
      esp_err_t ret = httpd_resp_sendstr_chunk(req, line);
      if (ret != ESP_OK)
      {
        ESP_LOGE(TAG_HTML, "httpd_resp_sendstr_chunk fail %d", ret);
      }
    }
    fclose(fhtml);
  }
  return ESP_OK;
}

// http
static esp_err_t faviconap_get_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "image/x-icon");
  httpd_resp_send(req, (const char *)favicon_ico, favicon_ico_len);
  return ESP_OK;
}

// take shot
static esp_err_t shot_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "image/jpg");
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  if (fb)
    esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb)
  {

    ESP_LOGE(TAG_0, "Camera capture failed");
    httpd_resp_send(req, (const char *)redirect, 59);
    return ESP_OK;
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
  // write_photo_to_sd(_jpg_buf, _jpg_buf_len);
  // esp_camera_fb_return(fb);
  httpd_resp_send(req, (const char *)_jpg_buf, _jpg_buf_len);
  return ESP_OK;
}

// save image
static esp_err_t save_img_handler(httpd_req_t *req)
{

  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
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
  // root_get_handler(req);
  httpd_resp_send(req, (const char *)redirect, 59);
  return ESP_OK;
}

// delete image handler
static esp_err_t delete_img_handler(httpd_req_t *req)
{
  esp_camera_fb_return(fb);
  // root_get_handler(req);
  httpd_resp_send(req, (const char *)redirect, 59);
  return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
  ESP_LOGI(TAG_HTML, "root_get_handler req->uri=[%s]", req->uri);
  char entrypath[FILE_PATH_MAX];
  char entrysize[16];
  const char *entrytype;

  struct dirent *entry;
  struct stat entry_stat;

  DIR *dir = opendir(MOUNT_POINT);
  const size_t MOUNT_POINT_len = strlen(MOUNT_POINT);

  /* Retrieve the base path of file storage to construct the full path */
  strlcpy(entrypath, MOUNT_POINT, sizeof(entrypath));

  if (!dir)
  {
    ESP_LOGE(TAG_SDCARD, "Failed to stat dir : %s", MOUNT_POINT);
    /* Respond with 404 Not Found */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
    return ESP_FAIL;
  }
  while ((entry = readdir(dir)) != NULL)
  {
    entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

    strlcpy(entrypath + MOUNT_POINT_len, entry->d_name, sizeof(entrypath) - MOUNT_POINT_len);
    // if (stat(entrypath, &entry_stat) == -1) {
    //     ESP_LOGE(TAG_SDCARD, "Failed to stat %s : %s", entrytype, entry->d_name);
    //     continue;
    // }
    // sprintf(entrysize, "%ld", entry_stat.st_size);
    ESP_LOGI(TAG_SDCARD, "Found %s : %s", entrytype, entry->d_name);
  }
  closedir(dir);
  /* Send index.html */
  Text2Html(req, "/sdcard/INDEX.HTM");

  /* Send empty chunk to signal HTTP response completion */
  httpd_resp_sendstr_chunk(req, NULL);

  return ESP_OK;
}

esp_err_t start_ap_server()
{
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  // sd_card_init();
  /* Use the URI wildcard matching function in order to
   * allow the same handler to respond to multiple different
   * target URIs which match the wildcard scheme */
  config.uri_match_fn = httpd_uri_match_wildcard;

  ESP_LOGI(TAG_HTML, "Starting HTTP Server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) != ESP_OK)
  {
    ESP_LOGE(TAG_HTML, "Failed to start file server!");
    return ESP_FAIL;
  }

  /* URI handler for get */
  httpd_uri_t _root_get_handler = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = root_get_handler,
      //.user_ctx  = server_data	// Pass server data as context
  };
  httpd_register_uri_handler(server, &_root_get_handler);

  /* URI handler for making shot */
  httpd_uri_t _shot_handler = {
      .uri = "/shot?*",
      .method = HTTP_GET,
      .handler = shot_handler,
      //.user_ctx  = server_data	// Pass server data as context
  };
  httpd_register_uri_handler(server, &_shot_handler);

  /* URI handler for save image */
  httpd_uri_t _save_img_handler = {
      .uri = "/save",
      .method = HTTP_GET,
      .handler = save_img_handler,
      //.user_ctx  = server_data	// Pass server data as context
  };
  httpd_register_uri_handler(server, &_save_img_handler);
  /* URI handler for delete current image */
  httpd_uri_t _delete_img_handler = {
      .uri = "/delete",
      .method = HTTP_GET,
      .handler = delete_img_handler,
      //.user_ctx  = server_data	// Pass server data as context
  };
  httpd_register_uri_handler(server, &_delete_img_handler);
  /* URI handler for favicon.ico */
  httpd_uri_t _favicon_get_handler = {
      .uri = "/favicon.ico",
      .method = HTTP_GET,
      .handler = faviconap_get_handler,
      //.user_ctx  = server_data	// Pass server data as context
  };
  httpd_register_uri_handler(server, &_favicon_get_handler);

  return ESP_OK;

  return ESP_OK;
}
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG_WIFI, "connect to the AP fail");
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(char *ssid, char *ssid_pass)
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_got_ip));

  wifi_config_t wifi_config = {};
  memset(&wifi_config, 0, sizeof(wifi_config));
  // strcpy((char *)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);
  // strcpy((char *)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);
  strcpy((char *)wifi_config.sta.ssid, ssid);
  strcpy((char *)wifi_config.sta.password, ssid_pass);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  // wifi_config_t wifi_config = {
  //     .sta = {
  //         .ssid = EXAMPLE_ESP_WIFI_SSID,
  //         .password = EXAMPLE_ESP_WIFI_PASS,
  //         /* Setting a password implies station will connect to all security modes including WEP/WPA.
  //          * However these modes are deprecated and not advisable to be used. Incase your Access point
  //          * doesn't support WPA2, these mode can be enabled by commenting below line */
  //         .threshold = {
  //             .authmode = WIFI_AUTH_WPA2_PSK,
  //         }},
  // };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  bits = xEventGroupWaitBits(s_wifi_event_group,
                             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                             pdFALSE,
                             pdFALSE,
                             portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */

  // read_config_file(ssid,  ssid_pass);

  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
             ssid, ssid_pass);
    //  XAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  }
  else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
             ssid, ssid_pass);
    //  EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  }
  else
  {
    ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
  }

  /* The event will not be processed after unregister */
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  vEventGroupDelete(s_wifi_event_group);
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
  Device_info *dev;
  // ESP_LOGI("TEST","device size if %d",sizeof(struct device_info_s));
  dev = (Device_info*)malloc(sizeof(struct device_info_s));
  memset(dev,0,sizeof(struct device_info_s));

  char ssid[40];
  char ssid_pass[40];

  i2c_master_init();
  i2c_scan();

  pct2075_read_temp(&(dev->temp));
  // xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);
  sd_card_init();
  int8_t read_conf_result = read_config_file(dev->ssid, dev->ssid_pass);

  print_device_info(dev);

  wifi_init_sta(dev->ssid, dev->ssid_pass);
  vTaskDelay(2000 / portTICK_RATE_MS);
  // sd_card_unmount();

  // if (ESP_OK != init_camera())
  // {
  //   return;
  // }
  start_ap_server();
  // camera_fb_t *fb = NULL;
  // fb = esp_camera_fb_get();
  // image_matrix = dl_matrix3du_alloc(1, 96,96,3);//fb->width, fb->height, 3);
  // image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  // if (!fb)
  // {

  //   ESP_LOGE(TAG_0, "Camera capture failed");
  //   return;
  // }

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