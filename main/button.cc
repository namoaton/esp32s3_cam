#include "button.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define GPIO_INPUT_IO_0     GPIO_NUM_13
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0) 
#define ESP_INTR_FLAG_DEFAULT 0

gpio_num_t BUTTON_PIN = GPIO_NUM_13;

static const char *TAG_BUTTON = "BUTTON";

static QueueHandle_t gpio_evt_queue = NULL;


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xQueueSendFromISR(gpio_evt_queue, &BUTTON_PIN, NULL);
}


static void gpio_task_example(void* arg)
{
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &BUTTON_PIN, portMAX_DELAY)) {
            ESP_LOGI(TAG_BUTTON,"GPIO[%d] intr, val: %d\n", BUTTON_PIN, gpio_get_level(BUTTON_PIN));
        }
    }
}

void init_interrupt(){
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
     xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}