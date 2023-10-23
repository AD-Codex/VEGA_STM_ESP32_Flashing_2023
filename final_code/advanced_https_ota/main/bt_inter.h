#ifndef BT_INTER_H
#define BT_INTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
extern "C" {
      // Get declaration for f(int i, char c, float x)
     #include "HTTP_MQTT_WIFI.h"
}

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//Rusira
#include "esp_log.h"
#include "esp_http_server.h"
#include "flash.h"
//


#define ESP_INTR_FLAG_DEFAULT 0

//Rusira
static const char *TAG_B = "BTN";
#define CONFIG_BUTTON_PIN GPIO_NUM_0
#define LONG_PRESS_IN_SECONDS 2
//

SemaphoreHandle_t xSemaphore = NULL;
//bool led_status = false;
extern  EventGroupHandle_t wifi_event_group;
extern const int CONNECTED_BIT_2;



// interrupt service routine, called when the button is pressed
void IRAM_ATTR button_isr_handler(void* arg) {
	
    // notify the button task
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

// task that will react to button clicks
void button_task(void* arg) {
	
	// infinite loop
	for(;;) {
		/*if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
			printf("Button pressed!\n");
			//led_status = !led_status;
			//gpio_set_level(LED, led_status);
		}*/
		uint16_t ticks = 0;
		// wait for the notification from the ISR
		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
			printf("Button pressed!\n");
			//led_status = !led_status;
			//gpio_set_level(CONFIG_LED_PIN, led_status);
            ESP_LOGI(TAG_B, "BTN Pressed Down.");

            ticks = 0;

            // Loop here while pressed until user lets go, or longer than set time
            while ((!gpio_get_level(CONFIG_BUTTON_PIN)) && (++ticks < LONG_PRESS_IN_SECONDS * 100))
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            /*if (ticks >= LONG_PRESS_IN_SECONDS * 100)
            {*/
            if(setup_flag==1){
                // setup flag is set to 1 if we need to start over and saved in NVS
                ESP_LOGI(TAG_B, "NVS setup_flag reset");
                xEventGroupClearBits(wifi_event_group, CONNECTED_BIT_2);
                setup_flag = 1;
                esp_err_t err_setup = save_int_value("setup_flag:", setup_flag);

                if (err_setup != ESP_OK)
                {
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_setup));
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);

                esp_restart();
            }
           // }

            // Wait here if the buttonis still held
            while (!gpio_get_level(CONFIG_BUTTON_PIN))
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            ESP_LOGI(TAG_B, "BTN Released.");
    
		}
	}
}

esp_err_t init_smart_config_btn(){
    xSemaphore = xSemaphoreCreateBinary();
	// configure button and led pins as GPIO pins
	gpio_pad_select_gpio(CONFIG_BUTTON_PIN);//for future development
	//gpio_pad_select_gpio(CONFIG_LED_PIN);
	// set the correct direction
	ESP_ERROR_CHECK(gpio_set_direction(CONFIG_BUTTON_PIN, GPIO_MODE_INPUT));
    //gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
	// enable interrupt on falling (1->0) edge for button pin
	ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_BUTTON_PIN, GPIO_INTR_NEGEDGE));
	// start the task that will handle the button
	xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
	// install ISR service with default configuration
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
	// attach the interrupt service routine
	ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_BUTTON_PIN, button_isr_handler, NULL));

    return ESP_OK;
}

#ifdef __cplusplus
}
#endif

#endif