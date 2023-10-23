#include <stdio.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "stm32_bin.h"

#define ESP_TXD_PIN2 (GPIO_NUM_17)
#define ESP_RXD_PIN2 (GPIO_NUM_16)

static const int RX_BUF_SIZE = 2048;
static uint32_t address = 0x08000000;

// ESP32 Pinout
#define STM32_RESET GPIO_NUM_18
#define STM32_BOOT0 GPIO_NUM_19

#define ledTogglePin 21

const int uart_num = UART_NUM_2;

static const char *TAG = "STM32";
static const char *TAG_GPIO = "GPIO";
static const char *TAG_UART = "UART";

static char tx_data = 0;
static char rx_data = 0;


void init_uart(void)
  {
    const uart_config_t uart_config_2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, ESP_TXD_PIN2, ESP_RXD_PIN2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE, 0, 0, NULL, 0);
   }

void gpio_init()
{
    gpio_reset_pin(STM32_RESET);
    gpio_reset_pin(STM32_BOOT0);
    gpio_reset_pin(ledTogglePin);

    gpio_set_direction(STM32_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(STM32_BOOT0, GPIO_MODE_OUTPUT);
    gpio_set_direction(ledTogglePin, GPIO_MODE_OUTPUT);

    gpio_set_level(STM32_RESET, 0);
    gpio_set_level(STM32_BOOT0, 0);
    gpio_set_level(ledTogglePin, 0);

    ESP_LOGI(TAG_GPIO, "Initialised GPIOs");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
}


static void gpio_boot_set()
{
    // latch boot pins
    gpio_set_level(STM32_BOOT0, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // toggle reset
    gpio_set_level(STM32_RESET, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static void gpio_boot_rset()
{
    // latch boot pins
    gpio_set_level(STM32_BOOT0, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // toggle reset
    gpio_set_level(STM32_RESET, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

uint8_t address_checksum(uint32_t address) {
    uint8_t checksum = 0;
    checksum = checksum ^ (address & 0xFF);
    checksum = checksum ^ ((address>>8) & 0xFF);
    checksum = checksum ^ ((address>>16) & 0xFF);
    checksum = checksum ^ ((address>>24) & 0xFF);
    return checksum;
}


static void enter_bootmode() {
    while(1) {
        tx_data = 0x7F;
        rx_data = 0;

        ESP_LOGI(TAG, "Sending: %X", tx_data);
        
        uart_flush_input(uart_num);
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        uart_read_bytes(uart_num, &rx_data, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG, "Received: %X", rx_data);

        if (rx_data==0x79)
        {
            ESP_LOGW(TAG, "Boot mode entered");
            break;
        }
        else {
            ESP_LOGW(TAG, "Fail: Boot mode entered");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }


    while (1) {
        uart_flush_input(uart_num);

        rx_data = 0;

        tx_data = 0x43;
        ESP_LOGI(TAG, "Sending: %X", tx_data);        
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        tx_data = 0xBC;
        ESP_LOGI(TAG, "Sending: %X", tx_data);        
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        uart_read_bytes(uart_num, &rx_data, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG, "Received: %X", rx_data);

        if (rx_data==0x79)
        {
            ESP_LOGW(TAG, "Request for flash erase");
        }
        else {
            ESP_LOGW(TAG, "Fail: Request for flash erase");
            break;
        }


        uart_flush_input(uart_num);

        rx_data = 0;

        tx_data = 0xFF;
        ESP_LOGI(TAG, "Sending: %X", tx_data);        
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        tx_data = 0x00;
        ESP_LOGI(TAG, "Sending: %X", tx_data);        
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        uart_read_bytes(uart_num, &rx_data, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG, "Received: %X", rx_data);

        if (rx_data==0x79) {
            ESP_LOGW(TAG, "flash erase success");
            break;
        }
        else {
            ESP_LOGW(TAG, "Fail: flash erase success");
            vTaskDelay( 2000 / portTICK_PERIOD_MS);

        }

    }

    int packet_size = 128;  // should 4x
    int send_packet_size = 0;
    int num_of_packet = 0;
    if (stm32_bin_len%packet_size >0) {
        num_of_packet = stm32_bin_len/packet_size + 1;
    }
    else {
        num_of_packet = stm32_bin_len/packet_size;
    }
    int chunks_in_packet = 1024/packet_size;
    int send_packet_num = 0;


    while (1)
    {

        address = 0x08000000 + packet_size*send_packet_num;
        ESP_LOGI(TAG, "Start address %X", address);

        rx_data = 0;
        uart_flush_input(uart_num);

        tx_data = 0x31;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        ESP_LOGI(TAG, "Sent %X", tx_data);
        tx_data = 0xCE;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
        ESP_LOGI(TAG, "Sent %X", tx_data);
        uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG, "Received: %X", rx_data);
        if (rx_data==0x79) {
            ESP_LOGI(TAG,"Write command sent");
        }
        else {
            ESP_LOGE(TAG, "Write command Failed");
            break;
        }


        rx_data = 0;
        uart_flush_input(uart_num);

        tx_data = (address>>24) & 0xFF;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            // ESP_LOGI(TAG, "Sent %X", tx_data);
        tx_data = (address>>16) & 0xFF;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            // ESP_LOGI(TAG, "Sent %X", tx_data);
        tx_data = (address>>8) & 0xFF;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            // ESP_LOGI(TAG, "Sent %X", tx_data);
        tx_data = address & 0xFF;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            // ESP_LOGI(TAG, "Sent %X", tx_data);

        tx_data = address_checksum(address);
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            // ESP_LOGI(TAG, "Sent %X", tx_data);
            
        uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG, "Received: %X", rx_data);
        if (rx_data==0x79) {
            ESP_LOGI(TAG,"Address sent");
        }
        else {
            ESP_LOGE(TAG, "Address Failed");
            break;
        }


        int checksum = 0;


        if (send_packet_num < num_of_packet-1) {
            send_packet_size = packet_size;
        }
        else {
            send_packet_size = stm32_bin_len - ((num_of_packet-1)*packet_size);
        }

        checksum = (send_packet_size-1);

        tx_data = (send_packet_size-1) & 0xFF;
        ESP_LOGI(TAG, "num byte %X", tx_data);
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            
        int count = 0;
        int bin_array_number = send_packet_num / chunks_in_packet ;
        int bin_array_pos = (send_packet_num % chunks_in_packet)*packet_size;


        while (count < send_packet_size)
        {
            tx_data = bin[bin_array_number][bin_array_pos];
            ESP_LOGI(TAG, "binvalue %X", tx_data);
            uart_write_bytes(uart_num, (const char *) &tx_data, 1);
            checksum = checksum ^ tx_data;
            bin_array_pos = bin_array_pos + 1;
            count = count +1;
        }


        ESP_LOGI(TAG, "checksum %X", checksum);
        tx_data = checksum & 0xFF;
        uart_write_bytes(uart_num, (const char *) &tx_data, 1);

        rx_data = 0;
        uart_flush_input(uart_num);
        uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG, "Received: %X", rx_data);
        if (rx_data==0x79) {
            ESP_LOGI(TAG,"Data sent");
            send_packet_num = send_packet_num +1;
        }
        else {
            ESP_LOGE(TAG, "Data send Failed");
            break;
        }

        if (send_packet_num == num_of_packet) {
            break;
        }
        

        
    }
    

}


static void stm32_flash() {
    gpio_set_level(ledTogglePin, 1);
    gpio_boot_set();
    
    enter_bootmode();

    gpio_set_level(ledTogglePin, 0);
    gpio_boot_rset();

}

// void app_main(void)
// {  
//     init_uart();
//     gpio_init();
    
//     stm32_flash();

//     ESP_LOGW(TAG, "flash success");


// }

