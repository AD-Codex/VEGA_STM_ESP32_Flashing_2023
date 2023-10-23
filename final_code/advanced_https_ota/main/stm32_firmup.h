// ---------------------------------------------------
// 2023/08
// Divakaran A.
// 
// ---------------------------------------------------

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

const int RX_BUF_SIZE_2 = 2048;
uint32_t write_address = 0x08000000;

// ESP32 Pinout
#define STM32_RESET_PIN GPIO_NUM_18
#define STM32_BOOT0_PIN GPIO_NUM_19

#define ledTogglePin_2 GPIO_NUM_21

const int uart_num_2 = UART_NUM_2;

const char *TAG_2 = "STM32";
const char *TAG_2_GPIO = "GPIO";
const char *TAG_2_UART = "UART";

char tx_data_2 = 0;
char rx_data_2 = 0;

// ----------------------- UART configuration
void stm32flash_init_uart(void)
  {
    const uart_config_t uart_config_2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(uart_num_2, &uart_config_2);
    uart_set_pin(uart_num_2, ESP_TXD_PIN2, ESP_RXD_PIN2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num_2, RX_BUF_SIZE_2, 0, 0, NULL, 0);
   }

// ------------------------- gpio pin initializing
void stm32flash_gpio_init()
{
    gpio_reset_pin(STM32_RESET_PIN);
    gpio_reset_pin(STM32_BOOT0_PIN);
    gpio_reset_pin(ledTogglePin_2);

    gpio_set_direction(STM32_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(STM32_BOOT0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ledTogglePin_2, GPIO_MODE_OUTPUT);

    gpio_set_level(STM32_RESET_PIN, 0);
    gpio_set_level(STM32_BOOT0_PIN, 0);
    gpio_set_level(ledTogglePin_2, 0);

    ESP_LOGI(TAG_2_GPIO, "Initialised GPIOs");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

// ---------------------- enable boot mode of the stm32
static void stm32flash_gpio_boot_set()
{
    // latch boot pins
    gpio_set_level(STM32_BOOT0_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // toggle reset
    gpio_set_level(STM32_RESET_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// --------------------- disable boot mode of the stm32
static void stm32flash_gpio_boot_rset()
{
    // latch boot pins
    gpio_set_level(STM32_BOOT0_PIN, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // toggle reset
    gpio_set_level(STM32_RESET_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(STM32_RESET_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}


uint8_t write_address_checksum(uint32_t write_address) {
    uint8_t checksum = 0;
    checksum = checksum ^ (write_address & 0xFF);
    checksum = checksum ^ ((write_address>>8) & 0xFF);
    checksum = checksum ^ ((write_address>>16) & 0xFF);
    checksum = checksum ^ ((write_address>>24) & 0xFF);
    return checksum;
}


static void stm32flash_enter_bootmode() {
    // Boot mode entered
    while(1) {
        tx_data_2 = 0x7F;
        rx_data_2 = 0;

        ESP_LOGI(TAG_2, "Sending: %X", tx_data_2);
        
        uart_flush_input(uart_num_2);
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);

        if (rx_data_2==0x79)
        {
            ESP_LOGW(TAG_2, "Boot mode entered");
            break;
        }
        else {
            ESP_LOGW(TAG_2, "Fail: Boot mode entered");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }

    // flash erase
    while (1) {
        uart_flush_input(uart_num_2);
        rx_data_2 = 0;

        tx_data_2 = 0x43;
        ESP_LOGI(TAG_2, "Sending: %X", tx_data_2);        
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        tx_data_2 = 0xBC;
        ESP_LOGI(TAG_2, "Sending: %X", tx_data_2);        
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);

        if (rx_data_2==0x79)
        {
            ESP_LOGW(TAG_2, "Request for flash erase");
        }
        else {
            ESP_LOGW(TAG_2, "Fail: Request for flash erase");
            break;
        }


        uart_flush_input(uart_num_2);

        rx_data_2 = 0;

        tx_data_2 = 0xFF;
        ESP_LOGI(TAG_2, "Sending: %X", tx_data_2);        
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        tx_data_2 = 0x00;
        ESP_LOGI(TAG_2, "Sending: %X", tx_data_2);        
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 1000 / portTICK_RATE_MS);
        
        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);

        if (rx_data_2==0x79) {
            ESP_LOGW(TAG_2, "flash erase success");
            break;
        }
        else {
            ESP_LOGW(TAG_2, "Fail: flash erase success");
            vTaskDelay( 2000 / portTICK_PERIOD_MS);

        }

    }


    // flash process
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

        write_address = 0x08000000 + packet_size*send_packet_num;
        ESP_LOGI(TAG_2, "Start write_address %X", write_address);

        rx_data_2 = 0;
        uart_flush_input(uart_num_2);

        tx_data_2 = 0x31;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
        tx_data_2 = 0xCE;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
        ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);
        if (rx_data_2==0x79) {
            ESP_LOGI(TAG_2,"Write command sent");
        }
        else {
            ESP_LOGE(TAG_2, "Write command Failed");
            break;
        }


        rx_data_2 = 0;
        uart_flush_input(uart_num_2);

        tx_data_2 = (write_address>>24) & 0xFF;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            // ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
        tx_data_2 = (write_address>>16) & 0xFF;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            // ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
        tx_data_2 = (write_address>>8) & 0xFF;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            // ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
        tx_data_2 = write_address & 0xFF;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            // ESP_LOGI(TAG_2, "Sent %X", tx_data_2);

        tx_data_2 = write_address_checksum(write_address);
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            // ESP_LOGI(TAG_2, "Sent %X", tx_data_2);
            
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);
        if (rx_data_2==0x79) {
            ESP_LOGI(TAG_2,"write_Address sent");
        }
        else {
            ESP_LOGE(TAG_2, "write_Address Failed");
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

        tx_data_2 = (send_packet_size-1) & 0xFF;
        ESP_LOGI(TAG_2, "num byte %X", tx_data_2);
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            
        int count = 0;
        int bin_array_number = send_packet_num / chunks_in_packet ;
        int bin_array_pos = (send_packet_num % chunks_in_packet)*packet_size;


        while (count < send_packet_size)
        {
            tx_data_2 = bin[bin_array_number][bin_array_pos];
            ESP_LOGI(TAG_2, "binvalue %X", tx_data_2);
            uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);
            checksum = checksum ^ tx_data_2;
            bin_array_pos = bin_array_pos + 1;
            count = count +1;
        }


        ESP_LOGI(TAG_2, "checksum %X", checksum);
        tx_data_2 = checksum & 0xFF;
        uart_write_bytes(uart_num_2, (const char *) &tx_data_2, 1);

        rx_data_2 = 0;
        uart_flush_input(uart_num_2);
        uart_read_bytes(uart_num_2, &rx_data_2, 1, 10000 / portTICK_RATE_MS);

        ESP_LOGI(TAG_2, "Received: %X", rx_data_2);
        if (rx_data_2==0x79) {
            ESP_LOGI(TAG_2,"Data sent");
            send_packet_num = send_packet_num +1;
        }
        else {
            ESP_LOGE(TAG_2, "Data send Failed");
            break;
        }

        if (send_packet_num == num_of_packet) {
            break;
        }
        

        
    }
    

}


static void stm32_flash() {
    gpio_set_level(ledTogglePin_2, 1);
    stm32flash_gpio_boot_set();
    
    stm32flash_enter_bootmode();

    gpio_set_level(ledTogglePin_2, 0);
    stm32flash_gpio_boot_rset();

}


// static void stm32_check() {

//     // toggle reset
//     gpio_set_level(STM32_RESET_PIN, 0);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     gpio_set_level(STM32_RESET_PIN, 1);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     gpio_set_level(STM32_RESET_PIN, 0);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);

//     uart_flush_input(uart_num_2);
//     rx_data_2 = 0;
//     uart_read_bytes(uart_num_2, &rx_data_2, 1, 10000 / portTICK_RATE_MS);

//     if ( rx_data_2 < 0x05) {
//         ESP_LOGW(TAG_2, "New version update");

//         stm32_flash();

//         ESP_LOGW(TAG_2, "flash done");
//     }
//     else {
//         ESP_LOGW(TAG_2, "New version on stm");
//     }

// }


// ---------------------- firm check

void stm32flash_firm_check(int curr_firm_number)
{
    int firm_number = bin_version;

    if (curr_firm_number < firm_number) {
        ESP_LOGW(TAG_2, "New version update");

        stm32flash_init_uart();
        stm32flash_gpio_init();
        stm32_flash();

        ESP_LOGW(TAG_2, "flash done");
    }
    else {
        ESP_LOGW(TAG_2, "New version on stm");
    }

}
