extern "C"
{
#include "HTTP_MQTT_WIFI.h"
#include "serialCom.h"
}
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_http_client.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ping/ping.h"
#include "esp_ping.h"
#include "mbedtls/md.h"
#include "cJSON.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"

#include "esp_spiffs.h"

#include "esp_timer.h"
/*#include "driver/spi_master.h"today
#include "esp_idf_lora/RH_98.h"
#include "esp_idf_lora/lora_encrypted_driver.h"
#include "crypto/Crypto.h"
#include "crypto/SpeckSmall.h"*/

// #include "ca_cert_1.h"

// smart config
#include "esp_wpa2.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "lwip/dns.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "driver/rmt.h"
#include "led_strip.h"

#include "stm32_firmup.h"

// #include "rahal_scanconfig.h"

extern "C"
{
#include "FileHandle.h"
#include "serialCom.h"
#include "PN532.h"
}

#if CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
#include "esp_efuse.h"
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#if CONFIG_BT_BLE_ENABLED || CONFIG_BT_NIMBLE_ENABLED
#include "ble_api.h"
#endif

#define ECHO_TEST_TXD 17
#define ECHO_TEST_RXD 16

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (100)

#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 36

// stm32 version
int stm32_version_num_1;
int stm32_version_num_2;
int stm32_version_num_3;
int stm32_version_num;

// int stopedFlag = 0;
int timeDiffFile = 0;

int startTransactionProcess = 0;
// bool checkFileHasUnfinishedTransaction = false;
isolatedStateType isolatedState;
powerSideStateType powerSideState;

// char tempUserId[25];

bool NO_AUTHORIZE = false;
#define mainONE_SHOT_TIMER_PERIOD pdMS_TO_TICKS(1000)

// int noInternetDisplay = 0;
int stopFromTimeout = 0;

int serialErrorTimeout = 0;
int serialRecevedFlag = 0;



//int temp2 = 0;
//int temp3 = 0;
//int temp4 = 0;
int counterMQTT = 0;
uint8_t toggle_pin = 0;
uint8_t bootOnce = 0;
uint8_t rebootTime = 0;
uint8_t configureChargerEnabledFlag = 0;

uint8_t no_reply_flag = 0;
uint8_t no_reply_flag_2 = 0;

uint8_t no_reply_count_is_device_paired = 0;
uint8_t no_reply_limit_is_device_paired = 20;
uint8_t reboot_flag = 0;

typedef enum
{
    NO_FAULT = 0,
    FAULT_STATE_ENTER
} faultType_t;

volatile faultType_t currentFault = NO_FAULT;
bool error_occurred_flag = false;

bool powerOnSerialBit = false;

#define TWDT_TIMEOUT_MS 3000
#define TASK_RESET_PERIOD_MS 2000
#define MAIN_DELAY_MS 10000

// #define CONFIG_EXAMPLE_RMT_TX_GPIO GPIO_NUM_26
#define CONFIG_EXAMPLE_RMT_TX_GPIO GPIO_NUM_33

void mainStateMachine();
void faultStateMachine();
void monitor();
int serialReadFunc();

extern const uint8_t server_cert_1_pem_start[] asm("_binary_ca_cert_1_pem_start");
extern const uint8_t server_cert_1_pem_end[] asm("_binary_ca_cert_1_pem_end");

#define OTA_URL_SIZE 256

/*#define IO_PIN_CS 10
#define IO_PIN_INTERRPUPT 20
#define IO_PIN_MISO 13
#define IO_PIN_MOSI 11
#define IO_PIN_CLK 12*/

/*#define IO_PIN_CS 15 today
#define IO_PIN_INTERRPUPT 27
#define IO_PIN_MISO 12
#define IO_PIN_MOSI 13
#define IO_PIN_CLK 14*/
// spi_device_handle_t _handle_spi;
// RH_RF98 lora(IO_PIN_CS, IO_PIN_INTERRPUPT, IO_PIN_MISO, IO_PIN_MOSI, IO_PIN_CLK);today
// SpeckSmall aes128;today
// lora_encrypted_driver myDriver(lora, aes128);today
const uint8_t *headerSending = (const uint8_t *)"cvega";
const uint8_t *headerReceiving = (const uint8_t *)"pvega";
const uint8_t *reset_code = (const uint8_t *)"$$$$";
const uint8_t *success_code = (const uint8_t *)"****";
const uint8_t *error_code = (const uint8_t *)"####";

const uint8_t *headerReset = (const uint8_t *)"cvega$$$$";
#define header_length_sending 5
#define header_length_receiving 5
#define timeout 3000
signed short int power_lora;
int vrms_recieved = 0;
int irms_recieved = 0;

signed short int power_lora_after_sign_bit;

signed short int currentPower;
signed short int previousPower;

uint8_t signBit;

uint16_t maxPower = 6600;
uint8_t serialState = 0;

unsigned long int replied_time;
// float frequency=433.0;
unsigned char encryptkey[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // The very secret key

// int r_int=0;
uint8_t no_reply_cnt = 0;
uint8_t no_reply_count_2 = 0;
int no_reply_limit = 20;
uint8_t no_reply_limit_over = 8;
uint8_t id_length = 14; // was 12
char result[13];
uint8_t body[6];
const uint8_t *device_id = (const uint8_t *)"0000";
const uint8_t *charger_id = (const uint8_t *)"1234";
int recommended_rssi = -90;

uint8_t ledBrightness[] = {1, 1, 2, 4, 6, 10, 14, 19, 25, 32, 39, 47, 56, 66, 77, 88, 100, 88, 77, 66, 56, 47, 39, 32, 25, 19, 14, 10, 6, 4, 2, 1, 1};

bool valid_header(uint8_t *buf)
{
    if (memcmp(buf, headerReceiving, header_length_receiving) == 0)
    {
        // Serial.println("Header Test Passed");
        return true;
    }
    else
    {
        // Serial.println("Header Test Failed");
        return false;
    }
}

void reset_device()
{
    uint8_t header_buffer[4 + header_length_sending];
    memcpy(header_buffer, headerReset, header_length_sending + 4);
    // memmove(header_buffer + header_length, reset_code, 4);//today
    for (int count = 0; count < 5; count++)
    {
#ifdef DEBUG
        ESP_LOGI("reset_device()", "Remote reset done");
#endif

        // Serial.println("Remote reset done");

        // myDriver.send(header_buffer, 4 + header_length); // send data today
        lora_send_packet((uint8_t *)header_buffer, 4 + header_length_sending);
#ifdef DEBUG
        ESP_LOGI("reset_device()", "sent:vega$$$$");
#endif
        // myDriver.waitPacketSent();today
        vTaskDelay(700 / portTICK_PERIOD_MS);
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
}

/*void pair_with_device()
{
    int pair_stage = 0;
    bool pair_error = false;
    uint8_t header_buffer[id_length + header_length];
    memcpy(header_buffer, header, header_length);
    while (pair_stage != 2)
    {
        ESP_LOGI("pair_with_device()", "%d initillizing..", pair_stage);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (pair_stage == 0)
        {

            memmove(header_buffer + header_length, charger_id, id_length);
            for (int count = 0; count < 6; count++) //today count was 12
            {
                myDriver.send(header_buffer, id_length + header_length); // send data
#ifdef DEBUG
                ESP_LOGI("pair_with_device()", "sent:%s", charger_id);
#endif
                myDriver.waitPacketSent();
                ESP_LOGI("pair_with_device()", "sent2:%s", charger_id);
                vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            countAttempts++;
            if (countAttempts >= 3)
            {
                countAttempts = 0;
#ifdef ENABLE_POWER_MEASURING_DEVICE
                if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
                {
                    loadBalancingEnableState = 0;
                    xSemaphoreGive(readStateMutex);
                }
#ifdef DEBUG
                ESP_LOGW("pair_with_device()", "loadBalancingEnableState:%d", loadBalancingEnableState);
#endif
                esp_err_t err_lbe = save_int_value("lbeState:", loadBalancingEnableState);

                if (err_lbe != ESP_OK)
                {
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_lbe));
                }
                return;

#endif
            }
        }
        else if (pair_stage == 1)
        {
            memmove(header_buffer + header_length, success_code, id_length);
            for (int count = 0; count < 12; count++)//today was 6
            {
                myDriver.send(header_buffer, id_length + header_length); // send data
#ifdef DEBUG
                ESP_LOGI("pair_with_device() new", "sent:veganode****");
#endif
                myDriver.waitPacketSent();
                vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            ESP_LOGI("pair_with_device()", "%d pairStage..", pair_stage);
            pair_stage += 1;
#ifdef DEBUG
            ESP_LOGI("pair_with_device()", "device initialization success");
            devicePairedStatus = 1;
#endif
            return;
        }
        if (pair_error)
        {
            // for(uint8_t i = 0; i<= id_length; i++) data[i] = (uint8_t)error_code[i];
            // delay(100);
            memmove(header_buffer + header_length, error_code, id_length);
            myDriver.send(header_buffer, id_length + header_length); // send data
#ifdef DEBUG
            ESP_LOGI("pair_with_device()", "sent:veganode#####");
#endif
            // Serial.println("sent: veganode####");
            myDriver.waitPacketSent();
            vTaskDelay(700 / portTICK_PERIOD_MS);
            pair_error = false;
        }

        if (myDriver.waitAvailableTimeout(timeout))
        {
            uint8_t buf[myDriver.maxMessageLength()];
            uint8_t len = sizeof(buf);
            if (myDriver.recv(buf, &len))
            {
#ifdef DEBUG
                ESP_LOGI("pair_with_device()", "got reply: %s", (char *)buf);
#endif
                if (valid_header(buf))
                {
                    replied_time = (unsigned long int)(esp_timer_get_time() / 1000);
                    if ((pair_stage == 0))
                    {
                        if ((device_id[0] == (char)buf[0 + header_length]) && (device_id[1] == (char)buf[1 + header_length]) && (device_id[2] == (char)buf[2 + header_length]) && (device_id[3] == (char)buf[3 + header_length]))
                        {
                            pair_stage += 1;
                            ESP_LOGI("pair_with_device()", "pair stage:%d", pair_stage);
                        }
                        else if (((char)buf[0 + header_length] == '#') && ((char)buf[1 + header_length] == '#') && ((char)buf[2 + header_length] == '#') && ((char)buf[3 + header_length] == '#'))
                        {
#ifdef DEBUG
                            ESP_LOGI("pair_with_device()", "charger id is wrong");
#endif
                        }
                        else
                        {
                            // ESP_LOGI("pair_with_device()", "device id wrong");
                            pair_error = true;
                            // memmove(header_buffer+header_length,error_code,id_length);
                            // myDriver.send(header_buffer, id_length+header_length); //send data
                            // ESP_LOGI("pair_with_device()", "sent: veganode####");
                            // myDriver.waitPacketSent();
                        }
                    }
                }
                else
                {
                }

                //      Serial.print("RSSI: ");
                //      Serial.println(rf95.lastRssi(), DEC);
            }
            else
            {
#ifdef DEBUG
                ESP_LOGI("pair_with_device()", "no reply from device");
#endif
            }
            if ((unsigned long int)(esp_timer_get_time() / 1000) - replied_time > timeout)
            {
#ifdef DEBUG
                ESP_LOGI("pair_with_device()", "hi hi");
#endif
                reset_device();
            }
            // no_reply_check();
        }
        else
        {
            reset_device();
            //if(pair_stage==0){today
           //   ESP_LOGI("pair_with_device()", "device doesn't respond.maybe power off or charger id wrong or device is out of range");
           //   reset_device();
           // }
           // else if(pair_stage==1){
          //      ESP_LOGI("pair_with_device()", "device doesn't respond.maybe power off or device id wrong or device is out of range");
           // }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}*/
void getAndSendMacAddress()
{

    uint8_t pair_stage = 0;
    while (pair_stage != 1)
    {
        ESP_LOGI("pair_with_device()", "pair_stage:%d", pair_stage);
        if (pair_stage == 0)
        {

            uint8_t header_buffer[id_length + header_length_sending];
            memcpy(header_buffer, headerSending, header_length_sending); // was memmove today
            memmove(header_buffer + header_length_sending, string_address, id_length);
            for (int count = 0; count < 12; count++) // today count was 6
            {
                // myDriver.send(header_buffer, id_length + header_length); // send data today
                lora_send_packet((uint8_t *)header_buffer, id_length + header_length_sending);
                // vTaskDelay(700 / portTICK_PERIOD_MS);
                // lora_send_packet((uint8_t*)"Hello", 5);

                ESP_LOGI("pair_with_device()", "send mac address");
                // myDriver.waitPacketSent(); today
                vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            pair_stage += 1;
            devicePairedStatus = 1;
            esp_err_t err_devicePairedStatus = save_int_value("dps:", devicePairedStatus);
            if (err_devicePairedStatus != ESP_OK)
            {
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_devicePairedStatus", esp_err_to_name(err_devicePairedStatus));
            }
        }
        /*else if (pair_stage == 1)
        {
            if (myDriver.waitAvailableTimeout(timeout))
            {
                uint8_t buf[myDriver.maxMessageLength()];
                uint8_t len = sizeof(buf);
                if (myDriver.recv(buf, &len))
                {
#ifdef DEBUG
                    ESP_LOGI("pair_with_device()", "got reply: %s", (char *)buf);
#endif
                    if (valid_header(buf))
                    {
                        if (check_mac(buf))
                        {
                            pair_stage += 1;
                            ESP_LOGI("pair_with_device()", "pair_stage:%d", pair_stage);
                        }
                        else
                        {
                            pair_stage = 0;
                            reset_device();
                        }
                    }
                }
            }
        }
        else if (pair_stage == 2)
        {
            uint8_t header_buffer[4 + header_length];
            memmove(header_buffer, header, header_length);
            memmove(header_buffer + header_length, success_code, 4);
            for (int count = 0; count < 6; count++) // today count was 12
            {

                myDriver.send(header_buffer, 4 + header_length); // send data
                ESP_LOGI("pair_with_device()", "success_code:****");
                myDriver.waitPacketSent();
                vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            pair_stage += 1;
#ifdef DEBUG
            ESP_LOGI("pair_with_device()", "device initialization success");
            devicePairedStatus = 1;
#endif
        }*/
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void no_reply_check()
{
    if (no_reply_cnt > no_reply_limit)
    {
        //    Serial.println("no reply limit exceeded|signal level:");
        //    Serial.println(lora.lastRssi());
        //    Serial.println();
        // no_reply_cnt = 0;
        // reset_device();
        if (no_reply_cnt > no_reply_limit)
        {
            no_reply_flag = 1;
        }

        if (no_reply_count_2 > no_reply_limit)
        {
            no_reply_flag_2 = 1;
        }

        // pair_with_device();today
        // getAndSendMacAddress();today
        //  disable load balancing enable state
        // if (no_reply_cnt > no_reply_limit_over) today
        //{
        no_reply_cnt = 0;
        no_reply_count_2 = 0;

        if (reboot_flag == 0)
        {
            reboot_flag = 1;
            /*esp_err_t err_reboot_flag = save_int_value("reboot_flag:", reboot_flag);

            if (err_reboot_flag != ESP_OK)
            {
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS reboot_flag", esp_err_to_name(err_reboot_flag));
            }*/

            gpio_set_level(loraReetPin, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // today was 1000
            gpio_set_level(loraReetPin, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // today was 1000

            loraInitialize();
            lora_set_frequency(433e6);
            lora_enable_crc();
            lora_receive();
            ESP_LOGE(TAG_B, "reboot_flag_inside:%d", reboot_flag);

            // esp_restart();
        }
        if (reboot_flag == 2)
        {
            reboot_flag = 0;
#ifdef ENABLE_POWER_MEASURING_DEVICE
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                loadBalancingEnableState = 0;
                xSemaphoreGive(readStateMutex);
            }
#ifdef DEBUG
            ESP_LOGW("no_reply_check()", "loadBalancingEnableState:%d", loadBalancingEnableState);
#endif
            esp_err_t err_lbe = save_int_value("lbeState:", loadBalancingEnableState);

            if (err_lbe != ESP_OK)
            {
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_lbe));
            }

#endif
        }
        if (reboot_flag == 1)
        {
            reboot_flag = 2;
        }
        //}
    }
}
/*bool error_check(uint8_t *buf)
{
    ESP_LOGI("error_check()", "got reply error check: %s", (char *)buf);

    if (lora.lastRssi() < recommended_rssi)
    {
#ifdef DEBUG
        ESP_LOGI("error_check()", "Low Signal Level");
#endif
        // digitalWrite(led_pin1,HIGH);
    }
    else
    {
#ifdef DEBUG
        ESP_LOGI("error_check()", "Recommeneded Signal Level|signal level:%d", lora.lastRssi());
#endif
        // digitalWrite(led_pin1,LOW);
    }

    if (((char)buf[0 + header_length] == '#') && ((char)buf[1 + header_length] == '#') && ((char)buf[2 + header_length] == '#') && ((char)buf[3 + header_length] == '#'))
    {
#ifdef DEBUG
        ESP_LOGI("error_check()####", "device is not paired yet. Pairing...");
#endif
        pair_with_device();
        return true;
    }
    else if ((device_id[0] == (char)buf[0 + header_length]) && (device_id[1] == (char)buf[1 + header_length]) && (device_id[2] == (char)buf[2 + header_length]) && (device_id[3] == (char)buf[3 + header_length]))
    {
        ESP_LOGI("error_check()", "device is not paired yet. Pairing...");
        uint8_t header_buffer[id_length + header_length];
        memcpy(header_buffer, header, header_length);
        memmove(header_buffer + header_length, success_code, id_length);
        for (int count = 0; count < 6; count++)
        {

            myDriver.send(header_buffer, id_length + header_length); // send data
#ifdef DEBUG
            ESP_LOGI("pair_with_device() 1", "sent:veganode****");
#endif
            myDriver.waitPacketSent();
            vTaskDelay(700 / portTICK_PERIOD_MS);
        }
        return true;
    }
    else
    {
        return false;
    }
}*/
void ecoPlusMode()
{

    if (signBit == 1)
    {
        power_lora_after_sign_bit = power_lora;
        ESP_LOGW("ecoPlusMode()", "signBit:%d", signBit);
    }
    else if (signBit == 2)
    {
        power_lora_after_sign_bit = power_lora * (-1);
        ESP_LOGW("ecoPlusMode()", "signBit:%d", signBit);
    }
    I_lora = fabsf(power_lora) / voltageL1 * 10;
    if (power_lora_after_sign_bit < 0)
    {
        if ((currentL1 + I_lora) > 30)
        {
            loadBalancingCurrentSetting = 30;
            printf("I_total in 30:%.2f\n", loadBalancingCurrentSetting);
        }
        else
        {
            loadBalancingCurrentSetting = currentL1 + I_lora;
            printf("I_total in else:%.2f\n", loadBalancingCurrentSetting);
        }
    }
    else
    {
        if ((currentL1 - I_lora) > 0)
        {
            loadBalancingCurrentSetting = currentL1 - I_lora;
            printf("I_total in power+:%.2f\n", loadBalancingCurrentSetting);
        }
        else
        {
            loadBalancingCurrentSetting = 0;
            printf("I_total in power+ else:%.2f\n", loadBalancingCurrentSetting);
        }
    }
}

/*void ecoMode()
{
    if (power_lora < maxPower && loadBalancingCurrentSetting < 30)
    {
        remainingPower = maxPower - power_lora;
        currentRemaining = remainingPower / voltageL1 * 10;
        if (currentL1 + currentRemaining <= 30)
        {
            loadBalancingCurrentSetting = currentL1+ currentRemaining;
            ESP_LOGW("ecoMode()", "loadBalancingCurrentSetting:%.2f", loadBalancingCurrentSetting);
        }
        else
        {
            loadBalancingCurrentSetting = 30;
            ESP_LOGW("ecoMode()", "loadBalancingCurrentSettingElse:%.2f", loadBalancingCurrentSetting);
        }
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}*/

void ecoMode()
{

    if (signBit == 1)
    {
        power_lora_after_sign_bit = power_lora;
        ESP_LOGW("ecoMode()", "signBit:%d", signBit);
    }
    else if (signBit == 2)
    {
        power_lora_after_sign_bit = (power_lora * (-1));
        ESP_LOGW("ecoMode()", "signBit:%d", signBit);
    }
    printf("loadBalancingCurrentSetting in early state :%.2f\n", loadBalancingCurrentSetting);
    if (power_lora_after_sign_bit < maxPower && loadBalancingCurrentSetting <= 30)
    {
        remainingPower = maxPower - power_lora_after_sign_bit;
        printf("remainingPower:%d\n", remainingPower);
        currentRemaining = remainingPower / voltageL1 * 10;
        printf("currentRemaining:%f\n", currentRemaining);
        if ((currentL1 + currentRemaining) <= 30)
        {
            loadBalancingCurrentSetting = currentL1 + currentRemaining;
            printf("loadBalancingCurrentSetting in if state :%.2f currentL1:%.2f\n", loadBalancingCurrentSetting, currentL1);
            // ESP_LOGW("ecoMode()", "loadBalancingCurrentSetting:%.2f", loadBalancingCurrentSetting);
        }
        else
        {
            loadBalancingCurrentSetting = 30;
            printf("loadBalancingCurrentSetting in else:%.2fcurrentL1:%.2f\n", loadBalancingCurrentSetting, currentL1);
            // ESP_LOGW("ecoMode()", "loadBalancingCurrentSettingElse:%.2f", loadBalancingCurrentSetting);
        }
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    else
    {
        if (power_lora_after_sign_bit > maxPower)
        {
            currentRemaining = (power_lora_after_sign_bit - maxPower) / voltageL1 * 10;
            if ((currentL1 - currentRemaining) >= 0)
            {
                loadBalancingCurrentSetting = currentL1 - currentRemaining;
                printf("loadBalancingCurrentSetting in powerlora>maxPower:%.2f\n", loadBalancingCurrentSetting);
            }
            else
            {
                loadBalancingCurrentSetting = 0;
                printf("loadBalancingCurrentSetting in powerlora>maxPower else:%.2f\n", loadBalancingCurrentSetting);
            }
        }
        if (loadBalancingCurrentSetting > 30)
        {
            loadBalancingCurrentSetting = 30;
            printf("loadBalancingCurrentSetting in lbcs>30:%.2f\n", loadBalancingCurrentSetting);
        }
    }
}
void lora_task(void *pvParameter)
{
#ifdef DEBUG
    ESP_LOGI("lora_task()", "setting up started");
#endif
    /* if (!myDriver.init()) today
     {
 #ifdef DEBUG
         ESP_LOGE("lora_task()", "init failed");
 #endif
     }
     else
     {
 #ifdef DEBUG
         ESP_LOGI("lora_task()", "init sucessfull");
 #endif
     }*/
    // lora.setFrequency(433.0);
    // aes128.setKey(encryptkey, 16);today
    lora_init();
    lora_set_frequency(433e6);
    lora_enable_crc();
    /*#ifdef DEBUG
        ESP_LOGI("lora_task()", "Encryption key set successful");
    #endif*/

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // lora.printRegisters();today
    esp_err_t err_lbe = load_int_value("lbeState:", &loadBalancingEnableState);
    if (err_lbe != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_lbe));
#endif
    }
    esp_err_t err_devicePairedStatus = load_int_value("dps:", &devicePairedStatus);
    if (err_devicePairedStatus != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS err_devicePairedStatus", esp_err_to_name(err_devicePairedStatus));
#endif
    }
    // devicePairedStatus = 0;
    if (loadBalancingEnableState == 1 && devicePairedStatus == 0)
    {
        // pair_with_device();
        getAndSendMacAddress();
    }

    uint8_t num = 0;

    esp_err_t err_signBit = load_int_value("signBit:", &signBit);
    if (err_signBit != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS signBit", esp_err_to_name(err_signBit));
#endif
    }

    esp_err_t err_configureChargerEnabledFlag = load_int_value("ccef:", &configureChargerEnabledFlag);
    if (err_configureChargerEnabledFlag != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS configureChargerEnabledFlag", esp_err_to_name(err_configureChargerEnabledFlag));
#endif
    }

    esp_err_t err_ecoPlusModeEnabled = load_int_value("epmEnable:", &ecoPlusModeEnabled);
    if (err_ecoPlusModeEnabled != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS ecoPlusModeEnabled", esp_err_to_name(err_ecoPlusModeEnabled));
#endif
    }
    /* esp_err_t err_reboot_flag = load_int_value("reboot_flag:", &reboot_flag);
     if (err_reboot_flag != ESP_OK)
     {
 #ifdef DEBUG
         ESP_LOGE(TAG, "Error (%s) loading to NVS reboot_flag", esp_err_to_name(err_reboot_flag));
 #endif
     }*/

    while (1)
    {
        if (loadBalancingEnableState == 1)
        {
            num = (num + 1) % 255 + 1;
// uint8_t header_buffer[id_length+header_length];
// memcpy(header_buffer,header,header_length);
// uint8_t data[24] = "veganode Send Data ";
// data[23]=num;
#ifdef DEBUG
            ESP_LOGI("\nlora_task()", "Sending to rf95_server...");
#endif
            // myDriver.setHeaderId(num);
            // myDriver.send(header, header_length);
            // myDriver.waitPacketSent();

            uint8_t buf[32];
            uint8_t len = sizeof(buf);
            ESP_LOGW("recivingLora", "devicePairedStatus:%d", devicePairedStatus);
            ESP_LOGW("recivingLora", "loadBalancingEnableState:%d", loadBalancingEnableState);
            ESP_LOGW("recivingLora", "reboot_flag:%d", reboot_flag);
            // put into receive mode
            int x;
            if (devicePairedStatus == 0)
            {
                getAndSendMacAddress();
                // reset_device();
                //  pair_with_device();
            }
            // else if (myDriver.waitAvailableTimeout(timeout)) today
            // reset_device();
            lora_receive();
            if (lora_received())
            {
                // Should be a reply message for us now
                // if (myDriver.recv(buf, &len)) today
                x = lora_receive_packet(buf, sizeof(buf));
                buf[x] = 0;
                printf("Received: %s\n", buf);
                no_reply_count_2 = 0;
                lora_receive();
                //{ today
                if (valid_header(buf))
                {

                    if (check_mac(buf))
                    {
                        if (reboot_flag == 2)
                        {
                            reboot_flag = 0;
                            /*esp_err_t err_reboot_flag = save_int_value("reboot_flag:", reboot_flag);

                            if (err_lbe != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS reboot_flag", esp_err_to_name(err_reboot_flag));
                            }*/
                        }
                        ESP_LOGI("pair_with_device()", "no_reply_count_is_device_paired:%d", no_reply_count_is_device_paired);
                        ESP_LOGW("lora_task()", "reboot_flag:%d", reboot_flag);
                        no_reply_count_is_device_paired = 0;
                        no_reply_cnt = 0;
                        replied_time = (unsigned long int)(esp_timer_get_time() / 1000);
                        // if (!error_check(buf))
                        //{
                        for (int i = 0; i < 12; i++)
                        {
                            result[i] = buf[i + 17]; // was 16 today;
                        }
                        sscanf(result, "%2hhX%2hhX%2hhX%2hhX%2hhX%2hhX", &body[0], &body[1], &body[2], &body[3], &body[4], &body[5]);
                        /*power_lora = ((buf[16] << 8) | (buf[17] & 0xff)); today
                        vrms_recieved = ((buf[18] << 8) | (buf[19] & 0xff));
                        irms_recieved = ((buf[20] << 8) | (buf[21] & 0xff));*/

                        power_lora = ((body[0] << 8) | (body[1] & 0xff));
                        vrms_recieved = ((body[2] << 8) | (body[3] & 0xff));
                        irms_recieved = ((body[4] << 8) | (body[5] & 0xff));
#ifdef DEBUG
                        ESP_LOGI("lora_task()", "got reply: %s power=%d", (char *)buf, power_lora);
                        ESP_LOGI("lora_task()", "got reply: %s vrms_recieved=%d", (char *)buf, vrms_recieved);
                        ESP_LOGI("lora_task()", "got reply: %s irms_recieved=%d", (char *)buf, irms_recieved);
                        ESP_LOGI("lora_task()", "RSSI     : %d", lora_packet_rssi());

                        // ESP_LOGI("lora_task()", "RSSI     : %d", lora.lastRssi());
                        // ESP_LOGI("lora_task()", "SNR     : %d", lora.lastSNR());
#endif
                        if (power_lora < 0)
                        {
#ifdef DEBUG
                            ESP_LOGI("lora_task()", "power flow direction solar to grid >>>>>>");
#endif
                        }
                        else if (power_lora > 0)
                        {
#ifdef DEBUG
                            ESP_LOGI("lora_task()", "power flow direction grid to charger <<<<<<");
#endif
                        }
                        else
                        {
#ifdef DEBUG
                            ESP_LOGI("lora_task()", "load balanced");
#endif
                        }

                        if (ecoPlusModeEnabled == 1 && configureChargerEnabledFlag == 0)
                        {
                            ecoPlusMode();
                        }
                        else if (configureChargerEnabledFlag == 0)
                        {
                            ecoMode();
                        }
                        else if (configureChargerEnabledFlag == 1)
                        {
                            ESP_LOGI("LORA Task", "configureChargerEnabledFlag:%d", configureChargerEnabledFlag);
                            ESP_LOGI("LORA Task", "signBit:%d", signBit);

                            if (CURRENT_STATE == IDLE_STATE)
                            {
                                previousPower = power_lora;
                                ESP_LOGI("LORA Task", "previousPower:%d", power_lora);
                            }
                            loadBalancingCurrentSetting = 7;
                            ESP_LOGI("LORA Task", "loadBalancingCurrentSetting:%f", loadBalancingCurrentSetting);
                            if (currentL1 >= 5)
                            {
                                if (CURRENT_STATE == START_STATE)
                                {
                                    currentPower = power_lora;
                                    if ((currentPower - previousPower) > 0)
                                    {
                                        signBit = 1;
                                        configureChargerEnabledFlag = 0;
                                        esp_err_t err_signBit = save_int_value("signBit:", signBit);
                                        if (err_signBit != ESP_OK)
                                        {
                                            ESP_LOGE(TAG, "Error (%s) saving to NVS signBit", esp_err_to_name(err_signBit));
                                        }

                                        esp_err_t err_configureChargerEnabledFlag = save_int_value("ccef:", configureChargerEnabledFlag);
                                        if (err_configureChargerEnabledFlag != ESP_OK)
                                        {
                                            ESP_LOGE(TAG, "Error (%s) saving to NVS ccef", esp_err_to_name(err_configureChargerEnabledFlag));
                                        }
                                    }
                                    else
                                    {
                                        signBit = 2;
                                        configureChargerEnabledFlag = 0;
                                        esp_err_t err_signBit = save_int_value("signBit:", signBit);
                                        if (err_signBit != ESP_OK)
                                        {
                                            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_signBit));
                                        }
                                        esp_err_t err_configureChargerEnabledFlag = save_int_value("ccef:", configureChargerEnabledFlag);
                                        if (err_configureChargerEnabledFlag != ESP_OK)
                                        {
                                            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_configureChargerEnabledFlag));
                                        }
                                    }
                                }
                            }
                        }
                        //}

                        //      Serial.print("RSSI: ");
                        //      Serial.println(rf95.lastRssi(), DEC);
                    }
                    else if (((char)buf[0 + header_length_receiving] == '#') && ((char)buf[1 + header_length_receiving] == '#') && ((char)buf[2 + header_length_receiving] == '#') && ((char)buf[3 + header_length_receiving] == '#'))
                    {
                        no_reply_count_is_device_paired++;
                        no_reply_cnt = 0;
#ifdef DEBUG
                        ESP_LOGI("pair_with_device()", "no_reply_count_is_device_paired:%d", no_reply_count_is_device_paired);
                        ESP_LOGI("pair_with_device()", "device is still in pair state");
#endif
                        if (no_reply_count_is_device_paired > no_reply_limit_is_device_paired)
                        {
                            no_reply_count_is_device_paired = 0;
                            devicePairedStatus = 0;
                            esp_err_t err_devicePairedStatus = save_int_value("dps:", devicePairedStatus);
                            if (err_devicePairedStatus != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_devicePairedStatus", esp_err_to_name(err_devicePairedStatus));
                            }
                        }
                    }
                }
                /*} today
                else
                {
    #ifdef DEBUG
                    ESP_LOGI("lora_task()", "recv failed");
    #endif
                }*/
                /*if ((unsigned long int)(esp_timer_get_time() / 1000) - replied_time > timeout)
                {
                    no_reply_cnt += 1;
                }*/
            }
            else
            {
#ifdef DEBUG
                ESP_LOGI("lora_task()", "No reply, is device running?");
#endif
                no_reply_cnt += 1;
                no_reply_count_2 += 1;
            }
            no_reply_check();
        }

        ESP_LOGI("lora_task()", "no_reply_flag:%d", no_reply_flag);
        ESP_LOGI("lora_task()", "no_reply_flag_2:%d", no_reply_flag_2);
        vTaskDelay(500 / portTICK_PERIOD_MS); // today was 1000
    }
}

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void sweepFlow(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t brightness, uint16_t delay, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 4); j++)
    {
        led_strip_hsv2rgb(hue, saturation, (brightness / 20), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 4, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 16), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 3, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 8), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 2, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 4), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, (j + 1), red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, (j), red, green, blue));
        vTaskDelay(pdMS_TO_TICKS(delay));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // ESP_LOGI(TAG, "j:%d", j);
        if (j == 32)
        {
            strip->clear(strip, 50);
            vTaskDelay(pdMS_TO_TICKS(delay_between));
        }
    }
}
void sweepFlowTailEnd(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t brightness, uint16_t delay)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int j = 0;
    for (j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 4); j++)
    {
        /*led_strip_hsv2rgb(hue, saturation, (brightness / 20), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 4, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 16), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 3, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 8), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j + 2, red, green, blue));
        led_strip_hsv2rgb(hue, saturation, (brightness / 4), &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, (j + 1), red, green, blue));*/
        // led_strip_hsv2rgb(hue, saturation, (brightness), &red, &green, &blue);
        //  Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, (j), 0, 255, 0));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // for(int i=0;i<1000000;i++){}
        vTaskDelay(delay / portTICK_RATE_MS);
        // ESP_LOGI(TAG, "j:%d", j);
    }

    for (j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 4); j++)
    {
        /* led_strip_hsv2rgb(hue, saturation, (brightness / 4), &red, &green, &blue);
         // Write RGB values to strip driver
         ESP_ERROR_CHECK(strip->set_pixel(strip, j + 4, red, green, blue));
         led_strip_hsv2rgb(hue, saturation, (brightness / 8), &red, &green, &blue);
         // Write RGB values to strip driver
         ESP_ERROR_CHECK(strip->set_pixel(strip, j + 3, red, green, blue));
         led_strip_hsv2rgb(hue, saturation, (brightness / 16), &red, &green, &blue);
         // Write RGB values to strip driver
         ESP_ERROR_CHECK(strip->set_pixel(strip, j + 2, red, green, blue));
         led_strip_hsv2rgb(hue, saturation, (brightness / 20), &red, &green, &blue);
         // Write RGB values to strip driver
         ESP_ERROR_CHECK(strip->set_pixel(strip, (j + 1), red, green, blue));*/
        // led_strip_hsv2rgb(hue, saturation, 0, &red, &green, &blue);
        //  Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, (j), 0, 0, 0));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // for(int i=0;i<1000000;i++){}
        vTaskDelay(delay / portTICK_RATE_MS);
    }
}

void allLEDSONOFF(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t brightness, uint16_t delay)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++)
    {
        led_strip_hsv2rgb(hue, saturation, brightness, &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
    }
    vTaskDelay(pdMS_TO_TICKS(delay));
    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(delay));
}

void allLEDSON(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t brightness, uint16_t delay)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++)
    {
        led_strip_hsv2rgb(hue, saturation, brightness, &red, &green, &blue);
        // Write RGB values to strip driver
        ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
    }
    vTaskDelay(pdMS_TO_TICKS(delay));
    // strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(delay));
}

void blink(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t maxBrightness, uint16_t speed, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int brightness = 0;

    for (brightness = 0; brightness < maxBrightness; brightness = brightness + speed)
    {
        // vTaskDelay(pdMS_TO_TICKS(delay));
        for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++)
        {
            led_strip_hsv2rgb(hue, saturation, brightness, &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }
    ESP_LOGI(TAG, "brightness=%d", brightness);

    for (brightness = maxBrightness; brightness >= 0; brightness = brightness - speed)
    {
        // vTaskDelay(pdMS_TO_TICKS(delay));
        for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++)
        {
            led_strip_hsv2rgb(hue, saturation, brightness, &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }
    ESP_LOGI(TAG, "brightness=%d", brightness);
    strip->clear(strip, 50);

    vTaskDelay(pdMS_TO_TICKS(delay_between));
}
/*void blinkMiddleold(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t maxBrightness, uint16_t speed, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int brightness = 0;
    int brightnessMiddle = 0;
    int maxBrightnessMiddle = 0;
    for (brightness = 0; brightness < maxBrightness; brightness = brightness + speed)
    {
        maxBrightnessMiddle = maxBrightnessMiddle + speed;
        if (maxBrightnessMiddle > maxBrightness)
        {
            maxBrightnessMiddle = maxBrightness;
        }
        brightnessMiddle = 0;
        // vTaskDelay(pdMS_TO_TICKS(delay));
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 4); j++)
        {
            if (j < 12)
            {
                brightnessMiddle = brightnessMiddle + maxBrightnessMiddle / 5;
            }
            else
            {
                brightnessMiddle = brightnessMiddle - maxBrightnessMiddle / 5;
            }
            if (brightnessMiddle > maxBrightnessMiddle)
            {
                brightnessMiddle = maxBrightnessMiddle;
            }
            if (brightnessMiddle < 0)
            {
                brightnessMiddle = 0;
            }

            ESP_LOGI(TAG, "brightnessMiddle=%d", brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }
    for (brightness = 0; brightness < maxBrightness; brightness = brightness + speed)
    {
        maxBrightnessMiddle = maxBrightnessMiddle - speed;
        if (maxBrightnessMiddle < 0)
        {
            maxBrightnessMiddle = 0;
        }
        brightnessMiddle = 0;
        // vTaskDelay(pdMS_TO_TICKS(delay));
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 4); j++)
        {
            if (j < 12)
            {
                brightnessMiddle = brightnessMiddle + maxBrightnessMiddle / 5;
            }
            else
            {
                brightnessMiddle = brightnessMiddle - maxBrightnessMiddle / 5;
            }
            if (brightnessMiddle > maxBrightnessMiddle)
            {
                brightnessMiddle = maxBrightnessMiddle;
            }
            if (brightnessMiddle < 0)
            {
                brightnessMiddle = 0;
            }

            ESP_LOGI(TAG, "brightnessMiddle=%d", brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }
    ESP_LOGI(TAG, "brightness=%d", brightness);

     for (brightness = maxBrightness; brightness >= 0; brightness = brightness - speed)
     {
         // vTaskDelay(pdMS_TO_TICKS(delay));
         for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++)
         {
             led_strip_hsv2rgb(hue, saturation, brightness, &red, &green, &blue);
             // Write RGB values to strip driver
             ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
             ESP_ERROR_CHECK(strip->refresh(strip, 100));
         }
     }
    ESP_LOGI(TAG, "brightness=%d", brightness);
    strip->clear(strip, 50);

    vTaskDelay(pdMS_TO_TICKS(delay_between));
}*/
void blinkMiddle(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t maxBrightness, uint16_t speed, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int gain = 0;
    int brightnessMiddle = 0;

    // int maxBrightnessMiddle = 0;
    for (gain = 0; gain <= maxBrightness; gain = gain + speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {
            if (j < 12)
            {
                brightnessMiddle = 0.694 * pow(j, 2) + 0.0053 * j;
                brightnessMiddle = brightnessMiddle * gain / 100;
                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            else
            {
                brightnessMiddle = 0.694 * pow(j, 2) - 33.317 * j + 399.868;
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                 {
                     brightnessMiddle = 100;
                 }*/
            }
            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }
    for (gain = maxBrightness; gain > 1; gain = gain - speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {
            if (j < 12)
            {
                brightnessMiddle = 0.694 * pow(j, 2) + 0.0053 * j;
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            else
            {
                brightnessMiddle = 0.694 * pow(j, 2) - 33.317 * j + 399.868;
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
    }

    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(delay_between));
}
void blinkMiddleNew(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t maxBrightness, uint16_t speed, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int gain = 0;
    int brightnessMiddle = 0;

    // int maxBrightnessMiddle = 0;
    for (gain = 0; gain <= maxBrightness; gain = gain + speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {
            if (j < 16)
            {
                brightnessMiddle = 0.390625 * pow(j, 2);
                brightnessMiddle = brightnessMiddle * gain / 100;
                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            else
            {
                brightnessMiddle = 0.390625 * pow(j, 2) - 25 * j + 400;
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                 {
                     brightnessMiddle = 100;
                 }*/
            }
            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    for (gain = maxBrightness; gain > 1; gain = gain - speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {
            if (j < 16)
            {
                brightnessMiddle = 0.390625 * pow(j, 2);
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            else
            {
                brightnessMiddle = 0.390625 * pow(j, 2) - 25 * j + 400;
                brightnessMiddle = brightnessMiddle * gain / 100;

                if (brightnessMiddle == 0)
                {
                    brightnessMiddle = 1;
                }
                /*if (brightnessMiddle >=100)
                {
                    brightnessMiddle = 100;
                }*/
            }
            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(delay_between));
}

void BlinkMiddleArray(led_strip_t *strip, uint16_t hue, uint16_t saturation, uint16_t maxBrightness, uint16_t speed, uint16_t delay_between)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    int gain = 0;
    int brightnessMiddle = 0;

    // int maxBrightnessMiddle = 0;
    for (gain = 0; gain <= maxBrightness; gain = gain + speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {

            brightnessMiddle = ledBrightness[j];
            brightnessMiddle = brightnessMiddle * gain / 100;
            if (brightnessMiddle == 0)
            {
                brightnessMiddle = 1;
            }

            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    for (gain = maxBrightness; gain > 1; gain = gain - speed)
    {
        for (int j = 0; j < (CONFIG_EXAMPLE_STRIP_LED_NUMBER - 3); j++)
        {
            brightnessMiddle = ledBrightness[j];
            brightnessMiddle = brightnessMiddle * gain / 100;

            if (brightnessMiddle == 0)
            {
                brightnessMiddle = 1;
            }
            // ESP_LOGI(TAG, "j:%d   brightnessMiddle=%d", j, brightnessMiddle);
            led_strip_hsv2rgb(hue, saturation, (brightnessMiddle), &red, &green, &blue);
            // Write RGB values to strip driver
            ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(delay_between));
}

void main_task(void *pvParameter)
{
    // wait for connection

    printf("connected!\n");

    /* ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
     printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
     printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
     printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
     printf("\n");*/
    // ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    // ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    // printf("Subscribed to TWDT\n");
    // xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        //  esp_task_wdt_reset();

        nowTime = esp_timer_get_time() / 1000;
        if (nowTime - serialTime > 500)
        {
            serialErrorTimeout++;
            serialTime = nowTime;
            // printf("! %lu",serialTime);
            int tmpState = 0;
            int tmpState2 = 0;
            if (ota_enabled_flag == false)
            {
                tmpState = serialReadFunc();
            }
            if (tmpState == 1)
            {

                // isolatedState = recevedSerial.state.bits.Status;removed
                // powerSideState = recevedSerial.state.bits.StatusPwr;removed
                controlSideState = recevedSerial.control_Side_Status.bits.curretState;
                connectorStartCharge = recevedSerial.network_SideRequest_Struct.bits.start_Charge;
                connectorStopCharge = recevedSerial.network_SideRequest_Struct.bits.stop_Charge;
                chargingActive = recevedSerial.control_Side_Status.bits.charging_active;
                if (chargeInProgress == 0)
                {
                    scheduleChargeStart = recevedSerial.network_SideRequest_Struct.bits.schedule_charge;
                }
                ESP_LOGW("msm", "connectorStartCharge:%d", connectorStartCharge);
                ESP_LOGW("msm", "scheduleChargeStart:%d", scheduleChargeStart);
                rtc_UpdateComplete = recevedSerial.network_SideRequest_Struct.bits.rtcUpdateComplete;
                rtc_UpdateAlarmComplete = recevedSerial.network_SideRequest_Struct.bits.rtcUpdateAlarmComplete;
                ESP_LOGW("msm", "rtc_UpdateAlarmComplete: %d,UpdateAlarmSerialFlag: %d", rtc_UpdateAlarmComplete, UpdateAlarmSerialFlag);
                if (rtc_UpdateAlarmComplete == 1)
                {
                    UpdateAlarmSerialFlag = 0;
                }

// Serial.println("SERIAL_IN:IsoStat: " + String(isolatedState) + " PwrStat: " + String(powerSideState) + " STRT:" + String(connectorStartCharge) + " STP:" + String(connectorStopCharge));
#ifdef DEBUG
                ESP_LOGI("msm", "SERIAL_IN: id %d: controlSideState:%d PwrStat:%d  STRT: %d STP: %d", recevedSerial.messegeID, (int)controlSideState, (int)powerSideState, connectorStartCharge, connectorStopCharge);
#endif
                // MeaningFull message receved
                serialErrorTimeout = 0;
                serialRecevedFlag = 1;

                if (recevedSerial.messegeID == 0)
                {
                    voltageL1 = recevedSerial.val1.all;
                    voltageL2 = recevedSerial.val2.all;
                    voltageL3 = recevedSerial.val3.all;

                    if (powerOnSerialBit == false)
                    {
                        powerOnSerialBit = true;
                        CURRENT_STATE = POWERON_STATE;
                    }
#ifdef DEBUG
                    ESP_LOGI("msm", "SERIAL_IN:Voltage:%fCurrent:%fpower:%f", voltageL1, currentL1, power);
#endif
                }
                if (recevedSerial.messegeID == 1)
                {
                    currentLive1 = recevedSerial.val1.all;
                    currentL1 = currentLive1 / 10;

                    currentLive2 = recevedSerial.val2.all;
                    currentL2 = currentLive2 / 10;

                    currentLive3 = recevedSerial.val3.all;
                    currentL3 = currentLive3 / 10;
                }

                if (recevedSerial.messegeID == 2)
                {

                    energy = recevedSerial.val1.all;
                    power = recevedSerial.val2.all * 0.001;
                    temp1 = recevedSerial.val3.bytes.LB;
                    // firmwareVersion=recevedSerial.val3.bytes.HB;
                    // stm32_version_num = firmwareVersion;
                }

                if (recevedSerial.messegeID == 3)
                {
                    stm32_version_num_1=recevedSerial.val1.bytes.LB;
                    stm32_version_num_2=recevedSerial.val1.bytes.HB;
                    stm32_version_num_3=recevedSerial.val2.bytes.LB;
                    stm32_version_num = 100*stm32_version_num_1 + 10*stm32_version_num_2 + 1*stm32_version_num_3;

                }

            }

            if (serialRecevedFlag == 1 && ota_enabled_flag == false)
            {
                sendSerial.StatusLB = CURRENT_STATE;
                sendSerial.StatusHB1 = stopCharge;
                sendSerial.StatusHB2 = scheduleChargeEnableState;
                sendSerial.isChargerLocked = isChargerLocked;
                sendSerial.scheduleChargeStart = scheduleChargeStart;
                ESP_LOGW("inside serial send", "scheduleChargeStart:%d", scheduleChargeStart);
                sendSerial.errorLB = is_internet_available;
                sendSerial.ledOffCommandRemote = led_off_command_remote;

                esp_err_t err_sstwd = load_key_value("sstwd:", scheduleStartTimeWeekDays, sizeof(scheduleStartTimeWeekDays));
                esp_err_t err_ssttwd = load_key_value("ssttwd:", scheduleStopTimeWeekDays, sizeof(scheduleStopTimeWeekDays));
                esp_err_t err_sstwe = load_key_value("sstwe:", scheduleStartTimeWeekEnds, sizeof(scheduleStartTimeWeekEnds));
                esp_err_t err_ssttwe = load_key_value("ssttwe:", scheduleStopTimeWeekEnds, sizeof(scheduleStopTimeWeekEnds));

                if (err_sstwd != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) loading to NVS err_sstwd", esp_err_to_name(err_sstwd));
#endif
                }
                if (err_ssttwd != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) loading to NVS err_ssttwd", esp_err_to_name(err_ssttwd));
#endif
                }
                if (err_sstwe != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) loading to NVS err_sstwe", esp_err_to_name(err_sstwe));
#endif
                }
                if (err_ssttwe != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) loading to NVS err_ssttwe", esp_err_to_name(err_ssttwe));
#endif
                }

                // sendSerial.messegeID = 1;
                switch (serialState)
                {
                case 0:
                {
                    esp_err_t err_lbe = load_int_value("lbeState:", &loadBalancingEnableState);
                    if (err_lbe != ESP_OK)
                    {
#ifdef DEBUG
                        ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_lbe));
#endif
                    }

                    sendSerial.messegeID = 0;
                    if (loadBalancingEnableState == 0)
                    {
                        sendSerial.val1.bytes.LB = maximumCurrentSetting;
                    }
                    else
                    {
                        sendSerial.val1.bytes.LB = loadBalancingCurrentSetting;
                    }

                    char HH0[3];
                    memcpy(HH0, &strftime_buf[11], 2);
                    HH0[2] = '\0';
                    ESP_LOGE("msm", "HH0:%s", HH0);
                    sendSerial.val2.bytes.HB = atoi(HH0);
                    char MM0[3];
                    memcpy(MM0, &strftime_buf[14], 2);
                    MM0[2] = '\0';
                    ESP_LOGE("msm", "MM0:%s", MM0);
                    sendSerial.val3.bytes.LB = atoi(MM0);
                    /*char SS0[3];
                    memcpy(SS0, &strftime_buf[6], 2);
                    SS0[2] = '\0';
                    ESP_LOGE("msm", "SS0:%s", SS0);
                    sendSerial.val3.bytes.HB = atoi(SS0);
                    */

                    sendSerial.val1.bytes.HB = time_setup_success_flag;
                    sendSerial.val2.bytes.LB = UpdateAlarmSerialFlag;
                    sendSerial.val3.bytes.HB = loadBalancingEnableState;
                    serialState = 1;
                }
                break;
                case 1:

                    char HH1[3];
                    memcpy(HH1, &scheduleStartTimeWeekDays[0], 2);
                    HH1[2] = '\0';
                    ESP_LOGE("msm", "HH1:%s", HH1);
                    sendSerial.val1.bytes.LB = atoi(HH1);
                    char MM1[3];
                    memcpy(MM1, &scheduleStartTimeWeekDays[3], 2);
                    MM1[2] = '\0';
                    ESP_LOGE("msm", "MM1:%s", MM1);
                    sendSerial.val1.bytes.HB = atoi(MM1);
                    /*char SS1[3];
                    memcpy(SS1, &scheduleStartTimeWeekDays[6], 2);
                    SS1[2] = '\0';
                    ESP_LOGE("msm", "SS1:%s", SS1);
                    sendSerial.val2.bytes.LB = atoi(SS1);*/

                    char HH2[3];
                    memcpy(HH2, &scheduleStopTimeWeekDays[0], 2);
                    HH2[2] = '\0';
                    ESP_LOGE("msm", "HH2:%s", HH2);
                    sendSerial.val2.bytes.LB = atoi(HH2);
                    char MM2[3];
                    memcpy(MM2, &scheduleStopTimeWeekDays[3], 2);
                    MM2[2] = '\0';
                    ESP_LOGE("msm", "MM2:%s", MM2);
                    sendSerial.val2.bytes.HB = atoi(MM2);
                    /*char SS2[3];
                    memcpy(SS2, &scheduleStopTimeWeekDays[6], 2);
                    SS2[2] = '\0';
                    ESP_LOGE("msm", "SS2:%s", SS2);
                    sendSerial.val3.bytes.HB = atoi(SS2);*/

                    char mm[3];
                    memcpy(mm, &strftime_buf[5], 2);
                    mm[2] = '\0';
                    ESP_LOGE("msm", "mm:%s", mm);
                    sendSerial.val3.bytes.LB = atoi(mm);

                    char dd[3];
                    memcpy(dd, &strftime_buf[8], 2);
                    dd[2] = '\0';
                    ESP_LOGE("msm", "dd:%s", dd);
                    sendSerial.val3.bytes.HB = atoi(dd);

                    sendSerial.messegeID = 1;
                    serialState = 2;
                    break;

                case 2:

                    char HH3[3];
                    memcpy(HH3, &scheduleStartTimeWeekEnds[0], 2);
                    HH3[2] = '\0';
                    ESP_LOGE("msm", "HH3:%s", HH3);
                    sendSerial.val1.bytes.LB = atoi(HH3);
                    char MM3[3];
                    memcpy(MM3, &scheduleStartTimeWeekEnds[3], 2);
                    MM3[2] = '\0';
                    ESP_LOGE("msm", "MM3:%s", MM3);
                    sendSerial.val1.bytes.HB = atoi(MM3);
                    /*char SS3[3];
                    memcpy(SS3, &scheduleStartTimeWeekEnds[6], 2);
                    SS3[2] = '\0';
                    ESP_LOGE("msm", "SS3:%s", SS3);
                    sendSerial.val2.bytes.LB = atoi(SS3);
                    */

                    char HH4[3];
                    memcpy(HH4, &scheduleStopTimeWeekEnds[0], 2);
                    HH4[2] = '\0';
                    ESP_LOGE("msm", "HH4:%s", HH4);
                    sendSerial.val2.bytes.LB = atoi(HH4);
                    char MM4[3];
                    memcpy(MM4, &scheduleStopTimeWeekEnds[3], 2);
                    MM4[2] = '\0';
                    ESP_LOGE("msm", "MM4:%s", MM4);
                    sendSerial.val2.bytes.HB = atoi(MM4);
                    /*char SS4[3];
                    memcpy(SS4, &scheduleStopTimeWeekEnds[6], 2);
                    SS4[2] = '\0';
                    ESP_LOGE("msm", "SS4:%s", SS4);
                    sendSerial.val3.bytes.HB = atoi(SS4);
                    */

                    char yy1[3];
                    memcpy(yy1, &strftime_buf[0], 2);
                    yy1[2] = '\0';
                    ESP_LOGE("msm", "yy1:%s", yy1);
                    sendSerial.val3.bytes.LB = atoi(yy1);

                    char yy2[3];
                    memcpy(yy2, &strftime_buf[2], 2);
                    yy2[2] = '\0';
                    ESP_LOGE("msm", "yy2:%s", yy2);
                    sendSerial.val3.bytes.HB = atoi(yy2);

                    sendSerial.messegeID = 2;
                    serialState = 0;
                    break;
                }

#ifdef DEBUG
                ESP_LOGE("msm", "SerialOut:CURRENT_STATE:%d", CURRENT_STATE);
#endif
                tmpState2 = serialSendFunc();

                if (tmpState2 == 1)
                {
                    serialRecevedFlag = 0;
                }
            }
            if (serialErrorTimeout > 20)
            {
#ifdef DEBUG
                ESP_LOGI("msm", "SERIAL_IN: Serial timeout error");
#endif
            }
        }
        else if (nowTime < serialTime)
        {
            serialTime = nowTime;
        }

        nowTime = esp_timer_get_time() / 1000;
        if (nowTime - previousMIllis_2 >= 500)
        {
#ifdef DEBUG
            ESP_LOGW("msm", "MQTT command:%s, mobileStartCommand:%d", MQTTCommand, mobileStartCommand);
#endif
            if (CURRENT_STATE == START_STATE)
            {
                gpio_set_level(ledTogglePin, 1);
            }
            else
            {

                if (toggle_pin == 0)
                {
                    gpio_set_level(ledTogglePin, 1);
                    toggle_pin = 1;
                }
                else if (toggle_pin == 1)
                {
                    gpio_set_level(ledTogglePin, 0);
                    toggle_pin = 0;
                }
            }
            previousMIllis_2 = nowTime;
            mainStateMachine();
            faultStateMachine();
            monitor();
            // printf("! %lu ",serialTime);
            // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        }
        else if (nowTime < previousMIllis_2)
        {
            previousMIllis_2 = nowTime;
        }
        if (timerCountDownStart)
        {
            nowTime = esp_timer_get_time() / 1000;
            if (nowTime - previousMIllis_3 >= 1000)
            {
#ifdef DEBUG
                ESP_LOGI("msm", "timerCountDownStart:  %d", timerCountDown);
#endif
                previousMIllis_3 = nowTime;
                timerCountDown--;
            }
            else if (nowTime < previousMIllis_3)
            {
                previousMIllis_3 = nowTime;
            }
        }

        nowTime = esp_timer_get_time() / 1000;
        if (nowTime - previousMIllis_1 >= 50000)
        {

            previousMIllis_1 = nowTime;
            ticker_count++;
            countDiff++;
#ifdef DEBUG
            ESP_LOGW("msm", "mqtt timer %d %d", ticker_count, countDiff);
#endif
            if (countDiff > 3)
            {
#ifdef DEBUG
                ESP_LOGI("msm", "MQTT connection is broken, more than 3 messages have been lost - Stopping Transaction");
#endif
                // connectorStopCharge=1;
                countDiff = 0;
            }
            // printf("! %lu ",serialTime);
            // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        }
        else if (nowTime < previousMIllis_1)
        {
#ifdef DEBUG
            ESP_LOGW("msm", "mqtt timer else if");
#endif
            previousMIllis_1 = nowTime;
        }
        // vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
        vTaskDelay(100 / portTICK_RATE_MS); // today 100
    }
}

extern "C"
{
    void app_main();
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
#endif
    }

#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0)
    {
#ifdef DEBUG
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
#endif
        return ESP_FAIL;
    }
#endif

#ifdef CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
    /**
     * Secure version check from firmware image header prevents subsequent download and flash write of
     * entire firmware image. However this is optional because it is also taken care in API
     * esp_https_ota_finish at the end of OTA update procedure.
     */
    const uint32_t hw_sec_version = esp_efuse_read_secure_version();
    if (new_app_info->secure_version < hw_sec_version)
    {
        ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed, %d < %d", new_app_info->secure_version, hw_sec_version);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}

void advanced_ota_example_task(void *pvParameter)
{
#ifdef DEBUG
    ESP_LOGI(TAG, "Starting Advanced OTA example");
#endif
    if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
    {
        led_off_command = 1; // added due to the led wrongly blinking issue during OTA update
        xSemaphoreGive(readStateMutex);
    }

    esp_err_t ota_finish_err = ESP_OK;
    esp_http_client_config_t config = {
        .url = "https://chargenet-fwu.s3.us-west-2.amazonaws.com/advanced_https_ota.bin",
        .cert_pem = (char *)server_cert_1_pem_start,
        .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0)
    {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    }
    else
    {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
#ifdef CONFIG_EXAMPLE_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .partial_http_download = true,
        .max_http_request_size = CONFIG_EXAMPLE_HTTP_REQUEST_SIZE,
#endif
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
#endif
        if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
        {
            led_off_command = 0;
            xSemaphoreGive(readStateMutex);
        }
        vTaskDelete(NULL);
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
#endif
        goto ota_end;
    }
    err = validate_image_header(&app_desc);
    if (err != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "image header verification failed");
#endif
        goto ota_end;
    }

    while (1)
    {

        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
        {
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                led_off_command = 0;
                xSemaphoreGive(readStateMutex);
            }
            break;
        }
// esp_https_ota_perform returns after every read operation which gives user the ability to
// monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
// data read so far.
#ifdef DEBUG
        ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
#endif
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true)
    {
// the OTA image was not completely received and user can customise the response to this situation.
#ifdef DEBUG
        ESP_LOGE(TAG, "Complete data was not received.");
#endif
        if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
        {
            led_off_command = 0;
            xSemaphoreGive(readStateMutex);
        }
    }
    else
    {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK))
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
#endif
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }
        else
        {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
#endif
            }
#ifdef DEBUG
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
#endif
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                led_off_command = 0;
                xSemaphoreGive(readStateMutex);
            }
            vTaskDelete(NULL);
        }
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);
#ifdef DEBUG
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
#endif
    if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
    {
        led_off_command = 0;
        xSemaphoreGive(readStateMutex);
    }
    vTaskDelete(NULL);
}
void counDiffTime(TimerHandle_t xTimer)
{
    // printf(" t ");

    if (xSemaphoreTake(timerMutex, 1) == pdTRUE)
    {
        // if (chargePause == 10) {
        timeDiffTask1 = timeDiffTask1 + 1;
        xSemaphoreGive(timerMutex);
        //}
    }
}
void ledStripOff(led_strip_t *strip)
{

    strip->clear(strip, 50);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void blink_led_task(void *pvParameter)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_26, RMT_TX_CHANNEL);

    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "install WS2812 driver failed");
#endif
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    esp_err_t err_led_off_command = load_int_value("led:", &led_off_command_remote);
    if (err_led_off_command != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_led_off_command));
#endif
    }
#ifdef DEBUG
    ESP_LOGW(TAG, "led_off_command_remote: %d", led_off_command_remote);
#endif
    while (true)
    {
        // ESP_LOGE(TAG, "LED TEST EXAPMLE");
        /*{
          PN532ReadState = CONFIGURE_PN532;
          xSemaphoreGive(readStateMutex);
        }*/
        if (led_off_command_remote == 0)
        {
            if (led_off_command == 0)
            {
                switch (currentLedState)
                {
                case sweepGreen:
                    // sweepFlowTailEnd(strip, 120, 100, 100, 20);
                    // blinkMiddleNew(strip, 120, 100, 100, 4, 500);
                    BlinkMiddleArray(strip, 120, 100, 100, 4, 500);
                    // blinkMiddle(strip, 180, 100, 100, 5, 500);
                    break;
                case sweepYellow:
                    sweepFlowTailEnd(strip, 60, 100, 100, 50);
                    break;
                case sweepBlue:
                    sweepFlowTailEnd(strip, 240, 100, 100, 50);
                    break;
                case sweepCyan:
                    sweepFlowTailEnd(strip, 180, 100, 100, 50);
                    break;
                case blinkYellow:
                    blink(strip, 60, 100, 100, 3, 500);
                    break;
                case blinkBlue:
                    // blink(strip, 180, 100, 100, 5, 500);
                    // blinkMiddleNew(strip, 180, 100, 100, 4, 500);
                    // BlinkMiddleArray(strip, 180, 100, 100, 4, 500);
                    allLEDSON(strip, 120, 100, 100, 500);
                    //  blink(strip, 240, 100, 100, 3, 500);
                    //  allLEDSONOFF(strip, 0, 100, 100, 500);
                    // sweepFlowTailEnd(strip, 0, 100, 100, 50);
                    // sweepFlow(strip, 120, 100, 100, 50,50);
                    break;
                case blinkMagenta:
                    blink(strip, 300, 100, 100, 3, 500);
                    break;
                case blinkRed:
                    blink(strip, 0, 100, 100, 3, 500);
                    break;
                case ledsOnOffError:
                    allLEDSONOFF(strip, 0, 100, 100, 500);
                    break;
                }
            }
            else
            {
                ledStripOff(strip);
            }
        }
        else
        {
            ledStripOff(strip);
        }
        vTaskDelay(40 / portTICK_RATE_MS);
    }
}

void initialize_memory()
{
    esp_err_t err_bootOnce = load_int_value("bootOnce:", &bootOnce);
    if (err_bootOnce != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS bootOnce", esp_err_to_name(err_bootOnce));
#endif
    }
    esp_err_t err_rebootTime = load_int_value("rTime:", &rebootTime);
    if (err_rebootTime != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG, "Error (%s) loading to NVS rebootTime", esp_err_to_name(err_rebootTime));
#endif
    }

    rebootTime += 1;
    esp_err_t err_rebootTime2 = save_int_value("rTime:", rebootTime);
    if (err_rebootTime2 != ESP_OK)
    {
#ifdef DEBUG
        ESP_LOGE(TAG_B, "Error (%s) saving to NVS rebootTime", esp_err_to_name(err_rebootTime2));
#endif
    }
    // bootOnce =0;
    if (bootOnce == 0)
    {

        bootOnce = 1;
        esp_err_t err_bootOnce2 = save_int_value("bootOnce:", bootOnce);
        if (err_bootOnce2 != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS bootOnce", esp_err_to_name(err_bootOnce2));
#endif
        }
        wifi_config_state = 1;
        esp_err_t err_wifi_config_state = save_int_value("wcstate:", wifi_config_state);
        if (err_wifi_config_state != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS wifi_config_state", esp_err_to_name(err_wifi_config_state));
#endif
        }

        isChargerLocked = 0;
        esp_err_t err_isLockedtrue = save_int_value("isLocked:", isChargerLocked);
        if (err_isLockedtrue != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS isChargerLocked", esp_err_to_name(err_isLockedtrue));
#endif
        }

        chargeInProgress = 0;
        esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

        if (err_chargeInProgress != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS chargeInProgress", esp_err_to_name(err_chargeInProgress));
#endif
        }

        isConfigState = 1;
        esp_err_t err_isConfigState = save_int_value("isCState:", isConfigState);
        if (err_isConfigState != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS isConfigState", esp_err_to_name(err_isConfigState));
#endif
        }
        maximumCurrentSetting = 7; // set a default value
        esp_err_t err_maximumCurrentSetting = save_int_value("mcs:", maximumCurrentSetting);
        if (err_maximumCurrentSetting != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) saving to NVS maximumCurrentSetting", esp_err_to_name(err_maximumCurrentSetting));
#endif
        }
        loadBalancingEnableState = 0;
        esp_err_t err_lbe = save_int_value("lbeState:", loadBalancingEnableState);
        if (err_lbe != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) saving to NVS loadBalancingEnableState", esp_err_to_name(err_lbe));
#endif
        }
        ecoPlusModeEnabled = 0;
        esp_err_t err_ecoPlusModeEnabled = save_int_value("epmEnable:", ecoPlusModeEnabled);
        if (err_ecoPlusModeEnabled != ESP_OK)
        {
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_ecoPlusModeEnabled));
        }
        // scheduleStartTimeWeekDays = "00:00:00";
        esp_err_t err_scheduleStartTimeWeekDays = save_key_value("sstwd:", scheduleStartTimeWeekDays);
        if (err_scheduleStartTimeWeekDays != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekDays));
        }
        // scheduleStopTimeWeekDays = "00:00:00";
        esp_err_t err_scheduleStopTimeWeekDays = save_key_value("ssttwd:", scheduleStopTimeWeekDays);
        if (err_scheduleStopTimeWeekDays != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekDays));
        }
        // scheduleStartTimeWeekEnds = "00:00:00";
        esp_err_t err_scheduleStartTimeWeekEnds = save_key_value("sstwe:", scheduleStartTimeWeekEnds);
        if (err_scheduleStartTimeWeekEnds != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekEnds));
        }
        esp_err_t err_scheduleStopTimeWeekEnds = save_key_value("ssttwe:", scheduleStopTimeWeekEnds);
        // scheduleStopTimeWeekEnds = "00:00:00";
        if (err_scheduleStopTimeWeekEnds != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekEnds));
        }
        configureChargerEnabledFlag = 1;
        esp_err_t err_configureChargerEnabledFlag = save_int_value("ccef:", configureChargerEnabledFlag);
        if (err_configureChargerEnabledFlag != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_configureChargerEnabledFlag));
        }
        signBit = 1;
        esp_err_t err_signBit = save_int_value("signBit:", signBit);
        // scheduleStopTimeWeekEnds = "00:00:00";
        if (err_signBit != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_signBit));
        }

#ifdef DEBUG
        ESP_LOGW(TAG_B, "bootOnce Executed now");
#endif
    }
}

void app_main()
{

    // displayMutex = xSemaphoreCreateMutex();
    // mqttMutex = xSemaphoreCreateMutex();
    timerMutex = xSemaphoreCreateMutex();
    internetMutex = xSemaphoreCreateMutex();
    readStateMutex = xSemaphoreCreateMutex();

    // checkInternetQueue = xQueueCreate( 1, sizeof( int ) );
    stopTransactionQueue = xQueueCreate(1, sizeof(int));
    // checkInternetQueueEnergy = xQueueCreate( 1, sizeof( int ) );
    stopTransactionQueueEnergy = xQueueCreate(1, sizeof(int));
    // checkInternetQueuePrev = xQueueCreate( 1, sizeof( int ) );
    stopTransactionQueuePrev = xQueueCreate(1, sizeof(int));
    HTTPCodeQueueEnergy = xQueueCreate(1, sizeof(int));
    HTTPCodeQueue = xQueueCreate(1, sizeof(int));
    HTTPCodeQueuePrev = xQueueCreate(1, sizeof(int));

    init_smart_config_setup();
#ifdef DEBUG
    ESP_LOGI(TAG, "init_smart_config_setup success");
#endif

    // init_PN532_I2C(sda_io_pin, scl_io_pin, pn532_rst_io_pin, pn532_irq_io_pin, I2C_NUM_0);
    // ESP_LOGI(TAG, "init_PN532_I2C success");
    init_uart();

    initialize_spiffs();
    initialise_pins();
    initialize_memory();
#ifdef DEBUG
    ESP_LOGI(TAG, "init_uart success");
#endif

    CURRENT_STATE = POWERON_STATE;
#ifdef DEBUG
    ESP_LOGI(TAG, "mqtt_app_start success");
#endif
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    countDiffTimer = xTimerCreate("CountDif", mainONE_SHOT_TIMER_PERIOD, pdTRUE, 0, counDiffTime);

    xTaskCreatePinnedToCore(&main_task, "main_task", 1024 * 10, NULL, 0, &Task_main, 0); // today changed to zero again
    
    // stm32 version check
    stm32flash_firm_check(stm32_version_num);

    
    initialise_wifi();
#ifdef DEBUG
    ESP_LOGI(TAG, "initialise_wifi success");
#endif
    xTaskCreate((TaskFunction_t)&check_internet_task, "check_internet_task", 2500, &Task_internet, 2, NULL);
    // initialize_ping(500, 2);
    // xTaskCreate((TaskFunction_t)&startConfig, "start_config", 2048 * 4, NULL, 1, NULL);
    xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task", 1024 * 8, NULL, 1, NULL);
    // xTaskCreate(&blink_led_task, "blink_led_task", 1024 * 10, NULL, 5, NULL);
    //  xTaskCreatePinnedToCore(&blink_led_task, "blink_led_task", 1024 * 10, (void *)1, 7, NULL, 1);
#ifdef DEBUG
    ESP_LOGI(TAG, "blink led task success");
#endif
    xTaskCreatePinnedToCore(networkTask, "networkTask", 8192, NULL, 2, &Task_net, 1);
    xTaskCreate(&lora_task, "lora_task", 1024 * 8, NULL, 2, NULL); // today priority was 2

    // xTaskCreatePinnedToCore (&blink_led_task,	"blink_led_task",1024 * 10, (void *)1, 5, NULL, 0);
    vTaskDelete(NULL);
}
void no_fault()
{
    if (error_occurred_flag == true)
    {
        currentFault = FAULT_STATE_ENTER;
    }
    else
    {
        currentFault = NO_FAULT;
    }
}

void faultStateEnter()
{
    if (recevedSerial.control_Side_Error.bits.serialAError == 1 || recevedSerial.control_Side_Error.bits.serialBError == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.terminalOT == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.diodeCheck_failed == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.powerSide_fault == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.gfit_error_count > 0)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.networkSide_fault == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.control_Side_Error.bits.error_count_updated > 0)
    {
        CURRENT_STATE = FAULT_STATE;
    }

    else if (recevedSerial.power_Side_Error.bits.error_OV_L1 == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.error_UV_L1 == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.error_SR_L1 == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.error_SR_N == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.error_SR_C == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.error_GFI_test == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.trip_OC_L1 == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }
    else if (recevedSerial.power_Side_Error.bits.trip_GFI == 1)
    {
        CURRENT_STATE = FAULT_STATE;
    }

    else
    {
        currentFault = NO_FAULT;
        //  CURRENT_STATE = IDLE_STATE;
    }
}

void faultStateMachine()
{

    switch (currentFault)
    {
    case NO_FAULT:
        no_fault();
        break;
    case FAULT_STATE_ENTER:
        faultStateEnter();
        break;
    default:
        break;
    }
}

void monitor()
{
    if (recevedSerial.power_Side_Error.all > 0 || recevedSerial.control_Side_Error.all > 0)
    {

        error_occurred_flag = true;
    }
    else
    {
        error_occurred_flag = false;
    }
}
void mainStateMachine()
{
#ifdef DEBUG
    printf("msm %d %d %d %d %d \n", uxTaskGetStackHighWaterMark(NULL), xPortGetFreeHeapSize(), esp_get_free_heap_size(), esp_get_minimum_free_heap_size(), esp_get_free_internal_heap_size());
#endif
    PREVIOUS_STATE = CURRENT_STATE;
    if (CURRENT_STATE == POWERON_STATE)
    {
        esp_err_t err_transaction_id_value = read_long_from_eeprom("tidvalue:", &transaction_id_value);

        if (err_transaction_id_value != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS transaction_id_value ", esp_err_to_name(err_transaction_id_value));
#endif
        }
        esp_err_t err_isPaired = load_int_value("isPaired:", &isPaired);

        if (err_isPaired != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS isPaired ", esp_err_to_name(err_isPaired));
#endif
        }

        esp_err_t err_scheduleChargeEnableState = load_int_value("sceState:", &scheduleChargeEnableState);

        if (err_scheduleChargeEnableState != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS scheduleChargeEnableState ", esp_err_to_name(err_scheduleChargeEnableState));
#endif
        }
        esp_err_t err_lbe = load_int_value("lbeState:", &loadBalancingEnableState);
        if (err_lbe != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS lbeState", esp_err_to_name(err_lbe));
#endif
        }

        esp_err_t err_wifi_config_state = load_int_value("wcstate:", &wifi_config_state);

        if (err_wifi_config_state != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS err_wifi_config_state ", esp_err_to_name(err_wifi_config_state));
#endif
        }

        esp_err_t err_is_charger_locked = load_int_value("isLocked:", &isChargerLocked);
        if (err_is_charger_locked != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS isLocked", esp_err_to_name(err_is_charger_locked));
#endif
        }

        esp_err_t err_chargeInProgress = load_int_value("progress:", &chargeInProgress);
        if (err_chargeInProgress != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS progress", esp_err_to_name(err_chargeInProgress));
#endif
        }

        esp_err_t err_isConfigState = load_int_value("isCState:", &isConfigState);
        if (err_isConfigState != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS isConfigState", esp_err_to_name(err_chargeInProgress));
#endif
        }

        esp_err_t err_maximumCurrentSetting = load_int_value("mcs:", &maximumCurrentSetting);
        if (err_maximumCurrentSetting != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS maximumCurrentSetting", esp_err_to_name(err_maximumCurrentSetting));
#endif
        }

        esp_err_t err_led_off_command_remote = load_int_value("led:", &led_off_command_remote);
        if (err_led_off_command_remote != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS led_off_command_remote", esp_err_to_name(err_led_off_command_remote));
#endif
        }

        esp_err_t err_scheduleChargeStart = load_int_value("scs:", &scheduleChargeStart);
        if (err_scheduleChargeStart != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
        }

        esp_err_t err_userId = load_key_value("userId:", userId, sizeof(userId));

        if (err_userId != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS err_userId", esp_err_to_name(err_userId));
#endif
        }

        // checkFileHasUnfinishedTransaction = false;
#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==POWERON_STATE/ ");
#endif
        CURRENT_STATE = IDLE_STATE;
        // isPaired=0;
    }
    else if (CURRENT_STATE == IDLE_STATE)
    {
        // wifi_config_state = 1;
        /*esp_err_t err_wifi_config_state = load_int_value("wcstate:", &wifi_config_state);

        if (err_wifi_config_state != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS err_wifi_config_state ", esp_err_to_name(err_wifi_config_state));
#endif
        }*/
        /* if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)

         {*/
        if (isPaired == 0 && is_internet_available == true && factoryResetOngoing == 0)
        {
            isPaired = 1;
            // xSemaphoreGive(readStateMutex);
            esp_err_t err_isPaired = save_int_value("isPaired:", isPaired);

            if (err_isPaired != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) saving to NVS isPaired", esp_err_to_name(err_isPaired));
#endif
            }
            factoryResetHTTP();
        }
        //}
        /*else
        {
            xSemaphoreGive(readStateMutex);
        }*/

#ifdef DEBUG
        ESP_LOGW(TAG, "isPaired:%d", isPaired);
        ESP_LOGW(TAG, "transaction_id:%u", transaction_id_value);

#endif
        /*  esp_err_t err_is_charger_locked = load_int_value("isLocked:", &isChargerLocked);
          if (err_is_charger_locked != ESP_OK)
          {
  #ifdef DEBUG
              ESP_LOGE(TAG, "Error (%s) loading to NVS isLocked", esp_err_to_name(err_is_charger_locked));
  #endif
          }*/
#ifdef DEBUG
        ESP_LOGW(TAG, "rebootTime:%d", rebootTime);
#endif

        /*if (isChargerLocked == false)
        {
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                currentLedState = blinkBlue;
                xSemaphoreGive(readStateMutex);
            }
        }
        else
        {
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                currentLedState = blinkYellow;
                xSemaphoreGive(readStateMutex);
            }
        }*/

        /*  if (toggle_pin == 0)
          {
              gpio_set_level(ledTogglePin, 1);
              toggle_pin = 1;
          }
          else if (toggle_pin == 1)
          {
              gpio_set_level(ledTogglePin, 0);
              toggle_pin = 0;
          }*/

#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==IDLE_STATE");
#endif
        counterMQTT++;
        if (counterMQTT > 2)
        {
            if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            {
                counterMQTT = 0;
                if (is_internet_available == true)
                {
                    xSemaphoreGive(internetMutex);
                    sendHeartBeatMQTT();
                }
                else
                {
                    xSemaphoreGive(internetMutex);
                }
            }
        }
        ESP_LOGI("msm", "CURRENT_STATE==IDLE_STATE maximumCurrentSetting:%d", maximumCurrentSetting);
        if (scheduleChargeEnableState == 1 && scheduleChargeStart == 1 && chargeInProgress == 0)
        {
            // mobileStartCommand = true;
            // scheduleChargeStart = false;
            CURRENT_STATE = START_STATE;
        }

        /* esp_err_t err_chargeInProgress = load_int_value("progress:", &chargeInProgress);
         if (err_chargeInProgress != ESP_OK)
         {
 #ifdef DEBUG
             ESP_LOGE(TAG, "Error (%s) loading to NVS progress", esp_err_to_name(err_chargeInProgress));
 #endif
         }

         esp_err_t err_isConfigState = load_int_value("isCState:", &isConfigState);
         if (err_isConfigState != ESP_OK)
         {
 #ifdef DEBUG
             ESP_LOGE(TAG, "Error (%s) loading to NVS isConfigState", esp_err_to_name(err_chargeInProgress));
 #endif
         }*/

        // connectorStartCharge = 1;
        //  mobileStartCommand=1;
        // if (chargeInProgress == 1 && scheduleChargeStart == 0)today
        if (chargeInProgress == 1)
        {
            // if (isChargerLocked == 0)
            //{
            CURRENT_STATE = PRE_START_STATE;
            /*if (controlSideState == 1)
            {
                timerCountDown = COUNTDOWN;
                timerCountDownStart = true;
            }*/
            if (connectorStartCharge == 1)
            {
                xTimerStart(countDiffTimer, 0);
            }
            //}
        }
        else if (isConfigState == 1)
        {
            CURRENT_STATE = CONFIG_STATE;
        }
        else if (mobileStartCommand == true || controlSideState == 3)
        {

            if (isChargerLocked == 0 || mobileStartCommand == true)
            {
                CURRENT_STATE = PRE_START_STATE;

                ESP_LOGE("msm", "PRE_START_STATE");
                // noInternetDisplay = 0;
                if (mobileStartCommand == true && controlSideState == 1)
                {
                    timerCountDown = COUNTDOWN;
                    timerCountDownStart = true;
                }

                // ESP_LOGI("msm", "mobileStartCommand:false");
                mobileStartCommand = false;
            }
        }
        // getMACAddress();
        //  uint8_t *macAddress[12]=(uint8_t*)90380CB047E4;
        //  bool mac =check_topic_mac(macAddress);
    }
    else if (CURRENT_STATE == CONFIG_STATE)
    {
#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==CONFIG_STATE");
#endif

        /* if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
         {
             currentLedState = sweepYellow;
             xSemaphoreGive(readStateMutex);
         }*/

// configure the charger
#ifdef DEBUG
        ESP_LOGW("config_State", "configuring the charger");
#endif
        CURRENT_STATE = IDLE_STATE;
        isConfigState = 0;
        esp_err_t err_isConfigState = save_int_value("isCState:", isConfigState);
        if (err_isConfigState != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG_B, "Error (%s) saving to NVS isConfigState", esp_err_to_name(err_isConfigState));
#endif
        }
    }

    else if (CURRENT_STATE == PRE_START_STATE)
    {
        /* if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
         {
             currentLedState = blinkMagenta;
             xSemaphoreGive(readStateMutex);
         }*/

        counterMQTT++;
        if (counterMQTT > 2)
        {
            counterMQTT = 0;
            if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            {
                if (is_internet_available == true)
                {
                    xSemaphoreGive(internetMutex);
                    sendHeartBeatMQTT();
                }
                else
                {
                    xSemaphoreGive(internetMutex);
                }
            }
        }
        if (connectorStartCharge == 1)
        {
            CURRENT_STATE = START_STATE;
            timerCountDownStart = false;
            connectorStartCharge = 0; // added
            xTimerStart(countDiffTimer, 0);

#ifdef DEBUG
            ESP_LOGI("msm", "PRE_START_STATE:START_STATE:%d ", connectorStartCharge);
#endif
            //   sendHeartBeatMQTT();
        }
        else
        {

#ifdef DEBUG
            ESP_LOGI("msm", "PRE_START_STATE:timerCountDown:%d ", timerCountDown);
#endif

            if (timerCountDown < 0)
            {
                timerCountDownStart = false;
#ifdef DEBUG
                ESP_LOGI("msm", "PRE_START_STATE:timerCountDown: displayStr(TimeOut)");
#endif
                // displayStr("TimeOut");
                stopFromTimeout = 1;
                //  connectorStopCharge = 1;
                CURRENT_STATE = STOP_STATE;
            }
            else if (timerCountDown == 0 && controlSideState == 1)
            {
                CURRENT_STATE = STOP_STATE;
                timerCountDownStart = false;
                stopFromTimeout = 1;
            }
            else if (controlSideState == 3)
            {
                timerCountDownStart = false;
                timerCountDown = 0;
            }
        }

        if (connectorStopCharge == 1)
        {
            CURRENT_STATE = STOP_STATE;
            ESP_LOGE("msm", "connectorStopCharge:%d", connectorStopCharge);
            timerCountDownStart = false;
        }
    }
    else if (CURRENT_STATE == START_STATE)
    {

#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==START_STATE %d", timeDiffTask1);
#endif
        /* if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
         {
             currentLedState = sweepGreen;
             xSemaphoreGive(readStateMutex);
         }*/
        if (chargeInProgress == 0)
        {
            // getLocalTime(&timeinfo);
#ifdef DEBUG
            ESP_LOGW(TAG, "TimerStart( countDiffTimer, 0 )");
            ESP_LOGW(TAG, "TimerStart( countDiffTimer, 0 )");
            ESP_LOGW(TAG, "TimerStart( countDiffTimer, 0 )");
#endif
            xTimerStart(countDiffTimer, 0);

            if (xSemaphoreTake(timerMutex, 0) == pdTRUE)
            {
                timeDiffTask1 = timeDiff;
                xSemaphoreGive(timerMutex);
            }
            chargeInProgress = 1;
            esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

            if (err_chargeInProgress != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_chargeInProgress));
#endif
            }

            esp_err_t err_scheduleChargeStart = save_int_value("scs:", scheduleChargeStart);

            if (err_scheduleChargeStart != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
            }
            startTransactionHTTP();

            timerCountDownStart = false;
        }
        /*if (xSemaphoreTake(timerMutex, 0) == pdTRUE)
        {
            timeDiff = timeDiffTask1;
            xSemaphoreGive(timerMutex);
        }*/

        /*esp_err_t err_maximumCurrentSetting = load_int_value("mcs:", &maximumCurrentSetting);
        if (err_maximumCurrentSetting != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_maximumCurrentSetting));
#endif
        }*/

        if ((loadBalancingEnableState == 0 && sendSerial.messegeID == 0) && configureChargerEnabledFlag == 0)
        {
            sendSerial.val1.bytes.LB = maximumCurrentSetting;
        }
        else if ((sendSerial.messegeID == 0 && loadBalancingEnableState == 1) || configureChargerEnabledFlag == 1)
        {
            sendSerial.val1.bytes.LB = loadBalancingCurrentSetting;
        }
        ESP_LOGW(TAG, "transaction_id:%u", transaction_id_value);
        counterMQTT++;
        if (counterMQTT > 2)
        {
            counterMQTT = 0;
            if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            {
                if (is_internet_available == true)
                {
                    xSemaphoreGive(internetMutex);
                    sendHeartBeatMQTT();
                }
                else
                {
                    xSemaphoreGive(internetMutex);
                }
            }
        }

        if (gpio_get_level(buttonInput) == 0)
         {
             printf("Button is LOW\n");
             stopCharge = 1;
         }
         else
         {
             printf("Button is HIGH\n");
         }

        // connectorStopCharge=1;
        if (connectorStopCharge == 1)
        {
            CURRENT_STATE = STOP_STATE;
            ESP_LOGE("msm", "CURRENT_STATE==STOP_STATE:connectorStopCharge = 1");
        }
        else if (controlSideState == 1)
        {
            CURRENT_STATE = STOP_STATE;
            ESP_LOGE("msm", "CURRENT_STATE==STOP_STATE:controlSideState = 1");
        }
    }
    else if (CURRENT_STATE == STOP_STATE)
    {
#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==STOP_STATE");
#endif
        if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
        {
            mode = 1;
            strcpy(userId, "0");
            xSemaphoreGive(readStateMutex);

            esp_err_t err_userId = save_key_value("userId:", userId);
            if (err_userId != ESP_OK)
            {
                ESP_LOGE(TAG, "Error (%s) saving to NVS err_userId", esp_err_to_name(err_userId));
            }
#ifdef DEBUG
            ESP_LOGI("msm", "userId:%s", userId);
#endif
        }
        else
        {
            xSemaphoreGive(readStateMutex);
        }

        if (stopFromTimeout == 1)
        { // ongoing charge
          // stopedFlag = 1;
            stopFromTimeout = 0;
            if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            {
                if (is_internet_available == true)
                {
                    xSemaphoreGive(internetMutex);
                    sendHeartBeatMQTT();
                }
                else
                {
                    xSemaphoreGive(internetMutex);
                }
            }
            stopTransactionHTTP();
            CURRENT_STATE = IDLE_STATE;
            scheduleChargeStart = 0;
            esp_err_t err_scheduleChargeStart = save_int_value("scs:", scheduleChargeStart);

            if (err_scheduleChargeStart != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
            }
            chargeInProgress = 0;
            esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

            if (err_chargeInProgress != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_chargeInProgress));
#endif
            }
            if (xSemaphoreTake(timerMutex, 1) == pdTRUE)
            {
                xTimerStop(countDiffTimer, 0);
                xSemaphoreGive(timerMutex);
            }

            // connectorStopCharge = 0;
        }
        else
        {
            if (connectorStopCharge == 1)
            {
                // stopedFlag = 1;
                connectorStopCharge = 0; // added
                scheduleChargeStart = 0;
                esp_err_t err_scheduleChargeStart = save_int_value("scs:", scheduleChargeStart);

                if (err_scheduleChargeStart != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
                }
                chargeInProgress = 0;
                esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

                if (err_chargeInProgress != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_chargeInProgress));
#endif
                }
                if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
                {
                    if (is_internet_available == true)
                    {
                        xSemaphoreGive(internetMutex);
                        sendHeartBeatMQTT();
                    }
                    else
                    {
                        xSemaphoreGive(internetMutex);
                    }
                }
                stopTransactionHTTP();
                CURRENT_STATE = IDLE_STATE;

                if (xSemaphoreTake(timerMutex, 1) == pdTRUE)
                {
                    xTimerStop(countDiffTimer, 0);
                    xSemaphoreGive(timerMutex);
                }
                stopCharge = 0;
            }
            else if (chargingActive == 0)
            {
                scheduleChargeStart = 0;
                esp_err_t err_scheduleChargeStart = save_int_value("scs:", scheduleChargeStart);

                if (err_scheduleChargeStart != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
                }
                chargeInProgress = 0; // newly added
                esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

                if (err_chargeInProgress != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_chargeInProgress));
#endif
                }
                stopTransactionHTTP();
                CURRENT_STATE = IDLE_STATE;
            }
            else
            {
                scheduleChargeStart = 0;
                esp_err_t err_scheduleChargeStart = save_int_value("scs:", scheduleChargeStart);

                if (err_scheduleChargeStart != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS err_scheduleChargeStart", esp_err_to_name(err_scheduleChargeStart));
#endif
                }
                chargeInProgress = 0; // newly added
                esp_err_t err_chargeInProgress = save_int_value("progress:", chargeInProgress);

                if (err_chargeInProgress != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_chargeInProgress));
#endif
                }
                stopTransactionHTTP();
                CURRENT_STATE = IDLE_STATE;
            }
        }
    }
    else if (CURRENT_STATE == FAULT_STATE)
    {
#ifdef DEBUG
        ESP_LOGI("msm", "CURRENT_STATE==FAULT_STATE");
#endif
        /* if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
         {
             currentLedState = ledsOnOffError;
             xSemaphoreGive(readStateMutex);
         }*/
        counterMQTT++;
        if (counterMQTT > 2)
        {
            if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            {
                counterMQTT = 0;
                if (is_internet_available == true)
                {
                    xSemaphoreGive(internetMutex);
                    sendHeartBeatMQTT();
                    ESP_LOGW("msm", "sent heartbeat mqtt");
                }
                else
                {
                    xSemaphoreGive(internetMutex);
                }
            }
        }

        timerCountDownStart = false;
        if (error_occurred_flag == false)
        {
            CURRENT_STATE = IDLE_STATE;
        }
    }
}
