
#ifndef HTTP_MQTT_WIFI_H
#define HTTP_MQTT_WIFI_H

#ifdef __cplusplus
extern "C"
{
#endif

#define DEBUG
// #define RTOSDEBUG
#include <stdio.h>
#include <string.h>

#include <cstring>
#include <cctype>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "ping.h"
#include "mbedtls/md.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "driver/uart.h"
// smart config
#include "esp_wpa2.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "lwip/dns.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "bt_inter.h"

// #include "rahal_crc.h"
#include "esp_spiffs.h"
#include "rahal_spiffs.h"

#include "esp_ota_ops.h"
#include "esp_https_ota.h"

#include "esp_task_wdt.h"

#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sntp.h"

#include "serialCom.h"

#include "components/lora/include/lora.h"

// ESP32 Pinout
#define STM32_RESET GPIO_NUM_18
#define STM32_BOOT0 GPIO_NUM_19
// #define STM32_BOOT1 GPIO_NUM_5
#define ledTogglePin GPIO_NUM_21
#define buttonInput GPIO_NUM_26
#define loraReetPin GPIO_NUM_25

#define ENABLE_POWER_MEASURING_DEVICE

// size of each data frame payload (actual frame size is bigger)
#define DATA_SIZE 512
    // size of each frame
    static int FRAME_SIZE = 3 + DATA_SIZE + 2;

    static const char *TOPIC_RECEIVE = "/topic/vega/update";
    static const char *TOPIC_FB = "/topic/vega/fb";

#define ESP_MAXIMUM_RETRY 10
#define ESP_MAXIMUM_RETRY_CONFIG 100
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
    /* FreeRTOS event group to signal when we are connected & ready to make a request */
    static EventGroupHandle_t s_wifi_event_group;
    flash_t flash;
    wifi_config_t wifi_config;
    esp_mqtt_client_handle_t mqtt_client;
    static int s_retry_num = 0;
    static const int CONNECTED_BIT = BIT0;
    // Event group
    EventGroupHandle_t wifi_event_group;
    const int CONNECTED_BIT_2 = BIT0;
    static const int ESPTOUCH_DONE_BIT = BIT1;
    static const char *TAG = "smartconfig_example";

#define CONFIG_RESOURCE "/data/2.5/forecast?id=524901&appid=25bb31dec81e9638ca08f49b03e351f4&city=maradana"
#define PING_WEBSITE_1 "www.google.com"
#define PING_WEBSITE_2 "www.apple.com"
#define CONFIG_WIFI_SSID "chargeNET" /*"VIP_ADSL"*/          /*"Vishva"*/
#define CONFIG_WIFI_PASSWORD "JB390BEAM2D" /* "0776638279"*/ /*"vishvaoshada"*/

    // WiFiClient espClient;
    // PubSubClient client(espClient);

    bool TimeBased = 1;
    bool sendMqtt = 0;
    uint8_t isPaired = 0;

    // IPAddress ping_host(142, 250, 4, 100);
    // const char* ping_host = "www.google.com";
    bool startResponseReceved = false;
    int eepromStat = 0;
    char userRefID[25] = "user.000000001";
#define NODEID_LEN 6
    char NODEID[NODEID_LEN] = "C0090";
    char NODE[NODEID_LEN] = "C0090";
    char mqtt_pub_topic[13 + 3];
    // 35.164.33.24

    // const char* startUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/StartTransaction";
    // const char* stopUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/StopTransaction";
    // const char* chargerAmountUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/ChargerReset";
    // const char* energyRateUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/GetChargePointDataByReference";
    // const char* energyBasedStopUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/EnergyBasedStopTransaction";
    // const char* stopPrevUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/StopTransaction";
    // const char* deepDiveUrl = "http://35.164.33.24:13000/HeartbeatHandler.asmx/RecordHeartbeat";
    // const char* getPinUrl = "http://35.164.33.24:13000/ChargerHandler.asmx/IsChargePointBusy";

    int POWERON_STATE = 1;
    int IDLE_STATE = 2;
    int CONFIG_STATE = 3;
    int PRE_START_STATE = 4;
    int START_STATE = 5;
    int STOP_STATE = 6;
    int CURRENT_STATE = 0;
    int PREVIOUS_STATE = 0;
    int FAULT_STATE = 7;

// number of frames per stream
#define STREAM_SIZE 1

// file size to save into spiffs
#define SPIFFS_BUFF_SIZE 512
#define UART_BUF_SIZE 1024

#define MS_DELAY 10

// CRC constants
#define CRC_POL 50585
#define CRC_POL_BIT 15

    static const char *TAG_UART = "UART";
    static const char *TAG_MQTT = "MQTT_TCP";
    static const char *TAG_GPIO = "GPIO";

    static const char *BROKER_URL = "mqtt://broker.hivemq.com";

    static bool receiving_update = false;
    static bool flashing_update = false;
    static uint32_t total_packets = 0;
    static uint32_t received_packets = 0;

    static uint32_t saved_bytes = 0; // number of bytes saved into spiffs buffer
    static uint8_t spiffs_buff[SPIFFS_BUFF_SIZE + 5] = {0};
    static int file_count = 0;
    static int checksum = 0;
    static int file_type = 0;

    static char tx_data = 0;
    static char rx_data = 0;

    static const int uart_num = UART_NUM_2;
    esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true};
    static uint32_t base_address = 0x08000000;

    /* const char *startUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/StartTransaction";
     const char *stopUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/StopTransaction";
     const char *chargerAmountUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/ChargerReset";
     const char *energyRateUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/GetChargePointDataByReference";
     const char *energyBasedStopUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/EnergyBasedStopTransaction";
     const char *stopPrevUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/StopTransaction";
     const char *deepDiveUrl = "http://api.chargenet.lk:13000/HeartbeatHandler.asmx/RecordHeartbeat";
     const char *getPinUrl = "http://api.chargenet.lk:13000/ChargerHandler.asmx/IsChargePointBusy";
     const char *getEnergyRateUrl = "https://api.chargenet.lk:13100/ChargerHandler.asmx/GetChargePointDataByReference";*/

    extern const uint8_t server_cert_2_pem_start[] asm("_binary_ca_cert_2_pem_start");
    extern const uint8_t server_cert_2_pem_end[] asm("_binary_ca_cert_2_pem_end");

    const char *startUrl = "https://api.chargenet.lk:13100/HomeChargerController.asmx/StartChargerTransaction";
    const char *stopUrl = "https://api.chargenet.lk:13100/HomeChargerController.asmx/StopChargerTransaction";
    const char *factoryResetUrl = "https://api.chargenet.lk:13100/HomeChargerController.asmx/ChangeChargerPairState";

    int chargerPIN = 0;
    bool jsonOutReady = false;
    unsigned long lastTime = 0;
    unsigned long lastTime1 = 0;

    unsigned long timerDelay = 1000;
    unsigned long timerDelay1 = 1000;

    char heartBeatObj[750] = ""; // was 600 today

    int sentHbCount = 0;

    unsigned long previousMIllis_1 = 0UL;
    unsigned long previousMIllis_2 = 0UL;
    unsigned long previousMIllis_3 = 0UL;
    const char *broker = "ec2-52-89-241-125.us-west-2.compute.amazonaws.com";
    const char *brokerUser = "";
    const char *brokerPass = "";

    bool getChargerDetails_HTTP_READY = 0;
    bool getPIN_HTTP_READY = 0;
    // String startMQTT = "";

    char userId[20] = "0";
    char MQTTCommand[22];
    char scheduleStartTimeWeekDays[] = "00:00";
    char scheduleStopTimeWeekDays[] = "00:00";
    char scheduleStartTimeWeekEnds[] = "00:00";
    char scheduleStopTimeWeekEnds[] = "00:00";

    uint8_t alarmUpdateCompleteFlag = 0;
    uint8_t UpdateAlarmSerialFlag = 1;
    uint8_t maximumCurrentSetting = 0;
    uint8_t maximumCurrentRequest = 0;
    float loadBalancingCurrentSetting = 0;
    float I_lora = 0;

    float currentRemaining = 0;
    uint8_t ecoPlusModeEnabled = 0;
    uint8_t countAttempts = 0;
    uint8_t devicePairedStatus = 0;
    uint8_t factoryResetOngoing = 0;

    uint8_t mode = 1;

    uint16_t remainingPower = 0;

    int countDiff = 0;
    int ticker_count = 0;
    bool sendHeartbeatReceved = false;

    // int doubleCharge = 1;
    int chargeHourlyAmount_IN_NetWork = 125;

    int wifiAttemptCounter = 0;

    char startStatus[10];
    char message_value[10];
    char started[10];
    char userReference[10] = "";
    char chargePointReference[10] = "";
    char username[10] = "";
    // char userID[10] = "";
    bool success;
    char message[10] = "";
    char errorMessage[10] = "";
    char transactionID[17];
    char transactionIDchrEmpty[17] = "";
    // char transactionIDPrev = "";
    int authFailErrorCode = 0;

    // float timeDiff;

#define READFILE_OK 1
#define HTTP_CODE_OK 200
    float energyRate = 0.0;
    // float accountBalance = 0.0;
    bool isStatusEqualToStop = false;
    // float timeWhenStopping = 0.0;
    // float energyWhenStopping = 0.0;

    float energySendPrev = 0;
    float timeDiffPrev = 0;
    float costCalcPrev = 0;
    char userReferenceIDPrev[21] = "";
    char transactionIDchrPrev[17];

    int wicount = 0;

    unsigned long debugTime = 0;
    unsigned long lastTime2 = 0;
    bool connectorBootDetect = false;
    bool GFI_flag = false;
    bool stopCondition = false;
    volatile bool startChargeTimer = false;
    uint8_t chargeInProgress = 0;
    // String authFailErroCode = "";
    bool stopButton = false;
    // bool internet_availability = false;
    unsigned long noInternetTxttimeout = 0;
    bool timerCountDownStart = false;
    int timerCountDown = 0;
    int COUNTDOWN = 50;
    int readFile_2State = 0;
    bool DEFINE_STANDALONE = true;
    float saved_last_time = 0;
    // String tempUserId = "";
    bool checkInitialFileRead = false;
    int connectorStartCharge = 0;
    int controlSideState = 0;
    int connectorStopCharge = 0;
    int chargingActive = 0;

    uint8_t rtc_UpdateComplete = 0;
    uint8_t rtc_UpdateAlarmComplete = 0;

    int stopCharge = 0;
    // int saved_flag = 1;
    bool timerCarStopOnOff = false;

    bool cardDetected = false;
    bool mobileStartCommand = false;
    float timeDiff = 0.0;
    // float costCalc = 0.0;
    //  float balance = 0;
    float balanceCalc = 0.0;
    float energySend = 0.0;
    float energyCalc = 0.0;
    // float saved_energySend = 0;

    int timeDiffTask1 = 0;
    int startConfigState = 0;
    bool configFlag1 = 0;
    bool configFlag2 = 0;
    bool configFlag3 = 0;
    bool configFlag4 = 0;

    uint8_t scheduleChargeEnableState = 0;
    uint8_t loadBalancingEnableState = 0;
    uint8_t scheduleChargeStart = 0;

    bool ota_enabled_flag = false;

    unsigned long cofigDelay1 = 0;
    unsigned long cofigDelay2 = 0;
    unsigned long cofigDelay3 = 0;
    unsigned long cofigDelay4 = 0;
    unsigned long deepDiveDelay = 0;
    unsigned long pingDelay = 0;
    unsigned long heartBeatDelay = 0;
    unsigned long mqttDelay = 0;
    unsigned long mqtt_deepDiveDelay = 0;
    unsigned long mqtt_fullDeepDiveDelay = 0;
    unsigned long timeStamp = 0;
    unsigned long chargeOngoingflag = 0;
    unsigned long tapCardFlag = 0;
    unsigned long serialTime = 0;
    unsigned long nowTime = 0;
    unsigned long debugDisplayTimer = 0;
    int wifiOk = 0;
    bool http_start_transaction_err = false;
    // int netOk = 0;
    int NFCOk = 0;
    int ntpOkFlag = 0;
    int EEpromOk = 0;
    uint8_t mac_base[6] = {0};
    char mac_address[6] = {0};
    uint8_t macAddress[6];
    char string_address2[13];
    char string_address[15];
    char mac_address_base[13];

    float voltageL1 = 0;
    float voltageL2 = 0;
    float voltageL3 = 0;

    float currentL1 = 0;
    float currentLive1 = 0;

    float currentL2 = 0;
    float currentLive2 = 0;

    float currentL3 = 0;
    float currentLive3 = 0;

    float power = 0;
    float energy = 0;

    uint8_t firmwareVersion = 0;

    int temp1 = 0;

    char *app_version = "";

    uint8_t led_off_command = 0;
    uint8_t led_off_command_remote = 0;
    uint8_t wifi_config_state = 0;

    char strftime_buf[20];
    uint8_t time_setup_success_flag = 0;

    uint32_t transaction_id_value = 0;

    char ipAddress[15];

    typedef enum
    {
        sweepGreen = 0,
        sweepYellow,
        sweepBlue,
        sweepCyan,
        blinkMagenta,
        blinkBlue,
        blinkYellow,
        blinkRed,
        ledsOnOffError
    } curr_led_state;

    volatile curr_led_state currentLedState = sweepGreen;

    // bool pingST = 0;
    //  bool onlineCore0 = false;
    bool connectorRegisterDetect;
    QueueHandle_t checkInternetQueue;
    QueueHandle_t HTTPCodeQueue;
    QueueHandle_t stopTransactionQueue;
    QueueHandle_t checkInternetQueueEnergy;
    QueueHandle_t HTTPCodeQueueEnergy;
    QueueHandle_t stopTransactionQueueEnergy;
    QueueHandle_t checkInternetQueuePrev;
    QueueHandle_t HTTPCodeQueuePrev;
    QueueHandle_t stopTransactionQueuePrev;
    QueueHandle_t displayQueue;
    QueueHandle_t displayQueueStruct;

    TimerHandle_t countDiffTimer;

    TaskHandle_t Task_net = 0;
    TaskHandle_t Task_main;
    TaskHandle_t Task_internet;
    TaskHandle_t loopTs = 0;
    TaskHandle_t DisplayTs = 0;
    TaskHandle_t NEOTs = 0;
    TaskHandle_t LOGOTs = 0;

    TaskHandle_t Task_rfid;

    SemaphoreHandle_t timerMutex;
    SemaphoreHandle_t mqttMutex;
    SemaphoreHandle_t displayMutex;
    SemaphoreHandle_t onlineCore0Mutex;
    SemaphoreHandle_t readStateMutex;

    static void smartconfig_example_task(void *parm);

    static void obtain_time(void);
    static void initialize_sntp(void);
    void factoryResetHTTP();

    void time_sync_notification_cb(struct timeval *tv)
    {
        ESP_LOGI(TAG, "Notification of a time synchronization event");
    }

    static void obtain_time(void)
    {
        initialize_sntp();

        // wait for time to be set
        time_t now = 0;
        struct tm timeinfo = {0};
        int retry = 0;
        const int retry_count = 50;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
        {
            ESP_LOGE(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    static void initialize_sntp(void)
    {
        ESP_LOGI(TAG, "Initializing SNTP");
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
        sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
        sntp_init();
    }

    void getTime()
    {
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        // Is time set? If not, tm_year will be (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900))
        {
            ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time();
            // update 'now' variable with current time
            time(&now);
        }

        // Set timezone to Sri Lanka Standard Time
        setenv("TZ", "UTC-5:30", 1);
        tzset();
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%Y.%m.%dT%H:%M:%S", &timeinfo);
        ESP_LOGE(TAG, "The current time in srilanka is: %s", strftime_buf);
        time_setup_success_flag = 1;
    }

    // function for checking CRC of received data packet
    static bool check_crc(const int arr_length, uint8_t *message_data)
    {
        uint8_t data[1024] = {0};

        for (int i = 0; i < arr_length; i++)
            data[i] = message_data[i];

        // compute received data CRC
        uint16_t received_crc = (data[arr_length - 2] << 8) + data[arr_length - 1];

        // remove received CRC for new calculation
        data[arr_length - 2] = 0;
        data[arr_length - 1] = 0;

        // create CRC mask based on CRC_POL
        uint8_t crc_arr[arr_length];
        for (int i = 0; i < arr_length; i++)
            crc_arr[i] = 0;
        crc_arr[0] = CRC_POL >> 8;
        crc_arr[1] = CRC_POL & 255;

        // compute CRC
        for (int i = arr_length * 8 - 1; i >= 16; i--)
        {
            if (((data[arr_length - 1 - (i >> 3)] >> (i & 7)) & 1) == 1)
            {
                for (int j = 0; j < arr_length; j++)
                    data[j] = data[j] ^ crc_arr[j];
            }
            for (int j = arr_length - 1; j > 0; j--)
                crc_arr[j] = (crc_arr[j] >> 1) | ((crc_arr[j - 1] & 1) << 7);
            crc_arr[0] = crc_arr[0] >> 1;
        }
        uint16_t calc_crc = (data[arr_length - 2] << 8) + data[arr_length - 1];

        return calc_crc == received_crc;
    }
    void enter_bootmode()
    {
        while (1)
        {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            // esp_task_wdt_reset();
            tx_data = 0x7F;
            rx_data = 0;
#ifdef DEBUG
            ESP_LOGI(TAG, "Sending: %X", tx_data);
#endif

            uart_flush_input(uart_num);
            uart_write_bytes(uart_num, (const char *)&tx_data, 1);
            uart_read_bytes(uart_num, &rx_data, 1, 1000 / portTICK_RATE_MS);
#ifdef DEBUG
            ESP_LOGI(TAG, "Received: %X", rx_data);
#endif

            if (rx_data == 0x79)
            {
#ifdef DEBUG
                ESP_LOGW(TAG, "Boot mode entered");
#endif
                break;
            }
        }
    }

    void erase_flash()
    {
#ifdef DEBUG
        ESP_LOGI(TAG, "Atempting to erase flash...");
#endif

        rx_data = 0;
        uart_flush_input(uart_num);

        tx_data = 0x43;
        uart_write_bytes(uart_num, (const char *)&tx_data, 1);
        tx_data = 0xBC;
        uart_write_bytes(uart_num, (const char *)&tx_data, 1);

        uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

        if (rx_data == 0x79)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Erasing Flash");
#endif
        }
        else
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Cannot Erase flash, No response");
#endif
        }

        rx_data = 0;
        uart_flush_input(uart_num);

        tx_data = 0xFF;
        uart_write_bytes(uart_num, (const char *)&tx_data, 1);
        tx_data = 0x00;
        uart_write_bytes(uart_num, (const char *)&tx_data, 1);

        uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

        if (rx_data == 0x79)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Flash Erase Success");
#endif
        }
        else
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Erase Failed");
#endif
        }
    }
    uint8_t address_checksum(uint32_t address)
    {
        uint8_t checksum = 0;
        checksum = checksum ^ (address & 0xFF);
        checksum = checksum ^ ((address >> 8) & 0xFF);
        checksum = checksum ^ ((address >> 16) & 0xFF);
        checksum = checksum ^ ((address >> 24) & 0xFF);
        return checksum;
    }

    void load_update()
    {
#ifdef DEBUG
        ESP_LOGI(TAG, "Loading update");
#endif

        FILE *f = fopen("/spiffs/0", "r");
        if (f == NULL)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Failed to open file for reading");
#endif
            esp_vfs_spiffs_unregister(conf.partition_label);
            return;
        }

        char file1[1024] = {0};
        fgets(file1, sizeof(file1), f);
        fclose(f);

        uint32_t n_bytes = (file1[4] << 16) + (file1[5] << 8) + file1[6];
        uint32_t file_size = (n_bytes - 1) / SPIFFS_BUFF_SIZE + 1;
        uint32_t rem_bytes = n_bytes;
        uint32_t sent_bytes = 0;
        uint32_t pack_size = 0;
        uint32_t checksum = 0;
        uint32_t address = 0;
#ifdef DEBUG
        ESP_LOGI(TAG, "No of files: '%d'", file_size);
        ESP_LOGI(TAG, "No of bytes: '%d'", n_bytes);
#endif

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // begin transmitting update

        for (int i = 1; i <= file_size; i++)
        {
            char file_name[100];
            snprintf(file_name, 100, "/spiffs/%d", i);

            f = fopen(file_name, "r");
            if (f == NULL)
            {
#ifdef DEBUG
                ESP_LOGE(TAG_SPIFFS, "Failed to open file %d for reading", i);
#endif
                return;
            }

            fread(file1, 1, SPIFFS_BUFF_SIZE + 2, f);
            fclose(f);
#ifdef DEBUG
            ESP_LOGI(TAG_SPIFFS, "Read file %d", i);
#endif

            int file_checksum = 0;

            for (int j = 0; j < SPIFFS_BUFF_SIZE; j++)
                file_checksum += file1[j];

            file_checksum = file_checksum & 0xFFFF;

            if (file_checksum != ((file1[SPIFFS_BUFF_SIZE] << 8) + file1[SPIFFS_BUFF_SIZE + 1]))
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Kernel File Checksum failure");
                ESP_LOGI(TAG, "Calculated checksum : %d", file_checksum);
                ESP_LOGI(TAG, "Read checksum : %d", (file1[SPIFFS_BUFF_SIZE] << 8) + file1[SPIFFS_BUFF_SIZE + 1]);
#endif
                return;
            }

            for (int j = 0; j < SPIFFS_BUFF_SIZE; j += 128)
            {
                vTaskDelay(MS_DELAY / portTICK_PERIOD_MS);

                address = base_address + sent_bytes;

                rx_data = 0;
                uart_flush_input(uart_num);

                tx_data = 0x31;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                ESP_LOGI(TAG, "Sent %X", tx_data);
                tx_data = 0xCE;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                ESP_LOGI(TAG, "Sent %X", tx_data);

                uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

                if (rx_data == 0x79)
                {
#ifdef DEBUG
                    ESP_LOGI(TAG, "Write command sent");
#endif
                }
                else
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Write command Failed");
#endif
                }

                rx_data = 0;
                uart_flush_input(uart_num);

                tx_data = (address >> 24) & 0xFF;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);
                tx_data = (address >> 16) & 0xFF;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);
                tx_data = (address >> 8) & 0xFF;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);
                tx_data = address & 0xFF;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);

                tx_data = address_checksum(address);
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);

                uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

                if (rx_data == 0x79)
                {
#ifdef DEBUG
                    ESP_LOGI(TAG, "Address sent");
#endif
                }
                else
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Address Failed");
#endif
                }

                rem_bytes = n_bytes - sent_bytes;

                if (rem_bytes < 128)
                    pack_size = rem_bytes;
                else
                    pack_size = 128;

                tx_data = pack_size - 1;
                rx_data = 0;
                uart_flush_input(uart_num);
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);

                checksum = tx_data;

                for (int k = 0; k < pack_size; k++)
                {
                    tx_data = file1[j + k];
                    checksum = checksum ^ tx_data;
                    uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                    // ESP_LOGI(TAG, "Sent %X", tx_data);
                }

                tx_data = checksum & 0xFF;
                uart_write_bytes(uart_num, (const char *)&tx_data, 1);
                // ESP_LOGI(TAG, "Sent %X", tx_data);

                uart_read_bytes(uart_num, &rx_data, 1, 10000 / portTICK_RATE_MS);

                if (rx_data == 0x79)
                {
#ifdef DEBUG
                    ESP_LOGI(TAG, "Data sent");
#endif
                }
                else
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Data Failed");
#endif
                }

                sent_bytes += pack_size;

                if (sent_bytes == n_bytes)
                    break;
            }
        }
#ifdef DEBUG
        ESP_LOGI(TAG, "Sent %d bytes", sent_bytes);
        ESP_LOGW(TAG, "Update Complete");
#endif
    }

    // Save data payload into spiffs buffer first
    // Once buffer is full, save into flash
    static void save(char *base_path, uint8_t *mes)
    {
        // Save message into buffer array
        for (int i = 0; i < DATA_SIZE; i++)
        {
            spiffs_buff[saved_bytes + i] = mes[i];
            checksum += mes[i];
        }
        saved_bytes += DATA_SIZE;
#ifdef DEBUG
        ESP_LOGI(TAG_SPIFFS, "Saved to buffer %d", saved_bytes);
#endif

        // If buffer is full or if all packets have been received, save data to flash memory
        if ((saved_bytes >= SPIFFS_BUFF_SIZE) | (received_packets == total_packets))
        {
            // pad remaining data of the buffer with zero
            for (int i = saved_bytes; i < SPIFFS_BUFF_SIZE; i++)
                spiffs_buff[i] = 0;

            // store checksum of file in last two bytes
            spiffs_buff[SPIFFS_BUFF_SIZE] = (char)((checksum >> 8) & 0xFF);
            spiffs_buff[SPIFFS_BUFF_SIZE + 1] = (char)(checksum & 0xFF);

            // construct file name and save to memory
            file_count++;
            char path[100];
            snprintf(path, 100, "%s%d", base_path, file_count);
            write_bytes(path, spiffs_buff, SPIFFS_BUFF_SIZE + 2);
#ifdef DEBUG
            ESP_LOGW(TAG_SPIFFS, "Written to memory %s", path);
#endif

            // reset variables
            saved_bytes = 0;
            checksum = 0;
        }
    }

    static void initialise_pins()
    {
        gpio_reset_pin(STM32_RESET);
        gpio_reset_pin(STM32_BOOT0);
        //  gpio_reset_pin(STM32_BOOT1);
        gpio_reset_pin(ledTogglePin);
        gpio_reset_pin(buttonInput);

        gpio_set_direction(STM32_RESET, GPIO_MODE_OUTPUT);
        gpio_set_direction(STM32_BOOT0, GPIO_MODE_OUTPUT);
        //   gpio_set_direction(STM32_BOOT1, GPIO_MODE_OUTPUT);
        gpio_set_direction(ledTogglePin, GPIO_MODE_OUTPUT);

        gpio_config_t io_conf;
        io_conf.pin_bit_mask = (1ULL << buttonInput);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // Disable internal pull-up
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Enable internal pull-down
        gpio_config(&io_conf);

        gpio_set_level(STM32_RESET, 0);
        gpio_set_level(STM32_BOOT0, 0);
        //   gpio_set_level(STM32_BOOT1, 1);
        gpio_set_level(ledTogglePin, 0);
        // gpio_set_level(GPIO_NUM_12, 0);
#ifdef DEBUG
        ESP_LOGI(TAG_GPIO, "Initialised GPIOs");
#endif

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    void flash_update()
    {
        // latch boot pins
        gpio_set_level(STM32_BOOT0, 1);
        // gpio_set_level(STM32_BOOT1, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ota_enabled_flag = true;
        // toggle reset
        gpio_set_level(STM32_RESET, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(STM32_RESET, 0);
#ifdef DEBUG
        ESP_LOGI(TAG_GPIO, "Reset STM32 into bootmode");
#endif

        enter_bootmode();

        erase_flash();

        load_update();

        // release latched pins
        gpio_set_level(STM32_BOOT0, 0);
        //  gpio_set_level(STM32_BOOT1, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ota_enabled_flag = false;
        // toggle reset
        gpio_set_level(STM32_RESET, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(STM32_RESET, 0);
#ifdef DEBUG
        ESP_LOGI(TAG_GPIO, "Reset STM32");
#endif
    }
    static void process_message(uint8_t *data, esp_mqtt_client_handle_t client)
    {
        // identify if update sequence
        if ((data[0] & 128) == 128)
        {
            // remove 1 from first bit
            data[0] = data[0] - 128;

            // receiving initial frame of an update
            if ((data[0] == 0) & (data[1] == 0) & (data[2] == 0) & !receiving_update)
            {
                receiving_update = true;
                received_packets = 0;

                // get total number of packets
                total_packets = (data[3] << 16) + (data[4] << 8) + data[5];

                // get file type
                file_type = data[6];

                // write data containing no of packets, file type and no of bytes to spiffs
                uint8_t mes[DATA_SIZE] = {0};
                for (int i = 0; i < DATA_SIZE; i++)
                    mes[i] = data[i + 3];

                if (file_type == 0)
                {
                    write_bytes("/spiffs/kernel/0", mes, DATA_SIZE);
#ifdef DEBUG
                    ESP_LOGI(TAG_SPIFFS, "Expecting kernel file");
#endif
                }
                else if (file_type == 1)
                {
                    write_bytes("/spiffs/0", mes, DATA_SIZE);
#ifdef DEBUG
                    ESP_LOGI(TAG_SPIFFS, "Expecting application file");
#endif
                }
                else if (file_type == 2)
                {
                    write_bytes("/spiffs/spc57/0", mes, DATA_SIZE);
#ifdef DEBUG
                    ESP_LOGI(TAG_SPIFFS, "Expecting spc570s file");
#endif
                }
                else if (file_type == 3)
                {
                    write_bytes("/spiffs/stm32/0", mes, DATA_SIZE);
#ifdef DEBUG
                    ESP_LOGI(TAG_SPIFFS, "Expecting stm32 file");
#endif
                }
                else
                {
                    write_bytes("/spiffs/c2000_can/0", mes, DATA_SIZE);
#ifdef DEBUG
                    ESP_LOGI(TAG_SPIFFS, "Expecting c2000 can file");
#endif
                }
#ifdef DEBUG
                ESP_LOGW(TAG_SPIFFS, "Expecting a total of %d packets", total_packets);
#endif
            }
            // receiving the next required dataframe of an update
            else if (((data[0] << 16) + (data[1] << 8) + data[2]) == (received_packets + 1))
            {
                received_packets++;
#ifdef DEBUG
                ESP_LOGI(TAG_SPIFFS, "Received packet %d of %d", received_packets, total_packets);
#endif

                char *base_path = "";
                if (file_type == 0)
                    base_path = "/spiffs/kernel/";
                else if (file_type == 1)
                    base_path = "/spiffs/";
                else if (file_type == 2)
                    base_path = "/spiffs/spc57/";
                else if (file_type == 3)
                    base_path = "/spiffs/stm32/";
                else
                    base_path = "/spiffs/c2000_can/";

                // last element should be 0
                uint8_t mes[DATA_SIZE + 1] = {0};
                for (int i = 0; i < DATA_SIZE; i++)
                    mes[i] = data[i + 3];

                save(base_path, mes);

                // update completed
                if (received_packets == total_packets)
                {
                    receiving_update = false;
#ifdef DEBUG
                    ESP_LOGW(TAG_SPIFFS, "Finished receiving update with a total of %d packets", received_packets);
#endif

                    if (file_type == 0)
                    {
#ifdef DEBUG
                        ESP_LOGI(TAG_SPIFFS, "Received kernel update");
#endif
                    }
                    else
                    {
#ifdef DEBUG
                        ESP_LOGI(TAG_SPIFFS, "Received application update");
#endif
                    }

                    total_packets = 0;
                    received_packets = 0;
                    saved_bytes = 0;
                    file_count = 0;
                    checksum = 0;
                    file_type = 0;
                }
            }

            // send feedback message requesting next stream of packets
            if ((received_packets > 0) & (received_packets % STREAM_SIZE == 0))
            {
                char return_mes[100];
                snprintf(return_mes, 100, "received_%d", received_packets);
                esp_mqtt_client_publish(client, TOPIC_FB, return_mes, 0, 1, 0);
#ifdef DEBUG
                ESP_LOGI(TAG_MQTT, "Sent FB requesting next stream");
#endif
            }
        }
        else if ((data[0] == 64) & !flashing_update)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Flashing Update");
#endif
            flashing_update = true;
            flash_update();
            flashing_update = false;
        }
    }
    void runningAppInfo()
    {
        const esp_partition_t *running2 = esp_ota_get_running_partition();
        esp_app_desc_t running_app_info2;
        if (esp_ota_get_partition_description(running2, &running_app_info2) == ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Running firmware version: %s", running_app_info2.version);
#endif
            vTaskDelay(pdMS_TO_TICKS(10));
            app_version = running_app_info2.version;
            // sprintf(app_version, "%s", running_app_info.version);
#ifdef DEBUG
            ESP_LOGI(TAG, "after app version: %s", running_app_info2.version);
#endif

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    static void event_handler(void *arg, esp_event_base_t event_base, /* system_event_t *system_event,*/ int32_t event_id, void *event_data)
    {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
        {
#ifdef DEBUG
            printf("SYSTEM_EVENT_STA_START\n");
#endif
            if (setup_flag == 1)
            {
                xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
            }
            else
            {
                esp_wifi_connect();
            }
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
#ifdef DEBUG
            printf("SYSTEM_EVENT_STA_DISCONNECTED\n");
#endif
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT_2);
            wifiOk = 0;
            /*esp_err_t err_wifi_config_state = load_int_value("wcstate:", &wifi_config_state);
            if (err_wifi_config_state != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_wifi_config_state));
#endif
            }
            if (wifi_config_state == 1)
            {

                if (s_retry_num < ESP_MAXIMUM_RETRY_CONFIG)
                {
                    esp_wifi_connect();
                    s_retry_num++;
#ifdef DEBUG
                    ESP_LOGI(TAG, "retry to connect to the AP");
#endif
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                else
                {
                    setup_flag = 1;
                    save_int_value("setup_flag:", setup_flag);

                    vTaskDelay(10 / portTICK_PERIOD_MS);

                    esp_restart();
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
            }
            else
            {
                if (s_retry_num < ESP_MAXIMUM_RETRY)
                {
                    esp_wifi_connect();
                    s_retry_num++;
#ifdef DEBUG
                    ESP_LOGI(TAG, "retry to connect to the AP");
#endif
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                else
                {
                    setup_flag = 1;
                    save_int_value("setup_flag:", setup_flag);

                    vTaskDelay(10 / portTICK_PERIOD_MS);

                    esp_restart();
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
            }*/
            s_retry_num++; // today
            esp_wifi_connect();
            ESP_LOGI(TAG, "retry to connect to the AP");
            if (setup_flag == 1 && (s_retry_num > ESP_MAXIMUM_RETRY_CONFIG))
            {
                esp_restart();

            } // today
            vTaskDelay(500 / portTICK_PERIOD_MS);
            // xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
        {
#ifdef DEBUG
            printf("SYSTEM_EVENT_STA_CONNECTED\n");
#endif
            wifiOk = 1;
            if (setup_flag == 1)
            {
                strcpy(flash.SSID_key, "SSID:");
                strcpy(flash.password_key, "PASSWORD:");
                strcpy(flash.SSID, STA_SSID);
                strcpy(flash.password, STA_PASSWORD);

                esp_err_t err_ssid = save_key_value(flash.SSID_key, flash.SSID);
                esp_err_t err_pw = save_key_value(flash.password_key, flash.password);

                if (err_ssid != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_ssid));
#endif
                }
                if (err_pw != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_pw));
#endif
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
                setup_flag = 0;
#ifdef DEBUG
                ESP_LOGI(TAG, "setup_flag_after = %d", setup_flag);
#endif

                esp_err_t err_setup = save_int_value("setup_flag:", setup_flag);

                if (err_setup != ESP_OK)
                {
#ifdef DEBUG
                    ESP_LOGE(TAG, "Error (%s) saving to NVS setup_Flag", esp_err_to_name(err_setup));
#endif
                }
            }
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_SUCCESS)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_FAILED)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_TIMEOUT)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_PIN)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_PROBEREQRECVED)
        {
#ifdef DEBUG
            printf("other wifi event %d\n", event_id);
#endif
        }
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
        {
#ifdef DEBUG
            printf("SYSTEM_EVENT_STA_GOT_IP\n");
#endif
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT_2);
        }
        else if (event_base == IP_EVENT)
        {
#ifdef DEBUG
            printf("other ip event %d\n", event_id);
#endif
        }

        else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
        {
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT_2);
#ifdef DEBUG
            ESP_LOGI(TAG, "now Scan done ");
#endif
            if (xSemaphoreTake(readStateMutex, 1) == pdTRUE)
            {
                currentLedState = sweepCyan;
                xSemaphoreGive(readStateMutex);
            }
        }
        else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Found channel");
#endif
        }
        else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
        {
#ifdef DEBUG
            ESP_LOGI(TAG, "Got SSID and password");
#endif

            smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;

            // uint8_t ssid[33] = {0};
            // uint8_t password[65] = {0};
            uint8_t rvd_data[33] = {0};

            bzero(&wifi_config, sizeof(wifi_config_t));
            memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
            wifi_config.sta.bssid_set = evt->bssid_set;
            if (wifi_config.sta.bssid_set == true)
            {
                memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
            }
            memcpy(STA_SSID, evt->ssid, sizeof(evt->ssid));
            memcpy(STA_PASSWORD, evt->password, sizeof(evt->password));
#ifdef DEBUG
            ESP_LOGI(TAG, "SSID:%s", STA_SSID);
            ESP_LOGI(TAG, "PASSWORD:%s", STA_PASSWORD);
#endif

            /*if (strlen(STA_SSID) > 0)
            {
                setup_flag = 0;
            }

            strcpy(flash.SSID_key, "SSID:");
            strcpy(flash.password_key, "PASSWORD:");
            strcpy(flash.SSID, STA_SSID);
            strcpy(flash.password, STA_PASSWORD);

            esp_err_t err_ssid = save_key_value(flash.SSID_key, flash.SSID);
            esp_err_t err_pw = save_key_value(flash.password_key, flash.password);

            if (err_ssid != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_ssid));
#endif
            }
            if (err_pw != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_pw));
#endif
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
#ifdef DEBUG
            ESP_LOGI(TAG, "setup_flag_after = %d", setup_flag);
#endif

            esp_err_t err_setup = save_int_value("setup_flag:", setup_flag);

            if (err_setup != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_setup));
#endif
            }*/

            vTaskDelay(10 / portTICK_PERIOD_MS);

            if (evt->type == SC_TYPE_ESPTOUCH_V2)
            {
                ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
#ifdef DEBUG
                ESP_LOGI(TAG, "RVD_DATA:");
#endif
                for (int i = 0; i < 33; i++)
                {
#ifdef DEBUG
                    printf("%02x ", rvd_data[i]);
#endif
                }
#ifdef DEBUG
                printf("\n");
#endif
            }

            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
            esp_wifi_connect();
        }
        else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
        {
            xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
        }
    }

    void init_smart_config_setup(void)

    {
        setup_flag = 1;
        isConfigState = 1;
        ESP_ERROR_CHECK(nvs_flash_init());
        esp_err_t err_setup = load_int_value("setup_flag:", &setup_flag);
        if (err_setup != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_setup));
#endif
        }

        esp_err_t err_config_state = load_int_value("isConfigState:", &isConfigState);
        if (err_config_state != ESP_OK)
        {
#ifdef DEBUG
            ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_config_state));
#endif
        }
        // init_smart_config_btn();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    static void initialise_wifi(void)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        s_wifi_event_group = xEventGroupCreate();
        wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        assert(sta_netif);
#ifdef DEBUG
        ESP_LOGI(TAG, "setup_flag_check = %d", setup_flag);
#endif

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        if (setup_flag == 1)
        {
            ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
            ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
            ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_start());
        }
        if (setup_flag != 1)
        {

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

            strcpy(flash.SSID_key, "SSID:");
            strcpy(flash.password_key, "PASSWORD:");

            esp_err_t err_ssid = load_key_value(flash.SSID_key, STA_SSID, sizeof(STA_SSID));
            esp_err_t err_pw = load_key_value(flash.password_key, STA_PASSWORD, sizeof(STA_PASSWORD));

            if (err_ssid != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_ssid));
#endif
            }

            if (err_pw != ESP_OK)
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_pw));
#endif
            }

            wifi_config_t wifi_config = {
                .sta = {
                    .ssid = {0},
                    .password = {0},
                    .threshold = {
                        .authmode = WIFI_AUTH_WPA2_PSK,
                    },
                    .pmf_cfg = {.capable = true, .required = false},
                },
            };
#ifdef DEBUG
            ESP_LOGI(TAG, "SSID:%s", STA_SSID);
            ESP_LOGI(TAG, "PASSWORD:%s", STA_PASSWORD);
#endif
            memcpy(wifi_config.sta.ssid, STA_SSID, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, STA_PASSWORD, sizeof(wifi_config.sta.password));

            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
            ESP_ERROR_CHECK(esp_wifi_start());

            EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                   pdFALSE,
                                                   pdFALSE,
                                                   portMAX_DELAY);

            if (bits & WIFI_CONNECTED_BIT)
            {
#ifdef DEBUG
                ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                         STA_SSID, STA_PASSWORD);
#endif
            }
            else if (bits & WIFI_FAIL_BIT)
            {
#ifdef DEBUG
                ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                         STA_SSID, STA_PASSWORD);
#endif
            }
            else
            {
#ifdef DEBUG
                ESP_LOGE(TAG, "UNEXPECTED EVENT");
#endif
            }

            // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
            // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
            // vEventGroupDelete(s_wifi_event_group);
        }
    }

    static void smartconfig_example_task(void *parm)
    {
        EventBits_t uxBits;
        ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));

        while (1)
        {
            uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
            if (uxBits & CONNECTED_BIT)
            {
#ifdef DEBUG
                ESP_LOGI(TAG, "smart config done...");
#endif
            }
            if ((uxBits & ESPTOUCH_DONE_BIT) /*| (setup_flag == 0)*/)
            {
#ifdef DEBUG
                ESP_LOGI(TAG, "ESP TOUCH done....");
#endif
                esp_smartconfig_stop();
                vTaskDelete(NULL);
            }
        }
    }

    esp_err_t _http_event_handler(esp_http_client_event_t *evt)
    {
        const char *TAG = "http_event_handler";
        static char *output_buffer; // Buffer to store response of http request from event handler
        static int output_len;      // Stores number of bytes read
        switch (evt->event_id)
        {
        case HTTP_EVENT_ERROR:
#ifdef DEBUG
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
#endif
            break;
        case HTTP_EVENT_ON_CONNECTED:
#ifdef DEBUG
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
#endif
            break;
        case HTTP_EVENT_HEADER_SENT:
#ifdef DEBUG
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
#endif
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client))
            {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data)
                {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                }
                else
                {
                    if (output_buffer == NULL)
                    {
                        output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL)
                        {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL)
            {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:

            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0)
            {
                if (output_buffer != NULL)
                {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
        }
        return ESP_OK;
    }

    void shaCalculate(char *tokenCal, char *shaToken, size_t payloadLength)
    {

        uint8_t shaResult[20];
        // char buffer[60];
        mbedtls_md_context_t ctx;
        mbedtls_md_type_t md_type = MBEDTLS_MD_SHA1;

        // const size_t payloadLength = sizeof(tokenCal);
        /*char vishva[5]="C0100";
        printf(" ");
        printf("hello boys %d", sizeof(tokenCal));
        printf("\n");*/
        mbedtls_md_init(&ctx);
        mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
        mbedtls_md_starts(&ctx);
        mbedtls_md_update(&ctx, (const unsigned char *)tokenCal, payloadLength);
        mbedtls_md_finish(&ctx, shaResult);
        mbedtls_md_free(&ctx);

        for (int i = 0; i < sizeof(shaResult); i++)
        {
            char str[3];
            sprintf(str, "%02X", (int)shaResult[i]);
            /*printf(" ");
            printf("%d", (int)shaResult[i]);
            printf(" ");*/
            memmove(shaToken + 2 * i, str, 2);
            // printf(str);
        }

        printf("\n");
        // printf(buffer);
        // return shaToken;
    }

    /* void getPIN_HTTP()
     {
         const char *TAG = "getPIN_HTTP";
 #ifdef DEBUG
         printf("HTTP_GETPIN:getPIN_HTTP");
 #endif
         char sha1String[40] = {0};
         shaCalculate(NODEID, sha1String, sizeof(NODEID) - 1);

 #ifdef DEBUG
         ESP_LOGI(TAG, "%s", sha1String);
 #endif
         char local_response_buffer[110];
         esp_http_client_config_t config = {
             .url = getPinUrl,
             .query = "esp",
             .disable_auto_redirect = true,
             .event_handler = _http_event_handler,
             .user_data = local_response_buffer, // Pass address of local buffer to get response

             // .max_redirection_count=2,
             //.crt_bundle_attach = esp_crt_bundle_attach,
         };
         esp_http_client_handle_t client_get_pin = esp_http_client_init(&config);
         esp_http_client_set_url(client_get_pin, getPinUrl);
         esp_http_client_set_method(client_get_pin, HTTP_METHOD_POST);
         // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client_get_pin, "Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client_get_pin, "User-Agent", "Super Agent/0.0.1");
         // String httpRequestData = "userID=" + String(0) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
         // sprintf(str, "%02X", (int)shaResult[i]);
         // const char *httpRequestData = "userID=0&chargePointReferenceID=",NODEID,"&token=",sha1String;
         char httpRequestData[92] = "userID=0&chargePointReferenceID=*****&token=****************************************";
         // char httpRequestData[67] = "pointReference=*****&token=****************************************";
         //  printf("%s\n",httpRequestData);
         memmove(httpRequestData + 32, NODEID, 5);
         memmove(httpRequestData + 38 + NODEID_LEN, sha1String, 40);
 #ifdef DEBUG
         printf("%s\n", httpRequestData);
 #endif
         esp_http_client_set_post_field(client_get_pin, httpRequestData, strlen(httpRequestData));
         esp_err_t err = esp_http_client_perform(client_get_pin);
         if (err == ESP_OK)
         {
 #ifdef DEBUG
             ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client_get_pin), esp_http_client_get_content_length(client_get_pin));
             printf("%d\n", chargerPIN);
             printf("%s\n", local_response_buffer);
 #endif
             cJSON *get_pin_json, *mobilePin_json;
             get_pin_json = cJSON_Parse(local_response_buffer);
             if (!get_pin_json)
             {
                 printf("\nnull get_pin_json\n");
             }
             else
             {
                 mobilePin_json = cJSON_GetObjectItem(get_pin_json, "mobilePin");
                 if (!mobilePin_json)
                 {
                     printf("\nnull json\n");
                 }
                 else
                 {
                     if (mobilePin_json->valuestring != NULL)
                     {
                         chargerPIN = (int)atoi(mobilePin_json->valuestring);
 #ifdef DEBUG
                         printf("got chargerPIN:%d\n", chargerPIN);
 #endif
                         getPIN_HTTP_READY = 1;
                     }
                 }
             }

             cJSON_Delete(get_pin_json); // Free memory
         }
         else
         {
             ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
         }

         // http.end();
         esp_http_client_cleanup(client_get_pin);
     }
 */
    /* void getEnergyRateHTTP()
     {
         const char *TAG = "getEnergyRateHTTP";
 #ifdef DEBUG
         printf("ENERGY_RATE_HTTP:getEnergyRate_HTTP\n");
 #endif
         char sha1String[41];
         sha1String[41] = '\0';
         shaCalculate(NODEID, sha1String, sizeof(NODEID) - 1);

 #ifdef DEBUG
         ESP_LOGI("main_task", "%s", sha1String);
 #endif
         char local_response_buffer[600];
         esp_http_client_config_t config = {
             .url = getEnergyRateUrl,
             .query = "esp",
             .disable_auto_redirect = false,
             .max_redirection_count = 2,
             .event_handler = _http_event_handler,
             .user_data = local_response_buffer, // Pass address of local buffer to get response
             .crt_bundle_attach = esp_crt_bundle_attach,
         };
         esp_http_client_handle_t client_rate = esp_http_client_init(&config);
         esp_http_client_set_url(client_rate, getEnergyRateUrl);
         esp_http_client_set_method(client_rate, HTTP_METHOD_POST);
         // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client_rate, "Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client_rate, "User-Agent", "Super Agent/0.0.1");
         // String httpRequestData = "userID=" + String(0) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
         char httpRequestData[67] = "pointReference=*****&token=";
         // printf("%s\n",httpRequestData);
         memmove(httpRequestData + 15, NODEID, 5);
         memmove(httpRequestData + strlen(httpRequestData), sha1String, 40);
 #ifdef DEBUG
         printf("%s\n", httpRequestData);
 #endif
         esp_http_client_set_post_field(client_rate, httpRequestData, NODEID_LEN + 61);
         esp_err_t err = esp_http_client_perform(client_rate);

         if (err == ESP_OK)
         {
 #ifdef DEBUG
             ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client_rate), esp_http_client_get_content_length(client_rate));

             printf("%d\n", chargerPIN);
             printf("%s\n", local_response_buffer);
 #endif
             cJSON *get_en_json, *energy_rate_json;
             get_en_json = cJSON_Parse(local_response_buffer);
             if (!get_en_json)
             {
                 printf("\nnull get_en_json\n");
             }
             else
             {
                 energy_rate_json = cJSON_GetObjectItem(get_en_json, "energy_rate");
                 if (!energy_rate_json)
                 {
                     printf("\nnull json\n");
                 }
                 else
                 {
                     energyRate = (float)atof(energy_rate_json->valuestring);
                     printf("got engergy_rate:%f\n", energyRate);
                 }
             }
             cJSON_Delete(get_en_json); // Free memory
         }
         else
         {
             ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
         }

         // http.end();
         esp_http_client_cleanup(client_rate);
     }*/

    /*  void getChargerDetails_HTTP()
      {

          const char *TAG = "getChargerDetails_HTTP";
          jsonOutReady = 0;
  #ifdef DEBUG
          printf("ENERGY_RATE_HTTP:getChargerDetails_HTTP\n");
  #endif

          if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
          {
              if (is_internet_available)
              {
                  xSemaphoreGive(internetMutex);
                  ESP_LOGI("getChargerDetails_HTTP", "ENERGY_RATE_HTTP:Online!!");
                  char sha1String[40] = {0};
                  shaCalculate(NODEID, sha1String, sizeof(NODEID) - 1);

  #ifdef DEBUG
                  ESP_LOGI("getChargerDetails_HTTP", "%s", sha1String);
  #endif

                  char local_response_buffer[600];
                  esp_http_client_config_t config = {
                      .url = getEnergyRateUrl,
                      .query = "esp",
                      .disable_auto_redirect = false,
                      .max_redirection_count = 2,
                      .event_handler = _http_event_handler,
                      .user_data = local_response_buffer, // Pass address of local buffer to get response

                      .crt_bundle_attach = esp_crt_bundle_attach,
                  };
                  esp_http_client_handle_t client = esp_http_client_init(&config);
                  esp_http_client_set_url(client, getEnergyRateUrl);
                  esp_http_client_set_method(client, HTTP_METHOD_POST);
                  // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
                  esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
                  esp_http_client_set_header(client, "User-Agent", "Super Agent/0.0.1");
                  // String httpRequestData = "userID=" + String(0) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
                  char data[68] = "pointReference=*****&token=";
                  data[67] = '\0';
                  // printf("%s\n",data);
                  memcpy(data + 15, NODEID, 5);
                  memcpy(data + 21 + NODEID_LEN, sha1String, 40);
  #ifdef DEBUG
                  printf("%s\n", data);
  #endif
                  esp_http_client_set_post_field(client, data, strlen(data));

                  esp_err_t err = esp_http_client_perform(client);

                  if (err == ESP_OK)
                  {
                      jsonOutReady = 1;
  #ifdef DEBUG
                      ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));

                      printf("%d\n", chargerPIN);
                      printf("%s\n", local_response_buffer);
  #endif
                      cJSON *get_en_json, *energy_rate_json, *tariff_json;
                      get_en_json = cJSON_Parse(local_response_buffer);
                      if (!get_en_json)
                      {
                          printf("\nnull get_en_json\n");
                      }
                      else
                      {
                          energy_rate_json = cJSON_GetObjectItem(get_en_json, "charge_point_amount");
                          tariff_json = cJSON_GetObjectItem(get_en_json, "tariff_mode");
                          if (!energy_rate_json)
                          {
                              printf("\nnull json\n");
                          }
                          else
                          {
                              chargeHourlyAmount_IN_NetWork = (int)atoi(energy_rate_json->valuestring);
                              printf("got charge_point_amount:%d type %s\n", chargeHourlyAmount_IN_NetWork, tariff_json->valuestring);
                              getChargerDetails_HTTP_READY = 1;
                              if (strcmp("ENERGY_BASED", tariff_json->valuestring) == 0)
                              {
                                  TimeBased = false;
                              }
                              else
                              {
                                  TimeBased = true;
                              }
                          }
                      }
                      cJSON_Delete(get_en_json); // Free memory
                  }
                  else
                  {
                      ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                  }

                  // http.end();
                  esp_http_client_cleanup(client);
              }
              else
              {
                  xSemaphoreGive(internetMutex);
                  ESP_LOGI("getChargerDetails_HTTP", "ENERGY_RATE_HTTP:Offline!!");
              }
          }
      }*/

    void startTransactionHTTP()
    {

        jsonOutReady = false;
        http_start_transaction_err = false;
        const char *TAG = "startTransactionHTTP";
        const char *authKey = "013ef816d1482a5e9d0208f76abd4768625c03da";

        char tokenCalId[54]; // was 52
        strcpy(tokenCalId, string_address);
        strcat(tokenCalId, authKey);

#ifdef DEBUG
        // printf("userRefID %s tokenCalId %s strlen %d\n", userRefID, tokenCalId, strlen(userRefID));
#endif
        char sha1String[42]; // was 40
        shaCalculate(tokenCalId, sha1String, strlen(string_address) + strlen(authKey));
        sha1String[41] = '\0';

        int len = std::strlen(sha1String);
        for (int i = 0; i < len; i++)
        {
            sha1String[i] = std::toupper(sha1String[i]);
        }

#ifdef DEBUG
        ESP_LOGI(TAG, "%s", sha1String);
#endif
        char local_response_buffer[600];
        /* esp_http_client_config_t config = {
             .url = startUrl,
             .query = "esp",
             .disable_auto_redirect = false,
             .max_redirection_count = 2,
             .event_handler = _http_event_handler,
             .user_data = local_response_buffer, // Pass address of local buffer to get response

             //.crt_bundle_attach = esp_crt_bundle_attach,
         };*/
        esp_http_client_config_t config_post = {
            .url = startUrl,
            .cert_pem = (char *)server_cert_2_pem_start,
            .method = HTTP_METHOD_POST,
            .event_handler = _http_event_handler,
            .user_data = local_response_buffer, // Pass address of local buffer to get response

            //.crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t start_client = esp_http_client_init(&config_post);
        esp_http_client_set_url(start_client, startUrl);
        esp_http_client_set_method(start_client, HTTP_METHOD_POST);
        esp_http_client_set_header(start_client, "Content-Type", "application/x-www-form-urlencoded");
        esp_http_client_set_header(start_client, "User-Agent", "Super Agent/0.0.1");

        char httpRequestData[140] = "referenceNo=";
        strcat(httpRequestData, string_address);

        strcat(httpRequestData, "&userId=");
        strcat(httpRequestData, userId);
        char modeString[] = "CHARGER";
        if (mode == 1)
        {
            strcat(httpRequestData, "&mode=");
            strcat(httpRequestData, modeString);
        }
        else if (mode == 2)
        {
            strcat(httpRequestData, "&mode=");
            strcpy(modeString, "APP");
            strcat(httpRequestData, modeString);
        }
        strcat(httpRequestData, "&status=START");
        strcat(httpRequestData, "&requestToken=");
        strcat(httpRequestData, sha1String);
        // strcat(httpRequestData, "}");
        ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);

        esp_http_client_set_post_field(start_client, httpRequestData, strlen(sha1String) + strlen(modeString) + strlen(userId) + 66); // was 64
        esp_err_t err = esp_http_client_perform(start_client);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(start_client), esp_http_client_get_content_length(start_client));
#ifdef DEBUG
            printf("%s\n", local_response_buffer);
#endif
            ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);
            cJSON *root, *energy_rate_json, *isSuccess_json;
            root = cJSON_Parse(local_response_buffer);
            if (!root)
            {
                printf("\nnull get_en_json\n");
            }
            else
            {
                // Get the values of the JSON object
                cJSON *payload = cJSON_GetObjectItem(root, "payload");
                // char *payload_value = payload->valuestring;
                /*char *payload_value = NULL;
                if (cJSON_IsString(payload) && payload->valuestring != NULL)
                {
                    cJSON *transaction_id = cJSON_GetObjectItem(payload, "transactionId");
                    transaction_id_value = (uint32_t)transaction_id->valueint;

                    esp_err_t err_transaction_id_value = save_long_to_eeprom("tidvalue:", transaction_id_value);
                    if (err_transaction_id_value != ESP_OK)
                    {
                        ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_transaction_id_value));
                    }

                    printf("transactionId: %u\n", transaction_id_value);
                }
                else
                {
                     printf("transactionId: <null>\n");
                }*/

                cJSON *success = cJSON_GetObjectItem(root, "success");
                bool success_value = cJSON_IsTrue(success);

                cJSON *message = cJSON_GetObjectItem(root, "message");
                char *message_value = NULL;
                if (cJSON_IsString(message) && message->valuestring != NULL)
                {
                    message_value = message->valuestring;
                }

                cJSON *error_message = cJSON_GetObjectItem(root, "errorMessage");
                char *error_message_value = error_message->valuestring;

                cJSON *error_code = cJSON_GetObjectItem(root, "errorCode");
                int error_code_value = error_code->valueint;

                // Print the values
                if (message_value != NULL)
                {
                    printf("message: %s\n", message_value);
                }
                else
                {
                    printf("message: <null>\n");
                }
                if (payload != NULL && cJSON_IsObject(payload))
                {
                    cJSON *transaction_id = cJSON_GetObjectItem(payload, "transactionId");
                    transaction_id_value = (uint32_t)transaction_id->valueint;

                    esp_err_t err_transaction_id_value = save_long_to_eeprom("tidvalue:", transaction_id_value);
                    if (err_transaction_id_value != ESP_OK)
                    {
                        ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_transaction_id_value));
                    }

                    printf("transactionId: %u\n", transaction_id_value);
                }
                else
                {
                    printf("transactionId: <null>\n");
                }
                printf("success: %d\n", success_value);
                // printf("message: %s\n", message_value);

                if (error_message_value != NULL)
                {
                    printf("errorMessage: %s\n", error_message_value);
                }
                else
                {
                    printf("errorMessage: <null>\n");
                }

                printf("errorCode: %d\n", error_code_value);
                // Clean up
                cJSON_Delete(root);
            }
        }
        esp_http_client_cleanup(start_client);
    }
    void stopTransactionHTTP()
    {

        jsonOutReady = false;
        http_start_transaction_err = false;
        const char *TAG = "stopTransactionHTTP";
        const char *authKey = "013ef816d1482a5e9d0208f76abd4768625c03da";

        char tokenCalId[54]; // was 52
        strcpy(tokenCalId, string_address);
        strcat(tokenCalId, authKey);

#ifdef DEBUG
        // printf("userRefID %s tokenCalId %s strlen %d\n", userRefID, tokenCalId, strlen(userRefID));
#endif
        char sha1String[40]; // was 40
        shaCalculate(tokenCalId, sha1String, strlen(string_address) + strlen(authKey));
        // sha1String[41] = '\0';
        int len = std::strlen(sha1String);
        for (int i = 0; i < len; i++)
        {
            sha1String[i] = std::toupper(sha1String[i]);
        }

#ifdef DEBUG
        ESP_LOGI(TAG, "%s", sha1String);
#endif
        char local_response_buffer[600];
        /* esp_http_client_config_t config = {
             .url = startUrl,
             .query = "esp",
             .disable_auto_redirect = false,
             .max_redirection_count = 2,
             .event_handler = _http_event_handler,
             .user_data = local_response_buffer, // Pass address of local buffer to get response

             //.crt_bundle_attach = esp_crt_bundle_attach,
         };*/
        esp_http_client_config_t config_post = {
            .url = stopUrl,
            .cert_pem = (char *)server_cert_2_pem_start,
            .method = HTTP_METHOD_POST,
            .event_handler = _http_event_handler,
            .user_data = local_response_buffer, // Pass address of local buffer to get response

            //.crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t start_client = esp_http_client_init(&config_post);
        esp_http_client_set_url(start_client, stopUrl);
        esp_http_client_set_method(start_client, HTTP_METHOD_POST);
        esp_http_client_set_header(start_client, "Content-Type", "application/x-www-form-urlencoded");
        esp_http_client_set_header(start_client, "User-Agent", "Super Agent/0.0.1");

        char httpRequestData[140] = "referenceNo=";
        strcat(httpRequestData, string_address);
        char transaction_id_value_str[10];
        strcat(httpRequestData, "&transactionId=");
        sprintf(transaction_id_value_str, "%u", transaction_id_value);
        strcat(httpRequestData, transaction_id_value_str);

        char energy_str[10];
        strcat(httpRequestData, "&energy=");
        sprintf(energy_str, "%.2f", energy);
        strcat(httpRequestData, energy_str);

        strcat(httpRequestData, "&status=STOP");
        strcat(httpRequestData, "&requestToken=");
        strcat(httpRequestData, sha1String);
        // strcat(httpRequestData, "}");
        ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);

        esp_http_client_set_post_field(start_client, httpRequestData, strlen(sha1String) + strlen(energy_str) + strlen(transaction_id_value_str) + 75); // was 60
        esp_err_t err = esp_http_client_perform(start_client);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(start_client), esp_http_client_get_content_length(start_client));
#ifdef DEBUG
            printf("%s\n", local_response_buffer);
#endif
            ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);
            cJSON *root, *energy_rate_json, *isSuccess_json;
            root = cJSON_Parse(local_response_buffer);
            if (!root)
            {
                printf("\nnull get_en_json\n");
            }
            else
            {
                // Get the values of the JSON object
                cJSON *payload = cJSON_GetObjectItem(root, "payload");
                char *payload_value = payload->valuestring;

                // cJSON *transaction_id = cJSON_GetObjectItem(payload, "transactionId");
                // int transaction_id_value = transaction_id->valueint;

                cJSON *success = cJSON_GetObjectItem(root, "success");
                bool success_value = cJSON_IsTrue(success);

                cJSON *message = cJSON_GetObjectItem(root, "message");
                char *message_value = NULL;
                if (cJSON_IsString(message) && message->valuestring != NULL)
                {
                    message_value = message->valuestring;
                }

                cJSON *error_message = cJSON_GetObjectItem(root, "errorMessage");
                char *error_message_value = error_message->valuestring;

                cJSON *error_code = cJSON_GetObjectItem(root, "errorCode");
                int error_code_value = error_code->valueint;

                // Print the values
                // printf("transactionId: %d\n", transaction_id_value);
                if (payload_value != NULL)
                {
                    printf("payload_value: %s\n", payload_value);
                }
                else
                {
                    printf("payload_value: <null>\n");
                }

                printf("success: %d\n", success_value);
                // printf("message: %s\n", message_value);
                if (message_value != NULL)
                {
                    printf("message: %s\n", message_value);
                }
                else
                {
                    printf("message: <null>\n");
                }
                if (error_message_value != NULL)
                {
                    printf("errorMessage: %s\n", error_message_value);
                }
                else
                {
                    printf("errorMessage: <null>\n");
                }

                printf("errorCode: %d\n", error_code_value);
                // Clean up
                cJSON_Delete(root);
            }
        }
        esp_http_client_cleanup(start_client);
    }

    /* void startTransactionHTTP()
      {
          jsonOutReady = false;
          http_start_transaction_err = false;
          const char *TAG = "startTransactionHTTP";
          // Check WiFi connection status
          // if (WiFi.status() == WL_CONNECTED) {
          char tokenCalId[28];
          // char userRefID[25]="0.1.0.255.0.4.91.0.";
          strcpy(tokenCalId, userRefID);
          strcat(tokenCalId, NODEID);

    #ifdef DEBUG
          printf("userRefID %s tokenCalId %s strlen %d\n", userRefID, tokenCalId, strlen(userRefID));
    #endif
          char sha1String[41];
          shaCalculate(tokenCalId, sha1String, strlen(userRefID) + NODEID_LEN - 1);
          sha1String[41] = '\0';

    #ifdef DEBUG
          ESP_LOGI("main_task", "%s", sha1String);
    #endif
          char local_response_buffer[600];
          esp_http_client_config_t config = {
              .url = startUrl,
              .query = "esp",
              .disable_auto_redirect = false,
              .max_redirection_count = 2,
              .event_handler = _http_event_handler,
              .user_data = local_response_buffer, // Pass address of local buffer to get response

              //.crt_bundle_attach = esp_crt_bundle_attach,
          };
          esp_http_client_handle_t start_client = esp_http_client_init(&config);
          esp_http_client_set_url(start_client, startUrl);
          esp_http_client_set_method(start_client, HTTP_METHOD_POST);
          esp_http_client_set_header(start_client, "Content-Type", "application/x-www-form-urlencoded");
          esp_http_client_set_header(start_client, "User-Agent", "Super Agent/0.0.1");
          // String httpRequestData = "userReferenceID=" + String(userRefID) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
          char httpRequestData[120] = "userReferenceID=";
          strcat(httpRequestData, userRefID);
          // printf("%s\n",httpRequestData);
          strcat(httpRequestData, "&chargePointReferenceID=");
          // printf("%s\n",httpRequestData);
          strcat(httpRequestData, NODEID);
          // printf("%s\n",httpRequestData);
          strcat(httpRequestData, "&token=");
          // printf("%s\n",httpRequestData);
          strcat(httpRequestData, sha1String);
          printf("%s\n", httpRequestData);
          esp_http_client_set_post_field(start_client, httpRequestData, strlen(userRefID) + strlen(NODEID) + 87);
          esp_err_t err = esp_http_client_perform(start_client);
          if (err == ESP_OK)
          {
              ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(start_client), esp_http_client_get_content_length(start_client));
    #ifdef DEBUG
              printf("%s\n", local_response_buffer);
    #endif
              cJSON *get_en_json, *energy_rate_json;
              get_en_json = cJSON_Parse(local_response_buffer);
              if (!get_en_json)
              {
                  printf("\nnull get_en_json\n");
              }
              else
              {
                  energy_rate_json = cJSON_GetObjectItem(get_en_json, "status");
                  if (!energy_rate_json)
                  {
                      printf("\nnull json\n");
                  }
                  else
                  {
                      sprintf(startStatus, energy_rate_json->valuestring);
                      printf("status %s\n", startStatus);
                      if ((esp_http_client_get_status_code(start_client) == 200) && (memcmp(startStatus, "STARTED", 7) == 0))
                      {
                          printf("inside if status %s\n", startStatus);
                          sprintf(started, startStatus);
                          cJSON *balance_json, *transac_id_json, *user_id_json, *user_ref_json, *charge_point_ref_json, *username_json, *success_json, *message_json, *error_msg_json, *rate_json;

                          balance_json = cJSON_GetObjectItem(get_en_json, "balance");
                          accountBalance = (float)atof(balance_json->valuestring);
                          printf("inside if status %f \n", accountBalance);

                          transac_id_json = cJSON_GetObjectItem(get_en_json, "transactionID");
                          sprintf(transactionID, transac_id_json->valuestring);
                          printf("inside if status %s\n", transac_id_json->valuestring);

                          error_msg_json = cJSON_GetObjectItem(get_en_json, "errorMessage");
                          if (error_msg_json->valuestring != NULL)
                          {
                              sprintf(errorMessage, error_msg_json->valuestring);
                              printf("inside if status 7 %s\n", error_msg_json->valuestring);
                          }

                          rate_json = cJSON_GetObjectItem(get_en_json, "energyRate");
                          energyRate = (float)atof(rate_json->valuestring);
                          printf("inside if status %f \n", energyRate);

                          user_id_json = cJSON_GetObjectItem(get_en_json, "userID");
                          sprintf(userID, user_id_json->valuestring);
                          printf("inside if status 1 %s\n", user_id_json->valuestring);

                          user_ref_json = cJSON_GetObjectItem(get_en_json, "userReference");
                          sprintf(userReference, user_ref_json->valuestring);
                          printf("inside if status 2 %s\n", user_ref_json->valuestring);

                          charge_point_ref_json = cJSON_GetObjectItem(get_en_json, "chargePointReference");
                          sprintf(chargePointReference, charge_point_ref_json->valuestring);
                          printf("inside if status 3 %s\n", charge_point_ref_json->valuestring);

                          username_json = cJSON_GetObjectItem(get_en_json, "username");
                          sprintf(username, username_json->valuestring);
                          printf("inside if status 4 %s\n", username_json->valuestring);

                          success_json = cJSON_GetObjectItem(get_en_json, "success");
                          // sprintf(success,success_json->valuestring);
                          printf("inside if status 5 %d\n", success_json->valueint);
                          success = success_json->valueint;

                          message_json = cJSON_GetObjectItem(get_en_json, "message");
                          if (message_json->valuestring != NULL)
                          {
                              sprintf(message, message_json->valuestring);
                              printf("inside if status 6 %s\n", message_json->valuestring);
                          }

                          jsonOutReady = true;
                          // printf("inside if jsonOutReady=true");
                      }
                      else
                      {

                          cJSON *error_code_json;
                          error_code_json = cJSON_GetObjectItem(get_en_json, "errorCode");
                          if (!error_code_json)
                          {
                              printf("\nnull error_code_json\n");
                          }
                          else
                          {

                              authFailErrorCode = error_code_json->valueint;
                              printf("authFailErrorCode %d\n", authFailErrorCode);
                          }
                          jsonOutReady = true;
                      }

                      // energyRate=(float)atof(energy_rate_json->valuestring);
                      // printf("got engergy_rate:%f\n",energyRate);
                  }
              }
              // printf("cJSON_Delete(get_en_json)");
              cJSON_Delete(get_en_json); // Free memory
                                         // printf("cJSON_Delete(get_en_json) done");
          }
          else
          {
              http_start_transaction_err = true;
              ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
          }
          esp_http_client_cleanup(start_client);
      }*/

    /*  void stoptTransactionHTTP()
      {
          jsonOutReady = 0;
          const char *TAG = "stoptTransactionHTTP";
    #ifdef DEBUG
          ESP_LOGI(TAG, "STOP_TRX_HTTP1:StopTrxCore1");
    #endif
          cJSON *json_ob1;
          json_ob1 = cJSON_CreateObject();
          char *ob1;

          if (userID == NULL || memcmp(userID, "", 1) == 0)
          {
              ESP_LOGI(TAG, "userID is null");
              cJSON_AddStringToObject(json_ob1, "command", "heartBeat");
              cJSON_AddStringToObject(json_ob1, "ticker", "0");
              cJSON_AddStringToObject(json_ob1, "difference", "0");
              cJSON_AddStringToObject(json_ob1, "soc", "");
              char timeDiff_chr[25];
              sprintf(timeDiff_chr, "%.1f", timeDiff / 60);
              cJSON_AddStringToObject(json_ob1, "chargeTime", (const char *)timeDiff_chr);
              cJSON_AddStringToObject(json_ob1, "chargeAmount", "0");
              cJSON_AddStringToObject(json_ob1, "userBalance", "0");
              cJSON_AddStringToObject(json_ob1, "stopCondition", "");
              char start_chr[4];
              sprintf(start_chr, "%d", START_STATE);
              cJSON_AddStringToObject(json_ob1, "state", (const char *)start_chr);
              cJSON_AddStringToObject(json_ob1, "userID", (const char *)userID);
              cJSON_AddStringToObject(json_ob1, "transactionID", (const char *)transactionID);
              char energySend_chr[25];
              sprintf(energySend_chr, "%.2f", energySend);
              cJSON_AddStringToObject(json_ob1, "energyCalc", energySend_chr);
              cJSON_AddStringToObject(json_ob1, "userReferenceID", userRefID);
              char doubleCharge_chr[4];
              sprintf(doubleCharge_chr, "%d", doubleCharge);
              cJSON_AddStringToObject(json_ob1, "doubleCharge", doubleCharge_chr);

              ob1 = cJSON_Print(json_ob1); //  take json Convert form to string
              printf("%s\n", ob1);
              free(ob1);
          }
          else
          {
              int readFilestat = 0;
              // readFilestat = ReadFile1(FILE_1);

              if (readFilestat == READFILE_OK)
              {

                  // ob1 = String(file_1);
              }
              else
              {
    #ifdef DEBUG
                  ESP_LOGE(TAG, "STOP_TRX_HTTP1:File_1 error");
    #endif
              }
          }

          cJSON_Delete(json_ob1);

          int stopTrx = 1;
          xQueueSend(stopTransactionQueue, &stopTrx, 0);
          ESP_LOGW(TAG, "xQueueSend(stopTransactionQueue");

          //   int checkNet = 0;
          //   xQueueReceive(checkInternetQueue, &checkNet, portMAX_DELAY);
          //   //ESP_LOGI("SqrecevchkInt1******");
          if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
          {
              if (is_internet_available == true)
              {
                  xSemaphoreGive(internetMutex);
                  int httpCode = 0;

                  if (HTTPCodeQueue != NULL)
                  {
                      ESP_LOGW(TAG, "xQueueReceive(HTTPCodeQueue0");
                      xQueueReceive(HTTPCodeQueue, &httpCode, portMAX_DELAY);
                      ESP_LOGW(TAG, " xQueueReceive(HTTPCodeQueue1");
                  }
                  else
                  {
                      ESP_LOGE(TAG, "HTTPCodeQueue==NULL");
                  }

                  if (httpCode == 1)
                  {
                      //   ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
                      memset(transactionID, '\0', sizeof(transactionID));
                      //   ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
                      // write to file 2
                  }
                  else
                  {

                      accountBalance = 0;
                      int writeError = 0;
                      // writeError = writeFile(ob1, FILE_2);
                      if (writeError != 1)
                      {
    #ifdef DEBUG
                          ESP_LOGE(TAG, "STOP_TRX_HTTP1:Write file 2 error");
    #endif
                      }
                  }
              }

              else
              {
                  xSemaphoreGive(internetMutex);
                   timeWhenStopping = timeDiff;
                   jsonOutReady = 1;
                   String tmp = "";
                   tmp = String(transactionID);
                   if (tmp != "null") {
                     int writeError2 = 0;
                     writeError2 = writeFile(ob1, FILE_2);
                     if (writeError2 != 1) {
               #ifdef DEBUG
                       ESP_LOGI("STOP_TRX_HTTP1:Write file 2 offline 2error");
               #endif
                     }
                   }*/
    /* }
    }
    jsonOutReady = 1;
    energySend = 0;
    // ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
    memset(transactionID, '\0', sizeof(transactionID));
    ESP_LOGW(TAG, "stoptTransactionHTTP() DONE");
    }*/

    /* void energyBasedStopTransaction()
     {
         jsonOutReady = 0;
         const char *TAG = "energyBasedStopTransaction";
    #ifdef DEBUG
         ESP_LOGI(TAG, "STOP_TRXENERGY_HTTP1");
    #endif
         cJSON *json_ob1;
         json_ob1 = cJSON_CreateObject();
         char *ob1;

         if (userID == NULL || memcmp(userID, "", 1) == 0)
         {
             ESP_LOGI(TAG, "userID is null");
             cJSON_AddStringToObject(json_ob1, "command", "heartBeat");
             cJSON_AddStringToObject(json_ob1, "ticker", "0");
             cJSON_AddStringToObject(json_ob1, "difference", "0");
             cJSON_AddStringToObject(json_ob1, "soc", "");
             char timeDiff_chr[25];
             sprintf(timeDiff_chr, "%.2f", timeDiff / 60);
             cJSON_AddStringToObject(json_ob1, "chargeTime", (const char *)timeDiff_chr);
             cJSON_AddStringToObject(json_ob1, "chargeAmount", "0");
             cJSON_AddStringToObject(json_ob1, "userBalance", "0");
             cJSON_AddStringToObject(json_ob1, "stopCondition", "");
             char start_chr[4];
             sprintf(start_chr, "%d", START_STATE);
             cJSON_AddStringToObject(json_ob1, "state", (const char *)start_chr);
             cJSON_AddStringToObject(json_ob1, "userID", (const char *)userID);
             cJSON_AddStringToObject(json_ob1, "transactionID", (const char *)transactionID);
             char energySend_chr[25];
             sprintf(energySend_chr, "%.2f", energySend);
             cJSON_AddStringToObject(json_ob1, "energyCalc", energySend_chr);
             cJSON_AddStringToObject(json_ob1, "userReferenceID", userRefID);
             char doubleCharge_chr[4];
             sprintf(doubleCharge_chr, "%d", doubleCharge);
             cJSON_AddStringToObject(json_ob1, "doubleCharge", doubleCharge_chr);

             ob1 = cJSON_Print(json_ob1); //  take json Convert form to string
             printf("%s\n", ob1);
             free(ob1);
         }
         else
         {
             int readFilestat = 0;
             // readFilestat = ReadFile1(FILE_1);

             if (readFilestat == READFILE_OK)
             {

                 // ob1 = String(file_1);
             }
             else
             {
    #ifdef DEBUG
                 ESP_LOGE(TAG, "File_1 error");
    #endif
             }
         }

         cJSON_Delete(json_ob1);

         int stopTrx = 1;
         xQueueSend(stopTransactionQueueEnergy, &stopTrx, 0);
         ESP_LOGW(TAG, "xQueueSend(stopTransactionQueueEnergy");

         //   int checkInternet = 0;
         //   ESP_LOGW(TAG,"xQueueReceive(checkInternetQueueEnergy0");
         //   xQueueReceive(checkInternetQueueEnergy, (void*)&checkInternet, portMAX_DELAY);
         //   ESP_LOGW(TAG,"xQueueReceive(checkInternetQueueEnergy1");
         if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
         {
             if (is_internet_available == true)
             {
                 xSemaphoreGive(internetMutex);
                 int httpCode = 0;

                 if (HTTPCodeQueueEnergy != NULL)
                 {
                     ESP_LOGW(TAG, "xQueueReceive(HTTPCodeQueueEnergy0");
                     xQueueReceive(HTTPCodeQueueEnergy, &httpCode, portMAX_DELAY);
                     ESP_LOGW(TAG, " xQueueReceive(HTTPCodeQueueEnergy1");
                 }
                 else
                 {
                     ESP_LOGE(TAG, "HTTPCodeQueueEnergy==NULL");
                 }

                 if (httpCode == 1)
                 {
                     //   ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
                     memset(transactionID, '\0', sizeof(transactionID));
                     //   ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
                     //   strcpy(transactionID, transactionIDchrEmpty);
                     // write to file 2
                 }
                 else
                 {

                     accountBalance = 0;
                     int writeError = 0;
                     // writeError = writeFile(ob1, FILE_2);
                     if (writeError != 1)
                     {
    #ifdef DEBUG
                         ESP_LOGE(TAG, "STOP_TRX_HTTP1:Write file 2 error");
    #endif
                     }
                 }
             }

             else
             {
                 xSemaphoreGive(internetMutex);
                  timeWhenStopping = timeDiff;
                  jsonOutReady = 1;
                  String tmp = "";
                  tmp = String(transactionID);
                  if (tmp != "null") {
                    int writeError2 = 0;
                    writeError2 = writeFile(ob1, FILE_2);
                    if (writeError2 != 1) {
              #ifdef DEBUG
                      ESP_LOGI("STOP_TRX_HTTP1:Write file 2 offline 2error");
              #endif
                    }
                  }*/
    /*  }
    }
    jsonOutReady = 1;
    energySend = 0;
    // ESP_LOGE(TAG,"(httpCode == 1 transactionIDchrEmpty");
    memset(transactionID, '\0', sizeof(transactionID));
    ESP_LOGW(TAG, "energyBasedStopTransaction() DONE");
    //   strcpy(transactionID, transactionIDchrEmpty);
    }*/

    /* void recordHeartBeat()
     {
    #ifdef DEBUG
         ESP_LOGI("recordHeartBeat", "HTTP_HEARTBEAT:Record Heartbeat");
    #endif
         char sha1String[41];
         sha1String[41] = '\0';
         shaCalculate(NODEID, sha1String, sizeof(NODEID) - 1);

    #ifdef DEBUG
         ESP_LOGI("recordHeartBeat", "sha1String %s", sha1String);
    #endif
         char local_response_buffer[100];
         esp_http_client_config_t config = {
             .url = deepDiveUrl,
             .query = "esp",
             .disable_auto_redirect = false,
             .max_redirection_count = 2,
             .event_handler = _http_event_handler,
             .user_data = local_response_buffer, // Pass address of local buffer to get response

             //.crt_bundle_attach = esp_crt_bundle_attach,
         };
         esp_http_client_handle_t client = esp_http_client_init(&config);
         esp_http_client_set_url(client, deepDiveUrl);
         esp_http_client_set_method(client, HTTP_METHOD_POST);
         // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
         esp_http_client_set_header(client, "User-Agent", "Super Agent/0.0.1");
         // String httpRequestData = "userID=" + String(0) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
         // sprintf(str, "%02X", (int)shaResult[i]);
         // const char *httpRequestData = "userID=0&chargePointReferenceID=",NODEID,"&token=",sha1String;
         char httpRequestData[182 + NODEID_LEN] = "chargePointReference=";
         strcat(httpRequestData, NODEID);
         // printf("%s\n",httpRequestData);
         strcat(httpRequestData, "&temp1=0&temp2=0&temp3=0&temp4=0&faults=0&ram_usage=0&cpu_usage=0&power_status=");
         // printf("%s\n",httpRequestData);
         char current_chr[3];
         sprintf(current_chr, "%02d", CURRENT_STATE);
         strcat(httpRequestData, current_chr);
         // printf("%s\n",httpRequestData);
         strcat(httpRequestData, "&fault_description=0&disk_space=0");
         // printf("%s\n",httpRequestData);
         strcat(httpRequestData, "&token=");
         // printf("%s\n",httpRequestData);
         strcat(httpRequestData, sha1String);
         printf("%s\n", httpRequestData);

         esp_http_client_set_post_field(client, httpRequestData, 181 + NODEID_LEN);
         esp_err_t err = esp_http_client_perform(client);

         if (err == ESP_OK)
         {
             ESP_LOGI("recordHeartBeat", "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
             printf("%s\n", local_response_buffer);
         }
         else
         {
             ESP_LOGE("recordHeartBeat", "HTTP POST request failed: %s", esp_err_to_name(err));
         }
         esp_http_client_cleanup(client);
         //  } else {
         //    ESP_LOGI("offline : recordHeartbeat");
         //  }
     }*/

    void factoryResetHTTP()
    {

        jsonOutReady = false;
        http_start_transaction_err = false;
        const char *TAG = "factoryResetHTTP";
        const char *authKey = "013ef816d1482a5e9d0208f76abd4768625c03da";

        char tokenCalId[54]; // was52
        strcpy(tokenCalId, string_address);
        strcat(tokenCalId, authKey);

#ifdef DEBUG
        // printf("userRefID %s tokenCalId %s strlen %d\n", userRefID, tokenCalId, strlen(userRefID));
#endif
        char sha1String[40]; // was 40
        shaCalculate(tokenCalId, sha1String, strlen(string_address) + strlen(authKey));
        // sha1String[41] = '\0';

        int len = std::strlen(sha1String);
        for (int i = 0; i < len; i++)
        {
            sha1String[i] = std::toupper(sha1String[i]);
        }

#ifdef DEBUG
        ESP_LOGI(TAG, "%s", sha1String);
#endif
        char local_response_buffer[150];
        esp_http_client_config_t config_post = {
            .url = factoryResetUrl,
            .cert_pem = (char *)server_cert_2_pem_start,
            .method = HTTP_METHOD_POST,
            .event_handler = _http_event_handler,
            .user_data = local_response_buffer, // Pass address of local buffer to get response

            //.crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t start_client = esp_http_client_init(&config_post);
        esp_http_client_set_url(start_client, factoryResetUrl);
        esp_http_client_set_method(start_client, HTTP_METHOD_POST);
        esp_http_client_set_header(start_client, "Content-Type", "application/x-www-form-urlencoded");
        esp_http_client_set_header(start_client, "User-Agent", "Super Agent/0.0.1");

        char httpRequestData[140] = "referenceNo=";
        strcat(httpRequestData, string_address);

        strcat(httpRequestData, "&userId=0");
        // strcat(httpRequestData, userId);

        strcat(httpRequestData, "&requestType=CHARGER");
        strcat(httpRequestData, "&isPaired=");
        char isPaired_str[10];
        if (isPaired == 1)
        {
            sprintf(isPaired_str, "%s", "true"); // convert Boolean to string
        }
        else
        {
            sprintf(isPaired_str, "%s", "false"); // convert Boolean to string
        }

        strcat(httpRequestData, isPaired_str);
        strcat(httpRequestData, "&requestToken=");
        strcat(httpRequestData, sha1String);
        // strcat(httpRequestData, "}");
        ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);

        esp_http_client_set_post_field(start_client, httpRequestData, strlen(sha1String) + strlen(isPaired_str) + 79); // was 64
        esp_err_t err = esp_http_client_perform(start_client);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(start_client), esp_http_client_get_content_length(start_client));
#ifdef DEBUG
            printf("%s\n", local_response_buffer);
#endif
            ESP_LOGI(TAG, "httpRequestData:%s", httpRequestData);
            cJSON *root, *energy_rate_json, *isSuccess_json;
            root = cJSON_Parse(local_response_buffer);
            if (!root)
            {
                printf("\nnull get_en_json\n");
            }
            else
            {
                // Get the values of the JSON object
                cJSON *payload = cJSON_GetObjectItem(root, "payload");
                char *payload_value = payload->valuestring;

                // cJSON *transaction_id = cJSON_GetObjectItem(payload, "transactionId");
                // int transaction_id_value = transaction_id->valueint;

                cJSON *success = cJSON_GetObjectItem(root, "success");
                bool success_value = cJSON_IsTrue(success);

                cJSON *message = cJSON_GetObjectItem(root, "message");
                char *message_value = NULL;
                if (cJSON_IsString(message) && message->valuestring != NULL)
                {
                    message_value = message->valuestring;
                }

                cJSON *error_message = cJSON_GetObjectItem(root, "errorMessage");
                char *error_message_value = error_message->valuestring;

                cJSON *error_code = cJSON_GetObjectItem(root, "errorCode");
                int error_code_value = error_code->valueint;

                // Print the values
                // printf("transactionId: %d\n", transaction_id_value);
                if (payload_value != NULL)
                {
                    printf("payload_value: %s\n", payload_value);
                }
                else
                {
                    printf("payload_value: <null>\n");
                }

                printf("success: %d\n", success_value);
                // printf("message: %s\n", message_value);
                if (message_value != NULL)
                {
                    printf("message: %s\n", message_value);
                }
                else
                {
                    printf("message: <null>\n");
                }
                if (error_message_value != NULL)
                {
                    printf("message: %s\n", error_message_value);
                }
                else
                {
                    printf("errorMessage: <null>\n");
                }
                printf("errorCode: %d\n", error_code_value);
                // Clean up
                cJSON_Delete(root);
            }
        }
        esp_http_client_cleanup(start_client);
    }

    void print_mac(uint8_t *mac)
    {
        printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        // char macStr[13]="";
        // char *thisString="this is the string";
        // for(int i=0;i<6;i++){
        //  const char* macStr=(const char*)(mac[0]);

        //}
        // printf("mac is:%s",macStr);
    }

    void getMACAddress()
    {

        esp_efuse_mac_get_default(mac_base);
        // esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
        //  unsigned char mac_local_base[6] = {0};
        //  unsigned char mac_uni_base[6] = {0};
        //  esp_derive_local_mac(mac_local_base, mac_uni_base);
        //  printf("Local Address: ");
        //  print_mac(mac_local_base);
        //  printf("\nUni Address: ");
        //  print_mac(mac_uni_base);
        printf("MAC Address: ");
        print_mac(mac_base);
        // memcpy(mac_address, mac_base, 6);

        int i = 0;
        int index = 0;
        int index2 = 0;
        /*for (i = 5; i >= 0; i--)
        {
            index += sprintf(&string_address[index], "%.2x", mac_base[i]);
        }*/

        macAddress[0] = mac_base[2];
        macAddress[1] = mac_base[0];
        macAddress[2] = mac_base[5];
        macAddress[3] = mac_base[1];
        macAddress[4] = mac_base[3];
        macAddress[5] = mac_base[4];

        for (i = 0; i < 6; i++)
        {

            index += sprintf(&string_address2[index], "%.2x", macAddress[i]);
            index2 += sprintf(&mac_address_base[index2], "%.2x", mac_base[i]);
        }

        ESP_LOGI("MQTT  Topic before wild card", "%s", string_address2);
        string_address[0] = 'o';
        string_address[1] = '3';
        memmove(string_address + 2, string_address2, sizeof(string_address2));
        ESP_LOGI("MQTT  Topic after wild card", "%s", string_address);

        ESP_LOGI("MAC Address", "%s", mac_address_base);
    }
    static bool check_topic(uint8_t *topic)
    {
        for (int i = 0; i < 18; i++)
            if (TOPIC_RECEIVE[i] != topic[i])
                return false;
        return true;
    }

    static bool check_topic2(uint8_t *topic)
    {
        for (int i = 0; i < 5; i++)
        {
            if (NODEID[i] != topic[i])
            {

                return false;
            }
            else
            {

                printf("NODEID[i]=%c topic[i]=%c\n", NODEID[i], topic[i]);
            }
        }
        return true;
    }
    static bool check_topic_mac(uint8_t *topic)
    {
        for (int i = 0; i < strlen(string_address); i++)
        {
            if (string_address[i] != topic[i])
            {
                printf("string_address[i]=%c topic[i]=%c\n", string_address[i], topic[i]);
                return false;
            }
            else
            {
                printf("string_address[i]=%c topic[i]=%c\n", string_address[i], topic[i]);
            }
        }
        return true;
    }

    static bool check_mac(uint8_t *topic)
    {
        for (int i = 5; i < (strlen(string_address) + 5); i++)
        {
            if (string_address[i - 5] != topic[i])
            {
                printf("string_address[i]=%c topic[i]=%c\n", string_address[i - 5], topic[i]);
                return false;
            }
            else
            {
                printf("string_address[i]=%c topic[i]=%c\n", string_address[i - 5], topic[i]);
            }
        }
        return true;
    }

    static esp_err_t
    mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
    {
        const char *TAG1 = "mqtt_event_handler_cb";
        mqtt_client = event->client;
        int msg_id;
        switch (event->event_id)
        {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG1, "MQTT_EVENT_BEFORE_CONNECT");
            break;
        case MQTT_EVENT_DELETED:
            ESP_LOGI(TAG1, "MQTT_EVENT_DELETED");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG1, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG1, "MQTT_EVENT_CONNECTED_TEST");

            printf("Subscribing to topic :%s\n", TOPIC_RECEIVE);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, TOPIC_RECEIVE, 1);
            printf("Subscribing to topic :%s\n", NODEID);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, NODEID, 1);
            // getMACAddress();
            //  char mac_address1[8]={0};
            //  memcpy(mac_address1,mac_base,6);

            printf("Subscribing to topic :%s\n", string_address);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, string_address, 1);
            // ESP_LOGI(TAG1, "mqtt api returned %d", msg_id);
            msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC_RECEIVE, "ESP32_connected", 0, 1, 0);

            // if(sendMqtt)
            // {
            // esp_mqtt_client_publish(client, "ChargeNET_mqtt",(const char*)heartBeatObj, 0, 1, 0);
            // vTaskDelay(10000/portTICK_PERIOD_MS);
            // }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG1, "MQTT_EVENT_DISCONNECTED");
            esp_restart();

            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG1, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG1, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG1, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ANY:
            ESP_LOGI(TAG1, "MQTT_EVENT_ANY");
            break;

        case MQTT_EVENT_DATA:
        {
            ESP_LOGI(TAG1, "MQTT_EVENT_DATA");
            printf("\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            uint8_t *topic_buff = (uint8_t *)event->topic;

            if (check_topic_mac(topic_buff))
            {
                cJSON *mqtt_json, *command_json, *userId_json, *scheduleStartTimeWeekDays_json, *scheduleStopTimeWeekDays_json, *scheduleStartTimeWeekEnds_json, *scheduleStopTimeWeekEnds_json, *maximumCurrentSetting_json;
                // if(strcmp(NODEID,event->topic)==0)

                mqtt_json = cJSON_Parse(event->data);
                if (!mqtt_json)
                {
                    printf("\nnull mqtt_json\n");
                }
                else
                {
                    userId_json = cJSON_GetObjectItem(mqtt_json, "userId");
                    if (!userId_json)
                    {
                        printf("\nnull userId_json json\n");
                    }
                    else
                    {
                        strcpy(userId, userId_json->valuestring);
                        printf("got UserId:%s\n", userId);

                        esp_err_t err_userId = save_key_value("userId:", userId);
                        if (err_userId != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) saving to NVS err_userId", esp_err_to_name(err_userId));
                        }
                    }
                    command_json = cJSON_GetObjectItem(mqtt_json, "command");
                    if (!command_json)
                    {
                        printf("\nnull json\n");
                    }
                    else
                    {
                        strcpy(MQTTCommand, command_json->valuestring);
                        printf("got MQTTCommand:%s\n", MQTTCommand);
                        if (memcmp(MQTTCommand, "start", 5) == 0 && MQTTCommand[5] != 'C')
                        {

#ifdef DEBUG
                            ESP_LOGI("MQTT_CALLBACK", "Mobile Start Command received");
#endif
                            esp_err_t err_is_charger_locked = load_int_value("isLocked:", &isChargerLocked);
                            if (err_is_charger_locked != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) loading to NVS", esp_err_to_name(err_is_charger_locked));
                            }
                            if (DEFINE_STANDALONE)
                            {
                                if (CURRENT_STATE == IDLE_STATE)
                                {
                                    mobileStartCommand = 1;
                                    mode = 2;
                                }
                            }
                            else
                            {
                                // enter code here
                            }
                        }
                        else if (memcmp(MQTTCommand, "stop", 4) == 0)
                        {
                            // connectorStopCharge = 1;
                            stopCharge = 1;
                        }
                        else if (memcmp(MQTTCommand, "enableEcoPlusMode", 16) == 0)
                        {
                            ecoPlusModeEnabled = 1;
                            esp_err_t err_ecoPlusModeEnabled = save_int_value("epmEnable:", ecoPlusModeEnabled);
                            if (err_ecoPlusModeEnabled != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_ecoPlusModeEnabled));
                            }
                        }
                        else if (memcmp(MQTTCommand, "disableEcoPlusMode", 18) == 0)
                        {
                            ecoPlusModeEnabled = 0;
                            esp_err_t err_ecoPlusModeEnabled = save_int_value("epmEnable:", ecoPlusModeEnabled);
                            if (err_ecoPlusModeEnabled != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_ecoPlusModeEnabled));
                            }
                        }
                        else if (memcmp(MQTTCommand, "registerAck", 11) == 0)
                        {
                            /*connectorRegisterDetect = 1;// today
                            // factoryResetHTTP();
                            transaction_id_value = 133;
                            esp_err_t err_transaction_id_value = save_long_to_eeprom("tidvalue:", transaction_id_value);
                            if (err_transaction_id_value != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_transaction_id_value));
                            }*/
                            gpio_set_level(loraReetPin, 0);
                            vTaskDelay(1000 / portTICK_PERIOD_MS); // today was 1000
                            gpio_set_level(loraReetPin, 1);
                            vTaskDelay(1000 / portTICK_PERIOD_MS); // today was 1000
                            lora_init();
                            lora_set_frequency(433e6);
                            lora_enable_crc();
                            lora_receive();
                        }
                        else if (memcmp(MQTTCommand, "Reboot", 6) == 0)
                        {
                            gpio_set_level(STM32_RESET, 1);
                            vTaskDelay(40 / portTICK_RATE_MS);
                            gpio_set_level(STM32_RESET, 0);
                            esp_restart();
                        }
                        else if (memcmp(MQTTCommand, "wifi_config_state_on", 20) == 0)
                        {
                            wifi_config_state = 1;
                            ESP_LOGI("MQTT_CALLBACK", "wifi_config_state_on Command received");
                            esp_err_t err_wifi_config_state = save_int_value("wcstate:", wifi_config_state);
                            if (err_wifi_config_state != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_wifi_config_state));
                            }
                        }
                        else if (memcmp(MQTTCommand, "wifi_config_state_off", 21) == 0)
                        {
                            wifi_config_state = 0;
                            ESP_LOGI("MQTT_CALLBACK", "wifi_config_state_off Command received");
                            esp_err_t err_wifi_config_state = save_int_value("wcstate:", wifi_config_state);
                            if (err_wifi_config_state != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_wifi_config_state));
                            }
                        }

                        else if (memcmp(MQTTCommand, "alarmUpdated", 12) == 0)
                        {
                            alarmUpdateCompleteFlag = 1;

                            esp_err_t err_alarmUpdated = save_int_value("alarmUpdated:", alarmUpdateCompleteFlag);
                            if (err_alarmUpdated != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_alarmUpdated));
                            }
                            ESP_LOGI("MQTT_CALLBACK", "alarmUpdated Command received");
                            UpdateAlarmSerialFlag = 1;
                        }
                        else if (memcmp(MQTTCommand, "updateAlarm", 11) == 0)
                        {
                            alarmUpdateCompleteFlag = 0;
                            esp_err_t err_alarmUpdated = save_int_value("alarmUpdated:", alarmUpdateCompleteFlag);
                            if (err_alarmUpdated != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_alarmUpdated));
                            }
                            ESP_LOGI("MQTT_CALLBACK", "alarmUpdate Command received");
                        }
                        else if (memcmp(MQTTCommand, "lockCharger", 11) == 0)
                        {
                            isChargerLocked = 1;
                            esp_err_t err_isLockedtrue = save_int_value("isLocked:", isChargerLocked);
                            if (err_isLockedtrue != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_isLockedtrue));
                            }
                            ESP_LOGI("MQTT_CALLBACK", "lockCharger Command received");
                        }
                        else if (memcmp(MQTTCommand, "unlockCharger", 13) == 0)
                        {
                            isChargerLocked = 0;
                            esp_err_t err_isLocked = save_int_value("isLocked:", isChargerLocked);
                            if (err_isLocked != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_isLocked));
                            }
                            ESP_LOGI("MQTT_CALLBACK", "unlockCharger Command received");
                        }
                        else if (memcmp(MQTTCommand, "maximumCurrentSetting", 21) == 0)
                        {

                            maximumCurrentSetting_json = cJSON_GetObjectItem(mqtt_json, "maximumCurrentSetting");
                            if (!maximumCurrentSetting_json)
                            {
                                printf("\nnull json\n");
                            }
                            else
                            {
                                maximumCurrentSetting = atoi(maximumCurrentSetting_json->valuestring);
                                printf("got maximumCurrentSetting:%d\n", maximumCurrentSetting);
                                esp_err_t err_maximumCurrentSetting = save_int_value("mcs:", maximumCurrentSetting);
                                if (err_maximumCurrentSetting != ESP_OK)
                                {
                                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_maximumCurrentSetting));
                                }
                            }
                        }
                        else if (memcmp(MQTTCommand, "wifiReset", 9) == 0)
                        {
                            // setup flag is set to 1 if we need to start over and saved in NVS
                            wifi_config_state = 1;
                            ESP_LOGI("MQTT_CALLBACK", "wifiReset Command received");
                            esp_err_t err_wifi_config_state = save_int_value("wcstate:", wifi_config_state);
                            if (err_wifi_config_state != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_wifi_config_state));
                            }
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
                        else if (memcmp(MQTTCommand, "factoryReset", 13) == 0)
                        {
                            factoryResetOngoing = 1;

                            wifi_config_state = 1;
                            ESP_LOGI("MQTT_CALLBACK", "wifi_config_state_on Command received");
                            esp_err_t err_wifi_config_state = save_int_value("wcstate:", wifi_config_state);
                            if (err_wifi_config_state != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_wifi_config_state));
                            }
                            // setup flag is set to 1 if we need to start over and saved in NVS
                            ESP_LOGI(TAG_B, "NVS setup_flag reset");
                            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT_2);
                            setup_flag = 1;
                            esp_err_t err_setup = save_int_value("setup_flag:", setup_flag);

                            if (err_setup != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_setup));
                            }
                            isConfigState = 1;
                            esp_err_t err_config_state = save_int_value("isConfigState:", isConfigState);
                            if (err_config_state != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_config_state));
                            }
                            esp_err_t err_scheduleStartTimeWeekDays = save_key_value("sstwd:", "00:00");
                            if (err_scheduleStartTimeWeekDays != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekDays));
                            }
                            esp_err_t err_scheduleStopTimeWeekDays = save_key_value("ssttwd:", "00:00");
                            if (err_scheduleStopTimeWeekDays != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekDays));
                            }
                            esp_err_t err_scheduleStartTimeWeekEnds = save_key_value("sstwe:", "00:00");
                            if (err_scheduleStartTimeWeekEnds != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekEnds));
                            }
                            esp_err_t err_scheduleStopTimeWeekEnds = save_key_value("ssttwe:", "00:00");
                            if (err_scheduleStopTimeWeekEnds != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekEnds));
                            }

                            esp_err_t err_maximumCurrentSetting = save_int_value("mcs:", 7);
                            if (err_maximumCurrentSetting != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_maximumCurrentSetting));
                            }

                            vTaskDelay(10 / portTICK_PERIOD_MS);
                            isPaired = 0;
                            esp_err_t err_isPaired = save_int_value("isPaired:", isPaired);

                            if (err_isPaired != ESP_OK)
                            {
#ifdef DEBUG
                                ESP_LOGE(TAG, "Error (%s) saving to NVS isPaired", esp_err_to_name(err_isPaired));
#endif
                            }
                            factoryResetHTTP();
                            factoryResetOngoing = 0;

                            esp_restart();
                        }
                        else if (memcmp(MQTTCommand, "scheduleChargeEnable", 20) == 0)
                        {
                            scheduleChargeEnableState = 1;
                            esp_err_t err_sce = save_int_value("sceState:", scheduleChargeEnableState);

                            if (err_sce != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_sce));
                            }
                        }

                        else if (memcmp(MQTTCommand, "scheduleChargeDisable", 21) == 0)
                        {
                            scheduleChargeEnableState = 0;
                            esp_err_t err_sce = save_int_value("sceState:", scheduleChargeEnableState);

                            if (err_sce != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_sce));
                            }
                        }
                        else if (memcmp(MQTTCommand, "loadBalancingEnable", 19) == 0)
                        {
                            loadBalancingEnableState = 1;
                            esp_err_t err_lbe = save_int_value("lbeState:", loadBalancingEnableState);

                            if (err_lbe != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_lbe));
                            }
                        }
                        else if (memcmp(MQTTCommand, "loadBalancingDisable", 21) == 0)
                        {
                            loadBalancingEnableState = 0;
                            esp_err_t err_lbd = save_int_value("lbeState:", loadBalancingEnableState);

                            if (err_lbd != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_lbd));
                            }
                        }
                        else if (memcmp(MQTTCommand, "scheduleUpdate", 14) == 0)
                        {
                            scheduleStartTimeWeekDays_json = cJSON_GetObjectItem(mqtt_json, "scheduleStartTimeWeekDays");
                            if (!scheduleStartTimeWeekDays_json)
                            {
                                printf("\nnull json\n");
                            }
                            else
                            {
                                strcpy(scheduleStartTimeWeekDays, scheduleStartTimeWeekDays_json->valuestring);
                                printf("got scheduleStartTimeWeekDays:%s\n", scheduleStartTimeWeekDays);
                                esp_err_t err_scheduleStartTimeWeekDays = save_key_value("sstwd:", scheduleStartTimeWeekDays);
                                if (err_scheduleStartTimeWeekDays != ESP_OK)
                                {
                                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekDays));
                                }
                            }
                            scheduleStopTimeWeekDays_json = cJSON_GetObjectItem(mqtt_json, "scheduleStopTimeWeekDays");
                            if (!scheduleStopTimeWeekDays_json)
                            {
                                printf("\nnull json\n");
                            }
                            else
                            {
                                strcpy(scheduleStopTimeWeekDays, scheduleStopTimeWeekDays_json->valuestring);
                                printf("got scheduleStopTimeWeekDays:%s\n", scheduleStopTimeWeekDays);
                                esp_err_t err_scheduleStopTimeWeekDays = save_key_value("ssttwd:", scheduleStopTimeWeekDays);
                                if (err_scheduleStopTimeWeekDays != ESP_OK)
                                {
                                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekDays));
                                }
                            }

                            scheduleStartTimeWeekEnds_json = cJSON_GetObjectItem(mqtt_json, "scheduleStartTimeWeekEnds");
                            if (!scheduleStartTimeWeekEnds_json)
                            {
                                printf("\nnull json\n");
                            }
                            else
                            {
                                strcpy(scheduleStartTimeWeekEnds, scheduleStartTimeWeekEnds_json->valuestring);
                                printf("got scheduleStartTimeWeekEnds:%s\n", scheduleStartTimeWeekEnds);
                                esp_err_t err_scheduleStartTimeWeekEnds = save_key_value("sstwe:", scheduleStartTimeWeekEnds);
                                if (err_scheduleStartTimeWeekEnds != ESP_OK)
                                {
                                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStartTimeWeekEnds));
                                }
                            }
                            scheduleStopTimeWeekEnds_json = cJSON_GetObjectItem(mqtt_json, "scheduleStopTimeWeekEnds");
                            if (!scheduleStopTimeWeekEnds_json)
                            {
                                printf("\nnull json\n");
                            }
                            else
                            {
                                strcpy(scheduleStopTimeWeekEnds, scheduleStopTimeWeekEnds_json->valuestring);
                                printf("got scheduleStopTimeWeekEnds:%s\n", scheduleStopTimeWeekEnds);
                                esp_err_t err_scheduleStopTimeWeekEnds = save_key_value("ssttwe:", scheduleStopTimeWeekEnds);
                                if (err_scheduleStopTimeWeekEnds != ESP_OK)
                                {
                                    ESP_LOGE(TAG, "Error (%s) saving to NVS", esp_err_to_name(err_scheduleStopTimeWeekEnds));
                                }
                            }
                        }
                        else if (memcmp(MQTTCommand, "led_off_command", 15) == 0)
                        {
                            led_off_command_remote = 1;
                            esp_err_t err_led = save_int_value("led:", led_off_command_remote);

                            if (err_led != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_led));
                            }
                        }
                        else if (memcmp(MQTTCommand, "led_on_command", 14) == 0)
                        {
                            led_off_command_remote = 0;
                            esp_err_t err_led = save_int_value("led:", led_off_command_remote);

                            if (err_led != ESP_OK)
                            {
                                ESP_LOGE(TAG_B, "Error (%s) saving to NVS", esp_err_to_name(err_led));
                            }
                        }
                    }
                }
                cJSON_Delete(mqtt_json); // Free memory
            }

            else if (check_topic(topic_buff))
            {
                // Get Message here
                uint8_t *message_buff = (uint8_t *)event->data;

                ESP_LOGI(TAG, "Recv int: %d %d %d %d %d %d %d %d %d %d %d %d %d", message_buff[0], message_buff[1], message_buff[2], message_buff[3], message_buff[4], message_buff[5], message_buff[6], message_buff[7], message_buff[8], message_buff[9], message_buff[10], message_buff[11], message_buff[12]);
                ESP_LOGI(TAG, "CRC check : %d", check_crc(FRAME_SIZE, message_buff));

                if (check_crc(FRAME_SIZE, message_buff))
                {
                    process_message(message_buff, mqtt_client);
                }
                else
                {
                    if (message_buff[0] == 64)
                    {
                        process_message(message_buff, mqtt_client);
                    }
                    ESP_LOGW(TAG, "CRC check failed");
                }
            }
            /*else if (check_topic_mac(topic_buff))
            {

                ESP_LOGW("topic comparision", "succesfull");
            }*/

            else
            {
                ESP_LOGE("topic comparision", "unsuccesfull");
            }
            break;
        }
        default:
            ESP_LOGI(TAG1, "Other event id:%d", event->event_id);
            break;
        }
        return ESP_OK;
    }

    static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
    {
        ESP_LOGD("mqtt_event_handler", "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
        mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
    }

    static void mqtt_app_start(void)
    {
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT_2, false, true, portMAX_DELAY);
        ESP_LOGI("mqtt_app_start", "mqtt started.");
        esp_mqtt_client_config_t mqtt_cfg = {
            // .uri = "mqtt://test.mosquitto.org",
            .uri = "mqtt://ec2-52-89-241-125.us-west-2.compute.amazonaws.com:1884",
            //  .client_id="espnext01"
            //  .task_stack=1024*8,
            .buffer_size = 1024,
        };
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
        esp_mqtt_client_start(mqtt_client);
    }

    void sendHeartBeatMQTT()
    {
        ESP_LOGI("sendHeartBeatMQTT", "sendHeartBeatMQTT");

        /* if (isStatusEqualToStop == true)
         {
             printf("isStatusEqualToStop == true");
             timeDiff = timeWhenStopping;
             energySend = energyWhenStopping;
             if (TimeBased == true)
             {
                 printf("2");
                 costCalc = (timeDiff / 60) * (chargeHourlyAmount_IN_NetWork / 60) * doubleCharge;
             }
             else
             {
                 printf("3");
                 costCalc = energySend * energyRate;
             }
             isStatusEqualToStop = false;
         }*/
        cJSON *json_ob1;
        json_ob1 = cJSON_CreateObject();
        char *ob1, *ob1File;
        // if (userID != NULL || memcmp(userID, "", 1) != 0)
        // {
        printf("4");
        cJSON_AddStringToObject(json_ob1, "command", "heartBeat");
        char current_state[10];
        printf("5");
        sprintf(current_state, "%d", CURRENT_STATE);

        cJSON_AddStringToObject(json_ob1, "state", (const char *)current_state);
        char I1[10];
        printf("6");
        sprintf(I1, "%.2f", currentL1);
        cJSON_AddStringToObject(json_ob1, "I1", (const char *)I1);

        char I2[10];
        printf("6");
        sprintf(I2, "%.2f", currentL2);
        cJSON_AddStringToObject(json_ob1, "I2", (const char *)I2);

        char I3[10];
        printf("6");
        sprintf(I3, "%.2f", currentL3);
        cJSON_AddStringToObject(json_ob1, "I3", (const char *)I3);

        char v1[10];
        printf("7");
        sprintf(v1, "%.2f", voltageL1);
        cJSON_AddStringToObject(json_ob1, "v1", (const char *)v1);

        char v2[10];
        printf("7");
        sprintf(v2, "%.2f", voltageL2);
        cJSON_AddStringToObject(json_ob1, "v2", (const char *)v2);

        char v3[10];
        printf("7");
        sprintf(v3, "%.2f", voltageL3);
        cJSON_AddStringToObject(json_ob1, "v3", (const char *)v3);

        char firmwareVersion_chr[10];
        sprintf(firmwareVersion_chr, "%d", firmwareVersion);
        cJSON_AddStringToObject(json_ob1, "fv", (const char *)firmwareVersion_chr);
        printf("9");

        char temp1_chr[10];
        sprintf(temp1_chr, "%d", temp1);
        cJSON_AddStringToObject(json_ob1, "temp1", (const char *)temp1_chr);
        printf("9");

        printf("8");
        char power_chr[10];
        sprintf(power_chr, "%.2f", power);
        cJSON_AddStringToObject(json_ob1, "power", (const char *)power_chr);
        printf("9");

        char time_Diff_chr[10];
        sprintf(time_Diff_chr, "%d", timeDiffTask1 / 60);
        cJSON_AddStringToObject(json_ob1, "chargeDuration", (const char *)time_Diff_chr);
        printf("10");
        char energy_chr[10];
        sprintf(energy_chr, "%.2f", energy);
        cJSON_AddStringToObject(json_ob1, "energy", (const char *)energy_chr);
        printf("11");
        char chargerLockState_chr[10];
        sprintf(chargerLockState_chr, "%d", isChargerLocked);
        cJSON_AddStringToObject(json_ob1, "chargerLockState", (const char *)chargerLockState_chr);
        printf("12");
        char faultState_chr[10];
        sprintf(faultState_chr, "%d", recevedSerial.control_Side_Error.all);
        cJSON_AddStringToObject(json_ob1, "faultStateControlSide", (const char *)faultState_chr);
        printf("13");

        char faultStatePowerSide_chr[10];
        sprintf(faultStatePowerSide_chr, "%d", recevedSerial.power_Side_Error.all);
        cJSON_AddStringToObject(json_ob1, "faultStatePowerSide", (const char *)faultStatePowerSide_chr);
        printf("14");

        char loadBalancingEnableState_chr[10];
        sprintf(loadBalancingEnableState_chr, "%d", loadBalancingEnableState);
        ESP_LOGW("heartBeat", "loadBalancingEnableState:%d", loadBalancingEnableState);
        cJSON_AddStringToObject(json_ob1, "loadBalancingEnableState", (const char *)loadBalancingEnableState_chr);
        printf("15");

        char scheduleChargeEnableState_chr[10];
        sprintf(scheduleChargeEnableState_chr, "%d", scheduleChargeEnableState);
        cJSON_AddStringToObject(json_ob1, "scheduleChargeEnableState", (const char *)scheduleChargeEnableState_chr);
        printf("16");

        char controlSideState_chr[10];
        sprintf(controlSideState_chr, "%d", controlSideState);
        cJSON_AddStringToObject(json_ob1, "connectorState", (const char *)controlSideState_chr);
        printf("17");
        cJSON_AddStringToObject(json_ob1, "userId", (const char *)userId);
        printf("18");

        char runningAppInformation[10];
        runningAppInfo();
        printf("21");
        vTaskDelay(pdMS_TO_TICKS(10));
        sprintf(runningAppInformation, "%s", app_version);
        vTaskDelay(pdMS_TO_TICKS(10));
        printf("22");
        cJSON_AddStringToObject(json_ob1, "runningAppVersion", (const char *)runningAppInformation);
        printf("16");

        wifi_ap_record_t ap;
        esp_wifi_sta_get_ap_info(&ap);
        // printf("wifiStrength:%d\n", ap.rssi);

        char wifiStrength_chr[10];
        sprintf(wifiStrength_chr, "%d", ap.rssi);
        cJSON_AddStringToObject(json_ob1, "wifiStrength", (const char *)wifiStrength_chr);
        printf("18");

        char maximumCurrentSetting_chr[10];
        sprintf(maximumCurrentSetting_chr, "%d", maximumCurrentSetting);
        cJSON_AddStringToObject(json_ob1, "maxCurrent", (const char *)maximumCurrentSetting_chr);
        printf("19");

        cJSON_AddStringToObject(json_ob1, "ipAddress", (const char *)ipAddress);
        printf("20");
        cJSON_AddStringToObject(json_ob1, "mac_address", (const char *)mac_address_base);

        printf("21");
        if (alarmUpdateCompleteFlag == 0)
        {
            cJSON_AddStringToObject(json_ob1, "scheduleStartTimeWeekDays", (const char *)scheduleStartTimeWeekDays);
            cJSON_AddStringToObject(json_ob1, "scheduleStopTimeWeekDays", (const char *)scheduleStopTimeWeekDays);
            cJSON_AddStringToObject(json_ob1, "scheduleStartTimeWeekEnds", (const char *)scheduleStartTimeWeekEnds);
            cJSON_AddStringToObject(json_ob1, "scheduleStopTimeWeekEnds", (const char *)scheduleStopTimeWeekEnds);
        }

        ob1 = cJSON_Print(json_ob1); //  take json Convert form to string
        printf(" 22");
        memmove(heartBeatObj, ob1, sizeof(heartBeatObj));
        printf(" 23");
        ob1File = cJSON_Print(json_ob1); //  take json Convert form to string
        printf(" %d ", sizeof(ob1File));
        //}

        printf(" 24");
        /* if(xSemaphoreTake(mqttMutex, 2)==pdTRUE){
           sendMqtt = true;
           xSemaphoreGive(mqttMutex);
           printf(" 20");
         }*/

#ifdef DEBUG
        ESP_LOGW("MQTT", "publishPre %s", mqtt_pub_topic);
#endif
        if (ota_enabled_flag == false)
        {
            esp_mqtt_client_publish(mqtt_client, mqtt_pub_topic, (const char *)heartBeatObj, 0, 1, 0);
            ESP_LOGI("mqtt_client", "mqtt_pub_topic:%s", mqtt_pub_topic);
        }
#ifdef DEBUG
        ESP_LOGW("MQTT", "publishPost");
#endif

        free(ob1);
        free(ob1File);
        cJSON_Delete(json_ob1);
#ifdef BUG
        printf("%s\n", heartBeatObj);
        printf("%s\n", ob1File);
#endif
        printf(" 22\n");
        // eepromStat = writeFile(ob1File, FILE_1);
        if (eepromStat != 1)
        {
#ifdef DEBUG
            ESP_LOGI("sendHeartBeatMQTT", "MQTT_HEARTBEAT:EEPROM write file problem");
#endif
        }
    }

    /* void sendDeepDiveData()
     {
         ESP_LOGI("sendDeepDiveData", "sendDeepDiveData");

         cJSON *json_ob1;
         json_ob1 = cJSON_CreateObject();
         char *ob1;

         printf("4");
         cJSON_AddStringToObject(json_ob1, "chargePointReference", (const char *)NODEID);

         char start_chr[4];
         sprintf(start_chr, "%d", CURRENT_STATE);
         cJSON_AddStringToObject(json_ob1, "power_status", (const char *)start_chr);

         // char count_chr[10];
         // printf("5");
         // sprintf(count_chr,"%d",ticker_count);

         cJSON_AddStringToObject(json_ob1, "fault_description", "[]");
         cJSON_AddStringToObject(json_ob1, "max_temp", "0");

         char timeDiff_chr[25];
         printf("6");
         sprintf(timeDiff_chr, "%.2f", timeDiff);
         cJSON_AddStringToObject(json_ob1, "percentage", (const char *)timeDiff_chr);

         cJSON_AddStringToObject(json_ob1, "started_percentage", "0");

         printf("7");
         char charging_option_chr[4];
         sprintf(charging_option_chr, "%d", stopCondition);
         printf(" 13");
         cJSON_AddStringToObject(json_ob1, "charging_option", (const char *)charging_option_chr);

         cJSON_AddStringToObject(json_ob1, "current_amperage", "");
         cJSON_AddStringToObject(json_ob1, "current_voltage", "");
         cJSON_AddStringToObject(json_ob1, "ctovStates", "0");
         cJSON_AddStringToObject(json_ob1, "vtocfaults", "0");
         cJSON_AddStringToObject(json_ob1, "vtocStates", "0");
        // cJSON_AddStringToObject(json_ob1, "transactionId", (const char *)transactionID);
         printf(" 11");
          char timeWhenStopping_chr[25];
          sprintf(timeWhenStopping_chr, "%.2f", timeWhenStopping);
          cJSON_AddStringToObject(json_ob1, "timeWhenStopping", (const char *)timeWhenStopping_chr);

         printf("9");
        char saved_energySend_chr[25];
         sprintf(saved_energySend_chr, "%.2f", energySend);
         cJSON_AddStringToObject(json_ob1, "energyCalc", (const char *)saved_energySend_chr);
         printf(" 10");*/
    /*char costCalc_chr[25];
    if (TimeBased == true)
    {

        //    sprintf(costCalc_chr, "%.2f", (timeWhenStopping / 60000) * (chargeHourlyAmount_IN_NetWork / 60));
    }
    else
    {
        sprintf(costCalc_chr, "%.2f", energySend * energyRate);
    }
    cJSON_AddStringToObject(json_ob1, "costCalc", (const char *)costCalc_chr);
    printf(" 12");
    char userBalance_chr[25];
    sprintf(userBalance_chr, "%.2f", balanceCalc);
    cJSON_AddStringToObject(json_ob1, "balance", userBalance_chr);

    ob1 = cJSON_Print(json_ob1); //  take json Convert form to string

    printf(" 19");
    char topic[15] = "/deepdive";
    memmove(topic + 9, NODEID, sizeof(NODEID));
    esp_mqtt_client_publish(mqtt_client, topic, (const char *)ob1, 0, 1, 0);

    #ifdef BUG
    printf("%s\n", ob1);

    #endif

    free(ob1);
    cJSON_Delete(json_ob1);
    printf(" 21\n");
    }*/

    /*void sendFullDeepDiveData()
    {
        ESP_LOGI("sendFullDeepDiveData", "sendFullDeepDiveData");

        cJSON *json_ob1;
        json_ob1 = cJSON_CreateObject();
        char *ob1;

        printf("4");
        cJSON_AddStringToObject(json_ob1, "chargePointReference", (const char *)NODEID);

        cJSON_AddStringToObject(json_ob1, "temp1", "0");
        cJSON_AddStringToObject(json_ob1, "temp2", "0");
        cJSON_AddStringToObject(json_ob1, "temp3", "0");
        cJSON_AddStringToObject(json_ob1, "temp4", "0");
        cJSON_AddStringToObject(json_ob1, "faults", "0");
        cJSON_AddStringToObject(json_ob1, "ram_usage", "0");
        cJSON_AddStringToObject(json_ob1, "ram_usage", "0");
        cJSON_AddStringToObject(json_ob1, "cpu_usage", "0");

        char start_chr[4];
        sprintf(start_chr, "%d", CURRENT_STATE);
        cJSON_AddStringToObject(json_ob1, "power_status", (const char *)start_chr);

        // char count_chr[10];
        // printf("5");
        // sprintf(count_chr,"%d",ticker_count);

        cJSON_AddStringToObject(json_ob1, "fault_description", "0");
        cJSON_AddStringToObject(json_ob1, "disk_space", "0");

        char sha1S[41] = {'\0'};
        shaCalculate(NODEID, sha1S, sizeof(NODEID) - 1);
        cJSON_AddStringToObject(json_ob1, "token", (const char *)sha1S);
        ob1 = cJSON_Print(json_ob1); //  take json Convert form to string

        printf(" 19");
        char topic[20] = "/fulldeepdive";
        memmove(topic + 13, NODEID, sizeof(NODEID));
        esp_mqtt_client_publish(mqtt_client, topic, (const char *)ob1, 0, 1, 0);

    #ifdef DEBUG
        printf("%s\n", ob1);

    #endif

        free(ob1);
        cJSON_Delete(json_ob1);
        printf(" 21\n");
    }*/

    /*   void stoptTransactionHTTP_task0()
       {

    #ifdef DEBUG
           const char *TAG = "stoptTransactionHTTP_task0";
           ESP_LOGI(TAG, " started");

           // int checkInternet = 0;
           if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
           {
               if (is_internet_available == true)
               {
                   xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                   ESP_LOGI(TAG, "Online!!");
    #endif
                   // checkInternet = 1;
               }
               else
               {
                   xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                   ESP_LOGI(TAG, "offline :(");
    #endif
               }
           }
    #endif
           // xQueueSend(checkInternetQueue,(void*)&checkInternet, portMAX_DELAY);
           // ESP_LOGI(TAG,"qsendChkNet0******");
           // add check internet function here
           if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
           {
               if (is_internet_available == true)
               {
                   xSemaphoreGive(internetMutex);
                   timeWhenStopping = timeDiff;
                   energyWhenStopping = energySend;
                   costCalc = (timeDiff) * (chargeHourlyAmount_IN_NetWork)*doubleCharge / 3600;
                   isStatusEqualToStop = true;

                   char sha1String[41];
                   sha1String[40] = '\0';
                   // char transactionID[17]="1234567890123";
                   shaCalculate(transactionID, sha1String, strlen(transactionID));

    #ifdef DBUG
                   ESP_LOGI(TAG, "%s", sha1String);
    #endif
                   char local_response_buffer[500];
                   esp_http_client_config_t config = {
                       .url = stopUrl,
                       .query = "esp",
                       .disable_auto_redirect = false,
                       .max_redirection_count = 2,
                       .event_handler = _http_event_handler,
                       .user_data = local_response_buffer, // Pass address of local buffer to get response

                       //.crt_bundle_attach = esp_crt_bundle_attach,
                   };
                   esp_http_client_handle_t client = esp_http_client_init(&config);
                   esp_http_client_set_url(client, stopUrl);
                   esp_http_client_set_method(client, HTTP_METHOD_POST);
                   // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
                   esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
                   esp_http_client_set_header(client, "User-Agent", "Super Agent/0.0.1");
                   // String httpRequestData = "userReferenceID=" + String(userRefID) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
                   char httpRequestData[204 + NODEID_LEN] = "userReferenceID=";
                   strcat(httpRequestData, userRefID);

                   // printf("%s\n",httpRequestData);
                   strcat(httpRequestData, "&chargePointReferenceID=");
                   strcat(httpRequestData, NODEID);

                   // printf("%s\n",httpRequestData);
                   strcat(httpRequestData, "&transactionID=");
                   strcat(httpRequestData, transactionID);

                   // printf("%s\n",httpRequestData);
                   strcat(httpRequestData, "&energyString=");
                   char energyWhenStopping_chr[25];
                   sprintf(energyWhenStopping_chr, "%.2f", energyWhenStopping);
                   strcat(httpRequestData, energyWhenStopping_chr);

                   // printf("%s\n",httpRequestData);
                   strcat(httpRequestData, "&timeString=");
                   char timeWhenStopping_chr[25];
                   sprintf(timeWhenStopping_chr, "%.2f", timeWhenStopping / 60.0);
                   strcat(httpRequestData, timeWhenStopping_chr);

                   // printf("%s\n",httpRequestData);
                   strcat(httpRequestData, "&token=");
                   memmove(httpRequestData + strlen(httpRequestData), sha1String, 40);

    #ifdef DEBUG
                   printf("%s\n", httpRequestData);
    #endif
                   esp_http_client_set_post_field(client, httpRequestData, strlen(httpRequestData));
                   esp_err_t err = esp_http_client_perform(client);
                   if (err == ESP_OK)
                   {
                       ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
                       printf("%s\n", local_response_buffer);
                   }
                   else
                   {
                       ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                   }
                   int HTTPCode = 0;
                   if (esp_http_client_get_status_code(client) == HTTP_CODE_OK)
                   {
                       HTTPCode = 1;
                       jsonOutReady = 1;
                       // StaticJsonDocument<1024> doc;
                       cJSON *task_json, *status_json;
                       task_json = cJSON_Parse(local_response_buffer);
                       if (!task_json)
                       {
                           printf("\nnull task_json\n");
                       }
                       else
                       {
                           status_json = cJSON_GetObjectItem(task_json, "status");
                           if (!status_json)
                           {
                               printf("\nnull json\n");
                           }
                           else
                           {
                               if (status_json->valuestring != NULL)
                               {
                                   sprintf(startStatus, status_json->valuestring);
                                   sprintf(started, startStatus);
                                   ESP_LOGI(TAG, "started = %s", started);
                               }
                               cJSON *balance_json;
                               balance_json = cJSON_GetObjectItem(task_json, "balance");
                               accountBalance = (float)atof(balance_json->valuestring);

                               // cJSON *balance_json,*transac_id_json,*user_id_json,*user_ref_json,*charge_point_ref_json,*username_json,*success_json,*message_json,*error_msg_json;
                               // balance_json=cJSON_GetObjectItem(get_en_json, "balance");
                               // energyRate=(float)atof(balance_json->valuestring);

                               // transac_id_json=cJSON_GetObjectItem(get_en_json, "transactionID");
                               // sprintf(transactionID,transac_id_json->valuestring);

                               // user_id_json=cJSON_GetObjectItem(get_en_json, "userID");
                               // sprintf(userID,user_id_json->valuestring);

                               // user_ref_json=cJSON_GetObjectItem(get_en_json, "userReference");
                               // sprintf(userReference,user_ref_json->valuestring);

                               // charge_point_ref_json=cJSON_GetObjectItem(get_en_json, "chargePointReference");
                               // sprintf(chargePointReference,charge_point_ref_json->valuestring);

                               // username_json=cJSON_GetObjectItem(get_en_json, "username");
                               // sprintf(username,charge_point_ref_json->valuestring);

                               // success_json=cJSON_GetObjectItem(get_en_json, "success");
                               // sprintf(success,success_json->valuestring);

                               // message_json=cJSON_GetObjectItem(get_en_json, "message");
                               // sprintf(message,message_json->valuestring);

                               // error_msg_json=cJSON_GetObjectItem(get_en_json, "errorMessage");
                               // sprintf(errorMessage,error_msg_json->valuestring);
                           }
                       }

                       cJSON_Delete(task_json); // Free memory

    #ifdef DEBUG
                       //     ESP_LOGI(TAG,started);
                       //      //ESP_LOGI(TAG,transactionID);
                       //     ESP_LOGI(TAG,userReference);
                       //     ESP_LOGI(TAG,chargePointReference);
                       //     ESP_LOGI(TAG, accountBalance );
                       //     ESP_LOGI(TAG,username);
                       //      //        if (userID = "null") {
                       //      //          Serial.print("this is a string");
                       //      //        }
                       //     ESP_LOGI(TAG,userID);
                       //     ESP_LOGI(TAG,success);
                       //     ESP_LOGI(TAG,message);
                       //     ESP_LOGI(TAG,errorMessage);
    #endif
                   }

                   esp_http_client_cleanup(client);
                   xQueueSend(HTTPCodeQueue, (void *)&HTTPCode, portMAX_DELAY);
                   ESP_LOGI(TAG, "send HTTPcode0 to HTTPCodeQueue ******");
               }
               else
               {
                   xSemaphoreGive(internetMutex);
               }
           }
       }*/

    /* void energyBasedStopTransaction_task0()
     {

    #ifdef DEBUG
         const char *TAG = "energyBasedStopTransaction_task0";
         ESP_LOGI(TAG, " started");

         //    int checkInternet = 0;
         if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
         {
             if (is_internet_available)
             {
                 xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                 ESP_LOGI(TAG, "Online!!");
    #endif
                 // checkInternet = 1;
             }
             else
             {
                 xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                 ESP_LOGI(TAG, "offline :(");
    #endif
             }
         }
    #endif
         // ESP_LOGI(TAG,"xQueueSend(checkInternetQueueEnergy0");
         // xQueueSend(checkInternetQueueEnergy, (void*)&checkInternet, portMAX_DELAY);
         // ESP_LOGI(TAG,"xQueueSend(checkInternetQueueEnergy1");
         // add check internet function here
         if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
         {
             if (is_internet_available == true)
             {
                 xSemaphoreGive(internetMutex);
                 timeWhenStopping = timeDiff;
                 energyWhenStopping = energySend;
                 costCalc = timeWhenStopping * chargeHourlyAmount_IN_NetWork * doubleCharge / 3600;
                 isStatusEqualToStop = true;

                 char sha1String[41];
                 sha1String[40] = '\0';
                 // char transactionID[17]="0.1.0.255.0.4.91.0.C0090";
                 shaCalculate(transactionID, sha1String, strlen(transactionID));

    #ifdef DEBUG
                 ESP_LOGI(TAG, "%s", sha1String);
    #endif
                 char local_response_buffer[500];
                 esp_http_client_config_t config = {
                     .url = energyBasedStopUrl,
                     .query = "esp",
                     .disable_auto_redirect = false,
                     .max_redirection_count = 2,
                     .event_handler = _http_event_handler,
                     .user_data = local_response_buffer, // Pass address of local buffer to get response

                     //.crt_bundle_attach = esp_crt_bundle_attach,
                 };
                 esp_http_client_handle_t client = esp_http_client_init(&config);
                 esp_http_client_set_url(client, energyBasedStopUrl);
                 esp_http_client_set_method(client, HTTP_METHOD_POST);
                 // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
                 esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
                 esp_http_client_set_header(client, "User-Agent", "Super Agent/0.0.1");
                 // String httpRequestData = "userReferenceID=" + String(userRefID) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
                 char httpRequestData[204 + NODEID_LEN] = "userReferenceID=";
                 strcat(httpRequestData, userRefID);

                 strcat(httpRequestData, "&chargePointReferenceID=");
                 strcat(httpRequestData, NODEID);

                 // printf("%s\n",httpRequestData);
                 strcat(httpRequestData, "&transactionID=");
                 strcat(httpRequestData, transactionID);

                 // printf("%s\n",httpRequestData);
                 char energyWhenStopping_chr[25];
                 sprintf(energyWhenStopping_chr, "%.2f", energyWhenStopping);
                 strcat(httpRequestData, "&energyString=");
                 strcat(httpRequestData, energyWhenStopping_chr);

                 // printf("%s\n",httpRequestData);
                 char timeWhenStopping_chr[25];
                 sprintf(timeWhenStopping_chr, "%.2f", timeWhenStopping / 60.0);
                 strcat(httpRequestData, "&timeString=");
                 strcat(httpRequestData, timeWhenStopping_chr);

                 // printf("%s\n",httpRequestData);
                 strcat(httpRequestData, "&token=");
                 memmove(httpRequestData + strlen(httpRequestData), sha1String, 40);

    #ifdef DEBUG
                 printf("%s\n", httpRequestData);
    #endif
                 esp_http_client_set_post_field(client, httpRequestData, strlen(httpRequestData));
                 // ESP_LOGI(TAG,"vishva oshada");
                 esp_err_t err = esp_http_client_perform(client);
                 if (err == ESP_OK)
                 {
                     ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
                     printf("%s\n", local_response_buffer);
                 }
                 else
                 {
                     ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                 }
                 int HTTPCode = 0;
                 if (esp_http_client_get_status_code(client) == HTTP_CODE_OK)
                 {
                     HTTPCode = 1;
                     // jsonOutReady = 1;
                     // StaticJsonDocument<1024> doc;
                     cJSON *task_json, *status_json;
                     // ESP_LOGI(TAG,"vishva oshada");
                     task_json = cJSON_Parse(local_response_buffer);
                     // ESP_LOGI(TAG,"vishva oshada");
                     if (!task_json)
                     {
                         printf("\nnull task_json\n");
                     }
                     else
                     {
                         status_json = cJSON_GetObjectItem(task_json, "status");
                         if (!status_json)
                         {
                             printf("\nnull json\n");
                         }
                         else
                         {
                             ESP_LOGI(TAG, "vishva oshada");
                             if (status_json->valuestring != NULL)
                             {
                                 sprintf(startStatus, status_json->valuestring);
                                 sprintf(started, startStatus);
                             }
                             ESP_LOGI(TAG, "started = %s", started);
                             // cJSON *balance_json,*transac_id_json,*user_id_json,*user_ref_json,*charge_point_ref_json,*username_json,*success_json,*message_json,*error_msg_json;
                             cJSON *balance_json;
                             balance_json = cJSON_GetObjectItem(task_json, "balance");
                             accountBalance = (float)atof(balance_json->valuestring);

                             // transac_id_json=cJSON_GetObjectItem(get_en_json, "transactionID");
                             // sprintf(transactionID,transac_id_json->valuestring);

                             // user_id_json=cJSON_GetObjectItem(get_en_json, "userID");
                             // sprintf(userID,user_id_json->valuestring);

                             // user_ref_json=cJSON_GetObjectItem(get_en_json, "userReference");
                             // sprintf(userReference,user_ref_json->valuestring);

                             // charge_point_ref_json=cJSON_GetObjectItem(get_en_json, "chargePointReference");
                             // sprintf(chargePointReference,charge_point_ref_json->valuestring);

                             // username_json=cJSON_GetObjectItem(get_en_json, "username");
                             // sprintf(username,charge_point_ref_json->valuestring);

                             // success_json=cJSON_GetObjectItem(get_en_json, "success");
                             // sprintf(success,success_json->valuestring);

                             // message_json=cJSON_GetObjectItem(get_en_json, "message");
                             // sprintf(message,message_json->valuestring);

                             // error_msg_json=cJSON_GetObjectItem(get_en_json, "errorMessage");
                             // sprintf(errorMessage,error_msg_json->valuestring);
                         }
                     }

                     cJSON_Delete(task_json); // Free memory

    #ifdef DEBUG
                     //     ESP_LOGI(TAG,started);
                     //      //ESP_LOGI(TAG,transactionID);
                     //     ESP_LOGI(TAG,userReference);
                     //     ESP_LOGI(TAG,chargePointReference);
                     //     ESP_LOGI(TAG, accountBalance );
                     //     ESP_LOGI(TAG,username);
                     //      //        if (userID = "null") {
                     //      //          Serial.print("this is a string");
                     //      //        }
                     //     ESP_LOGI(TAG,userID);
                     //     ESP_LOGI(TAG,success);
                     //     ESP_LOGI(TAG,message);
                     //     ESP_LOGI(TAG,errorMessage);
    #endif
                 }

                 esp_http_client_cleanup(client);
                 xQueueSend(HTTPCodeQueueEnergy, &HTTPCode, 0);
                 ESP_LOGI(TAG, "send HTTPcode0 to HTTPCodeQueueEnergy ******");
             }
             else
             {
                 xSemaphoreGive(internetMutex);
             }
         }
     }*/
    /*void stopTransactionPrevious_task0()
    {
        const char *TAG = "stopTransactionPrevious_task0";
    #ifdef DEBUG
        ESP_LOGI(TAG, " started");
    #endif
        // int checkInternet = 0;
        if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
        {
            if (is_internet_available == true)
            {
                xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                ESP_LOGI(TAG, "Online!!");
    #endif
                // checkInternet = true;
            }
            else
            {
                xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                ESP_LOGI(TAG, "offline :(");
    #endif
            }
        }
        // xQueueSend(checkInternetQueuePrev, (void*)&checkInternet, portMAX_DELAY);
        // ESP_LOGI(TAG,"qsendChkNet0******");
        // add check internet function here
        if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
        {
            if (is_internet_available == true)
            {
                xSemaphoreGive(internetMutex);
                timeWhenStopping = timeDiff * 60;
                // energyWhenStopping = energySend;
                // costCalc = (timeDiff * 16.66) * (chargeHourlyAmount_IN_NetWork * 16.66) * doubleCharge / 1000000;
                // isStatusEqualToStop = true;

                char sha1String[41];
                sha1String[40] = '\0';
                // char transactionIDchrPrev[17]="1234567890123";
                shaCalculate(transactionIDchrPrev, sha1String, strlen(transactionIDchrPrev));

    #ifdef DEBUG
                ESP_LOGI(TAG, "%s", sha1String);
    #endif
                char local_response_buffer[1000];
                esp_http_client_config_t config = {
                    .url = stopPrevUrl,
                    .query = "esp",
                    .disable_auto_redirect = false,
                    .max_redirection_count = 2,
                    .event_handler = _http_event_handler,
                    .user_data = local_response_buffer, // Pass address of local buffer to get response

                    //.crt_bundle_attach = esp_crt_bundle_attach,
                };
                esp_http_client_handle_t client = esp_http_client_init(&config);
                esp_http_client_set_url(client, stopPrevUrl);
                esp_http_client_set_method(client, HTTP_METHOD_POST);
                // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
                esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
                esp_http_client_set_header(client, "User-Agent", "Super Agent/0.0.1");
                // String httpRequestData = "userReferenceID=" + String(userRefID) + "&chargePointReferenceID=" + String(NODEID) + "&token=" + String(sha1String);
                char httpRequestData[204 + NODEID_LEN] = "userReferenceID=";
                strcat(httpRequestData, userReferenceIDPrev);

                // printf("%s\n",httpRequestData);
                strcat(httpRequestData, "&chargePointReferenceID=");
                strcat(httpRequestData, NODEID);

                // printf("%s\n",httpRequestData);
                strcat(httpRequestData, "&transactionID=");
                strcat(httpRequestData, transactionIDchrPrev);

                // printf("%s\n",httpRequestData);
                strcat(httpRequestData, "&energyString=");
                char energyWhenStopping_chr[25];
                sprintf(energyWhenStopping_chr, "%.2f", energySendPrev);
                strcat(httpRequestData, energyWhenStopping_chr);

                // printf("%s\n",httpRequestData);
                strcat(httpRequestData, "&timeString=");
                char timeWhenStopping_chr[25];
                sprintf(timeWhenStopping_chr, "%.2f", timeDiffPrev);
                strcat(httpRequestData, timeWhenStopping_chr);

                strcat(httpRequestData, "&token=");
                memmove(httpRequestData + strlen(httpRequestData), sha1String, 40);
                // printf("%s\n",httpRequestData);

    #ifdef DEBUG
                printf("%s\n", httpRequestData);
    #endif
                esp_http_client_set_post_field(client, httpRequestData, strlen(httpRequestData));
                esp_err_t err = esp_http_client_perform(client);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
                    printf("%s\n", local_response_buffer);
                }
                else
                {
                    ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
                }
                int HTTPCode = 0;
                if (esp_http_client_get_status_code(client) == HTTP_CODE_OK)
                {
                    HTTPCode = 1;
                    // jsonOutReady = 1;
                    // StaticJsonDocument<1024> doc;
                    cJSON *task_json, *status_json;
                    task_json = cJSON_Parse(local_response_buffer);
                    if (!task_json)
                    {
                        printf("\nnull task_json\n");
                    }
                    else
                    {
                        status_json = cJSON_GetObjectItem(task_json, "status");
                        if (!status_json)
                        {
                            printf("\nnull json\n");
                        }
                        else
                        {
                            // sprintf(startStatus,task_json->valuestring);
                            // sprintf(started,startStatus);
                            if (status_json->valuestring != NULL)
                            {
                                ESP_LOGI(TAG, "started = %s", status_json->valuestring);
                            }

                            // cJSON *balance_json,*transac_id_json,*user_id_json,*user_ref_json,*charge_point_ref_json,*username_json,*success_json,*message_json,*error_msg_json;
                            // balance_json=cJSON_GetObjectItem(get_en_json, "balance");
                            // energyRate=(float)atof(balance_json->valuestring);

                            // transac_id_json=cJSON_GetObjectItem(get_en_json, "transactionID");
                            // sprintf(transactionID,transac_id_json->valuestring);

                            // user_id_json=cJSON_GetObjectItem(get_en_json, "userID");
                            // sprintf(userID,user_id_json->valuestring);

                            // user_ref_json=cJSON_GetObjectItem(get_en_json, "userReference");
                            // sprintf(userReference,user_ref_json->valuestring);

                            // charge_point_ref_json=cJSON_GetObjectItem(get_en_json, "chargePointReference");
                            // sprintf(chargePointReference,charge_point_ref_json->valuestring);

                            // username_json=cJSON_GetObjectItem(get_en_json, "username");
                            // sprintf(username,charge_point_ref_json->valuestring);

                            // success_json=cJSON_GetObjectItem(get_en_json, "success");
                            // sprintf(success,success_json->valuestring);

                            // message_json=cJSON_GetObjectItem(get_en_json, "message");
                            // sprintf(message,message_json->valuestring);

                            // error_msg_json=cJSON_GetObjectItem(get_en_json, "errorMessage");
                            // sprintf(errorMessage,error_msg_json->valuestring);
                        }
                    }

                    cJSON_Delete(task_json); // Free memory

    #ifdef DEBUG
                    //     ESP_LOGI(TAG,started);
                    //      //ESP_LOGI(TAG,transactionID);
                    //     ESP_LOGI(TAG,userReference);
                    //     ESP_LOGI(TAG,chargePointReference);
                    //     ESP_LOGI(TAG, accountBalance );
                    //     ESP_LOGI(TAG,username);
                    //      //        if (userID = "null") {
                    //      //          Serial.print("this is a string");
                    //      //        }
                    //     ESP_LOGI(TAG,userID);
                    //     ESP_LOGI(TAG,success);
                    //     ESP_LOGI(TAG,message);
                    //     ESP_LOGI(TAG,errorMessage);
    #endif
                }

                esp_http_client_cleanup(client);
                xQueueSend(HTTPCodeQueuePrev, (void *)&HTTPCode, portMAX_DELAY);
                ESP_LOGI(TAG, "send HTTPcode0 to HTTPCodeQueuePrev ******");
            }
            else
            {
                xSemaphoreGive(internetMutex);
            }
        }
    }*/

    /*void stopTransactionPrevious()
    {
        const char *TAG = "stopTransactionPrevious";
    #ifdef DEBUG
        ESP_LOGI(TAG, "Stop transaction previous");
    #endif
        int stopTrx = true;
        xQueueSend(stopTransactionQueuePrev, &stopTrx, 0);

        //   int checkInternet = 0;
        //   xQueueReceive(checkInternetQueuePrev, &checkInternet, portMAX_DELAY);
        if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
        {
            if (is_internet_available == true)
            {
                xSemaphoreGive(internetMutex);
                int httpCode = 0;
                if (HTTPCodeQueuePrev != NULL)
                {
                    ESP_LOGW(TAG, "xQueueReceive(HTTPCodeQueueEnergy0");
                    xQueueReceive(HTTPCodeQueuePrev, &httpCode, portMAX_DELAY);
                    ESP_LOGW(TAG, " xQueueReceive(HTTPCodeQueueEnergy1");
                }
                else
                {
                    ESP_LOGE(TAG, "HTTPCodeQueuePrev==NULL");
                }
                if (httpCode == 1)
                {
                    // clearFile2(FILE_2);
                }
                else
                {
    #ifdef DEBUG
                    ESP_LOGI(TAG, "stop transaction previous failed - Unknown");
    #endif
                }
            }
            else
            {
                xSemaphoreGive(internetMutex);
    #ifdef DEBUG
                ESP_LOGI(TAG, "stop transaction previous failed - no internet");
    #endif
            }
        }
    }*/

    static void networkTask(void *pvParameters) // This is a task.
    {
        (void)pvParameters;
        unsigned long int now_time;
        getMACAddress();
        mqtt_pub_topic[0] = '/';
        // memmove(mqtt_pub_topic + 1, NODEID, sizeof(NODEID));
        memmove(mqtt_pub_topic + 1, string_address, sizeof(string_address));
        ESP_LOGW("networkTask", "mqtt_pub_topic:%s ", mqtt_pub_topic);
        mqtt_app_start();
        // print the local IP address
        tcpip_adapter_ip_info_t ip_info;
        ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
        sprintf(ipAddress, "%s", ip4addr_ntoa(&ip_info.ip));

        for (;;) // A Task shall never return or exit.
        {

#ifdef RTOSDEBUG
            if (esp_timer_get_time() / 1000 - lastTime2 > 1000)
            {
                lastTime2 = esp_timer_get_time() / 1000;
                ESP_LOGI("networkTask", "RTOS_DEBUG0:esp_get_free_heap_size:%d  ", esp_get_free_heap_size());
                ESP_LOGI("networkTask", "RTOS_DEBUG0:esp_get_minimum_free_heap_size:%d  ", esp_get_minimum_free_heap_size());
                ESP_LOGI("networkTask", "RTOS_DEBUG0:esp_get_free_internal_heap_size:%d  ", esp_get_free_internal_heap_size());
                ESP_LOGI("networkTask", "RTOS_DEBUG0:uxTaskGetStackHighWaterMark:%d  ", uxTaskGetStackHighWaterMark(NULL));
                ESP_LOGI("networkTask", "RTOS_DEBUG0:uxTaskGetStackHighWaterMark:%x  ", esp_task_wdt_status(NULL));
                //   Serial.println("RTOS_DEBUG0:Heapdata 0 :");
                //   Serial.print("RTOS_DEBUG0:HeapSize: ");
                //   Serial.println(); //total heap size
                //   Serial.print("RTOS_DEBUG0:LowestHeapSinceboot: ");
                //   Serial.println(ESP.getMinFreeHeap()); //lowest level of free heap since boot
                //   Serial.print("RTOS_DEBUG0:LargestHepBlockPossible: ");
                //   Serial.println(ESP.getMaxAllocHeap()); //largest block of heap that can be allocated at once
                //   Serial.println("RTOS_DEBUG0:StackHighwater 0= " + String(uxTaskGetStackHighWaterMark(Task_net)));
                // Serial.print("wdtStatus0 ");
                // Serial.println(esp_task_wdt_status(NULL), HEX);
            }
#endif

            /* int stopTrx = 0;
             if (stopTransactionQueue != NULL)
             {
                 xQueueReceive(stopTransactionQueue, &stopTrx, 0);
             }
             else
             {
                 ESP_LOGE(TAG, "stopTransactionQueue==NULL");
             }

             if (stopTrx)
             {
                 ESP_LOGI("networkTask", "SqRecevStpTrx0******");
                 stoptTransactionHTTP_task0();
             }

             int stopTrxEnergy = 0;
             if (stopTransactionQueueEnergy != NULL)
             {
                 xQueueReceive(stopTransactionQueueEnergy, &stopTrxEnergy, 0);
             }
             else
             {
                 ESP_LOGE(TAG, "stopTransactionQueueEnergy==NULL");
             }

             if (stopTrxEnergy)
             {
                 ESP_LOGI("networkTask", "he he he he******");
                 energyBasedStopTransaction_task0();
             }

             int stopTrxPrev = 0;
             if (stopTransactionQueuePrev != NULL)
             {
                 xQueueReceive(stopTransactionQueuePrev, &stopTrxPrev, 0);
             }
             else
             {
                 ESP_LOGE(TAG, "stopTransactionQueuePrev==NULL");
             }

             if (stopTrxPrev)
             {
                 ESP_LOGI("networkTask", "SqRecevStpTrx0******");
                 stopTransactionPrevious_task0();
             }*/

            if (wifiOk)
            {
#ifdef DEBUG
                now_time = esp_timer_get_time() / 1000;
                if (now_time - pingDelay > 1000)
                {

                    pingDelay = now_time;

                    // pingST =is_internet_available;

                    if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
                    {
                        if (is_internet_available)
                        {
                            xSemaphoreGive(internetMutex);
                            ESP_LOGI("networktask", "Online!!");
                            getTime();
                        }
                        else
                        {
                            xSemaphoreGive(internetMutex);
                            ESP_LOGI("networktask", "offline :(");
                        }
                    }
                }
                else if (now_time < pingDelay)
                {
                    pingDelay = nowTime;
                }
#endif

                if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
                {
                    if (is_internet_available == true)
                    {

                        xSemaphoreGive(internetMutex);
                        /*  if(xSemaphoreTake(mqttMutex, 0)==pdTRUE){
                              if (sendMqtt) {

                                  sendMqtt = false;
                                  xSemaphoreGive(mqttMutex);
                                  #ifdef DEBUG
                                  ESP_LOGW("MQTT", "publishPre %s",mqtt_pub_topic);
                                  #endif
                                  esp_mqtt_client_publish(mqtt_client,mqtt_pub_topic,(const char*)heartBeatObj, 0, 1, 0);
                                  #ifdef DEBUG
                                  ESP_LOGW("MQTT", "publishPost");
                                  #endif
                              }
                              else{
                                  xSemaphoreGive(mqttMutex);
                              }

                          }*/

                        //        if (millis() - mqttDelay > 1000) {
                        //          mqttDelay = millis();
                        //        }

                        // startConfig();    //Running startup config
                        now_time = esp_timer_get_time() / 1000;
                        /*if (now_time - deepDiveDelay > 60000)
                         {
                             deepDiveDelay = now_time;
                             recordHeartBeat();
                         }
                         else if (now_time < deepDiveDelay)
                         {
                             deepDiveDelay = now_time;
                         }*/

                        /*if (CURRENT_STATE != CLOSE_STATE)removed
                        {
                            now_time = esp_timer_get_time() / 1000;
                            if (now_time - heartBeatDelay > 30000)
                            {
                                heartBeatDelay = now_time;
                                sendHeartBeatMQTT();
                            }
                            else if (now_time < heartBeatDelay)
                            {
                                heartBeatDelay = now_time;
                            }
                        }*/

                        /*now_time = esp_timer_get_time() / 1000;
                        if (now_time - mqtt_deepDiveDelay > 2000)
                        {
                            mqtt_deepDiveDelay = now_time;
                            sendDeepDiveData();
                        }
                        else if (now_time < mqtt_deepDiveDelay)
                        {
                            mqtt_deepDiveDelay = now_time;
                        }*/

                        /*now_time = esp_timer_get_time() / 1000;
                        if (now_time - mqtt_fullDeepDiveDelay > 300000)
                        {
                            mqtt_fullDeepDiveDelay = now_time;
                            sendFullDeepDiveData();
                        }
                        else if (now_time < mqtt_fullDeepDiveDelay)
                        {
                            mqtt_fullDeepDiveDelay = now_time;
                        }*/
                    }
                    else
                    {
                        xSemaphoreGive(internetMutex);
                    }
                }
            }
            else
            {
                // ESP_LOGI("networkTask","no wifi %d",wifiOk);
            }

            vTaskDelay(100 / portTICK_PERIOD_MS); // one tick delay in between reads to give other processes processer time
        }
    }

    void check_internet_task(void *pvParameter)
    {
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT_2, false, true, portMAX_DELAY);
        // Subscribe this task to TWDT, then check if it is subscribed

        while (1)
        {

            if (wifiOk)
            {
                initialize_ping(100, 2);
                ESP_LOGE(TAG, "offline");
            }
            //  if ( is_internet_available==true) {
            // 	ESP_LOGI(TAG, "online");
            // } else {
            // 	ESP_LOGE(TAG, "offline");
            // }
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }
    void main_task(void *pvParameter);
    void startConfig()
    {
        char *TAG = "startConfig()";
        // #ifdef DEBUG
        //     ESP_LOGI(TAG,"startConfig");
        // #endif
        while (1)
        {
            // if (wifiOk)
            // {
            // if (xSemaphoreTake(internetMutex, 1) == pdTRUE)
            // {
            // if (is_internet_available == true)
            //  {
            xSemaphoreGive(internetMutex);
            switch (startConfigState)
            {
            // #if OPENLOG
            //     openLog.println("config 0");
            // #endif
            case 0:
#ifdef DEBUG
                ESP_LOGW(TAG, "STARTUP_CONFIG 0: RTC");
#endif
                //   if (TIME_FROM_RTC) {
                //     if (esp_timer_get_time()/1000- otpFlagtimeOut > 1000) {
                //       otpFlagtimeOut = millis();

                //       startConfigState = 1;   // change this************
                //       //            int tmp1=0;
                //       //            tmp1=receveTimeRTC();
                //       //              if (tmp1) {
                //       //                tm.tm_year = actualtime[6] + 100;
                //       //                tm.tm_mon =  actualtime[5] - 1;
                //       //                tm.tm_mday = actualtime[3];
                //       //                tm.tm_hour = actualtime[2];
                //       //                tm.tm_min = actualtime[1];
                //       //                tm.tm_sec = actualtime[0];
                //       //                time_t t = mktime(&tm);
                //       //               ESP_LOGI(TAG, asctime(&tm));
                //       //                struct timeval now2 = { .tv_sec = t };
                //       //                settimeofday(&now2, NULL);
                //       //                startConfigState = 1;
                //       //              }

                //     }
                //   }

                //   else {
                //     configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer2);
                //     if (!getLocalTime(&timeinfo)) {
                //      ESP_LOGI(TAG,"STARTUP_CONFIG:Failed to obtain time");
                //     }
                //     else {

                //       Serial.print("STARTUP_CONFIG: obtained time");
                //      ESP_LOGI(TAG,&timeinfo, "%A, %B %d %Y %H:%M:%S");
                //       startConfigState = 1;
                //       ntpOkFlag = 1;
                //       //          if (timeinfo.tm_hour > 12) {
                //       //            pmFlag = 1;
                //       //          }
                //       //          else {
                //       //            pmFlag = 0;
                //       //          }
                //       //          encodeTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_wday, timeinfo.tm_mon + 1, timeinfo.tm_year - 100, pmFlag);
                //       //          if (setTimeRTC()) {
                //       //            startConfigState = 1;
                //       //          }
                //     }
                //   }
                startConfigState = 1;
                ntpOkFlag = 1;
                break;

            case 1:
#ifdef DEBUG
                ESP_LOGW(TAG, "STARTUP_CONFIG 1: Getpin");
#endif
                // #if OPENLOG
                //       openLog.println("config 1");
                // #endif

                cofigDelay1 = esp_timer_get_time() / 1000;
                // getPIN_HTTP();
                /*if (getPIN_HTTP_READY == 1)
                {*/
                configFlag1 = 1;
                //}
                startConfigState = 2;
                break;

            case 2:

#ifdef DEBUG
                ESP_LOGW(TAG, "STARTUP_CONFIG 2:charger detail");
#endif
                // #if OPENLOG
                //       openLog.println("config 2");
                // #endif
                if (esp_timer_get_time() / 1000 - cofigDelay1 > 1000)
                {
                    // getChargerDetails_HTTP();
                    startConfigState = 3;
                    // if (getChargerDetails_HTTP_READY == 1)
                    //{
                    configFlag2 = 1;
                    cofigDelay2 = esp_timer_get_time() / 1000;
                    //}
                }

                break;

            case 3:
#ifdef DEBUG
                ESP_LOGW(TAG, "STARTUP_CONFIG 3:wait 5s");
#endif
                if (configFlag1 == 1 && configFlag2 == 1)
                {
                    startConfigState = 5;
                }
                else
                {
                    if (esp_timer_get_time() / 1000 - cofigDelay2 > 5000)
                    {
                        startConfigState = 4;
                    }
                }
                break;

            case 4:

                if (esp_timer_get_time() / 1000 - cofigDelay3 > 10000 || configFlag3 == 1)
                {
#ifdef DEBUG
                    ESP_LOGW(TAG, "STARTUP_CONFIG 4: retry getpin and charger details");
#endif

                    // #if OPENLOG
                    //         openLog.println("config 4");
                    // #endif
                    cofigDelay3 = esp_timer_get_time() / 1000;
                    if (getPIN_HTTP_READY == 1 && getChargerDetails_HTTP_READY == 1)
                    {
                        startConfigState = 5;
                    }
                    else
                    {
                        if (configFlag3 == 0)
                        {
                            // getPIN_HTTP();
                            configFlag3 = 1;
                            cofigDelay4 = esp_timer_get_time() / 1000;
                        }
                        if (esp_timer_get_time() / 1000 - cofigDelay4 > 2000)
                        {
                            // getChargerDetails_HTTP();
                            configFlag3 = 0;
                        }
                    }
                }

                break;

            case 5:
                if (configFlag4 == 0)
                {
#ifdef DEBUG
                    ESP_LOGW(TAG, "STARTUP_CONFIG 5: StartConfigOK");
#endif
                    uart_flush(UART_NUM_2);

                    ESP_LOGW(TAG, "Task about to be deleted");
                    vTaskDelete(NULL);

                    // #if OPENLOG
                    //         openLog.println("StartConfigOK");
                    // #endif
                    configFlag4 = 1;
                    ESP_LOGW(TAG, "Task deleted");
                }
                break;
            }
            // }
            // else
            // {
            //   xSemaphoreGive(internetMutex);
            // }
            //}
            //  }
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }

#ifdef __cplusplus
}
#endif

#endif
