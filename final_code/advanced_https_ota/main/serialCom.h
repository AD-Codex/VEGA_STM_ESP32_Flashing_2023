#ifndef SERIALCOM_H
#define SERIALCOM_H

#ifdef __cplusplus
extern "C"
{
#endif

// #include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "rom/crc.h"

#define TXD_PIN2 (GPIO_NUM_17)
#define RXD_PIN2 (GPIO_NUM_16)

  static const int RX_BUF_SIZE = 2048;
  // const uart_port_t uart_num = UART_NUM_2;
  // Setup UART buffered IO with event queue
  // const int uart_buffer_size = (1024 * 2);
  // QueueHandle_t uart_queue;
  // Install UART driver using an event queue here

  void init_uart(void)
  {
    const uart_config_t uart_config_2 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, TXD_PIN2, RXD_PIN2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE, 0, 0, NULL, 0);
    // const uart_config_t uart_config_2 ={
    //     .baud_rate = 9600,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_APB,
    // };
    // // uart_driver_install(uart_num,RX_BUF_SIZE*2,0,0,NULL,0);
    //  ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config_2));
    // ESP_ERROR_CHECK(uart_set_pin(uart_num,TXD_PIN2,RXD_PIN2,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,uart_buffer_size, 10, &uart_queue, 0));
  }

  uint8_t avrBuffer[5];
  uint8_t DataBuffer[19] = {'\0'};
  uint8_t uart_sendBuffer[17];
  // extern HardwareSerial MySerial;
  // extern SoftwareSerial swSer;
  uint8_t debugBuffer[17];
  uint8_t debugBufferOut[17];

  typedef enum
  {
    IDLE_POWER = 0,
    START_POWER,
    READY_POWER,
    CHARGE_POWER,
    STOP_POWER,
    FAULT_POWER
  } powerSideStateType;

  typedef enum
  {
    STATE_A1 = 0,
    STATE_A2,
    STATE_B1,
    STATE_B2,
    STATE_C1,
    STATE_C2,
    STATE_D,
    STATE_F
  } isolatedStateType;

  typedef struct
  {
    uint16_t LB : 8;
    uint16_t HB : 8;
  } VAL;

  typedef union
  {
    uint16_t all;
    VAL bytes;
  } VALBITS;

  typedef struct
  {
    powerSideStateType StatusPwr : 4;
    isolatedStateType Status : 4;

  } STATE1;

  typedef struct
  {
    uint8_t startCharge : 4;
    uint8_t stopCharge : 4;
  } CHARGTESTAR;

  typedef union
  {
    CHARGTESTAR bits;
    uint8_t all;
  } STARTCHARGE_BITS;

  typedef union
  {
    uint8_t all;
    STATE1 bits;

  } STATE1BITS;

  typedef struct
  {
    uint8_t serialAError : 1;
    uint8_t serialBError : 1;
    uint8_t terminalOT : 1;
    uint8_t diodeCheck_failed : 1;
    uint8_t powerSide_fault : 1;
    uint8_t networkSide_fault : 1;
    uint8_t gfit_error_count : 2;
    uint8_t error_count_updated : 1;
  } control_side_error;

  typedef struct
  {
    uint8_t trip_OC_L1 : 1;
    uint8_t trip_GFI : 1;
  } TRIPBITS;

  typedef struct
  {
    // add data here
    uint8_t control_side_errorHB1;
  } CHARGERERRORSHB1;

  typedef struct
  {
    uint8_t error_OV_L1 : 1;
    uint8_t error_UV_L1 : 1;
    uint8_t error_SR_L1 : 1;
    uint8_t error_SR_N : 1;
    uint8_t error_SR_C : 1;
    uint8_t error_GFI_test : 1;
    uint8_t trip_OC_L1 : 1;
    uint8_t trip_GFI : 1;

  } power_side_error;

  typedef struct
  {
    uint8_t curretState : 5;
    uint8_t charging_active : 1;
    uint8_t cpPWM_active : 1;
    uint8_t connector_state : 1;
  } control_side_status;

  typedef struct
  {
    uint8_t powerSideState : 5;
    uint8_t powerSide_ready : 1;
    uint8_t contactor_state : 2;
  } power_side_state;

  typedef struct
  {
    // add data here
  } STATEBITSHB2;
  /*typedef struct
{
  uint8_t start_Charge :1;
  uint8_t stop_Charge :1;
  uint8_t stp_btn_State :1;
  uint8_t vehicle_Check :1;
  uint8_t charge_pause :1;
} networkSide_request_struct;*/
  typedef struct
  {
    uint8_t start_Charge : 1;
    uint8_t stop_Charge : 1;
    uint8_t schedule_charge : 1;
    uint8_t vehicle_Check : 1;
    uint8_t charge_pause : 1;
    uint8_t rtcUpdateComplete : 1;
    uint8_t rtcUpdateAlarmComplete : 1;
  } networkSide_request_struct;

  /*struct networkSide_request_struct
  {
    uint8_t start_Charge : 1;
    uint8_t stop_Charge : 1;
    uint8_t schedule_charge : 1;
    uint8_t vehicle_Check : 1;
    uint8_t charge_pause : 1;
    uint8_t rtcUpdateComplete : 1;
    uint8_t rtcUpdateAlarmComplete : 1;
  };*/

  /*struct networkSide_request_struct
  {
    uint8_t start_Charge :1;
    uint8_t stop_Charge :1;
    uint8_t stp_btn_State :1;
    uint8_t vehicle_Check :1;
    uint8_t charge_pause :1;
    uint8_t rtcUpdateComplete:1;
    uint8_t rtcUpdateAlarmComplete:1;
  };*/

  typedef union
  {
    control_side_error bits;
    uint16_t all;
  } control_SideError;

  typedef union
  {
    TRIPBITS bits;
    uint8_t all;
  } TRIPBITS_BITS;

  /*typedef union {
  CHARGERERRORSHB bits;
  uint8_t all;
  }ERROR_BITSHB;*/

  typedef union
  {
    CHARGERERRORSHB1 bits;
    uint8_t all;
  } ERROR_BITSHB1;

  typedef union
  {
    power_side_error bits;
    uint8_t all;
  } power_SideError;

  typedef union
  {
    networkSide_request_struct bits;
    uint8_t all;
  } networkSideRequestStruct;

  typedef union
  {
    power_side_state bits;
    uint8_t all;
  } power_SideState;

  typedef union
  {
    control_side_status bits;
    uint8_t all;
  } controlSideStatus;

  typedef struct serialSend
  {
    unsigned int StatusLB;
    unsigned int StatusHB1;
    unsigned int StatusHB2;
    unsigned int errorLB;
    unsigned int ledOffCommandRemote;
    unsigned int isChargerLocked;
    unsigned int scheduleChargeStart;
   // unsigned int tripBits; today
    // unsigned int curruent_state;
    unsigned int messegeID;
    unsigned int maximumCurrentRequest;

    VALBITS val1;
    VALBITS val2;
    VALBITS val3;
  } serialSend;

  typedef struct serialReceve
  {
    controlSideStatus control_Side_Status;
    power_SideState power_Side_State;
    networkSideRequestStruct network_SideRequest_Struct;
    control_SideError control_Side_Error;
    ERROR_BITSHB1 errorHB1;
    power_SideError power_Side_Error;
    TRIPBITS_BITS tripBits;
    unsigned int messegeID;
    VALBITS val1;
    VALBITS val2;
    VALBITS val3;
    STARTCHARGE_BITS startChg;
  } serialReceve;

  serialReceve recevedSerial;
  serialSend sendSerial;

  // char DataBuffer[200]={'\0'};

  uint8_t uart_data_header[2] = {0x23, 0x43};
  uint8_t uart_debug_header[3] = {0x23, 0x23, 0x5A};
  uint16_t crc_reslt_lockA;
  uint16_t txA_crc_calc;
  uint16_t rxA_crc_calc;
  // const uint16_t data16[7]={0x1020,0x3040 ,0x5060 ,0x7080 ,0x90a0 ,0xb0c0};

  void update_bad_crcA(uint16_t ch)
  {
    unsigned short i, xor_flag;
    ch <<= 8;

    for (i = 0; i < 8; i++)
    {
      if ((crc_reslt_lockA ^ ch) & 0x8000)
      {
        xor_flag = 1;
      }
      else
      {
        xor_flag = 0;
      }
      crc_reslt_lockA = crc_reslt_lockA << 1;
      if (xor_flag)
      {
        crc_reslt_lockA = crc_reslt_lockA ^ 0x1021;
      }
      ch = ch << 1;
    }
  }
  uint16_t ModRTU_CRCA(uint8_t *buf, int len)
  {
    int pos, i;
    uint16_t crc = 0xFFFF;

    for (pos = 0; pos < len; pos++)
    {
      crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

      for (i = 8; i != 0; i--)
      { // Loop over each bit
        if ((crc & 0x0001) != 0)
        {            // If the LSB is set
          crc >>= 1; // Shift right and XOR 0xA001
          crc ^= 0xA001;
        }
        else
          // Else LSB is not set
          crc >>= 1; // Just shift right
      }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
  }

  uint16_t crc_1021A(uint16_t data_length, uint8_t *data_crc)
  {
    uint8_t countA;
    crc_reslt_lockA = 0xffff;
    uint16_t value;
    // printf("crc_1021A ");
    for (countA = 0; countA < data_length; countA += 2)
    {
      value = (data_crc[countA] << 8) | (data_crc[countA + 1] & 0xff);
      // printf("%x ",value);
      update_bad_crcA(value);
    }
    // printf("\n");
    return crc_reslt_lockA;
  }

  // uint16_t crc_1021B(uint16_t data_length, uint16_t *data_crc)
  // {
  //     uint8_t countA;
  //     crc_reslt_lockA = 0xffff;
  //     for (countA = 0; countA < data_length; countA++)
  //     {
  //         update_bad_crcA(data_crc[countA]);
  //     }
  //     return crc_reslt_lockA;
  // }

  int serialReadFunc()
  {
    int length = 0;
    int state = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t *)&length));
    if (length > 0)
    {

      ESP_LOGI("SeriaReadFunc", "available rx buffer length %d", length);
      length = uart_read_bytes(UART_NUM_2, DataBuffer, 19, 1000);
      //  ESP_LOGI(TAG, "read rx buffer length %d buffer %s",length,DataBuffer);
      //  #ifdef DEBUG
      //       printf("SERIAL_IN: SerialDataBuffer:");
      //       for (int i = 0; i < length; i++) {
      //         printf(" %x ",DataBuffer[i]);

      //       }
      //       printf("/\n");
      // #endif

      if (memcmp(DataBuffer, uart_data_header, 2) == 0 && DataBuffer[18] == 0x2A)
      { // data packet
        // crc_reslt_lockA=crc_1021A( 15,DataBuffer+1);
        // uint8_t txptr = (uint8_t) DataBuffer;
        txA_crc_calc = ModRTU_CRCA((uint8_t *)(DataBuffer + 1), 15);

        // crc_reslt_lockA=crc_1021B( 6,data16);
        //  printf("crc_reslt_lockA value %x \n",crc_reslt_lockA);
        // uncomment following to check crc
        if (1 /*crc_reslt_lockA==((DataBuffer[15] << 8 ) | (DataBuffer[16] & 0xff))*/)
        {

          state = 1;
          recevedSerial.control_Side_Status.all = DataBuffer[2];
          recevedSerial.power_Side_State.all = DataBuffer[3];
          recevedSerial.network_SideRequest_Struct.all = DataBuffer[4];
          recevedSerial.control_Side_Error.all = ((DataBuffer[6] << 8) | (DataBuffer[5] & 0xff));
          // recevedSerial.errorHB1.all = DataBuffer[6];
          recevedSerial.power_Side_Error.all = DataBuffer[7];
           ESP_LOGI("SerialReadFunction", "recevedSerial.power_Side_Error.all: %x",recevedSerial.power_Side_Error.all );
          recevedSerial.tripBits.all = DataBuffer[8];

          // recevedSerial.startChg.all = DataBuffer[7];
          recevedSerial.messegeID = DataBuffer[9];
          recevedSerial.val1.bytes.LB = DataBuffer[10];
          recevedSerial.val1.bytes.HB = DataBuffer[11];
          recevedSerial.val2.bytes.LB = DataBuffer[12];
          recevedSerial.val2.bytes.HB = DataBuffer[13];
          recevedSerial.val3.bytes.LB = DataBuffer[14];
          recevedSerial.val3.bytes.HB = DataBuffer[15];
#ifdef DEBUG
          printf("SERIAL_IN: length %d crc %x SerialDataBuffer: ", length, txA_crc_calc);
          for (int i = 0; i < length; i++)
          {
            printf(" %x ", DataBuffer[i]);
          }
          printf("\n");
#endif
          memset(DataBuffer, '\0', 15);
        }
      }
      else if (memcmp(DataBuffer, uart_debug_header, 3) == 0 && (char)DataBuffer[12] == '*' && (char)DataBuffer[13] == '\n')
      { // debug packet
        if (crc_reslt_lockA == ((DataBuffer[16] << 8) | (DataBuffer[17] & 0xff)))
        {
          state = 2;
        }
      }

      else
      {
      }
      uart_flush(UART_NUM_2);
    }
    return state;
  }

  int serialSendFunc()
  {
    //  uint8_t state = 0;
    uint8_t uart_sendBuffer[20] = {
        0x23,
        0x4E,
        (uint8_t)sendSerial.StatusLB,
        (uint8_t)sendSerial.StatusHB1,
        (uint8_t)sendSerial.StatusHB2,
        (uint8_t)sendSerial.errorLB,
        (uint8_t)sendSerial.ledOffCommandRemote,
        (uint8_t)sendSerial.isChargerLocked,
        (uint8_t)sendSerial.scheduleChargeStart,
        //(uint8_t)sendSerial.curruent_state,
        (uint8_t)sendSerial.messegeID,
        (uint8_t)sendSerial.val1.bytes.LB,
        (uint8_t)sendSerial.val1.bytes.HB,
        (uint8_t)sendSerial.val2.bytes.LB,
        (uint8_t)sendSerial.val2.bytes.HB,
        (uint8_t)sendSerial.val3.bytes.LB,
        (uint8_t)sendSerial.val3.bytes.HB,
        (uint8_t)0xff,
        (uint8_t)0xff,
        0x2A,
        0x0A};
    uint16_t buffer16 = (ModRTU_CRCA((uint8_t *)(uart_sendBuffer + 1), 15)) & 0xff;
    uart_sendBuffer[16] = (uint8_t)buffer16;
    uart_sendBuffer[17] = (ModRTU_CRCA((uint8_t *)(uart_sendBuffer + 1), 15)) >> 8;

    rxA_crc_calc = ModRTU_CRCA((uint8_t *)(uart_sendBuffer + 1), 15);

#ifdef DEBUG
    printf("SERIAL_OUT: crc %x SerialDataBuffer: ", rxA_crc_calc);
    for (int i = 0; i < 20; i++)
    {
      printf(" %x ", uart_sendBuffer[i]);
    }
    printf("\n");
#endif
    //uart_flush(UART_NUM_2);
    // Write data to UART, end with a break signal.
    uart_write_bytes(UART_NUM_2, uart_sendBuffer, 20);
    //vTaskDelay(100 / portTICK_RATE_MS);
    uart_wait_tx_done(UART_NUM_2, 500);
    return 1;
  }

#ifdef __cplusplus
}
#endif

#endif