// spi_generic_driver.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: lora_spi_driver.cpp,v 1.11 2017/11/06 00:04:08 mikem Exp $

#include <esp_idf_lora/lora_spi_driver.h>

lora_spi_driver::lora_spi_driver(uint8_t slaveSelectPin,uint8_t MISOPin,uint8_t MOSIPin,uint8_t SCLKPin,int SPIFrequency,uint8_t SPIMode)
    : 
   // _spi(spi),
    _slaveSelectPin(slaveSelectPin),
    _MOSIPin(MOSIPin),
    _MISOPin(MISOPin),
    _SCLKPin(SCLKPin),
    _SPIFrequency(SPIFrequency),
    _SPIMode(SPIMode)
{
}

bool lora_spi_driver::init()
{
    // start the SPI library with the default speeds etc:
    // On Arduino Due this defaults to SPI1 on the central group of 6 SPI pins
    //_spi.begin();
     esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   spi_bus_config_t bus = {
      .mosi_io_num = _MOSIPin,
      .miso_io_num = _MISOPin,
      .sclk_io_num = _SCLKPin,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 300
   };
           
   ret = spi_bus_initialize(SPI2_HOST, &bus, 3);

   //ESP_LOGI("lora_spi_driver", SPI2_HOST.);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
       /*.mode = 0,
        .clock_speed_hz = 2000000,
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL*/
         .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=_SPIMode,
        .duty_cycle_pos=128,        //50% duty cycle
        //.cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .clock_speed_hz=_SPIFrequency,
        //.spics_io_num=_slaveSelectPin,
        .flags = 0,
        .queue_size=1,
        .pre_cb = NULL
   };
   ret = spi_bus_add_device(SPI2_HOST, &dev, &_handle_spi);
   
   // ESP_LOGI("lora_spi_driver", "_spi.begin");
   assert(ret == ESP_OK);
   
    // Initialise the slave select pin
    // On Maple, this must be _after_ spi.begin
    //gpio_reset_pin((gpio_num_t)_slaveSelectPin);
    gpio_pad_select_gpio((gpio_num_t)_slaveSelectPin);
     /* Set the GPIO as a push/pull output */
      ESP_LOGI("lora_spi_driver", "setting up gpio pins:%d",_slaveSelectPin);
    gpio_set_direction((gpio_num_t)_slaveSelectPin, GPIO_MODE_OUTPUT); 
     ESP_LOGI("lora_spi_driver", "setting up gpio pins:%d",_slaveSelectPin); 
    //pinMode(_slaveSelectPin, OUTPUT);
    gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    //digitalWrite(_slaveSelectPin, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    /*int x=100;
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if(x==100){
            
            x=200;
        }
        else{
            x=100;
        }
        uint8_t out=0x42;
        uint8_t in[2];
        in[0]=12;
        in[1]=12;
        spi_transaction_t t = {
            .flags = 0,
            .length = 8 ,
            .tx_buffer = &out,
            .rx_buffer = in
        };
        gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
        spi_device_transmit(_spi._handle_spi, &t);
        gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
        ESP_LOGI("transmitted", "x:  %d",out);
        ESP_LOGI("received", "x:  %d,%d",in[0],in[1]);
        

    }*/
    /*while(1){
        spiRead(0x42);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }*/
    return true;
}
/*
uint8_t lora_spi_driver::lora_read_reg(uint8_t reg)
{
    uint8_t val;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    //digitalWrite(_slaveSelectPin, LOW);
     gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the address with the write mask off
    val = _spi.transfer(0); // The written value is ignored, reg value is read
     gpio_set_level((gpio_num_t)_slaveSelectPin,1);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return val;
}
*/
uint8_t lora_spi_driver::spiRead(uint8_t regi)
{
   uint8_t x = regi & ~RH_SPI_WRITE_MASK;
   uint8_t out[2]={x,0xff};
   uint8_t in[2];
    //in[0]=12;
    //in[1]=12;
   spi_transaction_t t = {
        .flags = 0,
        .length = 8*sizeof(out) ,
        .tx_buffer = out,
        .rx_buffer = in
   };
   ATOMIC_BLOCK_START;
   gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
   spi_device_transmit(_handle_spi, &t);
   gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
   ATOMIC_BLOCK_END;
    //ESP_LOGI("spiRead()", " Register Adresss:- %x, Data Received: %x,%x",regi,in[0],in[1]);
   //ESP_LOGI("spiRead()", "x:  %d,",in[1]);
   return in[1];
}

uint8_t lora_spi_driver::spiWrite(uint8_t regi, uint8_t val)
{
   uint8_t status = 0;
   uint8_t x= regi|RH_SPI_WRITE_MASK;
   uint8_t out[2] = {x, val };
   //uint8_t in[2];
   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = NULL
   };
   //ATOMIC_BLOCK_START;
   gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
   spi_device_transmit(_handle_spi, &t);
   gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
   //ESP_LOGI("spiWrite()", " Register Adresss:- %x, Data:-%x",regi,out[1]);
   //ATOMIC_BLOCK_END;
 //vTaskDelay(500 / portTICK_PERIOD_MS);
   return status;
}
/*
uint8_t lora_spi_driver::lora_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
     gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    //digitalWrite(_slaveSelectPin, LOW);
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the address with the write mask on
    _spi.transfer(val); // New value follows
    //digitalWrite(_slaveSelectPin, HIGH);
     gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}*/
uint8_t lora_spi_driver::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    uint8_t status = 0;
    //ESP_LOGI("lora_spi_driver::spiBurstRead", "started.................................");
    /*
    //ATOMIC_BLOCK_START;
    //_spi.beginTransaction();
    //digitalWrite(_slaveSelectPin, LOW);
    gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    //status = _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off
    //uint8_t status = 0;
    uint8_t x= reg & ~RH_SPI_WRITE_MASK;
    uint8_t out = x;
    spi_transaction_t tx = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = &out,
        .rx_buffer = NULL  
    };
    //ATOMIC_BLOCK_START;
    //gpio_set_level((gpio_num_t)12, 0);
    spi_device_transmit(_spi._handle_spi, &tx);
   // gpio_set_level((gpio_num_t)12, 1);
    //ATOMIC_BLOCK_END;
    //return status;
    //pi_device_transmit(_spi._handle_spi, &tx);
    //for(int i=0;i<10;i++){}
    tx.tx_buffer=NULL;
    tx.rx_buffer=dest;
    //tx.length=len*8;
    spi_device_transmit(_spi._handle_spi, &tx);
    //for(int i=0;i<8*len;i++){}
    gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    */
    /*uint8_t in[1];
    spi_transaction_t rx = {
        .flags = 0,
        //.length = 8 * sizeof(out),
        .tx_buffer = NULL,
        .rx_buffer = in
    };

    while (len--)
	//dest++ = _spi.transfer(0);
    {
        //ATOMIC_BLOCK_START;
        //gpio_set_level((gpio_num_t)12, 0);
        spi_device_transmit(_spi._handle_spi, &rx);
       // gpio_set_level((gpio_num_t)12, 1);
        //ATOMIC_BLOCK_END;
        *dest++ = in[0];
    }
    //digitalWrite(_slaveSelectPin, HIGH);
    gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
   //_spi.endTransaction();
    //ATOMIC_BLOCK_END;*/
   
   for(int i=0; i<len; i++) {


      *dest++ = spiRead(reg);
     
   }
      
    
  // ESP_LOGI("lora_spi_driver::spiBurstRead", "ended.................................");
    return status;
}

uint8_t lora_spi_driver::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
   
    uint8_t status = 0;
    gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    uint8_t x= RH_SPI_WRITE_MASK|reg;
    uint8_t out= x;
   
    spi_transaction_t tx = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = &out,
        .rx_buffer = NULL  
    };

    spi_device_transmit(_handle_spi, &tx);
    //for(int i=0;i<10;i++){}
    tx.tx_buffer=src;
    tx.rx_buffer=NULL;
    tx.length=len*8;
    /*for(int i=0;i<len;i++){
        ESP_LOGI("lora_spi_driver", "%d",src[i]);
    }*/
    //ESP_LOGI("lora_spi_driver::spiBurstWrite", " %d",len);
    spi_device_transmit(_handle_spi, &tx);
    //for(int i=0;i<8*len;i++){}
    gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    //vTaskDelay(500 / portTICK_PERIOD_MS);
    return status;
}
/*
uint8_t lora_spi_driver::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    //digitalWrite(_slaveSelectPin, LOW);
    gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    status = _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off
    while (len--)
	*dest++ = _spi.transfer(0);
    //digitalWrite(_slaveSelectPin, HIGH);
    gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t lora_spi_driver::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    //digitalWrite(_slaveSelectPin, LOW);
     gpio_set_level((gpio_num_t)_slaveSelectPin, 0);
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the start address with the write mask on
    while (len--)
	_spi.transfer(*src++);
    //digitalWrite(_slaveSelectPin, HIGH);
     gpio_set_level((gpio_num_t)_slaveSelectPin, 1);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}*/

void lora_spi_driver::setSlaveSelectPin(uint8_t slaveSelectPin)
{
    _slaveSelectPin = slaveSelectPin;
}

void lora_spi_driver::spiUsingInterrupt(uint8_t interruptNumber)
{
    //_spi.usingInterrupt(interruptNumber);
}

