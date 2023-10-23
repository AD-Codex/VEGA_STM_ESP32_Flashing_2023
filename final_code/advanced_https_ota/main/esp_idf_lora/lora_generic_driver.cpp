// lora_generic_driver.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: lora_generic_driver.cpp,v 1.23 2018/02/11 23:57:18 mikem Exp $

#include <esp_idf_lora/lora_generic_driver.h>
static const char *TAG = "lora_generic_driver";

lora_generic_driver::lora_generic_driver()
    : _mode(RHModeInitialising),
      _thisAddress(RH_BROADCAST_ADDRESS),
      _txHeaderTo(RH_BROADCAST_ADDRESS),
      _txHeaderFrom(RH_BROADCAST_ADDRESS),
      _txHeaderId(0),
      _txHeaderFlags(0),
      _rxBad(0),
      _rxGood(0),
      _txGood(0),
      _cad_timeout(0)
{
}

bool lora_generic_driver::init()
{
    return true;
}

// Blocks until a valid message is received
void lora_generic_driver::waitAvailable()
{
    while (!available())
        YIELD;
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool lora_generic_driver::waitAvailableTimeout(uint16_t timeout)
{
    unsigned long starttime = (unsigned long)(esp_timer_get_time() / 1000);
    while (((unsigned long)(esp_timer_get_time() / 1000) - starttime) < timeout)
    {
        if (available())
        {
            return true;
        }
        YIELD;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return false;
}

bool lora_generic_driver::waitPacketSent()
{
    ESP_LOGW("lora_generic_driver", "not timeout _mode:%d", _mode);
    while (_mode == RHModeTx)
    {
       // ESP_LOGW("lora_generic_driver", "not timeout in while loop _mode:%d", _mode);
        YIELD; // Wait for any previous transmit to finish
    }
    ESP_LOGW("lora_generic_driver", "not timeout after _mode:%d", _mode);
    return true;
}

bool lora_generic_driver::waitPacketSent(uint16_t timeout)
{
    unsigned long starttime = (unsigned long)(esp_timer_get_time() / 1000);
    while (((unsigned long)(esp_timer_get_time() / 1000) - starttime) < timeout)
    {
        ESP_LOGW("lora_generic_driver", "RHModeTx:%d", RHModeTx);
        if (_mode != RHModeTx) // Any previous transmit finished?
            return true;
        YIELD;
    }
    return false;
}

// Wait until no channel activity detected or timeout
bool lora_generic_driver::waitCAD()
{
    if (!_cad_timeout)
        return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = (unsigned long)(esp_timer_get_time() / 1000);
    while (isChannelActive())
    {
        if ((unsigned long)(esp_timer_get_time() / 1000) - t > _cad_timeout)
            return false;

        // delay(); // Should these values be configurable? Macros?
        vTaskDelay((1 + (random() % 10)) * 100 / portTICK_PERIOD_MS);
    }

    return true;
}

// subclasses are expected to override if CAD is available for that radio
bool lora_generic_driver::isChannelActive()
{
    return false;
}

void lora_generic_driver::setPromiscuous(bool promiscuous)
{
    _promiscuous = promiscuous;
}

void lora_generic_driver::setThisAddress(uint8_t address)
{
    _thisAddress = address;
}

void lora_generic_driver::setHeaderTo(uint8_t to)
{
    _txHeaderTo = to;
}

void lora_generic_driver::setHeaderFrom(uint8_t from)
{
    _txHeaderFrom = from;
}

void lora_generic_driver::setHeaderId(uint8_t id)
{
    _txHeaderId = id;
}

void lora_generic_driver::setHeaderFlags(uint8_t set, uint8_t clear)
{
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t lora_generic_driver::headerTo()
{
    return _rxHeaderTo;
}

uint8_t lora_generic_driver::headerFrom()
{
    return _rxHeaderFrom;
}

uint8_t lora_generic_driver::headerId()
{
    return _rxHeaderId;
}

uint8_t lora_generic_driver::headerFlags()
{
    return _rxHeaderFlags;
}

int16_t lora_generic_driver::lastRssi()
{
    return _lastRssi;
}

lora_generic_driver::RHMode lora_generic_driver::mode()
{
    return _mode;
}

void lora_generic_driver::setMode(RHMode mode)
{
    _mode = mode;
}

bool lora_generic_driver::sleep()
{
    return false;
}

// Diagnostic help
void lora_generic_driver::printBuffer(const char *prompt, const uint8_t *buf, uint8_t len)
{
    // #ifdef RH_HAVE_SERIAL
    // Serial.println(prompt);
    ESP_LOGI(TAG, "%s", (prompt));
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        if (i % 16 == 15)
            // Serial.println(buf[i], HEX);
            ESP_LOGI(TAG, "%x", buf[i]);
        else
        {
            ESP_LOGI(TAG, "%x", buf[i]);
            // Serial.print(buf[i], HEX);
            // Serial.print(' ');
        }
    }
    // Serial.println("");
    // #endif
}

uint16_t lora_generic_driver::rxBad()
{
    return _rxBad;
}

uint16_t lora_generic_driver::rxGood()
{
    return _rxGood;
}

uint16_t lora_generic_driver::txGood()
{
    return _txGood;
}

void lora_generic_driver::setCADTimeout(unsigned long cad_timeout)
{
    _cad_timeout = cad_timeout;
}

/*#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_PLATFORM_ATTINY)
// Tinycore does not have __cxa_pure_virtual, so without this we
// get linking complaints from the default code generated for pure virtual functions
extern "C" void __cxa_pure_virtual()
{
    while (1);
}
#endif*/
