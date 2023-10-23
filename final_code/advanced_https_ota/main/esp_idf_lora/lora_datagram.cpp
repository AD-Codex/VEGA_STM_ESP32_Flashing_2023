// lora_datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: lora_datagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <esp_idf_lora/lora_datagram.h>

lora_datagram::lora_datagram(lora_generic_driver& driver, uint8_t thisAddress) 
    :
    _driver(driver),
    _thisAddress(thisAddress)
{
}

////////////////////////////////////////////////////////////////////
// Public methods
bool lora_datagram::init()
{
    bool ret = _driver.init();
    if (ret)
	setThisAddress(_thisAddress);
    return ret;
}

void lora_datagram::setThisAddress(uint8_t thisAddress)
{
    _driver.setThisAddress(thisAddress);
    // Use this address in the transmitted FROM header
    setHeaderFrom(thisAddress);
    _thisAddress = thisAddress;
}

bool lora_datagram::sendto(uint8_t* buf, uint8_t len, uint8_t address)
{
    setHeaderTo(address);
    return _driver.send(buf, len);
}

bool lora_datagram::recvfrom(uint8_t* buf, uint8_t* len, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{
    if (_driver.recv(buf, len))
    {
	if (from)  *from =  headerFrom();
	if (to)    *to =    headerTo();
	if (id)    *id =    headerId();
	if (flags) *flags = headerFlags();
	return true;
    }
    return false;
}

bool lora_datagram::available()
{
    return _driver.available();
}

void lora_datagram::waitAvailable()
{
    _driver.waitAvailable();
}

bool lora_datagram::waitPacketSent()
{
    return _driver.waitPacketSent();
}

bool lora_datagram::waitPacketSent(uint16_t timeout)
{
    return _driver.waitPacketSent(timeout);
}

bool lora_datagram::waitAvailableTimeout(uint16_t timeout)
{
    return _driver.waitAvailableTimeout(timeout);
}

uint8_t lora_datagram::thisAddress()
{
    return _thisAddress;
}

void lora_datagram::setHeaderTo(uint8_t to)
{
    _driver.setHeaderTo(to);
}

void lora_datagram::setHeaderFrom(uint8_t from)
{
    _driver.setHeaderFrom(from);
}

void lora_datagram::setHeaderId(uint8_t id)
{
    _driver.setHeaderId(id);
}

void lora_datagram::setHeaderFlags(uint8_t set, uint8_t clear)
{
    _driver.setHeaderFlags(set, clear);
}

uint8_t lora_datagram::headerTo()
{
    return _driver.headerTo();
}

uint8_t lora_datagram::headerFrom()
{
    return _driver.headerFrom();
}

uint8_t lora_datagram::headerId()
{
    return _driver.headerId();
}

uint8_t lora_datagram::headerFlags()
{
    return _driver.headerFlags();
}



