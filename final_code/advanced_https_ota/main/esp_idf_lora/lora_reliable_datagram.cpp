// lora_reliable_datagram.cpp
//
// Define addressed datagram
// 
// Part of the Arduino RH library for operating with HopeRF RH compatible transceivers 
// (see http://www.hoperf.com)
// RHDatagram will be received only by the addressed node or all nodes within range if the 
// to address is RH_BROADCAST_ADDRESS
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: lora_reliable_datagram.cpp,v 1.17 2017/03/08 09:30:47 mikem Exp $

#include <esp_idf_lora/lora_reliable_datagram.h>
#include "esp_log.h"
////////////////////////////////////////////////////////////////////
// Constructors
lora_reliable_datagram::lora_reliable_datagram(lora_generic_driver& driver, uint8_t thisAddress) 
    : lora_datagram(driver, thisAddress)
{
    _retransmissions = 0;
    _lastSequenceNumber = 0;
    _timeout = RH_DEFAULT_TIMEOUT;
    _retries = RH_DEFAULT_RETRIES;
    memset(_seenIds, 0, sizeof(_seenIds));
}

////////////////////////////////////////////////////////////////////
// Public methods
void lora_reliable_datagram::setTimeout(uint16_t timeout)
{
    _timeout = timeout;
}

////////////////////////////////////////////////////////////////////
void lora_reliable_datagram::setRetries(uint8_t retries)
{
    _retries = retries;
}

////////////////////////////////////////////////////////////////////
uint8_t lora_reliable_datagram::retries()
{
    return _retries;
}

////////////////////////////////////////////////////////////////////
bool lora_reliable_datagram::sendtoWait(uint8_t* buf, uint8_t len, uint8_t address)
{
    // Assemble the message
    uint8_t thisSequenceNumber = ++_lastSequenceNumber;
    uint8_t retries = 0;
	//ESP_LOGI("lora_reliable_datagram::sendtoWait()", "retries-%d",_retries);
    while (retries++ <= _retries)
    {
	setHeaderId(thisSequenceNumber);
	setHeaderFlags(RH_FLAGS_NONE, RH_FLAGS_ACK); // Clear the ACK flag
	sendto(buf, len, address);
	waitPacketSent();

	// Never wait for ACKS to broadcasts:
	if (address == RH_BROADCAST_ADDRESS)
	    return true;

	if (retries > 1)
	    _retransmissions++;
	unsigned long thisSendTime =(unsigned long)(esp_timer_get_time()/1000); // Timeout does not include original transmit time

	// Compute a new timeout, random between _timeout and _timeout*2
	// This is to prevent collisions on every retransmit
	// if 2 nodes try to transmit at the same time
/*#if (RH_PLATFORM == RH_PLATFORM_RASPI) // use standard library random(), bugs in random(min, max)
	uint16_t timeout = _timeout + (_timeout * (random() & 0xFF) / 256);
#else*/
	uint16_t timeout = _timeout + (_timeout * (random() & 0xFF) / 256);
	//ESP_LOGI("lora_reliable_datagram::sendtoWait()", "timeout-%d",timeout);
//#endif
	int32_t timeLeft;
        while ((timeLeft = timeout - ((unsigned long)(esp_timer_get_time()/1000) - thisSendTime)) > 0)
	{
	    if (waitAvailableTimeout(timeLeft))
	    {
		uint8_t from, to, id, flags;
		if (recvfrom(0, 0, &from, &to, &id, &flags)) // Discards the message
		{
			/*ESP_LOGI("lora_reliable_datagram::sendtoWait()", "from-%d address-%d",from,address);
			ESP_LOGI("lora_reliable_datagram::sendtoWait()", "to-%d _thisAddress-%d",to,_thisAddress);
			ESP_LOGI("lora_reliable_datagram::sendtoWait()", "id-%d thisSequenceNumber-%d",id,thisSequenceNumber);
			ESP_LOGI("lora_reliable_datagram::sendtoWait()", "flags-%d RH_FLAGS_ACK-%d sum-%d",flags,RH_FLAGS_ACK,flags & RH_FLAGS_ACK);*/
		    // Now have a message: is it our ACK?
		    if (   from == address 
			   && to == _thisAddress 
			   && (flags & RH_FLAGS_ACK) 
			   && (id == thisSequenceNumber))
		    {
			// Its the ACK we are waiting for
			//ESP_LOGI("lora_reliable_datagram::sendtoWait()", "ack received!!!!!");
			return true;
		    }
		    else if (   !(flags & RH_FLAGS_ACK)
				&& (id == _seenIds[from]))
		    {
			// This is a request we have already received. ACK it again
			acknowledge(id, from);
		    }
		    // Else discard it
		}
	    }
	    // Not the one we are waiting for, maybe keep waiting until timeout exhausted
	    YIELD;
	}
	// Timeout exhausted, maybe retry
	YIELD;
    }
    // Retries exhausted
    return false;
}

////////////////////////////////////////////////////////////////////
bool lora_reliable_datagram::recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{  
    uint8_t _from;
    uint8_t _to;
    uint8_t _id;
    uint8_t _flags;
    // Get the message before its clobbered by the ACK (shared rx and tx buffer in some drivers
    if (available() && recvfrom(buf, len, &_from, &_to, &_id, &_flags))
    {
	// Never ACK an ACK
	if (!(_flags & RH_FLAGS_ACK))
	{
	    // Its a normal message not an ACK
	    if (_to ==_thisAddress)
	    {
	        // Its for this node and
		// Its not a broadcast, so ACK it
		// Acknowledge message with ACK set in flags and ID set to received ID
		acknowledge(_id, _from);
	    }
	    // If we have not seen this message before, then we are interested in it
	    if (_id != _seenIds[_from])
	    {
		if (from)  *from =  _from;
		if (to)    *to =    _to;
		if (id)    *id =    _id;
		if (flags) *flags = _flags;
		_seenIds[_from] = _id;
		return true;
	    }
	    // Else just re-ack it and wait for a new one
	}
    }
    // No message for us available
    return false;
}

bool lora_reliable_datagram::recvfromAckTimeout(uint8_t* buf, uint8_t* len, uint16_t timeout, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{
    unsigned long starttime = (unsigned long)(esp_timer_get_time()/1000);
    int32_t timeLeft;
    while ((timeLeft = timeout - ((unsigned long)(esp_timer_get_time()/1000) - starttime)) > 0)
    {
	if (waitAvailableTimeout(timeLeft))
	{
	    if (recvfromAck(buf, len, from, to, id, flags))
		return true;
	}
	YIELD;
    //vTaskDelay(10);
    }
    return false;
}

uint32_t lora_reliable_datagram::retransmissions()
{
    return _retransmissions;
}

void lora_reliable_datagram::resetRetransmissions()
{
    _retransmissions = 0;
}
 
void lora_reliable_datagram::acknowledge(uint8_t id, uint8_t from)
{
    setHeaderId(id);
    setHeaderFlags(RH_FLAGS_ACK);
    // We would prefer to send a zero length ACK,
    // but if an RH_RF22 receives a 0 length message with a CRC error, it will never receive
    // a 0 length message again, until its reset, which makes everything hang :-(
    // So we send an ACK of 1 octet
    // REVISIT: should we send the RSSI for the information of the sender?
    uint8_t ack = '!';
    sendto(&ack, sizeof(ack), from); 
    waitPacketSent();
}

