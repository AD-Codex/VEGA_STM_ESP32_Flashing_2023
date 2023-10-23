// lora_generic_spi.cpp
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// Contributed by Joanna Rutkowska
// $Id: lora_generic_spi.cpp,v 1.2 2014/04/12 05:26:05 mikem Exp $

#include <esp_idf_lora/lora_generic_spi.h>

lora_generic_spi::lora_generic_spi(Frequency frequency, BitOrder bitOrder, DataMode dataMode)
    :
    _frequency(frequency),
    _bitOrder(bitOrder),
    _dataMode(dataMode)
{
}

void lora_generic_spi::setBitOrder(BitOrder bitOrder)
{
    _bitOrder = bitOrder;
}

void lora_generic_spi::setDataMode(DataMode dataMode)
{
    _dataMode = dataMode; 
}

void lora_generic_spi::setFrequency(Frequency frequency)
{
    _frequency = frequency;
}

