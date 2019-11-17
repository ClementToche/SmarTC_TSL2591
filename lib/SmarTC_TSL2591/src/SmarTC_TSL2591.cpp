/*
* SmarTC TSL2591 Module
* See COPYRIGHT file at the top of the source tree.
*
* This product includes software developed by the SmarTC team.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, at version 3 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the SmarTC and the GNU General
* Public License along with this program.
*/

/**
 * @file SmarTC_TSL2591.cpp
 * 
 * @brief Source code for TSL2591 UV Sensor management.
 * @ingroup TSL2591
 * 
 * @author Clément TOCHE
 * Contact: ad@toche.fr
 */

#include "SmarTC_TSL2591.h"
#include <Wire.h>
#include <stdlib.h>

SmarTC_TSL2591::SmarTC_TSL2591()
{
  i_init = false;
}

boolean SmarTC_TSL2591::begin(void)
{
  Wire.begin();

  uint8_t id = read8(TSL2591_COMMAND_BIT | TSL2591_REG_DEVICE_ID);
  if (id != 0x50)
    return false;

  uint8_t pid = read8(TSL2591_COMMAND_BIT | TSL2591_REG_PACKAGE_PID);
  Serial.printf("\nInit. Package ID : %i\n", pid);

  setConfig(TSL2591_IT_100MS, TSL2591_GAIN_MED);

  i_init = true;
  disable();

  return true;
}

boolean SmarTC_TSL2591::begin(tsl2591_it_e it, tsl2591_g_e gain)
{
  Wire.begin();

  i_it = it;

  uint8_t id = read8(TSL2591_COMMAND_BIT | TSL2591_REG_DEVICE_ID);
  if (id != 0x50)
    return false;

  uint8_t pid = read8(TSL2591_COMMAND_BIT | TSL2591_REG_PACKAGE_PID);
  Serial.printf("\nInit. Package ID : %i\n", pid);

  setConfig(it, gain);

  i_init = true;
  disable();

  return true;
}

void SmarTC_TSL2591::enable(void)
{
  if (!i_init)
    return;

  write8(TSL2591_COMMAND_BIT | TSL2591_REG_ENABLE,
         TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);
}

void SmarTC_TSL2591::disable(void)
{
  if (!i_init)
    return;

  write8(TSL2591_COMMAND_BIT | TSL2591_REG_ENABLE, TSL2591_ENABLE_POWEROFF);
}

void SmarTC_TSL2591::setConfig(tsl2591_it_e integration, tsl2591_g_e gain)
{
  if (!i_init)
    return;
 
  enable();
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_CONFIG, integration | gain);
  disable();
}

uint32_t SmarTC_TSL2591::getFullLuminosity(void)
{
  if (!i_init)
    return 0;

  enable();

  // Wait for ADC cycles to integrate value (max 105ms per cycle)
  for (uint8_t c=0; c<=i_it; c++)
    delay(105);

  // Be sure we have an available value
  uint8_t st;
  while(true)
  {
    st = read8( TSL2591_COMMAND_BIT | TSL2591_REG_DEVICE_STATUS );
    if ( st & 0x1 )
      break;
    else
      delay(5);
  }

  // Both value must be read with chan 0 first (see datasheet)
  uint16_t c0;
  uint32_t c1;
  c0 = read16(TSL2591_COMMAND_BIT | TSL2591_REG_CHAN0_LOW);
  c1 = read16(TSL2591_COMMAND_BIT | TSL2591_REG_CHAN1_LOW);
  c1 <<= 16;
  c1 |= c0;

  disable();

  return c1;
}

uint16_t SmarTC_TSL2591::getChan0(void)
{
  uint16_t ret = (uint16_t) (getFullLuminosity() & 0xFFFF);
  return ret;
}

uint16_t SmarTC_TSL2591::getChan1(void)
{
  uint16_t ret = (uint16_t) ((getFullLuminosity()>>16) & 0xFFFF);
  return ret;
}

uint16_t SmarTC_TSL2591::getChan0(uint32_t fullValue)
{
  uint16_t ret = (uint16_t) (fullValue & 0xFFFF);
  return ret;
}

uint16_t SmarTC_TSL2591::getChan1(uint32_t fullValue)
{
  uint16_t ret = (uint16_t) ((fullValue>>16) & 0xFFFF);
  return ret;
}

void SmarTC_TSL2591::registerInt(uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591_pf_e persist = TSL2591_PERSIST_ANY)
{
  if (!i_init)
    return;

  enable();
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_PERSIST_FILTER, persist);
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_THRESHOLD_AILTL, lowerThreshold);
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_THRESHOLD_AILTH, lowerThreshold >> 8);
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_THRESHOLD_AIHTL, upperThreshold);
  write8(TSL2591_COMMAND_BIT | TSL2591_REG_THRESHOLD_AIHTH, upperThreshold >> 8);
  disable();
}

void SmarTC_TSL2591::clearInt()
{
  if (!i_init)
  return;

  enable();
  write8(TSL2591_CLEAR_INT);
  disable();
}

uint8_t SmarTC_TSL2591::getStatus(void)
{
  if (!i_init)
    return 0;

  enable();
  uint8_t x;
  x = read8(TSL2591_COMMAND_BIT | TSL2591_REG_DEVICE_STATUS);
  disable();
  return x;
}

uint8_t SmarTC_TSL2591::read8(uint8_t reg)
{
  uint8_t x;

  Wire.begin();

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 1);
  x = Wire.read();

  return x;
}

uint16_t SmarTC_TSL2591::read16(uint8_t reg)
{
  uint16_t x;
  uint16_t t;

  Wire.begin();

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 2);
  t = Wire.read();
  x = Wire.read();

  x <<= 8;
  x |= t;
  return x;
}

void SmarTC_TSL2591::write8(uint8_t reg, uint8_t value)
{
  Wire.begin();

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void SmarTC_TSL2591::write8(uint8_t reg)
{
  Wire.begin();

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
}
