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
 * @defgroup SmarTC_TSL2591 "SmarTC TSL2591"
 *
 * @brief TSL2591 Light Sensor management library over Arduino SDK.
 */

/**
 * @file SmarTC_TSL2591.h
 * 
 * @brief Header for TSL2591 Light Sensor management.
 * @ingroup SmarTC_TSL2591
 * 
 * @author Clément TOCHE
 * Contact: ad@toche.fr
 */

/**
 * @class SmarTC_TSL2591
 *
 * @ingroup SmarTC_TSL2591
 *
 * @brief TSL2591 Manager class
 *
 */

#ifndef SMARTC_TSL2591_H
#define SMARTC_TSL2591_H

#include <Arduino.h>

#define TSL2591_ADDR              (0x29)    ///< Default I2C address

#define TSL2591_COMMAND_BIT       (0xA0)    ///< 1010 0000: bits 7 and 5 for 'command normal'

///! Special Function Command for "Clear ALS and no persist ALS interrupt"
#define TSL2591_CLEAR_INT         (0xE7)
///! Special Function Command for "Interrupt set - forces an interrupt"
#define TSL2591_TEST_INT          (0xE4)

#define TSL2591_ENABLE_POWEROFF   (0x00)    ///< Flag for ENABLE register to disable
#define TSL2591_ENABLE_POWERON    (0x01)    ///< Flag for ENABLE register to enable
#define TSL2591_ENABLE_AEN        (0x02)    ///< ALS Enable. This field activates ALS function. Writing a one activates the ALS. Writing a zero disables the ALS.
#define TSL2591_ENABLE_AIEN       (0x10)    ///< ALS Interrupt Enable. When asserted permits ALS interrupts to be generated, subject to the persist filter.
#define TSL2591_ENABLE_NPIEN      (0x80)    ///< No Persist Interrupt Enable. When asserted NP Threshold conditions will generate an interrupt, bypassing the persist filter

#define TSL2591_REG_ENABLE            (0x00) // Enables states and interrupts
#define TSL2591_REG_CONFIG            (0x01) // ALS gain and integration time configuration
#define TSL2591_REG_THRESHOLD_AILTL   (0x04) // ALS interrupt low threshold low byte
#define TSL2591_REG_THRESHOLD_AILTH   (0x05) // ALS interrupt low threshold high byte
#define TSL2591_REG_THRESHOLD_AIHTL   (0x06) // ALS interrupt high threshold low byte
#define TSL2591_REG_THRESHOLD_AIHTH   (0x07) // ALS interrupt high threshold high byte
#define TSL2591_REG_THRESHOLD_NPAILTL (0x08) // No Persist ALS interrupt low threshold low byte
#define TSL2591_REG_THRESHOLD_NPAILTH (0x09) // No Persist ALS interrupt low threshold high byte
#define TSL2591_REG_THRESHOLD_NPAIHTL (0x0A) // No Persist ALS interrupt high threshold low byte
#define TSL2591_REG_THRESHOLD_NPAIHTH (0x0B) // No Persist ALS interrupt high threshold high byte
#define TSL2591_REG_PERSIST_FILTER    (0x0C) // Interrupt persistence filter
#define TSL2591_REG_PACKAGE_PID       (0x11) // Package ID
#define TSL2591_REG_DEVICE_ID         (0x12) // Device ID
#define TSL2591_REG_DEVICE_STATUS     (0x13) // Device Status
#define TSL2591_REG_CHAN0_LOW         (0x14) // CH0 low data byte
#define TSL2591_REG_CHAN0_HIGH        (0x15) // CH0 high data byte
#define TSL2591_REG_CHAN1_LOW         (0x16) // CH1 low data byte
#define TSL2591_REG_CHAN1_HIGH        (0x17) // CH1 high data byte

typedef enum tsl2591_it_e
{
  TSL2591_IT_100MS     = 0x00,  // 100 millis
  TSL2591_IT_200MS     = 0x01,  // 200 millis
  TSL2591_IT_300MS     = 0x02,  // 300 millis
  TSL2591_IT_400MS     = 0x03,  // 400 millis
  TSL2591_IT_500MS     = 0x04,  // 500 millis
  TSL2591_IT_600MS     = 0x05,  // 600 millis
} tsl2591_it_e;

typedef enum tsl2591_g_e
{
  TSL2591_GAIN_LOW                  = 0x00,    /// low gain (1x)
  TSL2591_GAIN_MED                  = 0x10,    /// medium gain (25x)
  TSL2591_GAIN_HIGH                 = 0x20,    /// medium gain (428x)
  TSL2591_GAIN_MAX                  = 0x30,    /// max gain (9876x)
} tsl2591_g_e;

typedef enum tsl2591_pf_e
{
  TSL2591_PERSIST_EVERY             = 0x00, // Every ALS cycle generates an interrupt
  TSL2591_PERSIST_ANY               = 0x01, // Any value outside of threshold range
  TSL2591_PERSIST_2                 = 0x02, // 2 consecutive values out of range
  TSL2591_PERSIST_3                 = 0x03, // 3 consecutive values out of range
  TSL2591_PERSIST_5                 = 0x04, // 5 consecutive values out of range
  TSL2591_PERSIST_10                = 0x05, // 10 consecutive values out of range
  TSL2591_PERSIST_15                = 0x06, // 15 consecutive values out of range
  TSL2591_PERSIST_20                = 0x07, // 20 consecutive values out of range
  TSL2591_PERSIST_25                = 0x08, // 25 consecutive values out of range
  TSL2591_PERSIST_30                = 0x09, // 30 consecutive values out of range
  TSL2591_PERSIST_35                = 0x0A, // 35 consecutive values out of range
  TSL2591_PERSIST_40                = 0x0B, // 40 consecutive values out of range
  TSL2591_PERSIST_45                = 0x0C, // 45 consecutive values out of range
  TSL2591_PERSIST_50                = 0x0D, // 50 consecutive values out of range
  TSL2591_PERSIST_55                = 0x0E, // 55 consecutive values out of range
  TSL2591_PERSIST_60                = 0x0F, // 60 consecutive values out of range
} tsl2591_pf_e;

typedef enum tsl2591_ch_e
{
  TSL2591_VISIBLE        = 0x2, // (channel 0) - (channel 1)
  TSL2591_INFRARED       = 0x1, // channel 1
  TSL2591_FULLSPECTRUM   = 0x0, // channel 0
} tsl2591_ch_e;

class SmarTC_TSL2591
{
public:
    SmarTC_TSL2591();
  
    boolean   begin     ( void );
    boolean   begin     (tsl2591_it_e it, tsl2591_g_e gain);
    void      enable    ( void );
    void      disable   ( void );
    uint8_t   getStatus ( void );
    void      setConfig (tsl2591_it_e integration, tsl2591_g_e gain);
    
    uint32_t  getFullLuminosity ( void );
    uint16_t  getChan0          ( void );
    uint16_t  getChan1          ( void );
    uint16_t  getChan0          ( uint32_t fullValue );
    uint16_t  getChan1          ( uint32_t fullValue );

    void      clearInt(void);
    void      registerInt(uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591_pf_e persist);
  
private:
    boolean      i_init;
    tsl2591_it_e i_it;

    void      write8  ( uint8_t r);
    void      write8  ( uint8_t r, uint8_t v );
    uint16_t  read16  ( uint8_t reg );
    uint8_t   read8   ( uint8_t reg );
};
#endif // SMARTC_TSL2591_H
