#include <Arduino.h>
#include <SmarTC_TSL2591.h>

/**
 * @brief Example for ESP8266 Module
 */

// Example with Generic TSL2591 Module (https://www.aliexpress.com/item/33012674292.html?spm=a2g0s.9042311.0.0.7a174c4dtwOCox)

SmarTC_TSL2591 light = SmarTC_TSL2591();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  if (light.begin(TSL2591_IT_100MS, TSL2591_GAIN_MAX ))
    Serial.printf("\n\nTSL2591 found and init ok\n");
  else
  {
    Serial.printf("TSL2591 not found !\n");
    Serial.printf("Press reset button to reboot and retry\n");
    while (1);
  }
}

void loop()
{
  uint32_t fl = light.getFullLuminosity();
  uint16_t c0 = light.getChan0(fl);
  uint16_t c1 = light.getChan1(fl);

  Serial.printf("[ %lu ] FL: %i  CH0: %i  CH1: %i\n", millis(), fl, c0, c1);
}