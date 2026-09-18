/*
   @file    ILPS22QS_DataLogTerminal_I3C.ino
   @author  STMicroelectronics
   @brief   Example to use the ILPS22QS barometer with I3C and SETDASA command
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.
*******************************************************************************
*/
#include <ILPS22QSSensor.h>

#define ILPS22QS_DYNAMIC_ADDRESS 0x30

ILPS22QSSensor sensor(&I3C, ILPS22QS_I3C_ADD);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    while (1) {}
  }
  if (!I3C.resetDynamicAddresses()) {
    while (1) {}
  }
  if (!I3C.assignDynamicAddress(sensor.getStaticAddress(), ILPS22QS_DYNAMIC_ADDRESS)) {
    while (1) {}
  }
  if (sensor.begin(ILPS22QS_DYNAMIC_ADDRESS) != ILPS22QS_OK) {
    while (1) {}
  }
  if (!I3C.setClock(12500000)) {
    while (1) {}
  }
  if (sensor.Enable() != ILPS22QS_OK) {
    while (1) {}
  }
}

void loop()
{
  float pressure;
  float temperature;

  if (sensor.GetPressure(&pressure) == ILPS22QS_OK && sensor.GetTemperature(&temperature) == ILPS22QS_OK) {
    Serial.print("Press[hPa]:");
    Serial.print(pressure, 2);
    Serial.print(", Temp[C]:");
    Serial.println(temperature, 2);
  }
  delay(500);
}
