/*
   @file    ILPS22QS_DataLogTerminal_I3C_ENTDAA.ino
   @author  STMicroelectronics
   @brief   Example to use the ILPS22QS barometer with I3C dynamic address assignment
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.
*******************************************************************************
*/
#include <ILPS22QSSensor.h>

ILPS22QSSensor sensor(&I3C);

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

  I3CDiscoveredDevice devices[8] = {};
  size_t found = 0;
  if (I3C.discover(devices, 8, &found)) {
    while (1) {}
  }

  uint8_t ilpDynAddr = 0U;

  for (size_t index = 0; index < found; ++index) {
    Serial.println(devices[index].pid, HEX);
    if (devices[index].pid == ILPS22QS_I3C_PID) {
      ilpDynAddr = devices[index].dynAddr;
      Serial.print("ilpDynAddr=");
      Serial.println(ilpDynAddr, HEX);
      break;
    }
  }

  if (ilpDynAddr == 0U) {
    Serial.println("ILPS22QS not found");
    while (1) {}
  }
  if (sensor.begin(ilpDynAddr) != ILPS22QS_OK) {
    Serial.println("sensor.begin() failed");
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
