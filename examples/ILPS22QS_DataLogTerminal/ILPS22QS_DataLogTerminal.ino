/*
   @file    ILPS22QS_DataLogTerminal.ino
   @author  STMicroelectronics
   @brief   Example to use the ILPS22QS absolute digital output barometer
 *******************************************************************************
   Copyright (c) 2025, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at: opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

// Includes
#include <ILPS22QSSensor.h>

// Create  an instance of the ILPS22QS sensor
ILPS22QSSensor sensor(&Wire);

void setup()
{
  // Initialize LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output
  Serial.begin(115200);

  // Initialize bus interface
  Wire.begin();

  // Initialize sensor component
  if (sensor.begin() != ILPS22QS_OK) {
    Serial.println("Failed to initialize ILPS22QS sensor.");
    while (1);
  }

  // Enable the sensor
  if (sensor.Enable() != ILPS22QS_OK) {
    Serial.println("Failed to enable ILPS22QS sensor.");
    while (1);
  }
  Serial.println("ILPS22QS sensor initialized and enabled.");
}

void loop()
{
  // Variables to store pressure and temperature
  float pressure, temperature;

  // Read pressure
  if (sensor.GetPressure(&pressure) != ILPS22QS_OK) {
    Serial.println("Failed to read pressure.");
  }
  // Read temperature
  if (sensor.GetTemperature(&temperature) != ILPS22QS_OK) {
    Serial.println("Failed to read temperature.");
  }

  // Print pressure and temperature values
  Serial.print("Press[hPa]:");
  Serial.print(pressure, 2);
  Serial.print(", Temp[C]:");
  Serial.println(temperature, 2);

  // LED blinking for status indication
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}