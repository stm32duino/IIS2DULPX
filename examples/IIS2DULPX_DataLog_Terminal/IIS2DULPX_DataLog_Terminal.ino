/*
   @file    IIS2DULPX_DataLog_Terminal.ino
   @author  STMicroelectronics
   @brief   Example to use the IIS2DULPX accelerometer sensor (X-axis only)
 *******************************************************************************
   Copyright (c) 2025, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

#include <IIS2DULPXSensor.h>

// Create an instance of the IIS2DULPX sensor
IIS2DULPXSensor sensor(&Wire);

void setup()
{
  // Initialize LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output
  Serial.begin(115200);

  // Initialize bus interface
  Wire.begin();

  // Initialize the sensor
  if (sensor.begin() != IIS2DULPX_OK) {
    Serial.println("Failed to initialize IIS2DULPX sensor!");
    while (1);
  }

  // Enable the accelerometer
  if (sensor.Enable_X() != IIS2DULPX_OK) {
    Serial.println("Failed to enable accelerometer!");
    while (1);
  }
  Serial.println("IIS2DULPX sensor initialized and accelerometer enabled.");
}

void loop()
{
  IIS2DULPX_Axes_t accel;
  // Get X-axis acceleration data
  if (sensor.Get_X_Axes(&accel) == IIS2DULPX_OK) {
    Serial.print("Accel-X [mg]: ");
    Serial.print(accel.x);
    Serial.print(",Accel-Y[mg]:");
    Serial.print(accel.y);
    Serial.print(",Accel-Z[mg]:");
    Serial.println(accel.z);
    // Led blinking.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("Failed to read acceleration data!");
  }
}

