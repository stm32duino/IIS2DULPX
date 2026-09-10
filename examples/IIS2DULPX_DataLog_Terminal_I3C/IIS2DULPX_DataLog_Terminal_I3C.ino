/*
   @file    IIS2DULPX_DataLog_Terminal_I3C.ino
   @author  STMicroelectronics
   @brief   Example to use the IIS2DULPX accelerometer sensor with I3C and SETDASA command
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/
#include <IIS2DULPXSensor.h>

#define IIS2DULPX_DYNAMIC_ADDRESS 0x30

IIS2DULPXSensor sensor(&I3C, IIS2DULPX_I3C_ADD_H);

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
  if (!I3C.assignDynamicAddress(sensor.getStaticAddress(), IIS2DULPX_DYNAMIC_ADDRESS)) {
    while (1) {}
  }
  if (sensor.begin(IIS2DULPX_DYNAMIC_ADDRESS) != IIS2DULPX_OK) {
    while (1) {}
  }
  if (!I3C.setClock(12500000)) {
    while (1) {}
  }
  if (sensor.Enable_X() != IIS2DULPX_OK) {
    while (1) {}
  }
}

void loop()
{
  IIS2DULPX_Axes_t accel;

  if (sensor.Get_X_Axes(&accel) == IIS2DULPX_OK) {
    Serial.print("Accel-X[mg]:");
    Serial.print(accel.x);
    Serial.print(",Accel-Y[mg]:");
    Serial.print(accel.y);
    Serial.print(",Accel-Z[mg]:");
    Serial.println(accel.z);
  }
  delay(500);
}
