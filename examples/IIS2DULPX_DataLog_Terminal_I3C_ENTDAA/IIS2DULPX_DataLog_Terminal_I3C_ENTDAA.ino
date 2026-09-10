/*
   @file    IIS2DULPX_DataLog_Terminal_I3C_ENTDAA.ino
   @author  STMicroelectronics
   @brief   Example to use the IIS2DULPX accelerometer sensor with I3C dynamic address assignment
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.
*******************************************************************************
*/
#include <IIS2DULPXSensor.h>

IIS2DULPXSensor sensor(&I3C);

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

  if (!I3C.isI3CDeviceReady(IIS2DULPX_I3C_ADD_H)) {
    Serial.println("isI3CDeviceReady() failed");
    while (1) {}
  }
  
  I3CDiscoveredDevice devices[8] = {};
  size_t found = 0;
  if (I3C.discover(devices, 8, &found)) {
    while (1) {}
  }

  for (size_t index = 0; index < found; ++index) {
    if (sensor.begin(devices[index].dynAddr) == IIS2DULPX_OK) {
      break;
    }
  }
  if (sensor.getDynAddress() == 0U) {
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
