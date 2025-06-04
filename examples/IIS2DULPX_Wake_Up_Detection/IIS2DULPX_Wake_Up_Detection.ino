/*
   @file    IIS2DULPX_Wake_Up_Detection.ino
   @author  STMicroelectronics
   @brief   Example to use the IIS2DULPX Wake Up Detection
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

#define INT1_pin A2

IIS2DULPXSensor IIS2DULPX(&Wire);

//Interrupts.
volatile int mems_event = 0;

void INT1Event_cb();

void setup()
{

  // Initialize LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output
  Serial.begin(115200);

  // Initialize bus interface
  Wire.begin();

  // Enable INT1 pin.
  attachInterrupt(INT1_pin, INT1Event_cb, RISING);

  // Initlialize components.
  IIS2DULPX.begin();
  IIS2DULPX.Enable_X();

  // Enable Wake Up Detection.
  IIS2DULPX.Enable_Wake_Up_Detection(IIS2DULPX_INT1_PIN);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    IIS2DULPX_Event_Status_t status;
    IIS2DULPX.Get_X_Event_Status(&status);
    if (status.WakeUpStatus) {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

      Serial.println("Wake up Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
