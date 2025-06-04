/*
   @file    IIS2DULPX_6D_Orientation.ino
   @author  STMicroelectronics
   @brief   Example to use the IIS2DULPX 6D Orientation
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
// Interrupts.
volatile int mems_event = 0;

char report[256];

void INT1Event_cb();
void sendOrientation();

void setup()
{

  // Initialize serial.
  Serial.begin(115200);
  delay(1000);

  // Initialize LED.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize I2C.
  Wire.begin();

  // Enable INT1 pin.
  attachInterrupt(INT1_pin, INT1Event_cb, RISING);

  // Initialize components.
  IIS2DULPX.begin();
  IIS2DULPX.Enable_X();
  // Enable 6D Orientation.
  IIS2DULPXStatusTypeDef ret = IIS2DULPX.Enable_6D_Orientation(IIS2DULPX_INT1_PIN);
}

void loop()
{
  if (true) {
    mems_event = 0;

    // Initialize the structure to zero at the beginning of each loop using memset
    IIS2DULPX_Event_Status_t status;
    memset(&status, 0, sizeof(IIS2DULPX_Event_Status_t));

    IIS2DULPX.Get_X_Event_Status(&status);

    if (status.D6DOrientationStatus) {
      // Send 6D Orientation
      sendOrientation();

      // LED blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
void sendOrientation()
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  IIS2DULPX.Get_6D_Orientation_XL(&xl);
  IIS2DULPX.Get_6D_Orientation_XH(&xh);
  IIS2DULPX.Get_6D_Orientation_YL(&yl);
  IIS2DULPX.Get_6D_Orientation_YH(&yh);
  IIS2DULPX.Get_6D_Orientation_ZL(&zl);
  IIS2DULPX.Get_6D_Orientation_ZH(&zh);

  if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1) {
    sprintf(report, "\r\n  __*_____________  " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n");
  }

  else {
    sprintf(report, "None of the 6D orientation axes is set in accelerometer.\r\n");
  }

  Serial.print(report);
}