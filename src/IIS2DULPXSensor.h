/**
 ******************************************************************************
 * @file    IIS2DULPXSensor.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    30 May 2025
 * @brief   Abstract Class of a IIS2DULPX sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __IIS2DULPXSensor_H__
#define __IIS2DULPXSensor_H__
/* Includes ------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
  #ifndef MSBFIRST
    #define MSBFIRST SPI_MSBFIRST
  #endif
#endif
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "iis2dulpx_reg.h"
/* Defines -------------------------------------------------------------------*/
#define IIS2DULPX_ACC_SENSITIVITY_FOR_FS_2G   0.061f  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DULPX_ACC_SENSITIVITY_FOR_FS_4G   0.122f  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DULPX_ACC_SENSITIVITY_FOR_FS_8G   0.244f  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DULPX_ACC_SENSITIVITY_FOR_FS_16G  0.488f  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */
#define IIS2DULPX_QVAR_GAIN  74.4f  /**< Gain value for bits AH_QVAR_GAIN_[1:0] == 1 in AH_QVAR_CFG register, 16 bits [LSB/mV] */
#define IIS2DULPX_QVAR_GAIN_MULTIPL_0_5X  0.5f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 0 in AH_QVAR_CFG register [-] */
#define IIS2DULPX_QVAR_GAIN_MULTIPL_1X    1.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 1 in AH_QVAR_CFG register [-] */
#define IIS2DULPX_QVAR_GAIN_MULTIPL_2X    2.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 2 in AH_QVAR_CFG register [-] */
#define IIS2DULPX_QVAR_GAIN_MULTIPL_4X    4.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 3 in AH_QVAR_CFG register [-] */
/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  IIS2DULPX_OK = 0,
  IIS2DULPX_ERROR = -1
} IIS2DULPXStatusTypeDef;
typedef enum {
  IIS2DULPX_INT1_PIN,
  IIS2DULPX_INT2_PIN,
} IIS2DULPX_SensorIntPin_t;
typedef enum {
  IIS2DULPX_ULTRA_LOW_POWER,
  IIS2DULPX_LOW_POWER,
  IIS2DULPX_HIGH_PERFORMANCE,
} IIS2DULPX_Power_Mode_t;
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} iis2dulpx_axis3bit16_t;
typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} iis2dulpx_axis1bit16_t;
typedef union {
  int32_t i32bit[3];
  uint8_t u8bit[12];
} iis2dulpx_axis3bit32_t;
typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} iis2dulpx_axis1bit32_t;
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} IIS2DULPX_AxesRaw_t;
typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} IIS2DULPX_Axes_t;
typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} IIS2DULPX_Event_Status_t;
/* Class Declaration ---------------------------------------------------------*/
/**
 * Abstract class of a IIS2DULPX pressure sensor.
 */
class IIS2DULPXSensor {
  public:
    IIS2DULPXSensor(TwoWire *i2c, uint8_t address = IIS2DULPX_I2C_ADD_H);
    IIS2DULPXSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
    IIS2DULPXStatusTypeDef begin();
    IIS2DULPXStatusTypeDef end();
    IIS2DULPXStatusTypeDef ExitDeepPowerDownI2C();
    IIS2DULPXStatusTypeDef ExitDeepPowerDownSPI();
    IIS2DULPXStatusTypeDef ReadID(uint8_t *Id);
    IIS2DULPXStatusTypeDef Enable_X();
    IIS2DULPXStatusTypeDef Disable_X();
    IIS2DULPXStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    IIS2DULPXStatusTypeDef Get_X_OutputDataRate(float *Odr);
    IIS2DULPXStatusTypeDef Set_X_OutputDataRate(float Odr);
    IIS2DULPXStatusTypeDef Set_X_OutputDataRate_With_Mode(float Odr, IIS2DULPX_Power_Mode_t Power);
    IIS2DULPXStatusTypeDef Get_X_FullScale(int32_t *FullScale);
    IIS2DULPXStatusTypeDef Set_X_FullScale(int32_t FullScale);
    IIS2DULPXStatusTypeDef Get_X_AxesRaw(IIS2DULPX_AxesRaw_t *Value);
    IIS2DULPXStatusTypeDef Get_X_Axes(IIS2DULPX_Axes_t *Acceleration);
    IIS2DULPXStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    IIS2DULPXStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    IIS2DULPXStatusTypeDef Set_Interrupt_Latch(uint8_t Status);
    IIS2DULPXStatusTypeDef Enable_X_DRDY_Interrupt();
    IIS2DULPXStatusTypeDef Disable_X_DRDY_Interrupt();
    IIS2DULPXStatusTypeDef Set_X_SelfTest(uint8_t Val);
    IIS2DULPXStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    IIS2DULPXStatusTypeDef Get_X_Init_Status(uint8_t *Status);
    IIS2DULPXStatusTypeDef Get_X_Event_Status(IIS2DULPX_Event_Status_t *Status);
    IIS2DULPXStatusTypeDef Enable_Wake_Up_Detection(IIS2DULPX_SensorIntPin_t IntPin);
    IIS2DULPXStatusTypeDef Disable_Wake_Up_Detection();
    IIS2DULPXStatusTypeDef Set_Wake_Up_Threshold(uint32_t Threshold);
    IIS2DULPXStatusTypeDef Set_Wake_Up_Duration(uint8_t Duration);
    IIS2DULPXStatusTypeDef Enable_6D_Orientation(IIS2DULPX_SensorIntPin_t IntPin);
    IIS2DULPXStatusTypeDef Disable_6D_Orientation();
    IIS2DULPXStatusTypeDef Set_6D_Orientation_Threshold(uint8_t Threshold);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_XL(uint8_t *XLow);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_XH(uint8_t *XHigh);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_YL(uint8_t *YLow);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_YH(uint8_t *YHigh);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_ZL(uint8_t *ZLow);
    IIS2DULPXStatusTypeDef Get_6D_Orientation_ZH(uint8_t *ZHigh);
    IIS2DULPXStatusTypeDef Set_Mem_Bank(uint8_t Val);
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);
        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);
        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }
        return 0;
      }
      return 1;
    }
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }
        dev_i2c->endTransmission(true);
        return 0;
      }
      return 1;
    }
  private:
    IIS2DULPXStatusTypeDef Set_X_OutputDataRate_When_Enabled(float_t Odr, IIS2DULPX_Power_Mode_t power_mode);
    IIS2DULPXStatusTypeDef Set_X_OutputDataRate_When_Disabled(float_t Odr, IIS2DULPX_Power_Mode_t power_mode);
    /* Helper classes. */
    TwoWire  *dev_i2c;
    SPIClass *dev_spi;
    /* Configuration */
    uint8_t  address;
    int      cs_pin;
    uint32_t spi_speed;
    uint8_t  is_initialized;
    uint8_t  acc_is_enabled;
    float   acc_odr;
    IIS2DULPX_Power_Mode_t power_mode;
    iis2dulpx_ctx_t reg_ctx;
};
#ifdef __cplusplus
extern "C" {
#endif
int32_t IIS2DULPX_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t IIS2DULPX_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif
#endif /* __IIS2DULPXSensor_H__ */
