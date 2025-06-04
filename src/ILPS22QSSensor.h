/**
 ******************************************************************************
 * @file    ILPS22QSSensor.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    21 May 2025
 * @brief   Abstract Class of a ILPS22QS sensor.
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

#ifndef __ILPS22QSSensor_H__
#define __ILPS22QSSensor_H__


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
#include "ilps22qs_reg.h"


/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  ILPS22QS_OK = 0,
  ILPS22QS_ERROR = -1
} ILPS22QSStatusTypeDef;


typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} ilps22qs_axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} ilps22qs_axis1bit16_t;

typedef union {
  int32_t i32bit[3];
  uint8_t u8bit[12];
} ilps22qs_axis3bit32_t;

typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} ilps22qs_axis1bit32_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a ILPS22QS pressure sensor.
 */
class ILPS22QSSensor {
  public:
    ILPS22QSSensor(TwoWire *i2c, uint8_t address = ILPS22QS_I2C_ADD);
    ILPS22QSSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
    ILPS22QSStatusTypeDef begin();
    ILPS22QSStatusTypeDef end();
    ILPS22QSStatusTypeDef ReadID(uint8_t *Id);
    ILPS22QSStatusTypeDef Get_Init_Status(uint8_t *Status);
    ILPS22QSStatusTypeDef Enable();
    ILPS22QSStatusTypeDef Disable();
    ILPS22QSStatusTypeDef GetOutputDataRate(float *Odr);
    ILPS22QSStatusTypeDef SetOutputDataRate(float Odr);
    ILPS22QSStatusTypeDef GetPressure(float *Value);
    ILPS22QSStatusTypeDef Get_Press_DRDY_Status(uint8_t *Status);
    ILPS22QSStatusTypeDef GetTemperature(float *Value);
    ILPS22QSStatusTypeDef Get_Temp_DRDY_Status(uint8_t *Status);
    ILPS22QSStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    ILPS22QSStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    ILPS22QSStatusTypeDef Set_One_Shot();
    ILPS22QSStatusTypeDef Get_One_Shot_Status(uint8_t *Status);
    ILPS22QSStatusTypeDef Set_AVG(uint8_t avg);
    ILPS22QSStatusTypeDef Set_LPF(uint8_t lpf);
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

    ILPS22QSStatusTypeDef SetOutputDataRate_When_Enabled(float Odr);
    ILPS22QSStatusTypeDef SetOutputDataRate_When_Disabled(float Odr);
    ILPS22QSStatusTypeDef Initialize();

    /* Helper classes. */
    TwoWire  *dev_i2c;
    SPIClass *dev_spi;

    /* Configuration */
    uint8_t  address;
    int      cs_pin;
    uint32_t spi_speed;

    uint8_t is_initialized;
    uint8_t is_enabled;
    ilps22qs_md_t last_odr;
    ilps22qs_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t ILPS22QS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t ILPS22QS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif /* __ILPS22QSSensor_H__ */
