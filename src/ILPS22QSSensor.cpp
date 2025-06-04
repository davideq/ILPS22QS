/**
 ******************************************************************************
 * @file    ILPS22QSSensor.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    21 May 2025
 * @brief   Implementation of a ILPS22QS sensor.
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


/* Includes ------------------------------------------------------------------*/

#include "ILPS22QSSensor.h"
#include <string.h>

/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the ilps22qs's instance
 */
ILPS22QSSensor::ILPS22QSSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = ILPS22QS_io_write;
  reg_ctx.read_reg = ILPS22QS_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  is_initialized = 0;
  is_enabled = 0;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
ILPS22QSSensor::ILPS22QSSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = ILPS22QS_io_write;
  reg_ctx.read_reg = ILPS22QS_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  is_initialized = 0;
  is_enabled = 0;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ILPS22QSStatusTypeDef ILPS22QSSensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }
  if (is_initialized == 0U) {
    if (Initialize() != ILPS22QS_OK) {
      return ILPS22QS_ERROR;
    }
  }
  is_initialized = 1U;
  return ILPS22QS_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ILPS22QSStatusTypeDef ILPS22QSSensor::end()
{
  if (is_initialized == 1U && Disable() != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }
  if (is_initialized == 1U) {
    if (Disable() != ILPS22QS_OK) {
      return ILPS22QS_ERROR;
    }

    if (Disable() != ILPS22QS_OK) {
      return ILPS22QS_ERROR;
    }
  }

  is_initialized = 0;

  return ILPS22QS_OK;
}



/**
  * @brief  Get WHO_AM_I value
  * @param  Id the WHO_AM_I value
  * @retval 0 in case of success, an error code otherwise
  */
ILPS22QSStatusTypeDef ILPS22QSSensor::ReadID(uint8_t *Id)
{
  ilps22qs_id_t val;
  if (ilps22qs_id_get(&reg_ctx, &val) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  *Id = val.whoami;

  return ILPS22QS_OK;
}


/**
* @brief  Set the ILPS22QS pressure sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::SetOutputDataRate(float Odr)
{
  /* Check if the component is enabled */
  if (is_enabled == 1U) {
    return SetOutputDataRate_When_Enabled(Odr);
  } else {
    return SetOutputDataRate_When_Disabled(Odr);
  }
}


/**
* @brief  Get the ILPS22QS pressure value
* @param  Value pointer where the pressure value is written
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::GetPressure(float *Value)
{
  ilps22qs_data_t data;
  ilps22qs_md_t md;

  if (ilps22qs_mode_get(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  if (md.fs != ILPS22QS_1260hPa) { /* NOTE: Currently only 1260 hPa full scale supported */
    md.fs = ILPS22QS_1260hPa;

    if (ilps22qs_mode_set(&reg_ctx, &md) != ILPS22QS_OK) {
      return ILPS22QS_ERROR;
    }
  }

  if (ilps22qs_data_get(&reg_ctx, &md, &data) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  *Value = data.pressure.hpa;

  return ILPS22QS_OK;
}


/**
* @brief  Get the ILPS22QS pressure data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Get_Press_DRDY_Status(uint8_t *Status)
{
  ilps22qs_status_t reg;

  if (ilps22qs_read_reg(&reg_ctx, ILPS22QS_STATUS, (uint8_t *) &reg, 1) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  *Status = reg.p_da;

  return ILPS22QS_OK;
}


/**
* @brief  Enable the ILPS22QS temperature sensor
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Enable()
{
  /* Check if the component is already enabled */
  if (is_enabled == 1U) {
    return ILPS22QS_OK;
  }

  /* Output data rate selection. */
  if (ilps22qs_mode_set(&reg_ctx, &last_odr) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  is_enabled = 1;

  return ILPS22QS_OK;
}


/**
* @brief  Disable the ILPS22QS temperature sensor
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Disable()
{
  /* Check if the component is already disabled */
  if (is_enabled == 0U) {
    return ILPS22QS_OK;
  }

  is_enabled = 0;

  return ILPS22QS_OK;
}


/**
* @brief  Get the ILPS22QS temperature value
* @param  Value pointer where the temperature value is written
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::GetTemperature(float *Value)
{
  ilps22qs_data_t data;
  ilps22qs_md_t md;

  if (ilps22qs_mode_get(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  if (md.fs != ILPS22QS_1260hPa) { /* NOTE: Currently only 1260 hPa full scale supported */
    md.fs = ILPS22QS_1260hPa;

    if (ilps22qs_mode_set(&reg_ctx, &md) != ILPS22QS_OK) {
      return ILPS22QS_ERROR;
    }
  }

  if (ilps22qs_data_get(&reg_ctx, &md, &data) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  *Value = data.heat.deg_c;

  return ILPS22QS_OK;
}


/**
* @brief  Get the ILPS22QS temperature data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Get_Temp_DRDY_Status(uint8_t *Status)
{
  ilps22qs_status_t reg;

  if (ilps22qs_read_reg(&reg_ctx, ILPS22QS_STATUS, (uint8_t *) &reg, 1) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  *Status = reg.t_da;

  return ILPS22QS_OK;
}


/**
* @brief  Get the ILPS22QS register value
* @param  Reg address to be written
* @param  Data value to be written
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (ilps22qs_read_reg(&reg_ctx, Reg, Data, 1) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Set the ILPS22QS register value
* @param  Reg address to be written
* @param  Data value to be written
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (ilps22qs_write_reg(&reg_ctx, Reg, &Data, 1) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}

/**
  * @}
  */

/** @defgroup ILPS22QS_Private_Functions ILPS22QS Private Functions
  * @{
  */


/**
* @brief  Get output data rate
* @param  Odr the output data rate value
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::GetOutputDataRate(float *Odr)
{
  ilps22qs_md_t val;

  if (ilps22qs_mode_get(&reg_ctx, &val) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  switch (val.odr) {
    case ILPS22QS_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case ILPS22QS_1Hz:
      *Odr = 1.0f;
      break;

    case ILPS22QS_4Hz:
      *Odr = 4.0f;
      break;

    case ILPS22QS_10Hz:
      *Odr = 10.0f;
      break;

    case ILPS22QS_25Hz:
      *Odr = 25.0f;
      break;

    case ILPS22QS_50Hz:
      *Odr = 50.0f;
      break;

    case ILPS22QS_75Hz:
      *Odr = 75.0f;
      break;

    case ILPS22QS_100Hz:
      *Odr = 100.0f;
      break;

    case ILPS22QS_200Hz:
      *Odr = 200.0f;
      break;

    default:
      break;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Set output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::SetOutputDataRate_When_Enabled(float Odr)
{
  ilps22qs_md_t new_val;

  if (ilps22qs_mode_get(&reg_ctx, &new_val) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  new_val.odr = (Odr <=   1.0f) ? ILPS22QS_1Hz
                : (Odr <=   4.0f) ? ILPS22QS_4Hz
                : (Odr <=  10.0f) ? ILPS22QS_10Hz
                : (Odr <=  25.0f) ? ILPS22QS_25Hz
                : (Odr <=  50.0f) ? ILPS22QS_50Hz
                : (Odr <=  75.0f) ? ILPS22QS_75Hz
                : (Odr <= 100.0f) ? ILPS22QS_100Hz
                :                   ILPS22QS_200Hz;

  if (ilps22qs_mode_set(&reg_ctx, &new_val) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  if (ilps22qs_mode_get(&reg_ctx, &last_odr) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Set output data rate when disabled
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::SetOutputDataRate_When_Disabled(float Odr)
{
  last_odr.odr = (Odr <=   1.0f) ? ILPS22QS_1Hz
                 : (Odr <=   4.0f) ? ILPS22QS_4Hz
                 : (Odr <=  10.0f) ? ILPS22QS_10Hz
                 : (Odr <=  25.0f) ? ILPS22QS_25Hz
                 : (Odr <=  50.0f) ? ILPS22QS_50Hz
                 : (Odr <=  75.0f) ? ILPS22QS_75Hz
                 : (Odr <= 100.0f) ? ILPS22QS_100Hz
                 :                   ILPS22QS_200Hz;

  return ILPS22QS_OK;
}


/**
* @brief  Initialize the ILPS22QS sensor
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Initialize()
{
  ilps22qs_md_t md;
  ilps22qs_bus_mode_t bus_mode;

  /* Set bdu and if_inc recommended for driver usage */
  if (ilps22qs_init_set(&reg_ctx, ILPS22QS_DRV_RDY) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }


  bus_mode.interface = ILPS22QS_SEL_BY_HW;


  bus_mode.filter = ILPS22QS_FILTER_AUTO;
  if (ilps22qs_bus_mode_set(&reg_ctx, &bus_mode) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  /* Set Output Data Rate in Power Down */
  md.odr = ILPS22QS_ONE_SHOT;

  /* Configure basic parameters */
  md.avg = ILPS22QS_4_AVG;
  md.lpf = ILPS22QS_LPF_ODR_DIV_4;
  md.fs = ILPS22QS_1260hPa;
  md.interleaved_mode = 0;

  if (ilps22qs_mode_set(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  if (ilps22qs_mode_get(&reg_ctx, &last_odr) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  last_odr.odr = ILPS22QS_25Hz;

  return ILPS22QS_OK;
}


/**
* @brief  Set the ILPS22QS One Shot Mode
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Set_One_Shot()
{
  ilps22qs_md_t md;

  if (ilps22qs_mode_get(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  /* Start One Shot Measurement */
  if (ilps22qs_trigger_sw(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Get the ILPS22QS One Shot Status
* @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Get_One_Shot_Status(uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

  /* Get DataReady for pressure */
  if (Get_Press_DRDY_Status(&p_da) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  /* Get DataReady for temperature */
  if (Get_Temp_DRDY_Status(&t_da) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  if (p_da && t_da) {
    *Status = 1;
  } else {
    *Status = 0;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Set the ILPS22QS averaging selection
* @param  avg averaging selection to be set
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Set_AVG(uint8_t avg)
{
  ilps22qs_md_t md;

  if (avg > 7) {
    return ILPS22QS_ERROR;
  }

  if (ilps22qs_mode_get(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  switch (avg) {
    case 0:
    default:
      md.avg = ILPS22QS_4_AVG;
      break;
    case 1:
      md.avg = ILPS22QS_8_AVG;
      break;
    case 2:
      md.avg = ILPS22QS_16_AVG;
      break;
    case 3:
      md.avg = ILPS22QS_32_AVG;
      break;
    case 4:
      md.avg = ILPS22QS_64_AVG;
      break;
    case 5:
      md.avg = ILPS22QS_128_AVG;
      break;
    case 6:
      md.avg = ILPS22QS_256_AVG;
      break;
    case 7:
      md.avg = ILPS22QS_512_AVG;
      break;
  }

  if (ilps22qs_mode_set(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}


/**
* @brief  Set the ILPS22QS low pass filter
* @param  lpf low pass filter mode to be set
* @retval 0 in case of success, an error code otherwise
*/
ILPS22QSStatusTypeDef ILPS22QSSensor::Set_LPF(uint8_t lpf)
{
  ilps22qs_md_t md;

  if (lpf != 0 && lpf != 1 && lpf != 3) {
    return ILPS22QS_ERROR;
  }

  if (ilps22qs_mode_get(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  switch (lpf) {
    case 0:
    default:
      md.lpf = ILPS22QS_LPF_DISABLE;
      break;
    case 1:
      md.lpf = ILPS22QS_LPF_ODR_DIV_4;
      break;
    case 3:
      md.lpf = ILPS22QS_LPF_ODR_DIV_9;
      break;
  }

  if (ilps22qs_mode_set(&reg_ctx, &md) != ILPS22QS_OK) {
    return ILPS22QS_ERROR;
  }

  return ILPS22QS_OK;
}



int32_t ILPS22QS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((ILPS22QSSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t ILPS22QS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((ILPS22QSSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
