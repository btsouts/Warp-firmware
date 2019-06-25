/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA260.h"

extern volatile WarpI2CDeviceState	deviceINA260State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

volatile uint32_t		gWarpI2cTimeoutMilliseconds2 = 1000;

/*!
 * @file Adafruit_INA260.h
 *
 * This is a library for the Adafruit INA260 breakout board
 * ----> https://www.adafruit.com/products/904
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#define INA260_I2CADDR_DEFAULT  0x40 ///< INA260 default i2c address
#define INA260_REG_CONFIG       0x00 ///< Configuration register
#define INA260_REG_CURRENT      0x01 ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE   0x02 ///< Bus voltage measurement register in mV
#define INA260_REG_POWER        0x03 ///< Power calculation register in mW
#define INA260_REG_MASK_ENABLE  0x06 ///< Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT  0x07 ///< Alert limit value register
#define INA260_REG_MFG_UID      0xFE ///< Manufacturer ID Register
#define INA260_REG_DIE_UID      0xFF ///< Die ID and Revision Register

/**
 * @brief Mode options.
 *
 * Allowed values for setMode.
 */
typedef enum _mode {
  INA260_MODE_SHUTDOWN = 0x00, /**< SHUTDOWN: Minimize quiescient current and turn
                                    off current into the device inputs. Set another
                                    mode to exit shutown mode **/
  INA260_MODE_TRIGGERED  = 0x03, /**< TRIGGERED: Trigger a one-shot measurement
                                      of current and bus voltage. Set the TRIGGERED
                                      mode again to take a new measurement **/
  INA260_MODE_CONTINUOUS = 0x07, /**< CONTINUOUS: (Default) Continuously update
                                      the current, bus voltage and power registers
                                      with new measurements **/
} INA260_MeasurementMode;


/**
 * @brief Conversion Time options.
 *
 * Allowed values for setCurrentConversionTime and setVoltageConversionTime.
 */
typedef enum _conversion_time {
  INA260_TIME_140_us, ///< Measurement time: 140us
  INA260_TIME_204_us, ///< Measurement time: 204us
  INA260_TIME_332_us, ///< Measurement time: 332us
  INA260_TIME_558_us, ///< Measurement time: 558us
  INA260_TIME_1_1_ms, ///< Measurement time: 1.1ms (Default)
  INA260_TIME_2_116_ms, ///< Measurement time: 2.116ms
  INA260_TIME_4_156_ms, ///< Measurement time: 4.156ms
  INA260_TIME_8_244_ms, ///< Measurement time: 8.224ms
} INA260_ConversionTime;

/**
 * @brief Averaging Count options.
 *
 * Allowed values forsetAveragingCount.
 */
typedef enum _count {
  INA260_COUNT_1, ///< Window size: 1 sample (Default)
  INA260_COUNT_4, ///< Window size: 4 samples
  INA260_COUNT_16, ///< Window size: 16 samples
  INA260_COUNT_64, ///< Window size: 64 samples
  INA260_COUNT_128, ///< Window size: 128 samples
  INA260_COUNT_256, ///< Window size: 256 samples
  INA260_COUNT_512, ///< Window size: 512 samples
  INA260_COUNT_1024, ///< Window size: 1024 samples
} INA260_AveragingCount;

// /*!
//  *    @brief  Create a slice of the register that we can address without touching other bits
//  *    @param  reg The Adafruit_BusIO_Register which defines the bus/register
//  *    @param  bits The number of bits wide we are slicing
//  *    @param  shift The number of bits that our bit-slice is shifted from LSB
//  */
// Adafruit_BusIO_RegisterBits::Adafruit_BusIO_RegisterBits(Adafruit_BusIO_Register *reg, uint8_t bits, uint8_t shift) {
//   _register = reg;
//   _bits = bits;
//   _shift = shift;
// }

// /*!
//  *    @brief  Read 4 bytes of data from the register
//  *    @return  data The 4 bytes to read
//  */
// uint32_t Adafruit_BusIO_RegisterBits::read(void) {
//   uint32_t val = _register->read();
//   val >>= _shift;
//   return val & ((1 << (_bits+1)) - 1);
// }


// /*!
//  *    @brief  Write 4 bytes of data to the register
//  *    @param  data The 4 bytes to write
//  */
// void Adafruit_BusIO_RegisterBits::write(uint32_t data) {
//   uint32_t val = _register->read();

//   // mask off the data before writing
//   uint32_t mask = (1 << (_bits+1)) - 1;
//   data &= mask;

//   mask <<= _shift;
//   val &= ~mask;      // remove the current data at that spot
//   val |= data << _shift; // and add in the new data
  
//   _register->write(val, _register->width());
// }

void
initINA260(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskCurrentCons
					);

    uint16_t	menuI2cPullupValue = 32768;

	// Adafruit_I2CRegister *die_register = new Adafruit_I2CRegister(i2c_dev, INA260_REG_DIE_UID, 2, MSBFIRST);
	// Adafruit_I2CRegister *mfg_register = new Adafruit_I2CRegister(i2c_dev, INA260_REG_MFG_UID, 2, MSBFIRST);
	// Adafruit_I2CRegisterBits *device_id = new Adafruit_I2CRegisterBits(die_register, 12, 4);

	// // make sure we're talking to the right chip
	// if ((mfg_register->read() != 0x5449) || (device_id->read() != 0x227)) {
	// 	return false;
	// }
	// // Read H 0x54 0x49
	// // Read B 0xFF 0xC0

	// Config = new Adafruit_I2CRegister(i2c_dev, INA260_REG_CONFIG, 2, MSBFIRST);
	// MaskEnable = new Adafruit_I2CRegister(i2c_dev, INA260_REG_MASK_ENABLE, 2, MSBFIRST);
	// AlertLimit = new Adafruit_I2CRegister(i2c_dev, INA260_REG_ALERT_LIMIT, 2, MSBFIRST);

	// reset();
	// delay(2); // delay 2ms to give time for first measurement to finish

    // configureSensorINA260(ina260_calValue,
	// 				config,
	// 				menuI2cPullupValue
	// 				);

	return;
}

WarpStatus
configureSensorINA260(uint16_t payload_CALIBRATION, uint16_t payload_CONFIGURATION, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	i2cWriteStatus1 = writeSensorRegisterINA260(kWarpSensorConfigurationRegisterINA260_CALIBRATION /* register address INA260_REG_CALIBRATION */,
	 						payload_CALIBRATION,
	 						menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterINA260(kWarpSensorConfigurationRegisterINA260_CONFIG /* register address INA260_REG_CONFIG */,
	 						payload_CONFIGURATION /* payload */,
	 						menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}	

WarpStatus
writeSensorRegisterINA260(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA260State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[1] = payload & 0xFF;
    payloadByte[0] = (payload >> 8) & 0xFF;

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA260(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		/* TODO check again */
        case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0xFE: case 0x0D: case 0x12:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceINA260State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA260State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA260(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

    //*value = ((_i2c->read() << 8) | _i2c->read());
	// 12 - 0D
	i2cReadStatus = readSensorRegisterINA260(INA260_REG_BUSVOLTAGE, 2 /* numberOfBytes */); //INA260_REG_BUSVOLTAGE INA260_REG_MFG_UID INA260_REG_CURRENT
	
	readSensorRegisterValueMSB = deviceINA260State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA260State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);
	readSensorRegisterValueCombined = readSensorRegisterValueCombined * 1.25; //Conversion according to manual

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "V 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, "H %d,", readSensorRegisterValueCombined);
		}
	}
}
