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

#include "devINA219.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

volatile uint32_t		gWarpI2cTimeoutMilliseconds2 = 1000;

/*!
 * @file Adafruit_INA219.h
 *
 * This is a library for the Adafruit INA219 breakout board
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

/** default I2C address **/
#define INA219_ADDRESS (0x40) // 1000000 (A0+A1=GND)

/** read **/
#define INA219_READ (0x01)

/*=========================================================================
    CONFIG REGISTER (R/W)
**************************************************************************/

/** config register address **/
#define INA219_REG_CONFIG (0x00)

/** reset bit **/
#define INA219_CONFIG_RESET (0x8000) // Reset Bit

/** mask for bus voltage range **/
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask

/** bus voltage range values **/
enum {
  INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
  INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** mask for gain bits **/
#define INA219_CONFIG_GAIN_MASK (0x1800) // Gain Mask

/** values for gain bits **/
enum {
  INA219_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
  INA219_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
  INA219_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
  INA219_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA219_CONFIG_BADCRES_MASK (0x0780)

/** values for bus ADC resolution **/
enum {
  INA219_CONFIG_BADCRES_9BIT = (0x0000),  // 9-bit bus res = 0..511
  INA219_CONFIG_BADCRES_10BIT = (0x0080), // 10-bit bus res = 0..1023
  INA219_CONFIG_BADCRES_11BIT = (0x0100), // 11-bit bus res = 0..2047
  INA219_CONFIG_BADCRES_12BIT = (0x0180), // 12-bit bus res = 0..4097
};

/** mask for shunt ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK                                             \
  (0x0078) // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
enum {
  INA219_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),   // 1 x 9-bit shunt sample
  INA219_CONFIG_SADCRES_10BIT_1S_148US = (0x0008), // 1 x 10-bit shunt sample
  INA219_CONFIG_SADCRES_11BIT_1S_276US = (0x0010), // 1 x 11-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_1S_532US = (0x0018), // 1 x 12-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_2S_1060US =
      (0x0048), // 2 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_4S_2130US =
      (0x0050), // 4 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_8S_4260US =
      (0x0058), // 8 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_16S_8510US =
      (0x0060), // 16 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_32S_17MS =
      (0x0068), // 32 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_64S_34MS =
      (0x0070), // 64 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_128S_69MS =
      (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

/** values for operating mode **/
enum {
  INA219_CONFIG_MODE_POWERDOWN,
  INA219_CONFIG_MODE_SVOLT_TRIGGERED,
  INA219_CONFIG_MODE_BVOLT_TRIGGERED,
  INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED,
  INA219_CONFIG_MODE_ADCOFF,
  INA219_CONFIG_MODE_SVOLT_CONTINUOUS,
  INA219_CONFIG_MODE_BVOLT_CONTINUOUS,
  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
};

/** shunt voltage register **/
#define INA219_REG_SHUNTVOLTAGE (0x01)

/** bus voltage register **/
#define INA219_REG_BUSVOLTAGE (0x02)

/** power register **/
#define INA219_REG_POWER (0x03)

/** current register **/
#define INA219_REG_CURRENT (0x04)

/** calibration register **/
#define INA219_REG_CALIBRATION (0x05)

void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskCurrentCons
					);

    uint16_t	menuI2cPullupValue = 32768;
	uint16_t 	ina219_calValue;
    
    // ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
    // ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)
	
	// ina219_calValue = 4096;
	// uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
    //                 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
    //                 INA219_CONFIG_SADCRES_12BIT_1S_532US |
    //                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	// ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
	// ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

	ina219_calValue = 8192; //setCalibration_16V_400mA

	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
					INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	
    configureSensorINA219(ina219_calValue,
					config,
					menuI2cPullupValue
					);

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
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
		.address = deviceINA219State.i2cAddress,
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
configureSensorINA219(uint16_t payload_CALIBRATION, uint16_t payload_CONFIGURATION, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CALIBRATION /* register address INA219_REG_CALIBRATION */,
	 						payload_CALIBRATION,
	 						menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CONFIG /* register address INA219_REG_CONFIG */,
	 						payload_CONFIGURATION /* payload */,
	 						menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		/* TODO check again */
        case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
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
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t		busVoltage=0, shuntVoltage=0, loadVoltage=0;

	WarpStatus	i2cReadStatus;

    //*value = ((_i2c->read() << 8) | _i2c->read());
	i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */); //INA219_REG_SHUNTVOLTAGE -- INA219_REG_CURRENT
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

	readSensorRegisterValueCombined = (readSensorRegisterValueCombined >> 3) * 4; // Enable this for bus voltage
	busVoltage = readSensorRegisterValueCombined;

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */); // -- INA219_REG_CURRENT
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

	shuntVoltage = readSensorRegisterValueCombined;

	loadVoltage = busVoltage + (shuntVoltage / 1000);

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "LV 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, "LV %d,", loadVoltage);
		}
	}
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
// void setCalibration_32V_2A(void) {
//   uint8_t ina219_i2caddr;
//   uint32_t ina219_calValue;
//   // The following multipliers are used to convert raw current and power
//   // values to mA and mW, taking into account the current config settings
//   uint32_t ina219_currentDivider_mA;
//   float ina219_powerMultiplier_mW;

//   // By default we use a pretty huge range for the input voltage,
//   // which probably isn't the most appropriate choice for system
//   // that don't use a lot of power.  But all of the calculations
//   // are shown below if you want to change the settings.  You will
//   // also need to change any relevant register settings, such as
//   // setting the VBUS_MAX to 16V instead of 32V, etc.

//   // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
//   // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
//   // RSHUNT = 0.1               (Resistor value in ohms)

//   // 1. Determine max possible current
//   // MaxPossible_I = VSHUNT_MAX / RSHUNT
//   // MaxPossible_I = 3.2A

//   // 2. Determine max expected current
//   // MaxExpected_I = 2.0A

//   // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
//   // MinimumLSB = MaxExpected_I/32767
//   // MinimumLSB = 0.000061              (61uA per bit)
//   // MaximumLSB = MaxExpected_I/4096
//   // MaximumLSB = 0,000488              (488uA per bit)

//   // 4. Choose an LSB between the min and max values
//   //    (Preferrably a roundish number close to MinLSB)
//   // CurrentLSB = 0.0001 (100uA per bit)

//   // 5. Compute the calibration register
//   // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
//   // Cal = 4096 (0x1000)

//   ina219_calValue = 4096;

//   // 6. Calculate the power LSB
//   // PowerLSB = 20 * CurrentLSB
//   // PowerLSB = 0.002 (2mW per bit)

//   // 7. Compute the maximum current and shunt voltage values before overflow
//   //
//   // Max_Current = Current_LSB * 32767
//   // Max_Current = 3.2767A before overflow
//   //
//   // If Max_Current > Max_Possible_I then
//   //    Max_Current_Before_Overflow = MaxPossible_I
//   // Else
//   //    Max_Current_Before_Overflow = Max_Current
//   // End If
//   //
//   // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
//   // Max_ShuntVoltage = 0.32V
//   //
//   // If Max_ShuntVoltage >= VSHUNT_MAX
//   //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
//   // Else
//   //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
//   // End If

//   // 8. Compute the Maximum Power
//   // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
//   // MaximumPower = 3.2 * 32V
//   // MaximumPower = 102.4W

//   // Set multipliers to convert raw current/power values
//   ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
//   ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

//   // Set Calibration register to 'Cal' calculated above
//   wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

//   // Set Config register to take into account the settings above
//   uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
//                     INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
//                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
//                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

//   wireWriteRegister(INA219_REG_CONFIG, config);
// }