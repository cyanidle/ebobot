/**
@file FaBoPWM_PCA9685.cpp
@brief This is a library for the FaBo PWM.

  Released under APACHE LICENSE, VERSION 2.0

  http://www.apache.org/licenses/
0x75
@author FaBo<info@fabo.io>
*/

#ifndef FABOPWM_PCA9685_H
#define FABOPWM_PCA9685_H

#include "Arduino.h"
#include "Wire.h"

/** PCA9685 Slave Address register */
#define PCA9685_SLAVE_ADDRESS 0x40
/** Mode register1 */
#define PCA9685_MODE1 0x00

#define PCA9685_ALL_LED_ON_L_REG 0xFA
#define PCA9685_ALL_LED_ON_H_REG 0xFB
#define PCA9685_ALL_LED_OFF_L_REG 0xFC
#define PCA9685_ALL_LED_OFF_H_REG 0xFD
#define PCA9685_MODE1_REG 0x00
#define PCA9685_MODE2_REG 0x01
#define PCA9685_PRE_SCALE_REG 0xFE
/** LED0 output and brightness control byte 0. */
#define PCA9685_LED0_ON_L_REG 0x06 
/** LED0 output and brightness control byte 1. */
#define PCA9685_LED0_ON_H_REG 0x07 
/** LED0 output and brightness control byte 2. */
#define PCA9685_LED0_OFF_L_REG 0x08 
/** LED0 output and brightness control byte 3. */
#define PCA9685_LED0_OFF_H_REG 0x09

#define PCA9685_OSC_CLOCK 25000000.0

#define PCA9685_DEFAULT_VALUE 300

/** Mode1 */
#define PCA9685_RESTART 0x80
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALL 0x01
/** Mode2 */
#define PCA9685_OUTDRV 0x04

/**
 @class FaBoPWM
 @brief PCA9685 Control
*/
class FaBoPWM
{
public:
	FaBoPWM(uint8_t addr = PCA9685_SLAVE_ADDRESS);
	bool begin(void);
	void init(uint8_t value = PCA9685_DEFAULT_VALUE);
	uint16_t calc_prescale(uint16_t hz);
	void set_hz(uint16_t hz);
	void calc_hz(uint16_t prescale, uint16_t *value);
	void set_channel_value(uint8_t channel, uint16_t value);
	uint16_t get_channel_value(uint8_t channel);
private:
	uint8_t _i2caddr;
	boolean checkI2c(uint8_t register_addr);
	void writeI2c(uint8_t register_addr, uint8_t value);
	void readI2c(uint8_t register_addr, int num, uint8_t *buf);
};

#endif
