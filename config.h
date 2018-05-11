/*
    config.h
    Copyright (C) 2017 Niryo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NIRYO_CONFIG_H
#define NIRYO_CONFIG_H


/*
 * Firmware version : 0.1.0
 */

#define NIRYO_STEPPER_VERSION_MAJOR 1
#define NIRYO_STEPPER_VERSION_MINOR 0
#define NIRYO_STEPPER_VERSION_PATCH 0

/*
 * All constants for Niryo One are defined below
 */


/*
 *    ----------- AS5600 Position sensor -------------
 */

#define AS5600_CPR 4096
#define AS5600_CPR_HALF 2048

#define AS5600_ADDRESS 0x36

#define AS5600_REG_CONF    0x07
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_REG_ANGLE_L 0x0F

/*
 *    ----------- A4954 Driver -------------
 */

#define KEEP_RESISTANCE_WHEN_DETACHED 1

//Defines for pins:
#define IN_1  15
#define IN_2  17
#define IN_3  3
#define IN_4  8

#define VREF_1 9
#define VREF_2 4

//for faster digitalWrite:
#define IN_1_HIGH() (REG_PORT_OUTSET1 = PORT_PB08)
#define IN_1_LOW() (REG_PORT_OUTCLR1 = PORT_PB08)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA04)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA04)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA09)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA09)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA14)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA14)

#define iMAX 2.0
#define rSense 0.150
#define uMAX (255/3.3)*(iMAX*10*rSense)   // 255 for 8-bit pwm, 1023 for 10 bit, must also edit analogFastWrite

#define UMAX_95_PERCENT  (int)(0.95 * uMAX) // 219
#define UMAX_90_PERCENT  (int)(0.90 * uMAX) // 207
#define UMAX_85_PERCENT  (int)(0.85 * uMAX) // 196
#define UMAX_80_PERCENT  (int)(0.80 * uMAX) // 184
#define UMAX_75_PERCENT  (int)(0.75 * uMAX) // 173
#define UMAX_70_PERCENT  (int)(0.70 * uMAX) // 161
#define UMAX_65_PERCENT  (int)(0.65 * uMAX) // 150
#define UMAX_60_PERCENT  (int)(0.60 * uMAX) // 138
#define UMAX_55_PERCENT  (int)(0.55 * uMAX) // 127

#define UMAX_50_PERCENT  (int)(0.50 * uMAX) // 115
#define UMAX_45_PERCENT  (int)(0.45 * uMAX) // 103
#define UMAX_40_PERCENT  (int)(0.40 * uMAX) // 92
#define UMAX_35_PERCENT  (int)(0.35 * uMAX) // 80
#define UMAX_30_PERCENT  (int)(0.30 * uMAX) // 69
#define UMAX_25_PERCENT  (int)(0.25 * uMAX) // 57
#define UMAX_20_PERCENT  (int)(0.20 * uMAX) // 46
#define UMAX_15_PERCENT  (int)(0.15 * uMAX) // 34
#define UMAX_10_PERCENT  (int)(0.10 * uMAX) // 23
#define UMAX_05_PERCENT  (int)(0.05 * uMAX) // 11

#define UMAX_DEFAULT UMAX_25_PERCENT
#define UMAX_PID     UMAX_50_PERCENT

#define spr 200   
#define aps 360.0/ spr       // angle per step

/*
 *    ----------- CAN bus -------------
 */

#define CAN_BROADCAST_ID  5

#define CAN_CMD_POSITION     0x03
#define CAN_CMD_TORQUE       0x04
#define CAN_CMD_MODE         0x07
#define CAN_CMD_MICRO_STEPS  0x13 
#define CAN_CMD_OFFSET       0x14
#define CAN_CMD_CALIBRATE    0x15
#define CAN_CMD_SYNCHRONIZE  0x16
#define CAN_CMD_MAX_EFFORT   0x17
#define CAN_CMD_MOVE_REL     0x18
#define CAN_CMD_RESET        0x19 // not yet implemented

#define CAN_DATA_POSITION    0x03
#define CAN_DATA_DIAGNOSTICS 0x08
#define CAN_DATA_CALIBRATION_RESULT 0x09
#define CAN_DATA_FIRMWARE_VERSION 0x10

// see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_zero/variant.cpp
#define CAN_PIN_CS  16  // PB09 
#define CAN_PIN_INT 38  // PA13 


/*
 *    ----------- Stepper controller -------------
 */

#define STEPPER_CPR 200

#define STEPPER_CONTROL_MODE_RELAX    0
#define STEPPER_CONTROL_MODE_STANDARD 1

#define STEPPER_DELAY_MIN 200

#define STEPPER_DEFAULT_MICRO_STEPS 8
#define ONE_FULL_STEP 1800

#define STEPPER_CALIBRATION_OK        1
#define STEPPER_CALIBRATION_TIMEOUT   2
#define STEPPER_CALIBRATION_BAD_PARAM 3

/*
 *    ----------- Driver temperature sensor -------------
 */

#define TEMPERATURE_SENSOR_PIN A4 


/*
 *    ----------- Fan -------------
 */

#define FAN_PIN 6 // PA20
#define fan_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define fan_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)

#endif
