/*
    NiryoStepper.ino
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

#include <Arduino.h>
#define Serial SerialUSB

/*
 * 
 * 
 * 
 * MOTOR UNIQUE ID
 * Each stepper on Niryo One has a different and unique ID (for can bus)
 * If you update this firmware, be sure to upload the code in each motor with the correct ID
 * ( Stepper 1 -> 1, Stepper 2 -> 2, Stepper 3 -> 3, Stepper 4 -> 4)
 * 
 * 
 * 
 * 
 */

#define MOTOR_ID   2  // <-- change this value for each stepper motor (1-4) on Niryo One

/*
 * 
 * --> If you did not read the comment above, please read it now :)
 * 
 */

#include <Wire.h>
#include <SPI.h>

#include "config.h"
#include "utils.h"
#include "A4954.h"
#include "AS5600.h"
#include "StepperController.h"
#include "CanBus.h"

uint8_t motor_id = MOTOR_ID;

// Stepper controller
StepperController stepper;

// Can driver
MCP_CAN can_driver(CAN_PIN_CS);

// Can Bus (will transfer commands and data between CAN driver and stepper controller)
CanBus canBus(&can_driver, &stepper, motor_id);

unsigned int driver_temperature = 0;

unsigned long time_last_write_position = micros();
unsigned long write_frequency_position = 80000; // 12.5Hz

unsigned long time_last_write_diagnostics = micros();
unsigned long write_frequency_diagnostics = 2000000; // 0.5Hz

unsigned long time_last_read_temperature = micros();
unsigned long read_temperature_frequency = 2000000; // 0.5 Hz

unsigned long time_last_write_firmware_version = micros();
unsigned long write_frequency_firmware_version = 5000000; // 0.2 Hz

// This is called every 0.5 seconds
void debug_serial()
{
  //SerialUSB.print("Sensor position : ");
  //SerialUSB.println(sensor_position);

  /*SerialUSB.print("Motor position : ");
  SerialUSB.print(motor_position_steps);
  //SerialUSB.print(", current steps : ");
  //SerialUSB.print(stepper.getCurrentStepNumber());
  SerialUSB.print(", goal steps : ");
  SerialUSB.println(stepper.getGoalStepNumber());
  SerialUSB.print("Sensor position : ");
  SerialUSB.println(sensor_position);
  */
}

long time_begin_debug_serial = micros();

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

void setup() {
  SerialUSB.begin(115200);  
  delay(2000);
  SerialUSB.println("-------------- START --------------");
  canBus.setup();
  
  Wire.begin();
  Wire.setClock(1000000); // 1 Mbits
  delay(100);

  // start fan
  setup_fan();
  fan_HIGH();

  // speed up analogRead() function
  init_analog_fast_read();

  // make sensor give a more recent value 
  // it is better not to activate it for normal Niryo One usage
  //speed_up_position_sensor_response_time();

  // set register to read once
  init_position_sensor();

  // setup pins for motor driver
  init_driver();

  delay(1000);
  
  time_begin_debug_serial = micros();
  
  stepper.start();
  stepper.setControlMode(STEPPER_CONTROL_MODE_RELAX);
  
  SerialUSB.println("-------------- SETUP FINISHED --------------");
}

//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////

bool action_available = true;
bool analog_read_enable = true;

void loop() {

  action_available = true;

  // read position from sensor
  update_current_position(stepper.getMicroSteps());

  // update stepper controller
  stepper.update();

  // read CAN if available
  if (action_available) {
    if(canBus.available()) 
    {
      canBus.read();
      action_available = false;
    }
  }
 
  // write position CAN
  if (action_available) {
    if (micros() - time_last_write_position > write_frequency_position) {
      time_last_write_position += write_frequency_position;
      canBus.writePosition();
      action_available = false;
    }
  }

  // write diagnostics CAN
  if (action_available) {
    if (micros() - time_last_write_diagnostics > write_frequency_diagnostics) {
      time_last_write_diagnostics += write_frequency_diagnostics;
      canBus.writeDiagnostics(stepper.getControlMode(), driver_temperature);
      action_available = false;
    }
  }

  // write firmware version
  if (action_available) {
    if (micros() - time_last_write_firmware_version > write_frequency_firmware_version) {
      time_last_write_firmware_version += write_frequency_firmware_version;
      canBus.writeFirmwareVersion(NIRYO_STEPPER_VERSION_MAJOR, NIRYO_STEPPER_VERSION_MINOR, NIRYO_STEPPER_VERSION_PATCH);
      action_available = false;
    }
  }
  
  // check read analog sensor
  if (action_available) {
    if (analog_read_enable) {
      if (micros() - time_last_read_temperature > read_temperature_frequency) {
        time_last_read_temperature += read_temperature_frequency;
        driver_temperature = analogRead(TEMPERATURE_SENSOR_PIN);
        action_available = false;
      }
    } 
  }

  // FOR DEBUG ONLY
  if (micros() - time_begin_debug_serial > 500000) {
    time_begin_debug_serial = micros();
    debug_serial();
  }
}

