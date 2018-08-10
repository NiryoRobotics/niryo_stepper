/*
    CanBus.cpp
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

#include "CanBus.h"

CanBus::CanBus(MCP_CAN* mcp, StepperController* sc, uint8_t motor_id) 
{
  can_driver = mcp;
  stepper_controller = sc;
  this->motor_id = motor_id;
}

void CanBus::setup()
{
  // Initialize MCP2515 running at 16MHz with a baudrate of 1000kb/s
  if(can_driver->begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    SerialUSB.println("MCP2515 Initialized Successfully!");
  }
  else {
    SerialUSB.println("Error Initializing MCP2515...");
  } 

  // Set interrupt pin as input
  pinMode(CAN_PIN_INT, INPUT);

  // Set filters - only accept frames for current motor ID, and CAN broadcast ID
  can_driver->init_Mask(0,0,0x000F0000);
  can_driver->init_Filt(0,0, motor_id * 65536);
  can_driver->init_Mask(1,0,0x000F0000);
  can_driver->init_Filt(2,0,CAN_BROADCAST_ID * 65536);

  can_driver->setMode(MCP_NORMAL);
  SerialUSB.println("MCP2515 CAN started");
}

/*
 * After the motor has finished (successfully or not) its calibration,
 * a result is sent back
 * 
 * (This is the only method to send back a result (from a request) by CAN bus !)
 * 
 */
void CanBus::writeCalibrationResult(uint8_t result, int sensor_steps)
{
  uint8_t data_res[] = { CAN_DATA_CALIBRATION_RESULT, result, (sensor_steps >> 8) & 0xFF, sensor_steps & 0xFF };
  can_driver->sendMsgBuf(0x10 + motor_id, 0, 4, data_res);
}

/*
 * Send current motor position (sensor position + rotations + offset)
 */
void CanBus::writePosition()
{
  int pos = motor_position_steps;
  uint8_t data_pos[] = { CAN_DATA_POSITION, (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF };
  can_driver->sendMsgBuf(0x10 + motor_id, 0, 4, data_pos); // 320 micros
}

/*
 * Send diagnostics ( ~ everything else than position )
 */
void CanBus::writeDiagnostics(uint8_t control_mode, unsigned int driver_temperature)
{
   uint8_t data_diag[] = { CAN_DATA_DIAGNOSTICS, control_mode, (driver_temperature >> 8) & 0xFF, driver_temperature & 0xFF };
   can_driver->sendMsgBuf(0x10 + motor_id, 0, 4, data_diag);
}

/*
 * Send firmware version
 */
void CanBus::writeFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch)
{
  uint8_t data_version[] = { CAN_DATA_FIRMWARE_VERSION, major, minor, patch };
  can_driver->sendMsgBuf(0x10 + motor_id, 0, 4, data_version);
} 

/*
 * Check if a CAN frame has arrived
 */
bool CanBus::available()
{
  return !digitalRead(CAN_PIN_INT);
}

/*
 * Read from CAN bus
 * - check ID and length
 * - get command (first data byte)
 * - execute command action with the rest of data
 */
void CanBus::read()
{   
    can_driver->readMsgBuf(&rxId, &len, rxBuf);     // read data (len 8) : 291 micros, (len : 1) : 224 micros  
                                                          
    // check id is standard, not extended
    if ((rxId & 0x80000000) == 0x80000000) {
      SerialUSB.println("Extended ID, nop nop nop.");
      return;
    }

    // check message is not a remote request frame
    if((rxId & 0x40000000) == 0x40000000){            
      SerialUSB.print("Remote request frame");
      return;
    } 

    if (len < 1) { return; }

    // check cmd
    uint8_t cmd = rxBuf[0];

    switch (cmd) {
      case CAN_CMD_POSITION: // 3 data bytes
        {
          if (len < 4) { break; } // pb

          int32_t data_position = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
        
          // check if negative
          if (data_position & (1 << 15)) {
            data_position = -1 * ((~data_position + 1) & 0xFFFF);
          }
        
          stepper_controller->setNewGoal(data_position);
        }
      break;
      case CAN_CMD_TORQUE: // 3 data bytes
        {
          if (len < 4) { break; } // pb                 
        }
      break;
      case CAN_CMD_MOVE_REL:
      {
          if (len < 7) { break; } // pb

          int32_t steps = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
          // check if negative
          if (steps & (1 << 15)) {
            steps = -1 * ((~steps + 1) & 0xFFFF);
          }

          unsigned long delay = (rxBuf[4] << 16) + (rxBuf[5] << 8) + rxBuf[6];

          stepper_controller->relativeMove(steps, delay);

          SerialUSB.print("Relative Move, steps : ");
          SerialUSB.print(steps);
          SerialUSB.print(", delay : ");
          SerialUSB.println(delay);
      }
      break;
      case CAN_CMD_OFFSET:
        {
          if (len < 6) { break; } // pb
          
          int32_t data_position_offset = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
                    // check if negative
          if (data_position_offset & (1 << 15)) {
            data_position_offset = -1 * ((~data_position_offset + 1) & 0xFFFF);
          }

          int32_t absolute_steps_at_offset_position = (rxBuf[4] << 8) + rxBuf[5];
          long current_steps = (sensor_position * stepper_controller->getMicroSteps() * STEPPER_CPR) / AS5600_CPR;
          int steps_half_rotation = stepper_controller->getMicroSteps() * STEPPER_CPR / 2;

          if (absolute_steps_at_offset_position < steps_half_rotation) {
            if (current_steps > absolute_steps_at_offset_position + steps_half_rotation) {
              motor_rotation_count = -1;
            }
            else {
              motor_rotation_count = 0;
            }
          }
          else if (absolute_steps_at_offset_position > steps_half_rotation) {
            if (current_steps < absolute_steps_at_offset_position - steps_half_rotation) {
              motor_rotation_count = 1;
            }
            else {
              motor_rotation_count = 0;
            }
          }

          offset = data_position_offset;

          SerialUSB.print("Set offset : ");
          SerialUSB.println(offset);
          SerialUSB.print("New motor rotation count : ");
          SerialUSB.println(motor_rotation_count);
        }
      break;
      case CAN_CMD_CALIBRATE:
        {
          if (len < 8) {break;} // pb
          
          int32_t data_position_offset = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
                    // check if negative
          if (data_position_offset & (1 << 15)) {
            data_position_offset = -1 * ((~data_position_offset + 1) & 0xFFFF);
          }

          unsigned int delay_steps = (rxBuf[4] << 8) + rxBuf[5];
          int direction = rxBuf[6];
          long timeout = rxBuf[7];
          
          uint8_t result = stepper_controller->calibrate(direction, delay_steps, data_position_offset, timeout);
          long absolute_sensor_steps = (sensor_position * stepper_controller->getMicroSteps() * STEPPER_CPR) / AS5600_CPR;

          SerialUSB.print("Absolute sensor steps : ");
          SerialUSB.println(absolute_sensor_steps);
          writeCalibrationResult(result, (int)absolute_sensor_steps);
        }
      break;
      case CAN_CMD_SYNCHRONIZE:
        {
          if (len < 2) { break; } // pb
          
          bool begin_traj = rxBuf[1];
          SerialUSB.println("Synchronize");
          stepper_controller->synchronizePosition(begin_traj);
        }
      break;
      case CAN_CMD_MODE: 
        {
          if (len < 2) { break; } // pb
                 
          uint8_t control_mode = rxBuf[1];
          stepper_controller->setControlMode(control_mode);
          SerialUSB.print("Set Mode : ");
          SerialUSB.println(control_mode);
          SerialUSB.print("Current step : ");
          SerialUSB.println(stepper_controller->getCurrentStepNumber());
          SerialUSB.print("Goal step : ");
          SerialUSB.println(stepper_controller->getGoalStepNumber());
          SerialUSB.print("Motor position : ");
          SerialUSB.println(motor_position_steps);

        }
      break;
      case CAN_CMD_MICRO_STEPS:
        {
          if (len < 2) { break; } // pb
          
          uint8_t micro_steps = rxBuf[1];
          stepper_controller->setMicroSteps(micro_steps);
          SerialUSB.print("Set Stepper Micro Steps : ");
          SerialUSB.println(micro_steps);
        }
      break;
      case CAN_CMD_MAX_EFFORT: // value between 0 and 255
        {
          if (len < 2) { break; } // pb

          uint8_t umax = rxBuf[1];
          stepper_controller->setMaxEffort(umax);
          SerialUSB.print("Set max effort : ");
          SerialUSB.println(umax);
        }
      break;
      default:
        {
          // nothing
        }
    }
}


