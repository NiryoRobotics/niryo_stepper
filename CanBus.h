/*
    CanBus.h
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

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "config.h"
#include "mcp_can.h"
#include "StepperController.h"


class CanBus {

  private:
    
    // CAN RX Variables
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];

    uint8_t motor_id;

    MCP_CAN* can_driver;
    StepperController* stepper_controller;

  public:
      CanBus() {}
      CanBus(MCP_CAN* mcp, StepperController* sc, uint8_t motor_id);

      void setup();

      void writeCalibrationResult(uint8_t result);
      void writePosition();
      void writeDiagnostics(uint8_t control_mode, unsigned int driver_temperature);
      void writeFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch);
      
      bool available();
      void read();
};

#endif
