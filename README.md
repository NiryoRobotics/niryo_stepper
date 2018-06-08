# NiryoStepper firmware 

Firmware to control a stepper motor with the custom Niryo Arduino-compatible board.
This board has a magnetic sensor (AS5600) to measure precisely the position of the stepper motor.
  
Communication interface : CAN bus (using MCP2515 to make an SPI-CAN interface)
  
This firmware is used on Niryo One, for the 4 stepper motors (axis 1-4). 
  
### How to upgrade the firmware on NiryoStepper boards  for Niryo One

Download the latest (recommended) version of the firmware.

You will need to install the Arduino IDE software, and install the “Arduino SAMD boards (32-bits ARM Cortex-M0+)” library from the boards manager (Tools -> Board -> Boards manager).

Choose “Arduino/Genuino Zero (Native USB Port)” in Tools -> Board

Then, make sure that you put the correct ID for each motor (1-2-3-4) on the NiryoStepper code, before you upload it to each board.

--> You can find a [complete tutorial on Niryo website](https://niryo.com/docs/niryo-one/update-your-robot/update-niryo-steppers/) (with photos and screenshots).

### How to communicate with a NiryoStepper board

First you need to find a device that can send data on a CAN Bus. We recommend using MCP2515 on both Raspberry Pi 3 and Arduino to communicate between the boards. The MCP2515 allows you to make an SPI-CAN interface.

Here you can find the function list directly implemented in C++ to communicate with a NiryoStepper :
- [Write functions](https://github.com/NiryoRobotics/niryo_one_ros/blob/master/niryo_one_driver/src/hw_driver/niryo_one_can_driver.cpp)
- [Read functions](https://github.com/NiryoRobotics/niryo_one_ros/blob/master/niryo_one_driver/src/hw_comm/can_communication.cpp#L216)

### NiryoStepper hardware architecture

You can see an overview of the NiryoStepper PCB [here](https://github.com/NiryoRobotics/niryo_one/blob/master/Electronics/Niryo_stepper_overview.pdf).
