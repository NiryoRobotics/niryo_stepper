/*
    StepperController.cpp
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

#include "StepperController.h"

StepperController::StepperController() 
{
  current_step_number = 0;
  goal_step_number = 0;
  delay_between_two_updates = STEPPER_DELAY_MIN;
  time_last_update = micros();
  time_last_step = micros();

  // default config below

  // cmd will be reached after 0.1 sec (will make trajectory smoothier if send cmd > 10 Hz)
  // if you want/need to send cmd at a lower rate, update this value
  micros_to_reach_goal = 100000; // todo get from CAN
  control_mode = STEPPER_CONTROL_MODE_RELAX;
  micro_steps = STEPPER_DEFAULT_MICRO_STEPS;
  umax = UMAX_DEFAULT;

  steps_to_add_to_goal = 0;
}

void StepperController::reset()
{
  // not yet implemented
}

/*
 * If standard mode is activated, you need to call this function at the beginning and end of trajectory
 * It will do 2 things :
 * - start at current sensor position if the motor missed some steps at previous trajectory
 * - start and finish with a little offset to compensate a small error between current steps and real motor steps
 */
void StepperController::synchronizePosition(bool begin_trajectory)
{ 
  long diff = motor_position_steps - current_step_number;

  if (abs(diff) < 50) {
    steps_to_add_to_goal = diff;
  }
  else {
    if (begin_trajectory) {
      current_step_number = motor_position_steps;
      steps_to_add_to_goal = 0;
    }
  }
}

void StepperController::attach() 
{
  steps_to_add_to_goal = 0;
  fan_HIGH();
  output(- 1800 * current_step_number / micro_steps, umax);
}

void StepperController::detach()
{
  output(-1800 * current_step_number / micro_steps, 0);
  fan_LOW();
}

void StepperController::start() 
{
    is_enabled = true;
    time_last_update = micros();
    time_last_step = micros();
}

void StepperController::stop()
{
    is_enabled = false;
}

/*
 * Allows micro steps between 1 and 32
 */
void StepperController::setMicroSteps(uint8_t new_micro_steps)
{
  if (new_micro_steps < 1 || new_micro_steps > 32) {
    return;
  }

  micro_steps = new_micro_steps;
  
}

/*
 * Modify max effort -> amount of used current
 */
void StepperController::setMaxEffort(uint8_t effort)
{
  umax = effort;
}

void StepperController::setControlMode(uint8_t control_mode)
{ 
  this->control_mode = control_mode;
  if (control_mode == STEPPER_CONTROL_MODE_RELAX) {
    detach();
  }
  else if (control_mode == STEPPER_CONTROL_MODE_STANDARD) {
    attach();
  }
}

void StepperController::relativeMove(long steps, unsigned long delay)
{
  current_step_number = motor_position_steps;
  goal_step_number = current_step_number + steps;
  delay_between_two_updates = (delay > STEPPER_DELAY_MIN) ? delay : STEPPER_DELAY_MIN;
}

/*
 * This method will :
 * - set new step goal
 * - calculate according delay between steps
 * - if standard_mode, a small amount can be added to compensate some small errors
 * 
 */
void StepperController::setNewGoal(long steps)
{ 
  long step_diff = steps - current_step_number;
  goal_step_number = steps - steps_to_add_to_goal;
  if (step_diff != 0) {
    long calc_delay = micros_to_reach_goal / abs(step_diff);
    delay_between_two_updates = (calc_delay > STEPPER_DELAY_MIN) ? calc_delay : STEPPER_DELAY_MIN;
  }
  else {
    delay_between_two_updates = 10000;
  }
  
  /*if (step_diff != 0) {
    SerialUSB.print("steps received : ");
    SerialUSB.print(steps);
    SerialUSB.print(", current_step : ");
    SerialUSB.print(current_step_number);
    SerialUSB.print(", step diff : ");
    SerialUSB.print(step_diff);
    SerialUSB.print(", steps_to_add_to_goal : ");
    SerialUSB.print(steps_to_add_to_goal);
    //SerialUSB.print(", delay : ");
    //SerialUSB.print(delay_between_two_updates);
    SerialUSB.print(", current sensor steps : ");
    SerialUSB.println(motor_position_steps);
  }*/
}

/*
 * Update controller depending on control mode
 * 
 * RELAX mode :
 * - disable motor, and copy sensor position to goal position
 * 
 * STANDARD mode :
 * - motor is powered with a constant current.
 * - it will follow the command step by step with precision
 * - no protection against missed steps (if many) during a trajectory, but
 * - every trajectory will correct previous trajectory missed steps
 * --> you need to call synchronizePosition(1) at the beginning of any given trajectory
 * --> you need to call synchronizePosition(0) at the end of any given trajectory
 * 
 */
void StepperController::update()
{
  if (micros() - time_last_update > delay_between_two_updates) {
    time_last_update += delay_between_two_updates;

    if (control_mode == STEPPER_CONTROL_MODE_RELAX) { 
      relaxModeUpdate();
    }
    else if (control_mode == STEPPER_CONTROL_MODE_STANDARD) {
      standardModeUpdate();
    }
    else {
      time_last_update = micros();
    }
  }
}

void StepperController::relaxModeUpdate()
{
  time_last_update = micros();
  current_step_number = motor_position_steps;
  goal_step_number = motor_position_steps;
}

void StepperController::standardModeUpdate()
{
  // minimum amount of time between 2 steps 
  // -> so the motor will never go too fast (and possibly miss steps due to code)
  if (micros() - time_last_step < 180) { 
    return;
  }
  
  if (current_step_number < goal_step_number) {
    ++current_step_number;
    output(-1800 * current_step_number / micro_steps, umax);
    time_last_step = micros();
  }
  else if (current_step_number > goal_step_number) {
    --current_step_number;
    output(-1800 * current_step_number / micro_steps, umax);
    time_last_step = micros();
  }
  else {
    // nothing to do, goal has been reached
    time_last_update = micros();
  }   
}

/*
 * This method will rotate the motor until it reaches an obstacle and misses some steps
 * The point where motor will miss step will define the offset
 * 
 * --> You can use this method to calibrate the motor with a home offset and a physical obstacle
 * If no step is missed (i.e. no obstacle), a timeout will be sent
 * If an offset has been set, a successful answer will be sent
 * 
 */
uint8_t StepperController::calibrate(int direction, unsigned long delay_steps, long steps_offset, unsigned long calibration_timeout) // timeout in seconds
{
  SerialUSB.println("Start calibration");
  SerialUSB.print("Direction : ");
  SerialUSB.println(direction);

  int MISS_STEPS_TRESHOLD = 4;
  long time_begin_calibration = micros();
  long timeout = 1000000 * (calibration_timeout - 3); // take 3 sec off calibration timeout
  if (timeout <= 0) { return STEPPER_CALIBRATION_BAD_PARAM; }
  if (delay_steps < 500) { delay_steps = 500; }
  if (delay_steps > 2000) { delay_steps = 2000; }
  long position = motor_position_steps;

  // attach motor and wait for stability
  attach();
  delay(500);

  long last_motor_position_steps = motor_position_steps;
  int miss_steps_counter = 0;
  bool calibration_ok = false;

  // accelerate a little bit, just for fun
  for (int i = 8000; i > delay_steps; i -= 50) { // about 4 steps to accelerate
    position = (direction) ? position + 1 : position - 1; // increment position
    output(-1800 * position / 32, UMAX_40_PERCENT); // 32 microsteps -> more precision
    delayMicroseconds(i);
  }

  // move at constant speed until reach an obstacle
  while (micros() - time_begin_calibration < timeout) {
    position = (direction) ? position + 1 : position - 1; // increment position
    output(-1800 * position / 32, UMAX_50_PERCENT); // 32 microsteps -> more precision
    delayMicroseconds(delay_steps);
    update_current_position(micro_steps); // read pos from sensor
    
    // Check if motor missed a step
    if (direction && (motor_position_steps - last_motor_position_steps < 0)) {
      miss_steps_counter++;
    }
    else if (!direction && (motor_position_steps - last_motor_position_steps > 0)) {
      miss_steps_counter++;
    }
    else {
      miss_steps_counter = 0;
    }
    
    if (miss_steps_counter > MISS_STEPS_TRESHOLD) {
      calibration_ok = true;
      break;
    }
    last_motor_position_steps = motor_position_steps;
  }

  // Now we are close
  // Continue to rotate (slowly) to get home position and set offset 
  long home_position = motor_position_without_offset;

  delay(100);
  for (int i = 0;  i < 32; ++i) {   // move 1 step more
    position = (direction) ? position + 1 : position - 1;
    output(-1800 * position / 32, UMAX_40_PERCENT); // 32 microsteps -> more precision
    delayMicroseconds(10000); // go slower
    update_current_position(micro_steps); // read pos from sensor
  }
  
  delay(500);

  // back to relax mode
  detach();

  if (calibration_ok) {
    SerialUSB.println("---- END -->");
    SerialUSB.print("Home position : ");
    SerialUSB.print(home_position);
    SerialUSB.print(", Motor position : ");
    SerialUSB.println(motor_position_without_offset);
    offset = motor_position_without_offset - steps_offset; 
    current_step_number = motor_position_steps;
    goal_step_number = motor_position_steps;
    SerialUSB.print("Calibration OK, set offset : ");
    SerialUSB.println(offset);
    return STEPPER_CALIBRATION_OK;
  }
  else {
    SerialUSB.println("Calibration timeout");
    return STEPPER_CALIBRATION_TIMEOUT;
  }
}


