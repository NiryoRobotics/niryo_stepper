/*
    StepperController.h
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

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "config.h"
#include "A4954.h"
#include "AS5600.h"


class StepperController {

  private:

    long steps_to_add_to_goal; // when synchronizing

    int micro_steps;
    int umax;
    
    bool is_enabled;
    uint8_t control_mode;

    int micros_to_reach_goal;

    long current_step_number;
    long goal_step_number;

    unsigned long delay_between_two_updates;
    unsigned long time_last_update; // last controller update time

    unsigned long time_last_step; // last time controller ordered to do a step

  public:

    StepperController();

    void reset();
    void synchronizePosition(bool begin_trajectory);

    void attach();
    void detach();

    void start();
    void stop();

    uint8_t calibrate(int direction, unsigned long delay_steps, long steps_offset, unsigned long calibration_timeout);

    void setMicroSteps(uint8_t micro_steps);
    void setMaxEffort(uint8_t effort);

    void setControlMode(uint8_t control_mode);
    void relativeMove(long steps, unsigned long delay);
    void setNewGoal(long steps);
    
    void update();
    void relaxModeUpdate();
    void standardModeUpdate();

    // getters
    long getCurrentStepNumber() { return current_step_number; }
    long getGoalStepNumber() { return goal_step_number; }
    unsigned long getDelay() { return delay_between_two_updates; }  
    uint8_t getControlMode() { return control_mode; }
    int getMicroSteps() { return micro_steps; }
    int getMaxEffort() { return umax; }
};


#endif
