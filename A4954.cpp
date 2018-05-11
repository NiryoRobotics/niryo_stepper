/*
    A4954.cpp
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

#include "A4954.h"

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

void init_driver()
{
  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);
  
  analogFastWrite(VREF_2, 80);
  analogFastWrite(VREF_1, 80);
}

void output(long theta, int effort) {
  int angle_1;
  int angle_2;
  int v_coil_A;
  int v_coil_B;
  
  int sin_coil_A;
  int sin_coil_B;
  int phase_multiplier = 500;

  angle_1 = mod((phase_multiplier * theta)/1000 , 3600);
  angle_2 = mod((phase_multiplier * theta)/1000 +900, 3600);
     
  sin_coil_A = sin_1[angle_1];
  sin_coil_B = sin_1[angle_2];
  
  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

  analogFastWrite(VREF_1, abs(v_coil_A));
  analogFastWrite(VREF_2, abs(v_coil_B));

  if (v_coil_A >= 0)  {
    IN_2_HIGH();
    IN_1_LOW();
  }
  else  {
    IN_2_LOW();
    IN_1_HIGH();
  }

  if (v_coil_B >= 0)  {
    IN_4_HIGH();
    IN_3_LOW(); 
  }
  else  {
    IN_4_LOW();
    IN_3_HIGH();
  }
}

/*
 * Setting all Pins to HIGH will
 * give more "resistance torque" to the motor
 * --> Useful on Niryo One so the axis of the motor
 * doesn't fall abruptely
 */
void relaxed_mode_with_resistance() {
  IN_1_HIGH();
  IN_2_HIGH();
  IN_3_HIGH();
  IN_4_HIGH();
}

