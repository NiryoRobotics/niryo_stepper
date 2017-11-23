/*
    analogFastWrite.h
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

    This file was originally licensed under Creative Commons 
    Attribution Share-Alike 4.0 License
    Copyright (C) Mechaduino-firmware
*/

//187kHz PWM implementation.  Stock analogWrite is much slower and is very audible!

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief SAMD products have only one reference for ADC
 */


extern void analogFastWrite( uint32_t ulPin, uint32_t ulValue ) ;


#ifdef __cplusplus
}
#endif








