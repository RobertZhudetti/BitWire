/*
  UMPins - The Unified Microcontroller Pin list
  Copyright (c) 2022 Robert M. Zhudetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  ...
  
  AtTiny85 pin definitions
 */

#ifndef __UMPINS_H__
#  error "Include <UMPins.h> instead of this file."
#endif

#ifndef __UMPINS_XXX_H__
#  define __UMPINS_XXX_H__ "UMPins_t85.h"
#else
#  error "Attempt to include more than one <UMPins_XXX.h> file."
#endif

#ifndef __UMPINS_AVR_t85_H__
#define __UMPINS_AVR_t85_H__

/* AtTiny85 doesn't have group A
#define PINS_PA 0x0100
#define PIN_PA0 0x0101
#define PIN_PA1 0x0102
etc..
#define PIN_PA7 0x0108
*/

#define PINS_PB_id 0x0200
#define PIN_PB0_id 0x0201
#define PIN_PB1_id 0x0202
#define PIN_PB2_id 0x0203
#define PIN_PB3_id 0x0204
#define PIN_PB4_id 0x0205
#define PIN_PB5_id 0x0206

#define PCINT0_id 0x11
#define PCINT1_id 0x12
#define PCINT2_id 0x13
#define PCINT3_id 0x14
#define PCINT4_id 0x15
#define PCINT5_id 0x16

#define PININFO_PB0() PinInfo(PIN_PB0_id, PCINT0_id)
#define PININFO_PB1() PinInfo(PIN_PB1_id, PCINT1_id)
#define PININFO_PB2() PinInfo(PIN_PB2_id, PCINT2_id)
#define PININFO_PB3() PinInfo(PIN_PB3_id, PCINT3_id)
#define PININFO_PB4() PinInfo(PIN_PB4_id, PCINT4_id)
#define PININFO_PB5() PinInfo(PIN_PB5_id, PCINT5_id)

volatile uint8_t * const PCMSKREGISTERS[] = {
	&PCMSK
};

#endif
