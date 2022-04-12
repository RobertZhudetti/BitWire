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
  
  AtMega328p pin definitions
 */

#ifndef __UMPINS_H__
#  error "Include <UMPins.h> instead of this file."
#endif

#ifndef __UMPINS_XXX_H__
#  define __UMPINS_XXX_H__ "UMPins_m328p.h"
#else
#  error "Attempt to include more than one <UMPins_XXX.h> file."
#endif

#ifndef __UMPINS_AVR_m328p_H__
#define __UMPINS_AVR_m328p_H__

/* AtMega328(p) doesn't have these
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
// Careful! PB6 & 7 are also crystal oscillator pins. Only available if crystal is not used.
#define PIN_PB6_id 0x0207
#define PIN_PB7_id 0x0208

#define PINS_PC_id 0x0300
#define PIN_PC0_id 0x0301
#define PIN_PC1_id 0x0302
#define PIN_PC2_id 0x0303
#define PIN_PC3_id 0x0304
#define PIN_PC4_id 0x0305
#define PIN_PC5_id 0x0306
// Careful! PC6 is also reset.
#define PIN_PC6_id 0x0307
//#define PIN_PC7 0x0308 // Doesn't exist.

#define PIN_SCL_id PIN_PC5_id
#define PIN_SDA_id PIN_PC4_id

#define PCINT12_id 0x14
#define PCINT13_id 0x15

#define PININFO_PC4 PinInfo(PIN_PC4_id, PCINT12_id)
#define PININFO_PC5 PinInfo(PIN_PC5_id, PCINT13_id)

#define PINS_PD_id 0x0400
#define PIN_PD0_id 0x0401
#define PIN_PD1_id 0x0402
#define PIN_PD2_id 0x0403
#define PIN_PD3_id 0x0404
#define PIN_PD4_id 0x0405
#define PIN_PD5_id 0x0406
#define PIN_PD6_id 0x0407
#define PIN_PD7_id 0x0408

#endif
