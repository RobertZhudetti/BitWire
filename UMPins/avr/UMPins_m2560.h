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

  AtMega2560 pin definitions
 */

#ifndef __UMPINS_H__
#  error "Include <UMPins.h> instead of this file."
#endif

#ifndef __UMPINS_XXX_H__
#  define __UMPINS_XXX_H__ "UMPins_m2560.h"
#else
#  error "Attempt to include more than one <UMPins_XXX.h> file."
#endif

#ifndef __UMPINS_AVR_m2560_H__
#define __UMPINS_AVR_m2560_H__

#define PINS_PA_id 0x0100
#define PIN_PA0_id 0x0101
#define PIN_PA1_id 0x0102
#define PIN_PA2_id 0x0103
#define PIN_PA3_id 0x0104
#define PIN_PA4_id 0x0105
#define PIN_PA5_id 0x0106
#define PIN_PA6_id 0x0107
#define PIN_PA7_id 0x0108

#define PINS_PB_id 0x0200
#define PIN_PB0_id 0x0201
#define PIN_PB1_id 0x0202
#define PIN_PB2_id 0x0203
#define PIN_PB3_id 0x0204
#define PIN_PB4_id 0x0205
#define PIN_PB5_id 0x0206
#define PIN_PB6_id 0x0207
#define PIN_PB7_id 0x0208

#define PINS_PC_id 0x0300
#define PIN_PC0_id 0x0301
#define PIN_PC1_id 0x0302
#define PIN_PC2_id 0x0303
#define PIN_PC3_id 0x0304
#define PIN_PC4_id 0x0305
#define PIN_PC5_id 0x0306
#define PIN_PC6_id 0x0307
#define PIN_PC7_id 0x0308

#define PINS_PD_id 0x0400
#define PIN_PD0_id 0x0401
#define PIN_PD1_id 0x0402
#define PIN_PD2_id 0x0403
#define PIN_PD3_id 0x0404
#define PIN_PD4_id 0x0405
#define PIN_PD5_id 0x0406
#define PIN_PD6_id 0x0407
#define PIN_PD7_id 0x0408

#define PIN_SCL_id PIN_PD0_id
#define PIN_SDA_id PIN_PD1_id

#define PINS_PE_id 0x0500
#define PIN_PE0_id 0x0501
#define PIN_PE1_id 0x0502
#define PIN_PE2_id 0x0503
#define PIN_PE3_id 0x0504
#define PIN_PE4_id 0x0505
#define PIN_PE5_id 0x0506
#define PIN_PE6_id 0x0507
#define PIN_PE7_id 0x0508

#define PINREF_PE0 MakePinRef(&DDRE, &PORTE, &PINE, 0 /* First bit in DDRx, PORTx, PINx */, 1 /* PCINT group 1 */, 0 /* PCMSK1 bit pos */)

#define PINS_PF_id 0x0600
#define PIN_PF0_id 0x0601
#define PIN_PF1_id 0x0602
#define PIN_PF2_id 0x0603
#define PIN_PF3_id 0x0604
#define PIN_PF4_id 0x0605
#define PIN_PF5_id 0x0606
#define PIN_PF6_id 0x0607
#define PIN_PF7_id 0x0608

#define PINS_PG_id 0x0700
#define PIN_PG0_id 0x0701
#define PIN_PG1_id 0x0702
#define PIN_PG2_id 0x0703
#define PIN_PG3_id 0x0704
#define PIN_PG4_id 0x0705
#define PIN_PG5_id 0x0706
//#define PIN_PG6_id 0x0707 // Doesn't exist
//#define PIN_PG7_id 0x0708 // Doesn't exist

#define PINS_PH_id 0x0800
#define PIN_PH0_id 0x0801
#define PIN_PH1_id 0x0802
#define PIN_PH2_id 0x0803
#define PIN_PH3_id 0x0804
#define PIN_PH4_id 0x0805
#define PIN_PH5_id 0x0806
#define PIN_PH6_id 0x0807
#define PIN_PH7_id 0x0808

//#define PINS_PI 0x0900 // Pin group I doesn't exist.

#define PINS_PJ_id 0x0A00
#define PIN_PJ0_id 0x0A01
#define PIN_PJ1_id 0x0A02
#define PIN_PJ2_id 0x0A03
#define PIN_PJ3_id 0x0A04
#define PIN_PJ4_id 0x0A05
#define PIN_PJ5_id 0x0A06
#define PIN_PJ6_id 0x0A07
#define PIN_PJ7_id 0x0A08

#define PINS_PK_id 0x0B00
#define PIN_PK0_id 0x0B01
#define PIN_PK1_id 0x0B02
#define PIN_PK2_id 0x0B03
#define PIN_PK3_id 0x0B04
#define PIN_PK4_id 0x0B05
#define PIN_PK5_id 0x0B06
#define PIN_PK6_id 0x0B07
#define PIN_PK7_id 0x0B08

#define PINS_PL_id 0x0C00
#define PIN_PL0_id 0x0C01
#define PIN_PL1_id 0x0C02
#define PIN_PL2_id 0x0C03
#define PIN_PL3_id 0x0C04
#define PIN_PL4_id 0x0C05
#define PIN_PL5_id 0x0C06
#define PIN_PL6_id 0x0C07
#define PIN_PL7_id 0x0C08

#endif
