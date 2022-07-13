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
 */

#ifndef __UMPINS_H__
#define __UMPINS_H__

#include <avr/io.h>

typedef uint16_t pinid_t;

#if defined (__AVR_ATmega32U2__)
#  include <UMPins/avr/UMPins_m32u2.h>
#elif defined (__AVR_ATmega2560__)
#  include <UMPins/avr/UMPins_m2560.h>
#elif defined (__AVR_ATmega328P__)
#  include <UMPins/avr/UMPins_m328p.h>
#elif defined (__AVR_ATtiny85__)
#  include <UMPins/avr/UMPins_t85.h>
#else
#  error "Could not find a pin list for target microcontroller."
#endif

volatile uint8_t * const DDR_REGISTERS[] = {
#ifdef DDRA
	&DDRA,
#else
	(uint8_t *)0,
#endif
#ifdef DDRB
	&DDRB,
#else
	(uint8_t *)0,
#endif
#ifdef DDRC
	&DDRC,
#else
	(uint8_t *)0,
#endif
#ifdef DDRD
	&DDRD,
#else
	(uint8_t *)0,
#endif
#ifdef DDRE
	&DDRE,
#else
	(uint8_t *)0,
#endif
#ifdef DDRF
	&DDRF,
#else
	(uint8_t *)0,
#endif
#ifdef DDRG
	&DDRG,
#else
	(uint8_t *)0,
#endif
#ifdef DDRH
	&DDRH,
#else
	(uint8_t *)0,
#endif
#ifdef DDRI
	&DDRI,
#else
	(uint8_t *)0,
#endif
#ifdef DDRJ
	&DDRJ,
#else
	(uint8_t *)0,
#endif
#ifdef DDRK
	&DDRK,
#else
	(uint8_t *)0,
#endif
#ifdef DDRL
	&DDRL,
#else
	(uint8_t *)0,
#endif
};

volatile uint8_t * const PORT_REGISTERS[] = {
#ifdef PORTA
	&PORTA,
#else
	(uint8_t *)0,
#endif
#ifdef PORTB
	&PORTB,
#else
	(uint8_t *)0,
#endif
#ifdef PORTC
	&PORTC,
#else
	(uint8_t *)0,
#endif
#ifdef PORTD
	&PORTD,
#else
	(uint8_t *)0,
#endif
#ifdef PORTE
	&PORTE,
#else
	(uint8_t *)0,
#endif
#ifdef PORTF
	&PORTF,
#else
	(uint8_t *)0,
#endif
#ifdef PORTG
	&PORTG,
#else
	(uint8_t *)0,
#endif
#ifdef PORTH
	&PORTH,
#else
	(uint8_t *)0,
#endif
#ifdef PORTI
	&PORTI,
#else
	(uint8_t *)0,
#endif
#ifdef PORTJ
	&PORTJ,
#else
	(uint8_t *)0,
#endif
#ifdef PORTK
	&PORTK,
#else
	(uint8_t *)0,
#endif
#ifdef PORTL
	&PORTL,
#else
	(uint8_t *)0,
#endif
};

volatile uint8_t * const PIN_REGISTERS[] = {
#ifdef PINA
	&PINA,
#else
	(uint8_t *)0,
#endif
#ifdef PINB
	&PINB,
#else
	(uint8_t *)0,
#endif
#ifdef PINC
	&PINC,
#else
	(uint8_t *)0,
#endif
#ifdef PIND
	&PIND,
#else
	(uint8_t *)0,
#endif
#ifdef PINE
	&PINE,
#else
	(uint8_t *)0,
#endif
#ifdef PINF
	&PINF,
#else
	(uint8_t *)0,
#endif
#ifdef PING
	&PING,
#else
	(uint8_t *)0,
#endif
#ifdef PINH
	&PINH,
#else
	(uint8_t *)0,
#endif
#ifdef PINI
	&PINI,
#else
	(uint8_t *)0,
#endif
#ifdef PINJ
	&PINJ,
#else
	(uint8_t *)0,
#endif
#ifdef PINK
	&PINK,
#else
	(uint8_t *)0,
#endif
#ifdef PINL
	&PINL,
#else
	(uint8_t *)0,
#endif
};

typedef struct {
  volatile uint8_t *ddrReg, *portReg, *pinReg;
  uint8_t bitValue;
} pinref_t;

typedef struct {
  uint16_t pinId;
  uint8_t pcIntId;
} pininfo_t;

pinref_t GetPinRef(pinid_t pin);
pininfo_t PinInfo(uint16_t pinId, uint8_t pcIntId);
pininfo_t PinInfo(uint16_t pinId);

void SetPinOutput(pinref_t pin);
bool IsPinOutput(pinref_t pin);
void SetPinInput(pinref_t pin);
void SetPinLow(pinref_t pin);
/* Reads the current value of the pin, whether the pin is configured
 * as input or output.
 */
bool ReadBidiPin(pinref_t pin);
void SetPinHigh(pinref_t pin);
bool ReadPin(pinref_t pin);
void TogglePin(pinref_t pin);

void MaskEnablePinChangeInt(pininfo_t pin);
void MaskDisablePinChangeInt(pininfo_t pin);

#endif
