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

#include <UMPins/UMPins.h>

pinref_t GetNullPinRef()
{
  pinref_t result;
  result.ddrReg = 0;
  result.portReg = 0;
  result.pinReg = 0;
  result.bitValue = 0;
  return result;
}

pinref_t GetPinRef(pinid_t pin)
{
	pinref_t result;
	int regidx = (pin >> 8) - 1;
	result.ddrReg = DDR_REGISTERS[regidx];
	result.portReg = PORT_REGISTERS[regidx];
	result.pinReg = PIN_REGISTERS[regidx];
	result.bitValue = _BV((pin & 0x00FF) - 1);
	return result;
}

pininfo_t PinInfo(uint16_t pinId, uint8_t pcIntId)
{
  pininfo_t result;
  result.pinId = pinId;
  result.pcIntId = pcIntId;
  return result;
}

pininfo_t PinInfo(uint16_t pinId)
{
  return PinInfo(pinId, 0);
}

void SetPinOutput(pinref_t pin)
{
	*(pin.ddrReg) |= pin.bitValue;
}

void SetPinOutput_v(volatile pinref_t &pin)
{
  *(pin.ddrReg) |= pin.bitValue;
}

bool IsPinOutput(pinref_t pin)
{
  return (*(pin.ddrReg) & pin.bitValue) == pin.bitValue;
}

void SetPinInput(pinref_t pin)
{
  *(pin.ddrReg) &= ~(pin.bitValue);
}

void SetPinInput_v(volatile pinref_t &pin)
{
  *(pin.ddrReg) &= ~(pin.bitValue);
}

void SetPinLow(pinref_t pin)
{
  *(pin.portReg) &= ~(pin.bitValue);
}

void SetPinLow_v(volatile pinref_t &pin)
{
  *(pin.portReg) &= ~(pin.bitValue);
}

/* Reads the current value of the pin, whether the pin is configured
 * as input or output.
 */
bool ReadBidiPin(pinref_t pin)
{
  if (IsPinOutput(pin))
    return *(pin.portReg) &= ~(pin.bitValue);
  else
    return (*(pin.pinReg) & pin.bitValue) == pin.bitValue;
}

void SetPinHigh(pinref_t pin)
{
	*(pin.portReg) |= pin.bitValue;
}

void SetPinHigh_v(volatile pinref_t &pin)
{
  *(pin.portReg) |= pin.bitValue;
}

void TogglePin(pinref_t pin)
{
  if ((*(pin.portReg) & pin.bitValue) == pin.bitValue)
    SetPinLow(pin);
  else
    SetPinHigh(pin);
}

bool ReadPin(pinref_t pin)
{
  return (*(pin.pinReg) & pin.bitValue) == pin.bitValue;
}

void MaskEnablePinChangeInt(pininfo_t pin)
{
  int regidx = (pin.pcIntId >> 4) - 1;
  *(PCMSKREGISTERS[regidx]) |= _BV((pin.pcIntId & 0x0F) - 1);
}

void MaskDisablePinChangeInt(pininfo_t pin)
{
  int regidx = (pin.pcIntId >> 4);
  *(PCMSKREGISTERS[regidx]) &= ~_BV((pin.pcIntId & 0x0F) - 1);
}

