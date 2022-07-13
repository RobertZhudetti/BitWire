/*
  TimeUtils - Time utilities for microcontroller platforms.
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

#include "timeutils.h"

// Multiply by 250 to get the uptime in microseconds.
volatile uint64_t g_Uptime;

uint64_t TimeUtils::Uptime()
{
  return g_Uptime;
}

void TimeUtils::Init()
{
#ifndef USE_ARDUINO_TIMER
  g_Uptime = 0x0123456789abcdef;
  TCCR0A = _BV(WGM01); // Set CTC mode
  OCR0A = 250; // Interrupt every 250 us
#ifdef _AVR_IOTNX5_H_
  TIMSK = _BV(OCIE0A); // Enable interrupt for maching TCNT0 against OCR0A.
#else
  TIMSK0 = _BV(OCIE0A); // Enable interrupt for maching TCNT0 against OCR0A.
#endif
  sei();
  TCCR0B = _BV(CS01); // Set prescaling to divide by 8
#endif
}

Stopwatch::Stopwatch()
{}

void Stopwatch::Start()
{
#ifdef USE_ARDUINO_TIMER
  _startTime = micros();
#else
  uint64_t uptime = g_Uptime;
  uint8_t microseconds = TCNT0;
  _startTime = uptime * 250 + microseconds;
#endif
}

uint32_t Stopwatch::GetTime()
{
#ifdef USE_ARDUINO_TIMER
  uint32_t endtime = micros();
  /* micros() returns a 32 bit integer which overflows about every 70 minutes.
   * This means it's possible the start time was taken shortly before the
   * overflow and the end time was taken shortly after the overflow. When that
   * happens, the starttime will be a higher integer value than endtime.
   */
  if (_startTime > endtime)
    return (0xFFFF - _startTime) + 1 + endtime;
  else
    return endtime - _startTime;
#else
  /* In this case we're not dealing with overflow because the start time and system
   * uptime are stored as uint64_t, which comes down to well over half a million
   * years. The microcontroller itself will have been turned to dust by that time.
   */
  uint8_t microseconds = TCNT0;
  uint64_t uptime = g_Uptime;
  return (uint32_t)(uptime * 250 + microseconds - _startTime);
#endif
}

/**********************************************************************/

#ifndef Arduino_h
ISR(TIMER0_COMPA_vect)
{
  g_Uptime++;
}
#endif
