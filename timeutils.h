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

#ifndef __TIMEUTILS_H__
#define __TIMEUTILS_H__

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef USE_ARDUINO_TIMER
#include <Arduino.h>
#endif

/* Stopwatch with microsecond resolution
 */
class Stopwatch
{
  private:
    int64_t _startTime;
  public:
    Stopwatch();
    void Start();
    uint32_t GetTime();
};

class TimeUtils
{
  public:
    static uint64_t Uptime();
    static void Init();
};

void InitTimeUtils();

#endif
