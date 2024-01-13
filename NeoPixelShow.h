/*--------------------------------------------------------------------
  This file is part of the NeoPixelShow library.

  NeoPixelShow is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixelShow is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixelShow. If not: <http://www.gnu.org/licenses/>.

  This minimal library only supports GRB data ordering and 800 KHz streaming.

  --------------------------------------------------------------------*/

#pragma once

#include <Arduino.h>

#if defined(ESP32)
#include "driver/rmt.h"
#include <stdint.h>
#endif

class NeoPixelShow
{
 public:

  // Constructor: pin number
  NeoPixelShow(uint8_t p);

  #if defined(ESP32)
  bool rmtInit(int index, uint16_t maxBytes);
  #endif

  void show(uint8_t *pixels, uint16_t numBytes);
  inline bool canShow(void) { return (micros() - endTime) >= 50L; }

 private:

  int8_t pin;               // Output pin number
  uint32_t endTime;         // Latch timing reference

  #if defined(ESP32)
  rmt_channel_t channel;    // RMT channel to use
  rmt_item32_t *pdata;      // RMT signal data

  #elif defined(__AVR__)
  volatile uint8_t *port;   // Output PORT register
  uint8_t pinMask;          // Output PORT bitmask
  #endif
};
