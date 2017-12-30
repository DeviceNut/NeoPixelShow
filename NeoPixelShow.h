/*--------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.

  This minimal library only supports GRB data ordering, 800 KHz streaming,
  and a modern version of the Arduino infrastructure. It does not support
  the ESP8266 or ESP32 processors.

  --------------------------------------------------------------------*/

#ifndef NEOPIXEL_SHOW_H
#define NEOPIXEL_SHOW_H

#if defined(ARDUINO)
#include "Arduino.h"
#elif defined(SPARK)
#include "Particle.h"
#endif

class NeoPixelShow
{
 public:

  // Constructor: pin number
  NeoPixelShow(uint8_t p);

  void show(uint8_t *pixels, uint16_t numBytes);
  inline bool canShow(void) { return (micros() - endTime) >= 50L; }

 private:

  int8_t pin;               // Output pin number
  uint32_t endTime;         // Latch timing reference

#ifdef __AVR__
  volatile uint8_t *port;   // Output PORT register
  uint8_t pinMask;          // Output PORT bitmask
#endif
};

#endif // NEOPIXEL_SHOW_H
