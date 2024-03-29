/*-------------------------------------------------------------------------
  Arduino library to control WS2812-based GRB 800 KHz bitstreams LED devices
  only, such as Adafruit's NeoPixel strips.

  8 MHz MCUs provide output on PORTB and PORTD, while 16 MHz chips can handle
  most output pins (possible exception with upper PORT registers on the 
  Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.

  Minified by Greg de Valois of Devicenut for use with PixelNut! devices.
  This minimal library only supports GRB data ordering, 800 KHz streaming,
  and a modern version of the Arduino infrastructure. 

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
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

  -------------------------------------------------------------------------*/

#include "NeoPixelShow.h"

#if defined(ESP32)
#define RMT_0_HIGH    3   // 0 bit high time
#define RMT_0_LOW     9   // 0 bit low time
#define RMT_1_HIGH    9   // 1 bit high time
#define RMT_1_LOW     3   // 1 bit low time
#define MAX_CHANNELS  8   // max of 8 RMT channels
#endif

NeoPixelShow::NeoPixelShow(uint8_t p) : pin(p), endTime(0)
{
  #ifdef __AVR__
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
  #elif !defined(ESP32)
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  #endif
}

#if defined(ESP32)
bool NeoPixelShow::rmtInit(int index, uint16_t numBytes)
{
  if (index >= MAX_CHANNELS) return false;

  // 3 bytes/pixel * 8 bits/byte
  pdata = (rmt_item32_t*)malloc(numBytes * sizeof(rmt_item32_t) * 8);
  if (pdata == NULL) return false;

  channel = (rmt_channel_t)((int)RMT_CHANNEL_0 + index);

  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel;
  config.gpio_num = (gpio_num_t)pin;
  config.tx_config.loop_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_en = false;
  config.tx_config.carrier_duty_percent = 50;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
  config.tx_config.carrier_freq_hz = 100;
  config.mem_block_num = 1;
  config.clk_div = 8;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(channel, 0, 0));

  return true;
}
#endif

void NeoPixelShow::show(uint8_t *pixels, uint16_t numBytes)
{
  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while(!canShow());
  // endTime is a private member (rather than global var) so that mutliple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  #if !defined(ESP32)
  noInterrupts();  // Need 100% focus on instruction timing
  #endif

  #if defined(ESP32)

  rmt_item32_t *p = pdata;
  for (int i = 0; i < numBytes; ++i, ++pixels)
  {
    byte data = *pixels;
    byte mask = 0x80;

    for (int bit = 0; bit < 8; ++bit, ++p)
    {
      *p = (data & mask) ? (rmt_item32_t){{{RMT_1_HIGH, 1, RMT_1_LOW, 0}}} :
                           (rmt_item32_t){{{RMT_0_HIGH, 1, RMT_0_LOW, 0}}};
      mask >>= 1;
    }
  }

  ESP_ERROR_CHECK(rmt_write_items(channel, pdata, (numBytes * 8), false));
  ESP_ERROR_CHECK(rmt_wait_tx_done(channel, portMAX_DELAY));

  #elif defined(__AVR__)

  // AVR MCUs -- ATmega & ATtiny (no XMEGA) ---------------------------------

  volatile uint16_t
    i   = numBytes; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low

  // Hand-tuned assembly code issues data to the LED drivers at a specific
  // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz).
  // The datastream timing for the LED drivers allows a little wiggle room each
  // way (listed in the datasheets), so the conditions for compiling each
  // case are set up for a range of frequencies rather than just the exact
  // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
  // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
  // on the datasheet figures and have not been extensively tested outside
  // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
  // close to the extremes (or possibly they could be pushed further).
  // Keep in mind only one CPU speed case actually gets compiled; the
  // resulting program isn't as massive as it might look from source here.

// 8 MHz(ish) AVR ---------------------------------------------------------
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

    volatile uint8_t n1, n2 = 0;  // First, next bits out

    // Squeezing an 800 KHz stream out of an 8 MHz chip requires code
    // specific to each PORT register.  At present this is only written
    // to work with pins on PORTD or PORTB, the most likely use case --
    // this covers all the pins on the Adafruit Flora and the bulk of
    // digital pins on the Arduino Pro 8 MHz (keep in mind, this code
    // doesn't even get compiled for 16 MHz boards like the Uno, Mega,
    // Leonardo, etc., so don't bother extending this out of hand).
    // Additional PORTs could be added if you really need them, just
    // duplicate the else and loop and change the PORT.  Each add'l
    // PORT will require about 150(ish) bytes of program space.

    // 10 instruction clocks per bit: HHxxxxxLLL
    // OUT instructions:              ^ ^    ^   (T=0,2,7)

  #ifdef PORTD // PORTD isn't present on ATtiny85, etc.

    if(port == &PORTD) {

      hi = PORTD |  pinMask;
      lo = PORTD & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      // Dirty trick: RJMPs proceeding to the next instruction are used
      // to delay two clock cycles in one instruction word (rather than
      // using two NOPs).  This was necessary in order to squeeze the
      // loop down to exactly 64 words -- the maximum possible for a
      // relative branch.

      asm volatile(
       "headD:"                   "\n\t" // Clk  Pseudocode
        // Bit 7:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 6"        "\n\t" // 1-2  if(b & 0x40)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 6:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 5"        "\n\t" // 1-2  if(b & 0x20)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 5:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 4"        "\n\t" // 1-2  if(b & 0x10)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 4:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 3"        "\n\t" // 1-2  if(b & 0x08)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 3:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 2"        "\n\t" // 1-2  if(b & 0x04)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 2:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 1"        "\n\t" // 1-2  if(b & 0x02)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 1:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 0"        "\n\t" // 1-2  if(b & 0x01)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "sbiw %[count], 1"        "\n\t" // 2    i-- (don't act on Z flag yet)
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++
        "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "brne headD"              "\n"   // 2    while(i) (Z flag set above)
      : [byte]  "+r" (b),
        [n1]    "+r" (n1),
        [n2]    "+r" (n2),
        [count] "+w" (i)
      : [port]   "I" (_SFR_IO_ADDR(PORTD)),
        [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

    } else if(port == &PORTB) {

  #endif // PORTD

      // Same as above, just switched to PORTB and stripped of comments.
      hi = PORTB |  pinMask;
      lo = PORTB & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      asm volatile(
       "headB:"                   "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 6"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 5"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 4"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 3"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 2"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 1"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 0"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "brne headB"              "\n"
      : [byte] "+r" (b), [n1] "+r" (n1), [n2] "+r" (n2), [count] "+w" (i)
      : [port] "I" (_SFR_IO_ADDR(PORTB)), [ptr] "e" (ptr), [hi] "r" (hi),
        [lo] "r" (lo));

  #ifdef PORTD
    }    // endif PORTB
  #endif

  // 12 MHz(ish) AVR --------------------------------------------------------
  #elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)

    // In the 12 MHz case, an optimized 800 KHz datastream (no dead time
    // between bytes) requires a PORT-specific loop similar to the 8 MHz
    // code (but a little more relaxed in this case).

    // 15 instruction clocks per bit: HHHHxxxxxxLLLLL
    // OUT instructions:              ^   ^     ^     (T=0,4,10)

    volatile uint8_t next;

  #ifdef PORTD

    if(port == &PORTD) {

      hi   = PORTD |  pinMask;
      lo   = PORTD & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Don't "optimize" the OUT calls into the bitTime subroutine;
      // we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
      asm volatile(
       "headD:"                   "\n\t" //        (T =  0)
        "out   %[port], %[hi]"    "\n\t" //        (T =  1)
        "rcall bitTimeD"          "\n\t" // Bit 7  (T = 15)
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 6
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 5
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 4
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 3
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 2
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 1
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi    (T =  1)
        "rjmp .+0"                "\n\t" // 2    nop nop      (T =  3)
        "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++   (T =  5)
        "out  %[port] , %[next]"  "\n\t" // 1    PORT = next  (T =  6)
        "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo    (T =  7)
        "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80) (T =  8)
         "mov %[next] , %[hi]"    "\n\t" // 0-1    next = hi  (T =  9)
        "nop"                     "\n\t" // 1                 (T = 10)
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo    (T = 11)
        "sbiw %[count], 1"        "\n\t" // 2    i--          (T = 13)
        "brne headD"              "\n\t" // 2    if(i != 0) -> (next byte)
         "rjmp doneD"             "\n\t"
        "bitTimeD:"               "\n\t" //      nop nop nop     (T =  4)
         "out  %[port], %[next]"  "\n\t" // 1    PORT = next     (T =  5)
         "mov  %[next], %[lo]"    "\n\t" // 1    next = lo       (T =  6)
         "rol  %[byte]"           "\n\t" // 1    b <<= 1         (T =  7)
         "sbrc %[byte], 7"        "\n\t" // 1-2  if(b & 0x80)    (T =  8)
          "mov %[next], %[hi]"    "\n\t" // 0-1   next = hi      (T =  9)
         "nop"                    "\n\t" // 1                    (T = 10)
         "out  %[port], %[lo]"    "\n\t" // 1    PORT = lo       (T = 11)
         "ret"                    "\n\t" // 4    nop nop nop nop (T = 15)
         "doneD:"                 "\n"
        : [byte]  "+r" (b),
          [next]  "+r" (next),
          [count] "+w" (i)
        : [port]   "I" (_SFR_IO_ADDR(PORTD)),
          [ptr]    "e" (ptr),
          [hi]     "r" (hi),
          [lo]     "r" (lo));

    } else if(port == &PORTB) {

  #endif // PORTD

      hi   = PORTB |  pinMask;
      lo   = PORTB & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Same as above, just set for PORTB & stripped of comments
      asm volatile(
       "headB:"                   "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "out  %[port] , %[next]"  "\n\t"
        "mov  %[next] , %[lo]"    "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[next] , %[hi]"    "\n\t"
        "nop"                     "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "brne headB"              "\n\t"
         "rjmp doneB"             "\n\t"
        "bitTimeB:"               "\n\t"
         "out  %[port], %[next]"  "\n\t"
         "mov  %[next], %[lo]"    "\n\t"
         "rol  %[byte]"           "\n\t"
         "sbrc %[byte], 7"        "\n\t"
          "mov %[next], %[hi]"    "\n\t"
         "nop"                    "\n\t"
         "out  %[port], %[lo]"    "\n\t"
         "ret"                    "\n\t"
         "doneB:"                 "\n"
        : [byte] "+r" (b), [next] "+r" (next), [count] "+w" (i)
        : [port] "I" (_SFR_IO_ADDR(PORTB)), [ptr] "e" (ptr), [hi] "r" (hi),
          [lo] "r" (lo));

  #ifdef PORTD
    }
  #endif

  // 16 MHz(ish) AVR --------------------------------------------------------
  #elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^   ^        ^       (T=0,5,13)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
      "nop"                      "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
       "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

  #else
  #error("CPU SPEED NOT SUPPORTED")
  #endif // end F_CPU ifdefs on __AVR__

  // END AVR ----------------------------------------------------------------

  #elif defined(__arm__)

  // ARM MCUs -- Teensy 3.0, 3.1, LC, Arduino Due ---------------------------

  #if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1

  #define CYCLES_800_T0H  (F_CPU / 4000000)
  #define CYCLES_800_T1H  (F_CPU / 1250000)
  #define CYCLES_800      (F_CPU /  800000)
  #define CYCLES_400_T0H  (F_CPU / 2000000)
  #define CYCLES_400_T1H  (F_CPU /  833333)
  #define CYCLES_400      (F_CPU /  400000)

  uint8_t          *p   = pixels,
                   *end = p + numBytes, pix, mask;
  volatile uint8_t *set = portSetRegister(pin),
                   *clr = portClearRegister(pin);
  uint32_t          cyc;

  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    cyc = ARM_DWT_CYCCNT + CYCLES_800;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_800);

#elif defined(__MKL26Z64__) // Teensy-LC

#if F_CPU == 48000000
  uint8_t          *p   = pixels,
		   pix, count, dly,
                   bitmask = digitalPinToBitMask(pin);
  volatile uint8_t *reg = portSetRegister(pin);
  uint32_t         num = numBytes;
  asm volatile(
	"L%=_begin:"				"\n\t"
	"ldrb	%[pix], [%[p], #0]"		"\n\t"
	"lsl	%[pix], #24"			"\n\t"
	"movs	%[count], #7"			"\n\t"
	"L%=_loop:"				"\n\t"
	"lsl	%[pix], #1"			"\n\t"
	"bcs	L%=_loop_one"			"\n\t"
	"L%=_loop_zero:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_loop_delay_T0H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T0H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_loop_delay_T0L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T0L"		"\n\t"
	"b	L%=_next"			"\n\t"
	"L%=_loop_one:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_loop_delay_T1H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T1H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_loop_delay_T1L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T1L"		"\n\t"
	"nop"					"\n\t"
	"L%=_next:"				"\n\t"
	"sub	%[count], #1"			"\n\t"
	"bne	L%=_loop"			"\n\t"
	"lsl	%[pix], #1"			"\n\t"
	"bcs	L%=_last_one"			"\n\t"
	"L%=_last_zero:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_last_delay_T0H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T0H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #10"			"\n\t"
	"L%=_last_delay_T0L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T0L"		"\n\t"
	"b	L%=_repeat"			"\n\t"
	"L%=_last_one:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_last_delay_T1H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T1H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #1"			"\n\t"
	"L%=_last_delay_T1L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T1L"		"\n\t"
	"nop"					"\n\t"
	"L%=_repeat:"				"\n\t"
	"add	%[p], #1"			"\n\t"
	"sub	%[num], #1"			"\n\t"
	"bne	L%=_begin"			"\n\t"
	"L%=_done:"				"\n\t"
	: [p] "+r" (p),
	  [pix] "=&r" (pix),
	  [count] "=&r" (count),
	  [dly] "=&r" (dly),
	  [num] "+r" (num)
	: [bitmask] "r" (bitmask),
	  [reg] "r" (reg)
  );
  #else
  #error("Sorry, only 48 MHz is supported, please set Tools > CPU Speed to 48 MHz")
  #endif // F_CPU == 48000000

  #elif defined(__SAMD21G18A__) // Arduino Zero

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask, portNum;
  uint32_t  pinMask;

  portNum =  g_APinDescription[pin].ulPort;
  pinMask =  1ul << g_APinDescription[pin].ulPin;
  ptr     =  pixels;
  end     =  ptr + numBytes;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint32_t *set = &(PORT->Group[portNum].OUTSET.reg),
                    *clr = &(PORT->Group[portNum].OUTCLR.reg);

    for(;;) {
      *set = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop;");
      if(p & bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
        *clr = pinMask;
      } else {
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }

  #elif defined (ARDUINO_STM32_FEATHER) // FEATHER WICED (120MHz)

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask;
  uint32_t  pinMask;

  pinMask =  BIT(PIN_MAP[pin].gpio_bit);
  ptr     =  pixels;
  end     =  ptr + numBytes;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint16_t *set = &(PIN_MAP[pin].gpio_device->regs->BSRRL);
  volatile uint16_t *clr = &(PIN_MAP[pin].gpio_device->regs->BSRRH);

    for(;;) {
      if(p & bitMask) { // ONE
        // High 800ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
        // Low 450ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
      } else { // ZERO
        // High 400ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop;");
        // Low 850ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
        // Move on to the next pixel
        asm("nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }

  #else // Other ARM architecture -- Presumed Arduino Due

  #define SCALE      VARIANT_MCK / 2UL / 1000000UL
  #define INST       (2UL * F_CPU / VARIANT_MCK)
  #define TIME_800_0 ((int)(0.40 * SCALE + 0.5) - (5 * INST))
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))

  int             pinMask, time0, time1, period, t;
  Pio            *port;
  volatile WoReg *portSet, *portClear, *timeValue, *timeReset;
  uint8_t        *p, *end, pix, mask;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0,
    TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC1, 0);

  pinMask   = g_APinDescription[pin].ulPin; // Don't 'optimize' these into
  port      = g_APinDescription[pin].pPort; // declarations above.  Want to
  portSet   = &(port->PIO_SODR);            // burn a few cycles after
  portClear = &(port->PIO_CODR);            // starting timer to minimize
  timeValue = &(TC1->TC_CHANNEL[0].TC_CV);  // the initial 'while'.
  timeReset = &(TC1->TC_CHANNEL[0].TC_CCR);
  p         =  pixels;
  end       =  p + numBytes;
  pix       = *p++;
  mask      = 0x80;

    time0  = TIME_800_0;
    time1  = TIME_800_1;
    period = PERIOD_800;

  for(t = time0;; t = time0) {
    if(pix & mask) t = time1;
    while(*timeValue < period);
    *portSet   = pinMask;
    *timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG;
    while(*timeValue < t);
    *portClear = pinMask;
    if(!(mask >>= 1)) {   // This 'inside-out' loop logic utilizes
      if(p >= end) break; // idle time to minimize inter-byte delays.
      pix = *p++;
      mask = 0x80;
    }
  }
  while(*timeValue < period); // Wait for last bit
  TC_Stop(TC1, 0);

  #endif // end Due

  // END ARM ----------------------------------------------------------------

  #elif defined(__ARDUINO_ARC__)

  // Arduino 101  -----------------------------------------------------------

  #define NOPx7 { __builtin_arc_nop(); \
    __builtin_arc_nop(); __builtin_arc_nop(); \
    __builtin_arc_nop(); __builtin_arc_nop(); \
    __builtin_arc_nop(); __builtin_arc_nop(); }

  PinDescription *pindesc = &g_APinDescription[pin];
  register uint32_t loop = 8 * numBytes; // one loop to handle all bytes and all bits
  register uint8_t *p = pixels;
  register uint32_t currByte = (uint32_t) (*p);
  register uint32_t currBit = 0x80 & currByte;
  register uint32_t bitCounter = 0;
  register uint32_t first = 1;

  // The loop is unusual. Very first iteration puts all the way LOW to the wire -
  // constant LOW does not affect NEOPIXEL, so there is no visible effect displayed.
  // During that very first iteration CPU caches instructions in the loop.
  // Because of the caching process, "CPU slows down". NEOPIXEL pulse is very time sensitive
  // that's why we let the CPU cache first and we start regular pulse from 2nd iteration
  if (pindesc->ulGPIOType == SS_GPIO) {
    register uint32_t reg = pindesc->ulGPIOBase + SS_GPIO_SWPORTA_DR;
    uint32_t reg_val = __builtin_arc_lr((volatile uint32_t)reg);
    register uint32_t reg_bit_high = reg_val | (1 << pindesc->ulGPIOId);
    register uint32_t reg_bit_low  = reg_val & ~(1 << pindesc->ulGPIOId);

    loop += 1; // include first, special iteration
    while(loop--) {
      if(!first) {
        currByte <<= 1;
        bitCounter++;
      }

      // 1 is >550ns high and >450ns low; 0 is 200..500ns high and >450ns low
      __builtin_arc_sr(first ? reg_bit_low : reg_bit_high, (volatile uint32_t)reg);
      if(currBit) { // ~400ns HIGH (740ns overall)
        NOPx7
        NOPx7
      }
      // ~340ns HIGH
      NOPx7
     __builtin_arc_nop();

      // 820ns LOW; per spec, max allowed low here is 5000ns */
      __builtin_arc_sr(reg_bit_low, (volatile uint32_t)reg);
      NOPx7
      NOPx7

      if(bitCounter >= 8) {
        bitCounter = 0;
        currByte = (uint32_t) (*++p);
      }

      currBit = 0x80 & currByte;
      first = 0;
    }
  } else if(pindesc->ulGPIOType == SOC_GPIO) {
    register uint32_t reg = pindesc->ulGPIOBase + SOC_GPIO_SWPORTA_DR;
    uint32_t reg_val = MMIO_REG_VAL(reg);
    register uint32_t reg_bit_high = reg_val | (1 << pindesc->ulGPIOId);
    register uint32_t reg_bit_low  = reg_val & ~(1 << pindesc->ulGPIOId);

    loop += 1; // include first, special iteration
    while(loop--) {
      if(!first) {
        currByte <<= 1;
        bitCounter++;
      }
      MMIO_REG_VAL(reg) = first ? reg_bit_low : reg_bit_high;
      if(currBit) { // ~430ns HIGH (740ns overall)
        NOPx7
        NOPx7
        __builtin_arc_nop();
      }
      // ~310ns HIGH
      NOPx7

      // 850ns LOW; per spec, max allowed low here is 5000ns */
      MMIO_REG_VAL(reg) = reg_bit_low;
      NOPx7
      NOPx7

      if(bitCounter >= 8) {
        bitCounter = 0;
        currByte = (uint32_t) (*++p);
      }

      currBit = 0x80 & currByte;
      first = 0;
    }
  }

  #else
  #error("No processor selected for NeoPixelShow")
  #endif

  // END ARCHITECTURE SELECT ------------------------------------------------

  #if !defined(ESP32)
  interrupts();
  #endif
  
  endTime = micros(); // Save EOD time for latch on next call
}
