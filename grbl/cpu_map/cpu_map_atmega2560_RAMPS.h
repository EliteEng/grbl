/*
  cpu_map_atmega2560.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This cpu_map file serves as a central pin mapping settings file for AVR Mega 2560 */


#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "Atmega2560"

  // Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
#define RX_BUFFER_SIZE    256
#define TX_BUFFER_SIZE    128

  // Define step pulse output pins.
#define X_STEP_DDR      DDRF
#define X_STEP_PORT     PORTF
#define X_STEP_PIN      PINF
#define X_STEP_BIT        0 // MEGA2560 A0

#define Y_STEP_DDR      DDRF
#define Y_STEP_PORT     PORTF
#define Y_STEP_PIN      PINF
#define Y_STEP_BIT        6 // MEGA2560 A6

#define Z_STEP_DDR      DDRL
#define Z_STEP_PORT     PORTL
#define Z_STEP_PIN      PINL
#define Z_STEP_BIT        3 // MEGA2560 Digital Pin 46


  // Define step direction output pins.
#define X_DIRECTION_DDR      DDRF
#define X_DIRECTION_PORT     PORTF
#define X_DIRECTION_PIN      PINF
#define X_DIRECTION_BIT   1 // MEGA2560 A1

#define Y_DIRECTION_DDR      DDRF
#define Y_DIRECTION_PORT     PORTF
#define Y_DIRECTION_PIN      PINF
#define Y_DIRECTION_BIT   7 // MEGA2560 A7

#define Z_DIRECTION_DDR      DDRL
#define Z_DIRECTION_PORT     PORTL
#define Z_DIRECTION_PIN      PINL
#define Z_DIRECTION_BIT   1 // MEGA2560 Digital Pin 48

  // Define stepper driver enable/disable output pin.
#define X_STEPPERS_DISABLE_DDR   DDRD
#define X_STEPPERS_DISABLE_PORT  PORTD
#define X_STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 38
#define X_STEPPERS_DISABLE_MASK (1<<X_STEPPERS_DISABLE_BIT)

#define Y_STEPPERS_DISABLE_DDR   DDRF
#define Y_STEPPERS_DISABLE_PORT  PORTF
#define Y_STEPPERS_DISABLE_BIT   2 // MEGA2560 Digital Pin 56
#define Y_STEPPERS_DISABLE_MASK (1<<Y_STEPPERS_DISABLE_BIT)

#define Z_STEPPERS_DISABLE_DDR   DDRK
#define Z_STEPPERS_DISABLE_PORT  PORTK
#define Z_STEPPERS_DISABLE_BIT   0 // MEGA2560 Digital Pin 62
#define Z_STEPPERS_DISABLE_MASK (1<<Z_STEPPERS_DISABLE_BIT)

  // NOTE: All limit bit pins must be on the same port
  // FIXME this does not work on RAMPS needs to be split out to separate pins
#define LIMIT_DDR       DDRB
#define LIMIT_PORT      PORTB
#define LIMIT_PIN       PINB
#define X_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
#define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
#define Z_LIMIT_BIT     6 // MEGA2560 Digital Pin 12
#define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT0_vect
#define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR   DDRH
#define SPINDLE_ENABLE_PORT  PORTH
#define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6
#define SPINDLE_DIRECTION_DDR   DDRE
#define SPINDLE_DIRECTION_PORT  PORTE
#define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
#define COOLANT_FLOOD_DDR   DDRH
#define COOLANT_FLOOD_PORT  PORTH
#define COOLANT_FLOOD_BIT   5 // MEGA2560 Digital Pin 8
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_DDR   DDRH
#define COOLANT_MIST_PORT  PORTH
#define COOLANT_MIST_BIT   6 // MEGA2560 Digital Pin 9
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_DDR       DDRK
#define CONTROL_PIN       PINK
#define CONTROL_PORT      PORTK
#define RESET_BIT         4  // MEGA2560 Analog Pin 8
#define FEED_HOLD_BIT     1  // MEGA2560 Analog Pin 9
#define CYCLE_START_BIT   2  // MEGA2560 Analog Pin 10
#define SAFETY_DOOR_BIT   3  // MEGA2560 Analog Pin 11
#define CONTROL_INT       PCIE2  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT2_vect
#define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

  // Define probe switch input pin.
#define PROBE_DDR       DDRD
#define PROBE_PIN       PIND
#define PROBE_PORT      PORTD
#define PROBE_BIT       3  // MEGA2560 Digital pin 18 (ZMIN)
#define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER2B which is attached to Digital Pin 9
#define TCCRA_REGISTER    TCCR2A
#define TCCRB_REGISTER    TCCR2B
#define OCR_REGISTER    OCR2B

#define COMB_BIT      COM2B1
#define WAVE0_REGISTER    WGM20
#define WAVE1_REGISTER    WGM21
#define WAVE2_REGISTER    WGM22
#define WAVE3_REGISTER    WGM23

#define SPINDLE_PWM_DDR   DDRH
#define SPINDLE_PWM_PORT    PORTH
#define SPINDLE_PWM_BIT   6 // MEGA2560 Digital Pin 9
#endif // End of VARIABLE_SPINDLE
