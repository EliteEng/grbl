/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl v0.9

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2015 Rob Brown

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

/* The cpu_map.h file serves as a central pin mapping settings file for different processor
   types, i.e. AVR 328p or AVR Mega 2560. Grbl officially supports the Arduino Uno, but the 
   other supplied pin mappings are supplied by users, so your results may vary. */

// NOTE: This is still a work in progress. We are still centralizing the configurations to
// this file, so your success may vary for other CPUs.

#ifndef cpu_map_h
#define cpu_map_h

#include "grbl.h"

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define X_STEP_DDR    DDRD
  #define X_STEP_PORT   PORTD
  #define X_STEP_BIT	  2  // Uno Digital Pin 2

  #define Y_STEP_DDR    DDRD
  #define Y_STEP_PORT   PORTD
  #define Y_STEP_BIT	  3  // Uno Digital Pin 3

  #define Z_STEP_DDR    DDRD
  #define Z_STEP_PORT   PORTD
  #define Z_STEP_BIT	  4  // Uno Digital Pin 4
  

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define X_DIRECTION_DDR     DDRD
  #define X_DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT	    5  // Uno Digital Pin 5

  #define Y_DIRECTION_DDR     DDRD
  #define Y_DIRECTION_PORT    PORTD
  #define Y_DIRECTION_BIT     6  // Uno Digital Pin 6

  #define Z_DIRECTION_DDR     DDRD
  #define Z_DIRECTION_PORT    PORTD
  #define Z_DIRECTION_BIT     7  // Uno Digital Pin 7

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
  #define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect 
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRB
  #define SPINDLE_ENABLE_PORT     PORTB
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif  
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
  #endif  

  // Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRC
  #define CONTROL_PIN       PINC
  #define CONTROL_PORT      PORTC
  #define RESET_BIT         0  // Uno Analog Pin 0
  #define FEED_HOLD_BIT     1  // Uno Analog Pin 1
  #define CYCLE_START_BIT   2  // Uno Analog Pin 2
  #define SAFETY_DOOR_BIT   1  // Uno Analog Pin 1 NOTE: Safety door is shared with feed hold. Enabled by config define.
  #define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
  
  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // Uno Analog Pin 5
  #define PROBE_MASK      (1<<PROBE_BIT)

  
  #ifdef VARIABLE_SPINDLE
    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
    #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
    #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
  #endif // End of VARIABLE_SPINDLE

#endif


//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA2560 // (Arduino Mega 2560) Working @EliteEng

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define X_STEP_DDR      DDRA
  #define X_STEP_PORT     PORTA
  #define X_STEP_BIT      2 // MEGA2560 Digital Pin 24

  #define Y_STEP_DDR      DDRA
  #define Y_STEP_PORT     PORTA
  #define Y_STEP_BIT      3 // MEGA2560 Digital Pin 25

  #define Z_STEP_DDR      DDRA
  #define Z_STEP_PORT     PORTA
  #define Z_STEP_BIT      4 // MEGA2560 Digital Pin 26

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define X_DIRECTION_DDR     DDRC
  #define X_DIRECTION_PORT    PORTC
  #define X_DIRECTION_BIT     7 // MEGA2560 Digital Pin 30

  #define Y_DIRECTION_DDR     DDRC
  #define Y_DIRECTION_PORT    PORTC
  #define Y_DIRECTION_BIT     6 // MEGA2560 Digital Pin 31

  #define Z_DIRECTION_DDR     DDRC
  #define Z_DIRECTION_PORT    PORTC
  #define Z_DIRECTION_BIT     5 // MEGA2560 Digital Pin 32

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRB
  #define STEPPERS_DISABLE_PORT  PORTB
  #define STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 13
  #define STEPPERS_DISABLE_MASK  (1<<STEPPERS_DISABLE_BIT)

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
  #define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
  #define Z_LIMIT_BIT     6 // MEGA2560 Digital Pin 12
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK      ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRH
  #define SPINDLE_ENABLE_PORT     PORTH
  #define SPINDLE_ENABLE_BIT      3 // MEGA2560 Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR     DDRH
  #define COOLANT_FLOOD_PORT    PORTH
  #define COOLANT_FLOOD_BIT     5 // MEGA2560 Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR    DDRH
    #define COOLANT_MIST_PORT   PORTH
    #define COOLANT_MIST_BIT    6 // MEGA2560 Digital Pin 9
  #endif  

  // Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRK
  #define CONTROL_PIN       PINK
  #define CONTROL_PORT      PORTK
  #define RESET_BIT         0  // MEGA2560 Analog Pin 8
  #define FEED_HOLD_BIT     1  // MEGA2560 Analog Pin 9
  #define CYCLE_START_BIT   2  // MEGA2560 Analog Pin 10
  #define SAFETY_DOOR_BIT   3  // MEGA2560 Analog Pin 11
  #define CONTROL_INT       PCIE2  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT2_vect
  #define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRK
  #define PROBE_PIN       PINK
  #define PROBE_PORT      PORTK
  #define PROBE_BIT       7  // MEGA2560 Analog Pin 15
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    #define SPINDLE_PWM_DDR		DDRH
    #define SPINDLE_PWM_PORT    PORTH
    #define SPINDLE_PWM_BIT		4 // MEGA2560 Digital Pin 7
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_RAMPS // (Arduino Mega 2560 with RAMPS)

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define X_STEP_DDR      DDRF
  #define X_STEP_PORT     PORTF
  #define X_STEP_BIT      0 // MEGA2560 Analog Pin 0

  #define Y_STEP_DDR      DDRF
  #define Y_STEP_PORT     PORTF
  #define Y_STEP_BIT      6 // MEGA2560 Analog Pin 6

  #define Z_STEP_DDR      DDRL
  #define Z_STEP_PORT     PORTL
  #define Z_STEP_BIT      3 // MEGA2560 Digital Pin 26

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define X_DIRECTION_DDR     DDRF
  #define X_DIRECTION_PORT    PORTF
  #define X_DIRECTION_BIT     1 // MEGA2560 Analog Pin 1

  #define Y_DIRECTION_DDR     DDRF
  #define Y_DIRECTION_PORT    PORTF
  #define Y_DIRECTION_BIT     7 // MEGA2560 Analog Pin 7

  #define Z_DIRECTION_DDR     DDRL
  #define Z_DIRECTION_PORT    PORTL
  #define Z_DIRECTION_BIT     1 // MEGA2560 Digital Pin 48

  // Define stepper driver enable/disable output pin.
  #define MULTI_STEPPER_DISABLE
  
  #define X_STEPPERS_DISABLE_DDR   DDRD
  #define X_STEPPERS_DISABLE_PORT  PORTD
  #define X_STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 38
  
  #define Y_STEPPERS_DISABLE_DDR   DDRF
  #define Y_STEPPERS_DISABLE_PORT  PORTF
  #define Y_STEPPERS_DISABLE_BIT   2 // MEGA2560 Analog Pin 2
  
  #define Z_STEPPERS_DISABLE_DDR   DDRK
  #define Z_STEPPERS_DISABLE_PORT  PORTK
  #define Z_STEPPERS_DISABLE_BIT   1 // MEGA2560 Analog Pin 9


  // NOTE: All limit bit pins must be on the same port
  // Needs changing so Limits can be on different ports
  #define LIMIT_DDR       DDRK
  #define LIMIT_PORT      PORTK
  #define LIMIT_PIN       PINK
  #define X_LIMIT_BIT     1 // MEGA2560 Analog Pin 9
  #define Y_LIMIT_BIT     2 // MEGA2560 Analog Pin 10
  #define Z_LIMIT_BIT     3 // MEGA2560 Analog Pin 11
  #define LIMIT_INT       PCIE2  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT2_vect 
  #define LIMIT_PCMSK     PCMSK2 // Pin change interrupt register
  #define LIMIT_MASK      ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRL
  #define SPINDLE_ENABLE_PORT     PORTL
  #define SPINDLE_ENABLE_BIT      2 // MEGA2560 Digital Pin 47
  #define SPINDLE_DIRECTION_DDR   DDRH
  #define SPINDLE_DIRECTION_PORT  PORTH
  #define SPINDLE_DIRECTION_BIT   6 // MEGA2560 Digital Pin 9

  // Define flood coolant enable output pins.
  #define COOLANT_FLOOD_DDR     DDRH
  #define COOLANT_FLOOD_PORT    PORTH
  #define COOLANT_FLOOD_BIT     5 // MEGA2560 Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR  DDRL
    #define COOLANT_MIST_PORT PORTL
    #define COOLANT_MIST_BIT  4 // MEGA2560 Digital Pin 45
  #endif
  

  // Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRB
  #define CONTROL_PIN       PINB
  #define CONTROL_PORT      PORTB
  #define RESET_BIT         0  // MEGA2560 Digital Pin 53
  #define FEED_HOLD_BIT     1  // MEGA2560 Digital Pin 52
  #define CYCLE_START_BIT   3  // MEGA2560 Digital Pin 50
  #define SAFETY_DOOR_BIT   2  // MEGA2560 Digital Pin 51
  #define CONTROL_INT       PCIE0  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT0_vect
  #define CONTROL_PCMSK     PCMSK0 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // MEGA2560 Digital Pin 32
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    #define SPINDLE_PWM_DDR   DDRB
    #define SPINDLE_PWM_PORT  PORTB
    #define SPINDLE_PWM_BIT	4 // MEGA2560 Digital Pin 10
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_MEGA_GSHIELD // (Arduino Mega 2560 with GSHIELD)

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Define step pulse output pins.
  #define X_STEP_DDR    DDRE
  #define X_STEP_PORT   PORTE
  #define X_STEP_BIT    4 // MEGA2560 Digital Pin 2
  
  #define Y_STEP_DDR    DDRE
  #define Y_STEP_PORT   PORTE
  #define Y_STEP_BIT    5 // MEGA2560 Digital Pin 3
  
  #define Z_STEP_DDR    DDRG
  #define Z_STEP_PORT   PORTG
  #define Z_STEP_BIT    5 // MEGA2560 Digital Pin 4
  
  // Define step direction output pins.
  
  #define X_DIRECTION_DDR   DDRE
  #define X_DIRECTION_PORT  PORTE
  #define X_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5
  
  #define Y_DIRECTION_DDR   DDRH
  #define Y_DIRECTION_PORT  PORTH
  #define Y_DIRECTION_BIT   3 // MEGA2560 Digital Pin 6
  
  #define Z_DIRECTION_DDR   DDRH
  #define Z_DIRECTION_PORT  PORTH
  #define Z_DIRECTION_BIT   4 // MEGA2560 Digital Pin 7

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRH
  #define STEPPERS_DISABLE_PORT   PORTH
  #define STEPPERS_DISABLE_BIT    5 // MEGA2560 Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)


  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRD
  #define LIMIT_PORT      PORTD
  #define LIMIT_PIN       PIND
  #define X_LIMIT_BIT     0 // MEGA2560 Digital Pin 21
  #define Y_LIMIT_BIT     1 // MEGA2560 Digital Pin 20
  #define Z_LIMIT_BIT     2 // MEGA2560 Digital Pin 19
  #define LIMIT_INT       PCIE0 // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK      ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRB
  #define SPINDLE_ENABLE_PORT     PORTB
  #define SPINDLE_ENABLE_BIT      6 // MEGA2560 Digital Pin 34
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   7 // MEGA2560 Digital Pin 35

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRF
  #define COOLANT_FLOOD_PORT  PORTF
  #define COOLANT_FLOOD_BIT   3 // MEGA2560 Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR  DDRF
    #define COOLANT_MIST_PORT PORTF
    #define COOLANT_MIST_BIT  4 // MEGA2560 Analog Pin 4
  #endif 
  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRK
  #define CONTROL_PIN       PINK
  #define CONTROL_PORT      PORTK
  #define RESET_BIT         0 // MEGA2560 Analog Pin 8
  #define FEED_HOLD_BIT     1 // MEGA2560 Analog Pin 9
  #define CYCLE_START_BIT   2 // MEGA2560 Analog Pin 10
  #define SAFETY_DOOR_BIT   3 // MEGA2560 Analog Pin 11
  #define CONTROL_INT       PCIE2 // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT2_vect
  #define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))

  // Define probe switch input pin.
  #define PROBE_DDR   DDRF
  #define PROBE_PIN   PINF
  #define PROBE_PORT  PORTF
  #define PROBE_BIT   5 // MEGA2560 Analog Pin 5
  #define PROBE_MASK  (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    #define SPINDLE_PWM_DDR   DDRH
    #define SPINDLE_PWM_PORT  PORTH
    #define SPINDLE_PWM_BIT	6 // MEGA2560 Digital Pin 9
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

/* 
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and paste one of the default cpu map
  // settings above and modify it to your needs. Then, make sure the defined name is also
  // changed in the config.h file.
#endif
*/

typedef struct {
	
	volatile uint8_t* step_ddr[N_AXIS];
	volatile uint8_t* step_port[N_AXIS];
	uint8_t step_bit[N_AXIS];
	
	volatile uint8_t* direction_ddr[N_AXIS];
	volatile uint8_t* direction_port[N_AXIS];
	uint8_t direction_bit[N_AXIS];
	
	#ifdef MULTI_STEPPER_DISABLE
	  volatile uint8_t* stepper_disable_ddr[N_AXIS];
	  volatile uint8_t* stepper_disable_port[N_AXIS];
	  uint8_t stepper_disable_bit[N_AXIS];
	#endif
	
} cpu_t;
extern cpu_t cpu;

void cpu_init(void);

#endif
