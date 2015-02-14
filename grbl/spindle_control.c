/*
  spindle_control.c - spindle control methods
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
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2012 Sungeun K. Jeon
*/ 

#include "grbl.h"


void spindle_init()
{    
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    #ifdef CPU_MAP_ATMEGA328P
      SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
      TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20);
      TCCR2B = (TCCR2B & 0xF8) | 0x02; // set to 1/8 Prescaler
      OCR2A = 0;
    #else
      SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
      TCCR4A = (1<<COM4B1) | (1<<WGM41) | (1<<WGM40);
      TCCR4B = (TCCR4B & 0xF8) | 0x02 | (1<<WGM42) | (1<<WGM43); // set to 1/8 Prescaler
      OCR4A = 0xFFFF; // set the top 16bit value
      OCR4B = 0;
    #endif     
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  #endif
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    #ifdef CPU_MAP_ATMEGA328P
	    TCCR2A &= ~(1<<COM2A1); // Disable PWM. Output voltage is zero.
    #else
	    TCCR4A &= ~(1<<COM4B1); // Disable PWM. Output voltage is zero. 
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
    #endif
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
  #endif  
}


void spindle_set_state(uint8_t state, float rpm)
{
  // Halt or set spindle direction and rpm. 
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

    if (state == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }

    #ifdef VARIABLE_SPINDLE
      // TODO: Install the optional capability for frequency-based output for servos.
    
      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
      else { 
        rpm -= SPINDLE_MIN_RPM; 
        if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
      }
    
      #ifdef CPU_MAP_ATMEGA328P
        uint8_t current_pwm = floor( rpm*(255.0/SPINDLE_RPM_RANGE) + 0.5);
      #else
        uint16_t current_pwm = floor( rpm*(65535.0/SPINDLE_RPM_RANGE) + 0.5);;
      #endif

      #ifdef MINIMUM_SPINDLE_PWM
        if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
      #endif
        
      #ifdef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared. 
        OCR2A = current_pwm; // Set PWM pin output
      #else 
        OCR4B = current_pwm; // Set PWM pin output
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    #else   
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
    #endif

  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}
