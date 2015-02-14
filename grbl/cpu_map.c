/*
  cpu_map.c - CPU and pin mapping loading values into cpu struct
  Part of Grbl v0.9

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

#include "cpu_map.h"

void cpu_init() 
{
	
	cpu.step_ddr[X_AXIS] = &X_STEP_DDR;
	cpu.step_port[X_AXIS] = &X_STEP_PORT;
	cpu.step_bit[X_AXIS] = X_STEP_BIT;
	
	cpu.step_ddr[Y_AXIS] = &Y_STEP_DDR;
	cpu.step_port[Y_AXIS] = &Y_STEP_PORT;
	cpu.step_bit[Y_AXIS] = Y_STEP_BIT;
	
	cpu.step_ddr[Z_AXIS] = &Z_STEP_DDR;
	cpu.step_port[Z_AXIS] = &Z_STEP_PORT;
	cpu.step_bit[Z_AXIS] = Z_STEP_BIT;

	cpu.direction_ddr[X_AXIS] = &X_DIRECTION_DDR;
	cpu.direction_port[X_AXIS] = &X_DIRECTION_PORT;
	cpu.direction_bit[X_AXIS] = X_DIRECTION_BIT;
	
	cpu.direction_ddr[Y_AXIS] = &Y_DIRECTION_DDR;
	cpu.direction_port[Y_AXIS] = &Y_DIRECTION_PORT;
	cpu.direction_bit[Y_AXIS] = Y_DIRECTION_BIT;
	
	cpu.direction_ddr[Z_AXIS] = &Z_DIRECTION_DDR;
	cpu.direction_port[Z_AXIS] = &Z_DIRECTION_PORT;
	cpu.direction_bit[Z_AXIS] = Z_DIRECTION_BIT;
	
	#ifdef MULTI_STEPPER_DISABLE
	  cpu.stepper_disable_ddr[X_AXIS] = &X_STEPPERS_DISABLE_DDR;
	  cpu.stepper_disable_port[X_AXIS] = &X_STEPPERS_DISABLE_PORT;
	  cpu.stepper_disable_bit[X_AXIS] = X_STEPPERS_DISABLE_BIT;
	  
	  cpu.stepper_disable_ddr[Y_AXIS] = &Y_STEPPERS_DISABLE_DDR;
	  cpu.stepper_disable_port[Y_AXIS] = &Y_STEPPERS_DISABLE_PORT;
	  cpu.stepper_disable_bit[Y_AXIS] = Y_STEPPERS_DISABLE_BIT;
	  
	  cpu.stepper_disable_ddr[Z_AXIS] = &Z_STEPPERS_DISABLE_DDR;
	  cpu.stepper_disable_port[Z_AXIS] = &Z_STEPPERS_DISABLE_PORT;
	  cpu.stepper_disable_bit[Z_AXIS] = Z_STEPPERS_DISABLE_BIT;
	#endif

}
