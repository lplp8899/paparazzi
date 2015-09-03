/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include <stdio.h>
#include <stdlib.h>

#include "vision_sim.h"

// define void functions to do simulation
void opticflow_module_init(void) 
{
  //printf("do nothing\n");
}
void opticflow_module_start(void) 
{
  //printf("do nothing\n");
}
void opticflow_module_run(void) 
{
//yaw_ref_fun(); // will be run 512 Hz, check the airframe and module file
  //printf("do nothing\n");
}
void opticflow_module_stop(void) 
{
  //printf("do nothing\n");
}

float yaw_ref_sim;

float yaw_ref_fun() 
{
	float m[10] = {0,0,0,0,0,45,0,0,0,-45};
	int n;
	n = rand() % 10;
	
	yaw_ref_sim = m[n];
	printf("fun[%d] = %f\n",n,yaw_ref_sim);
	return yaw_ref_sim;
	
	//return m[n];
}





