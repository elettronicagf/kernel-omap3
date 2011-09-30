/*
 * drivers/media/video/tvp515x.h
 *
 * Copyright (C) 2011 Elettronica GF S.r.l.
 * Andrea Collamati <andrea.collamati@elettronicagf.it>
 *
 * Using code from drivers/media/video/tvp514x.h
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _TVP515X_H
#define _TVP515X_H

#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

/*
 * Other macros
 */
#define TVP515X_MODULE_NAME		"tvp515x"

#define TVP515X_XCLK_BT656		(27000000)

/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS		(720)
#define NTSC_NUM_ACTIVE_LINES		(480)
#define PAL_NUM_ACTIVE_PIXELS		(720)
#define PAL_NUM_ACTIVE_LINES		(576)

/**
 * enum tvp515x_input - enum for different decoder input pin
 *		configuration.
 */
enum tvp515x_input {
	/*
	 * Input selection
	 */
	INPUT_CVBS_P1A = 0x0,
	INPUT_SVIDEO_PI1A_P1B,
	INPUT_CVBS_P1B,
	/* 	In Som336 module the port CVBS_P1A and CVBS_P1B
	 * 	are mapped to different connectors using a gpio
	 * 	This is how the input 0-3 are mapped on Som336 module.
		Connector CVBS CN39 = 0x0,		 Gpio 138 = 0   Port = CVBS_P1A
		Connector CVBS CN36 = 0x1,		 Gpio 138 = 0   Port = CVBS_P1B
		Connector CVBS CN38 = 0x2,		 Gpio 138 = 1   Port = CVBS_P1A
		Connector CVBS CN37 = 0x3,		 Gpio 138 = 1   Port = CVBS_P1B
	*/
#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	INPUT_INVALID=4,
#else
	INPUT_INVALID=3,
#endif
};


/**
 * enum tvp515x_output - enum for output format
 *			supported.
 *
 */
enum tvp515x_output {
	OUTPUT_8BIT_422_SEPERATE_SYNC = 0,
	OUTPUT_8BIT_422_EMBEDDED_SYNC = 7,
	OUTPUT_INVALID
};

/**
 * struct tvp515x_platform_data - Platform data values and access functions.
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function.
 * @priv_data_set: Device private data (pointer) access function.
 */
struct tvp515x_platform_data {
    int (*s_power) (struct v4l2_subdev *subdev, u32 on);
};


#endif				/* ifndef _TVP515X_H */
