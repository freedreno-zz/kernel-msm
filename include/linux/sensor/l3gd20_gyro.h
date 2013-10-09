/*
 *	Copyright (C) 2011, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 */

#ifndef __L3GD20_H__
#define __L3GD20_H__

#define L3GD20_GYRO_DEV_NAME	"l3gd20_gyro"
#define L3GD20_GYR_INPUT_NAME "gyro_sensor"
#define GYR_DEV_FILE_NAME "l3gd20_gyro_misc"

#define L3GD20_GYRO_IOCTL_BASE	80
#define L3GD20_GYRO_IOCTL_SET_DELAY\
	_IOW(L3GD20_GYRO_IOCTL_BASE, 0, int64_t)
#define L3GD20_GYRO_IOCTL_GET_DELAY\
	_IOR(L3GD20_GYRO_IOCTL_BASE, 1, int64_t)
#define L3GD20_GYRO_IOCTL_READ_DATA_XYZ\
	_IOR(L3GD20_GYRO_IOCTL_BASE, 2, int)

struct l3gd20_gyro_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
	int gpio_DRDY;
};

#endif /* __L3GD20_H__ */
