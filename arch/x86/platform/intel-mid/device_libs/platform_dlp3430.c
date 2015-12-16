/*
 * platform_display.c: put platform display configuration in
 * this file. If any platform level display related configuration
 * has to be made, then that configuration shoule be in this
 * file.
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/printk.h>
#include <linux/platform_data/dlp3430.h>
#include <asm/spid.h>


static struct i2c_board_info __initdata dlp3430_i2c_device = {
	I2C_BOARD_INFO("dlp3430", 0x1b),
};
static struct dlp3430_rom_data dlp3430_rom_data[1] = {0x10, 0x84};

struct dlp3430_platform_data dlp3430_platform_data = {
	.name = "dlp3430",
	.device_control = 0,
	.initial_brightness = 0,
	.period_ns = 5000000, /* 200 Hz */
	.size_program = 1,
	.rom_data = dlp3430_rom_data,
};

void *dlp3430_get_platform_data(void)
{

	return (void *)&dlp3430_platform_data;
}

static int __init platform_dlp3430_module_init(void)
{

	dlp3430_i2c_device.platform_data = dlp3430_get_platform_data();

	if (dlp3430_i2c_device.platform_data == NULL) {
		pr_debug("failed to get platform data for dlp3430");
		return -EINVAL;
	}

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR0) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR1))
		return i2c_register_board_info(3, &dlp3430_i2c_device, 1);

	return -EPERM;
}

rootfs_initcall(platform_dlp3430_module_init);

