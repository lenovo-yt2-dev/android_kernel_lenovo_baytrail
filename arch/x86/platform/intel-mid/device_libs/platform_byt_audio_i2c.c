/*
 * platform_byt_audio.c: Baytrail audio platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Md Abudllah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/platform_sst_audio.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_device.h>
#include <linux/sfi.h>
#include <linux/spi/spi.h>
#include <asm/intel-mid.h>
#include "platform_wm5102.h"

static struct regulator_consumer_supply dc1v8_consumers[] = {
	REGULATOR_SUPPLY("AVDD", "2-001b"), /* wm5102 */
	REGULATOR_SUPPLY("DBVDD1", "2-001b"), /* wm5102 */
	REGULATOR_SUPPLY("LDOVDD", "2-001b"),
	REGULATOR_SUPPLY("DBVDD2", "wm5102-codec"),
	REGULATOR_SUPPLY("DBVDD3", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDL", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "2-001b"),
};

static struct regulator_init_data dc1v8_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(dc1v8_consumers),
	.consumer_supplies = dc1v8_consumers,
};

static struct fixed_voltage_config dc1v8vdd_pdata = {
	.supply_name = "DC_1V8",
	.microvolts = 1800000,
	.init_data = &dc1v8_data,
	.gpio = -1,
};

static struct platform_device dc1v8_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data = &dc1v8vdd_pdata,
	},
};
static struct byt_audio_platform_data byt_wm5102_pdata;

static struct i2c_board_info __initdata wm5102_board_info = {
	I2C_BOARD_INFO("wm5102", 0x1b),
	.platform_data = &wm5102_platform_data,
};

static int __init byt_audio_platform_init(void)
{
	struct platform_device *pdev;
	int ret;

	pr_info("in %s\n", __func__);

	ret = add_sst_platform_device();

	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return 0;
	}

	platform_device_register(&dc1v8_device);

	wm5102_board_info.platform_data = wm5102_platform_data(&wm5102_board_info);
	i2c_register_board_info(2, &wm5102_board_info, 1);

	pdev = platform_device_alloc("byt_wm5102", -1);
	
	if (!pdev) {
		pr_err("failed to allocate byt_wm5102 platform device\n");
		return 0;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add byt_wm5102 platform device\n");
		platform_device_put(pdev);
		return 0;
	}
	/*if (platform_device_add_data(pdev, &byt_wm5102_pdata,
				     sizeof(byt_wm5102_pdata))) {
		pr_err("failed to add byt_wm5102 platform data\n");
		platform_device_put(pdev);
		return 0;
	}*/
	return 0;
}
device_initcall(byt_audio_platform_init);
