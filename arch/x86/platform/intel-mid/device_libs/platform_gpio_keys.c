/*
 * platform_gpio_keys.c: gpio_keys platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/pnp.h>
#include <asm/intel_vlv2.h>
#include <asm/io_apic.h>
#include <asm/acpi.h>
#include <asm/hw_irq.h>
#include <asm/intel-mid.h>
#include "platform_gpio_keys.h"

enum {
	PWRBTN_KEY,
	ROLOCK_KEY,
	VOL_KEYS,
	KEY_TYPE_NUMS,
};

static struct gpio_keys_button lesskey_button_powerbtn[] = {
	{KEY_POWER,		-1, 1, "power_btn", EV_KEY, .acpi_idx = 0, 1},
	{ },
};

static struct gpio_keys_button lesskey_button_rolock[] = {
	{KEY_RO,		-1, 1, "rotationlock",	EV_KEY, .acpi_idx = 4},
	{ },
};

static struct gpio_keys_button lesskey_button_vol[] = {
	{KEY_VOLUMEUP,		-1, 1, "volume_up",	EV_KEY, .acpi_idx = 2},
	{KEY_VOLUMEDOWN,	-1, 1, "volume_down",	EV_KEY, .acpi_idx = 3},
};

struct gpio_keys_init_data {
	struct gpio_keys_button *keys_button;
	int nkeys;
};

static struct gpio_keys_init_data lesskey_init_data[KEY_TYPE_NUMS] = {
	{
		.keys_button = lesskey_button_powerbtn,
		.nkeys = 1,
	}, {
		.keys_button = lesskey_button_rolock,
		.nkeys = 1,
	}, {
		.keys_button = lesskey_button_vol,
		.nkeys = sizeof(lesskey_button_vol) /
			 sizeof(struct gpio_keys_button),
	},
};

static struct gpio_keys_platform_data lesskey_keys[KEY_TYPE_NUMS] = {
	{
		.buttons	= lesskey_button_powerbtn,
		.rep		= 0,
		.nbuttons	= 0,
	}, {
		.buttons	= lesskey_button_rolock,
		.rep		= 0,
		.nbuttons	= 0,
	}, {
		.buttons	= lesskey_button_vol,
		.rep		= 1,
		.nbuttons	= 0,
	},
};

static struct platform_device lesskey_device[KEY_TYPE_NUMS] = {
	{
		.name		= "gpio-lesskey",
		.id		= PLATFORM_DEVID_AUTO,
		.dev		= {
			.platform_data	= &lesskey_keys[PWRBTN_KEY],
		},
	}, {
		.name		= "gpio-lesskey-rolock",
		.id		= PLATFORM_DEVID_AUTO,
		.dev		= {
			.platform_data	= &lesskey_keys[ROLOCK_KEY],
		},
	}, {
		.name		= "gpio-keys",
		.id		= PLATFORM_DEVID_AUTO,
		.dev		= {
			.platform_data	= &lesskey_keys[VOL_KEYS],
		},
	},
};

static int
lesskey_pnp_probe(struct pnp_dev *pdev, const struct pnp_device_id *id)
{
	int type, i, num, good;
	struct gpio_keys_button *gb;
	struct acpi_gpio_info info;
	int ret = 0;

	for (type = 0; type < KEY_TYPE_NUMS; type++) {
		good = 0;
		gb = lesskey_init_data[type].keys_button;
		num = lesskey_init_data[type].nkeys;

		for (i = 0; i < num; i++) {
			gb[i].gpio = acpi_get_gpio_by_index(&pdev->dev,
							gb[i].acpi_idx, &info);
			pr_info("lesskey [%2d]: name = %s, gpio = %d\n",
				 i, gb[i].desc, gb[i].gpio);
			if (gb[i].gpio < 0)
				continue;
			if (i != good)
				gb[good] = gb[i];
			good++;
		}

		if (good) {
			lesskey_keys[type].nbuttons = good;
			ret = platform_device_register(&lesskey_device[type]);
			if (ret) {
				dev_err(&pdev->dev, "register platform device %s failed\n",
					lesskey_device[type].name);
				return ret;
			}
		}
	}

	return 0;
}

static const struct pnp_device_id lesskey_pnp_match[] = {
	{ "INTCFD9", 0},
	{ }
};
MODULE_DEVICE_TABLE(pnp, lesskey_pnp_match);

static struct pnp_driver lesskey_pnp_driver = {
	.name		= "lesskey",
	.id_table	= lesskey_pnp_match,
	.probe          = lesskey_pnp_probe,
};

static int __init lesskey_init(void)
{
	return pnp_register_driver(&lesskey_pnp_driver);
}

late_initcall(lesskey_init);
