/*
 * Dollar Cove  --  Device access for Intel PMIC for CR
 *
 * Copyright (c) 2014, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>
#include "./pmic.h"

enum {
	VBUS_FALLING_IRQ = 2,
	VBUS_RISING_IRQ,
	VBUS_OV_IRQ,
	VBUS_FALLING_ALT_IRQ,
	VBUS_RISING_ALT_IRQ,
	VBUS_OV_ALT_IRQ,

	CHARGE_DONE_IRQ = 10,
	CHARGE_CHARGING_IRQ,
	BAT_SAFE_QUIT_IRQ,
	BAT_SAFE_ENTER_IRQ,
	BAT_ABSENT_IRQ,
	BAT_APPEND_IRQ,

	QWBTU_IRQ = 16,
	WBTU_IRQ,
	QWBTO_IRQ,
	WBTO_IRQ,
	QCBTU_IRQ,
	CBTU_IRQ,
	QCBTO_IRQ,
	CBTO_IRQ,

	WL2_IRQ = 24,
	WL1_IRQ,
	GPADC_IRQ,
	OT_IRQ = 31,

	GPIO0_IRQ = 32,
	GPIO1_IRQ,
	POKO_IRQ,
	POKL_IRQ,
	POKS_IRQ,
	POKN_IRQ,
	POKP_IRQ,
	EVENT_IRQ,

	MV_CHNG_IRQ = 40,
	BC_USB_CHNG_IRQ,
};

static struct resource power_button_resources[] = {
	{
		.start	= POKN_IRQ,
		.end	= POKN_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= POKP_IRQ,
		.end	= POKP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct resource gpio_resources[] = {
	{
		.start	= GPIO0_IRQ,
		.end	= GPIO1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource adc_resources[] = {
	{
		.start = GPADC_IRQ,
		.end   = GPADC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pwrsrc_resources[] = {
	{
		.start = VBUS_FALLING_IRQ,
		.end   = VBUS_FALLING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = VBUS_RISING_IRQ,
		.end   = VBUS_RISING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MV_CHNG_IRQ,
		.end   = MV_CHNG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BC_USB_CHNG_IRQ,
		.end   = BC_USB_CHNG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource charger_resources[] = {
	{
		.start = VBUS_OV_IRQ,
		.end   = VBUS_OV_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CHARGE_DONE_IRQ,
		.end   = CHARGE_DONE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CHARGE_CHARGING_IRQ,
		.end   = CHARGE_CHARGING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BAT_SAFE_QUIT_IRQ,
		.end   = BAT_SAFE_QUIT_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BAT_SAFE_ENTER_IRQ,
		.end   = BAT_SAFE_ENTER_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QCBTU_IRQ,
		.end   = QCBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CBTU_IRQ,
		.end   = CBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QCBTO_IRQ,
		.end   = QCBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CBTO_IRQ,
		.end   = CBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource battery_resources[] = {
	{
		.start = QWBTU_IRQ,
		.end   = QWBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WBTU_IRQ,
		.end   = WBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QWBTO_IRQ,
		.end   = QWBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WBTO_IRQ,
		.end   = WBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WL2_IRQ,
		.end   = WL2_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WL1_IRQ,
		.end   = WL1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell dollar_cove_dev[] = {
	{
		.name = "dollar_cove_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "dollar_cove_gpio",
		.id = 0,
		.num_resources = ARRAY_SIZE(gpio_resources),
		.resources = gpio_resources,
	},
	{
		.name = "dollar_cove_power_button",
		.id = 0,
		.num_resources = ARRAY_SIZE(power_button_resources),
		.resources = power_button_resources,
	},
	{
		.name = "dollar_cove_pwrsrc",
		.id = 0,
		.num_resources = ARRAY_SIZE(pwrsrc_resources),
		.resources = pwrsrc_resources,
	},
	{
		.name = "dollar_cove_charger",
		.id = 0,
		.num_resources = ARRAY_SIZE(charger_resources),
		.resources = charger_resources,
	},
	{
		.name = "dollar_cove_battery",
		.id = 0,
		.num_resources = ARRAY_SIZE(battery_resources),
		.resources = battery_resources,
	},
	{NULL, },
};

#define DOLLAR_COVE_IRQREGMAP(irq) \
	[irq] = { \
		{(0x40 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_INV},\
		{(0x48 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_W1C},\
		{(0x48 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_W1C},\
	}

struct intel_pmic_irqregmap dollar_cove_irqregmap[] = {
	DOLLAR_COVE_IRQREGMAP(VBUS_FALLING_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_RISING_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_OV_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_FALLING_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_RISING_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_OV_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(CHARGE_DONE_IRQ),
	DOLLAR_COVE_IRQREGMAP(CHARGE_CHARGING_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_SAFE_QUIT_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_SAFE_ENTER_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_ABSENT_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_APPEND_IRQ),
	DOLLAR_COVE_IRQREGMAP(QWBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(WBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(QWBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(WBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(QCBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(CBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(QCBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(CBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(WL2_IRQ),
	DOLLAR_COVE_IRQREGMAP(WL1_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPADC_IRQ),
	DOLLAR_COVE_IRQREGMAP(OT_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPIO0_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPIO1_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKO_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKL_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKS_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKN_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKP_IRQ),
	DOLLAR_COVE_IRQREGMAP(EVENT_IRQ),
	DOLLAR_COVE_IRQREGMAP(MV_CHNG_IRQ),
	DOLLAR_COVE_IRQREGMAP(BC_USB_CHNG_IRQ),
};

static int dollar_cove_init(void)
{
	pr_info("Dollar Cove: IC_TYPE 0x%02X\n", intel_mid_pmic_readb(0x03));
	return 0;
}

struct intel_mid_pmic dollar_cove_pmic = {
	.label		= "dollar cove",
	.irq_flags	= IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.init		= dollar_cove_init,
	.cell_dev 	= dollar_cove_dev,
	.irq_regmap	= dollar_cove_irqregmap,
	.irq_num	= 48,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");

