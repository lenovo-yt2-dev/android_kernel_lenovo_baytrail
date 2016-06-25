/*
 * platform_ekth3250.c: ekth3250 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_s7508.h"
#include <linux/input/synaptics_dsx.h>

#define TM1940 (1)
#define TM2448 (2)

#define SYNAPTICS_MODULE TM2448

/* Synaptics changes for Panda Board */
static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state);

#if (SYNAPTICS_MODULE == TM2448)
#if 0
#define DSX_I2C_ADDR 0x20
#define DSX_ATTN_GPIO 183  // 183 in yt2
#define DSX_ATTN_MUX_NAME "gpmc_ad15.gpio_39"
#define DSX_RESET_GPIO 175   // 175 in yt2
#endif 
#define SYNAPTICS_I2C_DEVICE
//#define DSX_I2C_ADDR 0x20
#define DSX_I2C_ADDR 0x38

#define DSX_ATTN_GPIO 142
#define DSX_ATTN_MUX_NAME "gpmc_ad15.gpio_39"
#define DSX_POWER_GPIO 211
#define DSX_POWER_MUX_NAME "mcspi1_cs3.gpio_140"
#define DSX_POWER_ON_STATE 1
#define DSX_POWER_DELAY_MS 160
#define DSX_RESET_GPIO 60
#define DSX_RESET_ON_STATE 0
#define DSX_RESET_DELAY_MS 100
#define DSX_RESET_ACTIVE_MS 20
#define DSX_IRQ_FLAGS IRQF_TRIGGER_FALLING
static unsigned char regulator_name[] = "";
static unsigned char cap_button_codes[] = {};

#if 0
static unsigned char tm2448_cap_button_codes[] = {};

static struct synaptics_dsx_cap_button_map tm2448_cap_button_map = {
	.nbuttons = ARRAY_SIZE(tm2448_cap_button_codes),
	.map = tm2448_cap_button_codes,
};
#endif 

static struct synaptics_dsx_cap_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};

//static struct synaptics_dsx_board_data dsx_board_data = {
static struct synaptics_dsx_platform_data dsx_board_data = {
	.irq_gpio = DSX_ATTN_GPIO,
	//.irq_flags = DSX_IRQ_FLAGS,
	.irq_type = IRQF_ONESHOT | IRQ_TYPE_LEVEL_LOW,    
	.power_gpio = DSX_POWER_GPIO,
	.power_on_state = DSX_POWER_ON_STATE,
	.power_delay_ms = DSX_POWER_DELAY_MS,
	.reset_gpio = DSX_RESET_GPIO,
	.reset_on_state = DSX_RESET_ON_STATE,
	.reset_delay_ms = DSX_RESET_DELAY_MS,
	.reset_active_ms = DSX_RESET_ACTIVE_MS,
 	.gpio_config = synaptics_gpio_setup,
 	.regulator_name = regulator_name,
 	.cap_button_map = &cap_button_map,
	.x_flip = false,
	.y_flip = true,
	.swap_axes = true,
#ifdef SYNAPTICS_SPI_DEVICE
	.byte_delay_us = DSX_SPI_BYTE_DELAY_US,
	.block_delay_us = DSX_SPI_BLOCK_DELAY_US,
#endif
};


void *s7805_platform_data(void *info)
{
#if 0
        static struct synaptics_dsx_platform_data dsx_platformdata = {
	    //.irq_flags = IRQF_TRIGGER_FALLING | IRQCHIP_ONESHOT_SAFE,
	    .irq_type = IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING,    
	    .irq_gpio = DSX_ATTN_GPIO,
	    .reset_delay_ms = 100,
	    .reset_gpio = DSX_RESET_GPIO,
 	    .gpio_config = synaptics_gpio_setup,
 	    .cap_button_map = &tm2448_cap_button_map,
	};
	return &dsx_platformdata;
#endif 

	return &dsx_board_data;

}

#elif (SYNAPTICS_MODULE == TM1940)    // #if (SYNAPTICS_MODULE == TM2448)
#define DSX_I2C_ADDR 0x20
#define DSX_ATTN_GPIO 39
#define DSX_ATTN_MUX_NAME "gpmc_ad15.gpio_39"
#define DSX_RESET_GPIO -1

static unsigned char tm1940_cap_button_codes[] = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH};

static struct synaptics_dsx_cap_button_map tm1940_cap_button_map = {
	.nbuttons = ARRAY_SIZE(tm1940_cap_button_codes),
	.map = tm1940_cap_button_codes,
};

void *s7805_platform_data(void *info)
{
        static struct synaptics_dsx_platform_data dsx_platformdata = {
	    .irq_flags = IRQF_TRIGGER_FALLING,
	    .irq_gpio = DSX_ATTN_GPIO,
	    .reset_delay_ms = 100,
	    .reset_gpio = DSX_RESET_GPIO,
 	    .gpio_config = synaptics_gpio_setup,
 	    .cap_button_map = &tm1940_cap_button_map,
	}
	return &dsx_platformdata;
}


static struct i2c_board_info bus4_i2c_devices[] = {
	{
		I2C_BOARD_INFO("synaptics_dsx_i2c", DSX_I2C_ADDR),
		.platform_data = &dsx_platformdata,
	},
};
#endif

#if 0
static int synaptics_gpio_setup(int gpio, bool configure)
{
	int retval = 0;
	unsigned char buf[16];

	printk("%s, gpio=%d\n", __func__, gpio);

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}
#endif 

/* End of Synaptics changes for Merrifield VV Board */

static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	printk(KERN_ERR"lxh****:%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, configure, dir, state);

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		
	        if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}
			
		gpio_export(gpio,1);
		
		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}


