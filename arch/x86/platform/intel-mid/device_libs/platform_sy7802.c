/*
 * platform_sy7802.c: sy7802 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef DEBUG
#define DEBUG 1
#endif

#include <linux/init.h>
#include <linux/types.h>
#include <asm/intel-mid.h>

#include <media/sy7802.h>
#include "platform_sy7802.h"
#include "platform_camera.h"

/* workround - pin defined for byt */
/* GP_CAMERASB09 is NO.15, the number is 102+15 = 117
 * So, for GP_CAMERASB09 = 117 + 9 = 126
 */
#define FLASH_RESET			118	/* MCSI_GPIO_01, High to reset , EN pin */
#define FLASH_HOLD			119	/* MCSI_GPIO_02, High to enable, TX2 or INT pin */
#define FLASH_TRIGGER		121	/* MCSI_GPIO_04, High to enable, FLEN pin, flash mode or trigger */
#define FLASH_TORCH			122	/* MCSI_GPIO_05, High to enable, TX1 or TORCH pin, torch mode */

void *sy7802_platform_data_func(void *info)
{
	static struct sy7802_platform_data platform_data;

	platform_data.gpio_reset	=	FLASH_RESET;
	platform_data.gpio_strobe	=	FLASH_TRIGGER;
	platform_data.gpio_torch	=	FLASH_TORCH;
	platform_data.gpio_hold		=	FLASH_HOLD;

	if (platform_data.gpio_reset == -1) {
		pr_err("%s: Unable to find GP_FLASH_RESET\n", __func__);
		return NULL;
	}
	if (platform_data.gpio_strobe == -1) {
		pr_err("%s: Unable to find GP_FLASH_STROBE\n", __func__);
		return NULL;
	}
	if (platform_data.gpio_torch == -1) {
		pr_err("%s: Unable to find GP_FLASH_TORCH\n", __func__);
		return NULL;
	}

	if (platform_data.gpio_hold == -1) {
		pr_err("%s: Unable to find GP_FLASH_HOLD\n", __func__);
		return NULL;
	}	

	pr_info("flash ic platform_data : sy7802: reset: %d strobe %d torch %d hold %d\n",
		platform_data.gpio_reset, platform_data.gpio_strobe,
		platform_data.gpio_torch, platform_data.gpio_hold);

	/* Set to TX2 mode, then ENVM/TX2 pin is a power amplifier sync input:
	 * ENVM/TX pin asserted, flash forced into torch;
	 * ENVM/TX pin desserted, flash set back;
	 */
	platform_data.envm_tx2 = 1;
	platform_data.tx2_polarity = 0;

	/* set peak current limit to be 1000mA */
	platform_data.current_limit = 0;

	return &platform_data;
}


