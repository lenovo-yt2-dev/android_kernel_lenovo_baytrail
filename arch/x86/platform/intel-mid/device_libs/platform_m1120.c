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
#include "platform_m1120.h"
#include <linux/m1120.h>

typedef struct m1120_platform_data_t m1120_board_data = {
	.interrupt_gpio = 25,
	.interrupt_irq = 25 + 256,
};

void *s7805_platform_data(void *info)
{
	return &m1120_board_data;

};
