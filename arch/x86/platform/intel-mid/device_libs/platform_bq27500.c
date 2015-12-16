/*
 * platform_bq27541.c: bq27541 initilization file
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
#include <asm/intel-mid.h>
#include <linux/power/bq27x00_battery.h>
#include <asm/gpio.h>

#define BQ27541_SLAVE_ADDR 0x55
#define BQ27541_I2C_MASTER 1
/*
 * Translate temperatures to compensate for thermistor circuit problem (EVT2).
 * Precision is limited but should provide accurate values.
 *
 * FIXME: reference code, need change
 */
static int bq27541_translate_temp(int temperature)
{
	if (temperature <= 980)
		/*
		 * This indicates that the thermistor is disconnected on EVT2. Return the same
		 * value as would be measured on EVT3 in case of missing thermistor so that the
		 * gas gauge driver can recognize the disconnected state.
		 */
		temperature = -408;
	else if (temperature <= 986)
		temperature = 0;
	else if (temperature <= 989)
		temperature = 50;
	else if (temperature <= 993)
		temperature = 100;
	else if (temperature <= 998)
		temperature = 150;
	else if (temperature <= 1003)
		temperature = 200;
	else if (temperature <= 1009)
		temperature = 250;
	else if (temperature <= 1016)
		temperature = 300;
	else if (temperature <= 1024)
		temperature = 350;
	else if (temperature <= 1044)
		temperature = 400;
	else if (temperature <= 1055)
		temperature = 450;
	else if (temperature <= 1067)
		temperature = 500;
	else if (temperature <= 1080)
		temperature = 550;
	else if (temperature <= 1095)
		temperature = 600;
	else if (temperature <= 1116)
		temperature = 650;
	else
		temperature = 700;

	return temperature;
}

static struct bq27x00_platform_data bq27541_platform_data = {
	.soc_int_irq = -1,
	.bat_low_irq = -1,
};

static struct i2c_board_info __initdata bq27541_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq27500", BQ27541_SLAVE_ADDR),
		.platform_data = &bq27541_platform_data,
	},
};

static int __init bq27541_platform_init(void)
{
	int soc_int_gpio, soc_int_irq;
	int res;

	//soc_int_gpio = get_gpio_by_name("max_fg_alert");
	soc_int_gpio = 148; // GPIOS_18,102 + 28 + 18
	soc_int_irq = gpio_to_irq(soc_int_gpio);

	printk("%s: gpio = %d,irq = %d\n",__func__,soc_int_gpio,soc_int_irq);
	
	res = irq_set_irq_wake(soc_int_irq, 1);
	if (res) {
		pr_err("%s: Failed to set irq wake for soc_int: %d\n", __func__, res);
		return 0;
	}

	bq27541_platform_data.soc_int_irq = soc_int_irq;
	//bq27541_platform_data.translate_temp = bq27541_translate_temp;
	res = i2c_register_board_info(BQ27541_I2C_MASTER, 
							&bq27541_i2c_boardinfo, ARRAY_SIZE(bq27541_i2c_boardinfo));
	if(res < 0){
		pr_err("%s: fail register bq27541 i2c device\n");
	}
	
	return 0;
}
rootfs_initcall(bq27541_platform_init);
//device_initcall(bq27541_platform_init);
