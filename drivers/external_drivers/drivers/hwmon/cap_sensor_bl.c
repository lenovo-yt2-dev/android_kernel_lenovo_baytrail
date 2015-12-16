/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>

static struct i2c_board_info __initdata i2c_cy8c= { I2C_BOARD_INFO("capsensor-bl", (0x0b))};

static struct cap_switch_data {
        struct switch_dev sdev;
        int irq_gpio;
        u32 irq_gpio_flags;
        struct i2c_client *client;
        const char *name_on;
        const char *name_off;
        const char *state_on;
        const char *state_off;
        int irq;
};

static struct cap_switch_data *switch_data;


static ssize_t cap_switch_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	int count = 3;

	ret = i2c_master_recv(client, buf, count);
	
    if(ret == count)
    {
            printk("%s [0x%02x 0x%02x 0x%02x]\n", __func__, buf[0], buf[1], buf[2]);
	}else{
		ret=0;
	}
	return ret;

}
static DEVICE_ATTR(status, S_IRUGO, cap_switch_status_show,
	NULL);

static ssize_t cap_switch_block_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = switch_data->client;
	int ret;
	int i;

	printk("count=%d\n",count);
        
        if(count != 0)
        {
            
	    for(i=0; i<count; i++)
            {
                printk("%x ", buf[i]);
            }

            printk("\n");

	    ret = i2c_master_send(client, buf, count);

            if(ret == count)
            {
                printk("%s, send %d byte data\n", __func__, count);
            }

        }

	return count;
}

static DEVICE_ATTR(block, S_IWUSR | S_IWGRP, NULL, cap_switch_block_store);


static struct attribute *cap_switch_attr[] = {
	&dev_attr_block.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group cap_switch_attr_group = {
	.name = NULL,
	.attrs = cap_switch_attr,
};


static int cap_switch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	printk(KERN_INFO "%s\n", __FUNCTION__);

	switch_data = kzalloc(sizeof(struct cap_switch_data), GFP_KERNEL);

        if (!switch_data)
            return -ENOMEM;

        switch_data->client = client;

#if 0
	ret = i2c_smbus_read_byte_data(client, 0x00);
        printk(KERN_INFO "cap_switch reg 00 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x01);
        printk(KERN_INFO "cap_switch reg 01 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x09);
        printk(KERN_INFO "cap_switch reg 09 %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, 0x0D);
        printk(KERN_INFO "cap_switch reg 0D %d\n", ret);
#endif 	

	ret = sysfs_create_group(&client->dev.kobj, &cap_switch_attr_group);

	i2c_set_clientdata(client, (void*)switch_data);

	if (ret) {
		printk(KERN_ERR "%s():failed to create sysfs device attributes\n", __FUNCTION__);
		goto err_create_group;
	}

	return 0;

err_create_group:
	sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);

	return ret;
}

static int  cap_switch_remove(struct i2c_client *client)
{
	struct cap_switch_data *data = (struct cap_switch_data *)i2c_get_clientdata(client);

	gpio_free(data->irq_gpio);
    switch_dev_unregister(&data->sdev);
	kfree(data);
	sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);

	return 0;
}

static const struct i2c_device_id capsensor_id[] = {
	{ "capsensor-bl", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, capsensor_id);

#ifdef CONFIG_OF
static struct of_device_id capsensor_match_table[] = {
	{ .compatible = "cypress,capsensor",},
	{ },
};
#else
#define capsensor_match_table NULL
#endif

static struct i2c_driver cap_switch_driver = {
	.driver		= {
		.name	= "capsensor-bl",
		.of_match_table = capsensor_match_table,
	},
	.probe		= cap_switch_probe,
	.remove		= cap_switch_remove,
	.id_table   = capsensor_id,
};

static int __init cap_bl_switch_init(void)
{

	i2c_register_board_info(5, &i2c_cy8c, 1);

    printk("%s, i2c-bus=5, addr=0x0B\n",__func__);

	return i2c_add_driver(&cap_switch_driver);
}

static void __exit cap_bl_switch_exit(void)
{
	i2c_del_driver(&cap_switch_driver);
}

module_init(cap_bl_switch_init);
module_exit(cap_bl_switch_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("CAP Sensor bootloader driver");
MODULE_LICENSE("GPL");
