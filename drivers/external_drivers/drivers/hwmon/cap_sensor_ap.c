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
#include <linux/gpio.h>
#include <linux/i2c.h>

#define REG_DEVID 0x00
#define REG_FWVERSION 0x01
#define REG_KEYSTATUS 0x02
#define REG_CMD 0x03
#define CMD_ENTER_BL 0x55

static struct i2c_board_info __initdata i2c_cy8c= { I2C_BOARD_INFO("capsensor-ap", (0x0a))};

struct cap_switch_data {
    int irq_gpio;
    struct i2c_client *client;
    int irq;
};

struct cap_switch_data *switch_data;

static int cy8c_i2c_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf;
    int ret = 0;
    
    buf = addr;
    ret = i2c_master_send(client, (const char*)&buf, 1);

    if (ret < 0) {
        printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
        return ret;
    }

    ret = i2c_master_recv(client, (char*)&buf, 1);

    if (ret < 0) {
        printk(KERN_ERR "%s():reads data error!!\n", __FUNCTION__);
        return ret;
    }

    *data = buf;
    return 0;
}

static int cy8c_i2c_write_byte(struct i2c_client *client, u8 addr, u8 data)
{
    u8 buf[] = {addr, data};
    int ret = 0;

    ret = i2c_master_send(client, (const char*)buf, sizeof(buf));
    if (ret < 0) {
        printk(KERN_ERR "%s():send command error!!\n", __FUNCTION__);
        return ret;
    }

    return 0;
}

static irqreturn_t cap_irq_handler(int irq, void *dev_id)
{
    printk("%s\n", __func__);

    return IRQ_HANDLED;
}

static ssize_t cap_switch_deviceid_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    u8 value = 0xff;
    int ret;
    struct i2c_client *client = to_i2c_client(dev);

    ret = cy8c_i2c_read_byte(client, REG_DEVID, &value);
    if(!ret) {
        ret = sprintf(buf, "0x%02x\n", value);
        printk("%s, %s\n", __func__, buf);
        return ret;
    }

    return 0;
}

static DEVICE_ATTR(deviceid, S_IRUGO, cap_switch_deviceid_show, NULL);

static ssize_t cap_switch_fwversion_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    u8 value = 0xff;
    int ret;
    struct i2c_client *client = to_i2c_client(dev);

    ret = cy8c_i2c_read_byte(client, REG_FWVERSION, &value);
    if(!ret) {
        ret = sprintf(buf, "0x%02x\n", value);
        printk("%s, %s\n",__func__, buf);
        return ret;
    }

    return 0;
}

static DEVICE_ATTR(fwversion, S_IRUGO, cap_switch_fwversion_show, NULL);

static ssize_t cap_switch_mode_store(struct device *dev, 
                     struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = switch_data->client;
    int ret;
    int i;
    
    printk("%s, count=%d\n", __func__, count);

    for(i=0; i<count; i++)
    {
        printk("%x ", buf[i]);
    }

    if (client){
        ret = cy8c_i2c_write_byte(client, REG_CMD, CMD_ENTER_BL);
        printk("%s, client: 0x%x, send 0x%x to reg[%d]\n", __func__, client, CMD_ENTER_BL, REG_CMD);
    }
    else
       printk("%s client is NULL\n",__func__);

    return count;
    
}

static DEVICE_ATTR(mode, S_IWUSR | S_IWGRP, NULL, cap_switch_mode_store);

static ssize_t cap_switch_keystatus_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int status;

    status = (gpio_get_value(switch_data->irq_gpio))? 0 : 1;

    return sprintf(buf, "0x%02x\n", status);
}

static DEVICE_ATTR(keystatus, S_IRUGO | S_IWUSR, cap_switch_keystatus_show, NULL);

static struct attribute *cap_switch_attr[] = {
    &dev_attr_deviceid.attr,
    &dev_attr_fwversion.attr,
    &dev_attr_mode.attr,
    &dev_attr_keystatus.attr,
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
    switch_data->irq_gpio = 94;

    ret = gpio_request(switch_data->irq_gpio, "capsensor");
    if (ret < 0)
        goto err_request_gpio;

    ret = gpio_direction_input(switch_data->irq_gpio);
    if (ret < 0)
        goto err_set_gpio_input;

    switch_data->irq = gpio_to_irq(switch_data->irq_gpio);
    
    if (switch_data->irq < 0) {
        ret = switch_data->irq;
        goto err_detect_irq_num_failed;
    }

    ret = request_irq(switch_data->irq, cap_irq_handler,
              IRQF_TRIGGER_MASK, "capsensor", switch_data);

    if (ret < 0)
        goto err_request_irq;

#if 1
    ret = i2c_smbus_read_byte_data(client, 0x00);
        printk(KERN_INFO "cap_switch reg 00 %d\n", ret);
    ret = i2c_smbus_read_byte_data(client, 0x01);
        printk(KERN_INFO "cap_switch reg 01 %d\n", ret);
    ret = i2c_smbus_read_byte_data(client, 0x09);
        printk(KERN_INFO "cap_switch reg 09 %d\n", ret);
    ret = i2c_smbus_read_byte_data(client, 0x0D);
        printk(KERN_INFO "cap_switch reg 0D %d\n", ret);
#endif

    i2c_set_clientdata(client, (void*)switch_data);

    ret = sysfs_create_group(&client->dev.kobj, &cap_switch_attr_group);

    if (ret) {
        printk(KERN_ERR "%s():failed to create sysfs device attributes\n", __FUNCTION__);
        goto err_create_group;
    }

    return 0;

err_create_group:
    sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
    gpio_free(switch_data->irq_gpio);
err_request_gpio:
    kfree(switch_data);

    return ret;
}

static int  cap_switch_remove(struct i2c_client *client)
{
    struct cap_switch_data *data = (struct cap_switch_data *)i2c_get_clientdata(client);

    gpio_free(data->irq_gpio);
    kfree(data);
    sysfs_remove_group(&client->dev.kobj, &cap_switch_attr_group);

    return 0;
}

static const struct i2c_device_id capsensor_id[] = {
    { "capsensor-ap", 0 },
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
    .driver        = {
        .name    = "capsensor-ap",
        .of_match_table = capsensor_match_table,
    },
    .probe        = cap_switch_probe,
    .remove        = cap_switch_remove,
    .id_table   = capsensor_id,
};

static int __init cap_ap_switch_init(void)
{
    i2c_register_board_info(5, &i2c_cy8c, 1);

    printk("%s, i2c-bus=5, addr=0x0A\n",__func__);

    return i2c_add_driver(&cap_switch_driver);
}

static void __exit cap_ap_switch_exit(void)
{
    i2c_del_driver(&cap_switch_driver);
}

module_init(cap_ap_switch_init);
module_exit(cap_ap_switch_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("CAP Sensor ap driver");
MODULE_LICENSE("GPL");
