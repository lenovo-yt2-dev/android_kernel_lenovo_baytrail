/* drivers/input/misc/cm36671.c - cm36671 optical sensors driver
 *
 * Copyright (C) 2014 Capella Microsystems Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "cm36671.h"
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>

#if 1
#define PS_DBG(format,...) do{ printk("[SK3810]");\
						printk(format, ## __VA_ARGS__);	\
			}while(0)
#else
#define PS_DBG(format,...) do{ 
			}while(0)
#endif

#define SK3018_GPIO     133 
#define SK3018_NAME   "SK3018"
struct sk3018_info {
	struct class *sk3018_class;
	struct platform_device *pdev;
	struct mutex lock;
	struct device *ha_dev;
	struct input_dev *hall_input_dev;
	
	int sk3018_gpio_pin;
	int enable;
	int data;
	int sk3018_irq;

	struct early_suspend early_suspend;
	struct workqueue_struct *lp_wq;

};



static ssize_t sk3018_enable_show(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sk3018_info *lpi = dev_get_drvdata(dev);
	int enable;
	enable=lpi->enable;
	return sprintf(buf, "%d\n", enable);
}

static ssize_t sk3018_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sk3018_info *lpi = dev_get_drvdata(dev);
	int enable;
	int en = -1;
	
	/*sscanf(buf, "%d", &en);

	if (en != 0 && en != 1)
		return -EINVAL;
	
	if(lpi->enable){
		if(en)
			return 0;
	       else{
	       	enable_irq(lpi->sk3018_irq);
	       	lpi->enable=en;
	       	}
		}
	else{
		if(en){
			disable_irq(lpi->sk3018_irq);
			lpi->enable=en;
			}
		//else
		printk("the hall sensor already disabled!\n");
		}*/
	return count;
}

static ssize_t sk3018_data_show(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int gpio_value;
	gpio_value=gpio_get_value(SK3018_GPIO);
	return sprintf(buf, "%d\n", gpio_value);
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, sk3018_enable_show, sk3018_enable_store);
static DEVICE_ATTR(Hdata, S_IRUGO, sk3018_data_show, NULL);

static struct attribute *hall_attributes[] = {
	&dev_attr_Hdata.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group hall_attribute_group = {
	.attrs = hall_attributes
};

static irqreturn_t sk3018_interrupt(int irq, void *data)
{
    struct sk3018_info *lpi = data;
    int gpio_value;

    gpio_value = gpio_get_value(lpi->sk3018_gpio_pin);

	if(gpio_value){
		 /*hall near*/
	    input_event(lpi->hall_input_dev, EV_KEY, KEY_SHOP, 1);
	    input_sync(lpi->hall_input_dev);
	    input_event(lpi->hall_input_dev, EV_KEY, KEY_SHOP, 0);
	    input_sync(lpi->hall_input_dev);
    }
	else{

	    /*hall far*/
	    input_event(lpi->hall_input_dev, EV_KEY, KEY_SPORT, 1);
	    input_sync(lpi->hall_input_dev);
	    input_event(lpi->hall_input_dev, EV_KEY, KEY_SPORT, 0);
	    input_sync(lpi->hall_input_dev);

    }

	return IRQ_HANDLED;
}

static irqreturn_t sk3018_irq_isr(int irq, void *devid)
{
	struct sk3018_info *lpi = (struct sk3018_info *)devid;

	PS_DBG("[sk3018]IRQ Handled for hall sensor interrupt: %d\n",irq);

	return IRQ_WAKE_THREAD;
}
static int hall_setup(struct sk3018_info *lpi)
{
	int ret;

	lpi->hall_input_dev = input_allocate_device();
	if (!lpi->hall_input_dev) {
		pr_err(
			"[sk3018]%s: could not allocate hall input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->hall_input_dev->name = "sk3018";
	__set_bit(EV_KEY, lpi->hall_input_dev->evbit);
	__set_bit(KEY_SHOP, lpi->hall_input_dev->keybit);
	__set_bit(KEY_SPORT, lpi->hall_input_dev->keybit);

	ret = input_register_device(lpi->hall_input_dev);
	if (ret < 0) {
		pr_err(
			"[sk3018 error]%s: could not register hall input device\n",
			__func__);
		goto err_free_hall_input_device;
	}
	input_set_capability(lpi, EV_KEY, KEY_POWER);

	return ret;

err_free_hall_input_device:
	input_free_device(lpi->hall_input_dev);
	return ret;
}

static int sk3018_setup(struct sk3018_info *lpi)
{
	int ret = 0;
	msleep(5);
	ret = gpio_request(lpi->sk3018_gpio_pin, "gpio_sk3018_pin");
	if (ret < 0) {
		pr_err("[sk3018 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->sk3018_gpio_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->sk3018_gpio_pin);
	if (ret < 0) {
		pr_err(
			"[sk3018 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->sk3018_gpio_pin, ret);
		goto fail_free_gpio_pin;
	}
	//wqf add 
	lpi->sk3018_irq= gpio_to_irq(lpi->sk3018_gpio_pin);
	PS_DBG("[sk3018 ]sk3018_irq= %d the gpio is %d\n", lpi->sk3018_irq,lpi->sk3018_gpio_pin);
	ret = request_threaded_irq(lpi->sk3018_irq, sk3018_irq_isr, sk3018_interrupt, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
				   "sk3018", lpi);

	if (ret) {
		PS_DBG("[sk3018 error]Failed to request IRQ: %d\n", ret);
		}
	
	
	//INIT_DELAYED_WORK(&lpi->delay_work, sk3018_poll);

	return ret;

fail_free_gpio_pin:
	gpio_free(lpi->sk3018_gpio_pin);
	return ret;
}

static void sk3018_early_suspend(struct early_suspend *h)
{
	/*if(lpi->enable) {
		enable=0;
		disable_irq(sk3018_irq);
		}*/
	return ;
}
static void sk3018_late_resume(struct early_suspend *h)
{
	/*if(enable!=1){
		enable=1;
		enable_irq(sk3018_irq);
		}*/
	return ;
}

static int sk3018_probe(struct platform_device *pdev)
{
	int ret=0 ;
	struct sk3018_info *lpi;
	
	lpi = kzalloc(sizeof(struct sk3018_info), GFP_KERNEL);
	
	if (!lpi)
		return -ENOMEM;

	platform_set_drvdata(pdev, lpi);

	lpi->pdev = pdev;
	mutex_init(&lpi->lock);
	
	lpi->sk3018_gpio_pin=SK3018_GPIO;

	ret=hall_setup(lpi);
	if (ret < 0) {
		pr_err("[sk3018 error]%s: hall_setup error!!\n",
			__func__);
		return ret;
	}
	ret=sk3018_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][sk3018 error]%s: sk3018_setup error!!\n",
			__func__);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &hall_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs can not create group\n");
		goto err_sys_init;
	}

	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = sk3018_early_suspend;
	lpi->early_suspend.resume = sk3018_late_resume;
	register_early_suspend(&lpi->early_suspend);

	return ret;

err_sys_init:
	free_irq(lpi->sk3018_irq, lpi);
	gpio_free(lpi->sk3018_gpio_pin);
	return ret;
	
}

static int sk3018_remove(struct platform_device *pdev)
{
	struct sk3018_info *lpi = platform_get_drvdata(pdev);
	int irq = gpio_to_irq(lpi->sk3018_gpio_pin);

	input_unregister_device(lpi->hall_input_dev);
	free_irq(irq, lpi);
	gpio_free(lpi->sk3018_gpio_pin);
	kfree(lpi);
	return 0;
}

static struct platform_device hall_device = {
         .name    = SK3018_NAME,
};

static struct platform_driver hall_driver = {
	.probe		= sk3018_probe,
	.remove		= sk3018_remove,
	.driver	= {
		.name	= SK3018_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init sk3018_init(void)
{
	int ret;
       ret=platform_device_register(&hall_device);
       if(ret < 0)
       	printk(KERN_ERR "Fail to register  sarsensor device\n");
	ret = platform_driver_register(&hall_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register  sarsensor driver\n");

	return ret;
}

static void __exit sk3018_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(sk3018_init);
module_exit(sk3018_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sk3018 Driver");
