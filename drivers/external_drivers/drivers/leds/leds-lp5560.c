/*
 * =====================================================================================
 *
 *       Filename:  drivers/leds/leds-lp5560.c
 *
 *    Description:  lp5560 led driver
 *
 *        Version:  0.1
 *        Created:  06/07/2013 04:28:25 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  LENOVO 
 *        Company:  LENOVO Inc.
 *
 * =====================================================================================
 */
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include "leds-lp5560.h"

//#define USE_HRTIMER

#ifndef USE_HRTIMER 
DEFINE_SPINLOCK(lp5560_lock);
#endif

/* ---------------------------------------------------------------------------- */
#define LED_DEBUG

#if defined(LED_DEBUG)
#define LED_TAG                  "[LP5560] "
#define LED_FUN(f)               printk(KERN_INFO LED_TAG"%s\n", __FUNCTION__)
#define LED_ERR(fmt, args...)    printk(KERN_ERR  LED_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define LED_LOG(fmt, args...)    printk(KERN_ERR LED_TAG fmt, ##args)
#define LED_DBG(fmt, args...)    printk(KERN_INFO LED_TAG fmt, ##args) 
#else
#define LED_FUN(f)
#define LED_ERR(fmt, args...)    printk(KERN_ERR  LED_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define LED_LOG(fmt, args...)
#define LED_DBG(fmt, args...)
#endif


#define CALI_LENGHT		500
#define MAX_CUST_TIME   32
u16 delay_onms[MAX_CUST_TIME]  = {13,  26,  52,  105, 158, 211, 264, 316, 369, 435,
                                 501, 594, 699, 805, 910, 1016,1122,1227,1353,1478,
                                 1603,1729,1854,1980,2105,2230,2356,2481,2613,2745,
                                 2877,3009};
u16 delay_offms[MAX_CUST_TIME] = {26,  52,  105, 211, 316, 422, 528, 633, 739, 871,
                                 1001,1188,1399,1610,1821,2032,2244,2455,2706,2956,
                                 3207,3458,3709,3960,4210,4461,4712,4963,5227,5491,
                                 5755,6019};

union u_full_train common_train = {
	.train.reset_start[0] = {
		.high = 50, .low = 50,
	},
	.train.reset_start[1] = {
		.high = 50, .low = 50 + 800,
	},
	.train.reset_end[0 ... 2] = {
		.high = 50, .low = 50,
	},
	.train.train_start[0] = {
		.high = 50, .low = 50,
	},
	.train.train_start[1] = {
		.high = 50, .low = 50 + 800,
	},
	.train.calib = {
		.high = CALI_LENGHT, .low = 50,
	},
	.train.cur = {
		.high = CALI_LENGHT * 7,
		.low  = CALI_LENGHT * 7,
	},
	.train.train_seq[0 ... 2] = {
		.rise = CALI_LENGHT * 5,
		.on   = CALI_LENGHT * 8,
		.fall = CALI_LENGHT * 5,
		.off  = CALI_LENGHT * 8,
	},
	.train.train_end[0 ... 2] = {
		.high = 50, .low = 50,
	},
};

#ifdef USE_HRTIMER
static struct hrtimer lp5560_hrtimer;
#endif
static int g_array_i = 0;
u16 g_array_max = 0;
u32 g_array[MAX_COUNT * 2] = {0};
static unsigned long onMS;
static unsigned long offMS;
static unsigned char on_pulse;
static unsigned char off_pulse;
static unsigned char blink;
static int led_blink_flag = 0;

static ssize_t show_delay_on(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", onMS);
}

static ssize_t store_delay_on(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    if (strict_strtoul(buf, 0, &onMS))
        return -EINVAL;

	if (onMS > delay_onms[MAX_CUST_TIME - 1]) {
		onMS = delay_onms[MAX_CUST_TIME - 1];
	}

    return len;
}

static ssize_t show_delay_off(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", offMS);
}

static ssize_t store_delay_off(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    if (strict_strtoul(buf, 0, &offMS))
        return -EINVAL;

	if (offMS > delay_offms[MAX_CUST_TIME - 1]) {
		offMS = delay_offms[MAX_CUST_TIME - 1];
	}

    return len;
}
/*
static ssize_t show_blink(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", blink);
}

static ssize_t store_blink(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    LED_LOG("offMS : %s\n", buf);
    if (strict_strtoul(buf, 0, &blink))
        return -EINVAL;
	onMS = 900;
	offMS = 500;
	lp5560_mode_set(LED_FLASH);
    return len;
}

static LP5560_ATTR_RW(delay_on);
static LP5560_ATTR_RW(delay_off);
static LP5560_ATTR_RW(blink);

static struct attribute *lp5560_led_attributes[] = {
    &dev_attr_delay_on.attr,
    &dev_attr_delay_off.attr,
    &dev_attr_blink.attr,
    NULL,
};

static struct attribute_group lp5560_led_attribute_group = {
    .attrs      = lp5560_led_attributes
};
*/
#ifdef USE_HRTIMER
static enum hrtimer_restart lp5560_callback(struct hrtimer *timer)
{
	if (g_array_i < g_array_max) {
		if (g_array_i % 2 == 0)
			gpio_set_value(112, 1);
		else
			gpio_set_value(112, 0);

		hrtimer_add_expires_ns(&lp5560_hrtimer, 1000 * g_array[g_array_i++]);

		return HRTIMER_RESTART;
	}

	gpio_set_value(112, 1);

	return HRTIMER_NORESTART;
}
#endif

static void lp5560_mode_set(u16 mode)
{
	int i;

	/* standby mode */
	gpio_set_value(112, 0);
	msleep(1);

	switch(mode) {
		case LED_TURN_ON:
			common_train.train.reset_end[2].low = 1000*200;		/* reset done. delay 200ms */

			for (i = 0; i < 3; i++) {
				common_train.train.train_seq[i].rise = 0;
				common_train.train.train_seq[i].on   = 0;
				common_train.train.train_seq[i].fall = 0;
				common_train.train.train_seq[i].off  = 0;
			}
			
			common_train.train.train_seq[0].rise = CALI_LENGHT * 8;
			common_train.train.train_seq[0].on   = CALI_LENGHT * 8;
			common_train.train.train_end[2].low = 5000;			/* command done. delay 5ms */
			break;
		case LED_TURN_OFF:
			gpio_set_value(112, 0);
			break;
		case LED_FLASH:
			common_train.train.reset_end[2].low = 1000*200;

			for (i = 0; i < 3; i++) {
				common_train.train.train_seq[i].rise = CALI_LENGHT * 5;
				common_train.train.train_seq[i].on   = (CALI_LENGHT * on_pulse + CALI_LENGHT)>>1;
				common_train.train.train_seq[i].fall = CALI_LENGHT * 5;
				common_train.train.train_seq[i].off  = (CALI_LENGHT * off_pulse + CALI_LENGHT)>>1;
			}
			common_train.train.train_end[2].low = 5000;			/* command down. delay 5ms */
			break;
		case LED_RESET:
			break;
		case LED_SET_CURRENT:
			break;
		default:
			break;
	}

	if (mode != LED_TURN_OFF)
	{
		g_array_max = 0;
		for (i = 0; i < MAX_COUNT * 2; i++) {
			if (common_train.train_array[i] != 0)
				g_array[g_array_max++] = common_train.train_array[i];
		}

#ifdef USE_HRTIMER
		g_array_i = 0;
		hrtimer_cancel(&lp5560_hrtimer);
		hrtimer_start(&lp5560_hrtimer, ktime_set(0, 0), HRTIMER_MODE_REL);
#else
		spin_lock(&lp5560_lock);
		for (i = 0; i < g_array_max; i++) {
			if (i % 2 == 0)
				gpio_set_value(112,1);
			else
				gpio_set_value(112, 0);

			udelay(g_array[i]);
		}
		spin_unlock(&lp5560_lock);

		if (mode == LED_RESET)
			gpio_set_value(112, 0);
		else
			gpio_set_value(112, 1);
#endif
	}
}

static ssize_t show_blink(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%ld\n", blink);
}

static ssize_t store_blink(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t len)
{
    if (strict_strtoul(buf, 0, &blink))
        return -EINVAL;
    printk("set led blink = %d.\n", blink);
	if(blink != 0){
		led_blink_flag = 1;
		lp5560_mode_set(LED_FLASH);
	} else {
		led_blink_flag = 0;
		lp5560_mode_set(LED_TURN_OFF);
	}


    return len;
}

static LP5560_ATTR_RW(delay_on);
static LP5560_ATTR_RW(delay_off);
static LP5560_ATTR_RW(blink);

static struct attribute *lp5560_led_attributes[] = {
    &dev_attr_delay_on.attr,
    &dev_attr_delay_off.attr,
    &dev_attr_blink.attr,
    NULL,
};

static struct attribute_group lp5560_led_attribute_group = {
    .attrs      = lp5560_led_attributes
};
static void lp5560_led_white_set(struct led_classdev *led_cdev,
                   enum led_brightness value)
{
	int i;

	if(led_blink_flag == 1){
    		printk("set led brightness = %d.\n", value);
		return;
	}
	
	printk("set led brightness = %d.\n", value);

	common_train.train.cur.high = ((64 / 32) * CALI_LENGHT + CALI_LENGHT)>>1;
	common_train.train.cur.low  = ((64 / 32) * CALI_LENGHT + CALI_LENGHT)>>1;

	switch (value) {
		case 0:
			lp5560_mode_set(LED_TURN_OFF);
			break;
		case 255:
			lp5560_mode_set(LED_TURN_ON);
			break;
		case 128:
			onMS = 1980;
			offMS = 4210;
			on_pulse = 23;
			off_pulse = 29;
			lp5560_mode_set(LED_FLASH);
			break;
		case 64:
			onMS = 1016;
			offMS = 2032;
			on_pulse = 15;
			off_pulse = 15;
			lp5560_mode_set(LED_FLASH);
			break;
		default:
			lp5560_mode_set(LED_TURN_ON);
			break;
			printk("set led error\n");
/*			if (onMS == 0)
				lp5560_mode_set(LED_TURN_OFF);
			else if (offMS == 0)
				lp5560_mode_set(LED_TURN_ON);
			else {
				for (i = 0; i < MAX_CUST_TIME; i++) {
					if (onMS <= delay_onms[i]) {
						on_pulse = i;
						break;
					}
				}

				for (i = 0; i < MAX_CUST_TIME; i++) {
					if (offMS <= delay_offms[i]) {
						off_pulse = i;
						break;
					}
				}

				lp5560_mode_set(LED_FLASH);
			}
			break;
			*/
	}
}

static struct led_classdev lp5560_white_led = { 
    .name			    = "white",
    .default_trigger    = "ide-disk",
    .brightness_set     = lp5560_led_white_set,
};

static int lp5560_led_probe(struct platform_device *pdev)
{
    int ret;

    ret = led_classdev_register(&pdev->dev, &lp5560_white_led);
    if (ret < 0)
        led_classdev_unregister(&lp5560_white_led);

    ret = sysfs_create_group(&lp5560_white_led.dev->kobj,
                        &lp5560_led_attribute_group);
    if (ret) {
        led_classdev_unregister(&lp5560_white_led);
        return -1;
    }

	/* GPIO init */
    gpio_request(112, "led-ctrl");
    gpio_direction_output(112, 0);
    gpio_set_value(112, 0);

#ifdef USE_HRTIMER
	hrtimer_init(&lp5560_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lp5560_hrtimer.function=lp5560_callback;
#endif

    return ret;
}

static int lp5560_led_remove(struct platform_device *pdev)
{
#ifdef USE_HRTIMER
	hrtimer_cancel(&lp5560_hrtimer);
#endif
    sysfs_remove_group(&lp5560_white_led.dev->kobj, &lp5560_led_attribute_group);
    led_classdev_unregister(&lp5560_white_led);

    return 0;
}

static struct platform_driver lp5560_led_driver= {
        .probe      = lp5560_led_probe,
        .remove     = lp5560_led_remove,
        .driver     = {
			.name   = "leds-lp5560",
			.owner  = THIS_MODULE,
        },
};

static struct platform_device led_pdev = { 
	        .name= "leds-lp5560",
		        .id= -1, 
};
MODULE_ALIAS("platform:leds-blade");

static int __init gpio_led_init(void)
{
	        platform_device_register(&led_pdev);
		        return platform_driver_register(&lp5560_led_driver);
}

static void __exit gpio_led_exit(void)
{
	        platform_device_unregister(&led_pdev);
		        platform_driver_unregister(&lp5560_led_driver);
}


module_init(gpio_led_init);
module_exit(gpio_led_exit);
MODULE_AUTHOR("LENOVO");
MODULE_DESCRIPTION("LED driver for LP5560 controllers");
MODULE_LICENSE("GPL");

