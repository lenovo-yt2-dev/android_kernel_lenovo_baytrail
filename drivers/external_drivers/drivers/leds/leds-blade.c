#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ctype.h>
//#include <mach/pmic.h>
//#include <asm/gpio.h>
//#include "leds.h"
#include <linux/gpio.h>
#include <linux/delay.h>

#define MAX_BLINK_TIMES 1000
#define LED_NUMS 4
//int led_blink = 0;
struct wake_lock led_wake_lock;
struct blade_led_data {
	char name[10];
	struct led_classdev cdev;
	unsigned led_num;
	unsigned mix_led_num;
	int led_blink;
	bool is_blinking;
//	u8 mix_led_type;
//	u8 led_type;
	u8 led_brightness;
	struct delayed_work led_work;
};
struct blade_led_data led_array[LED_NUMS]=
{
	
	{
		.name = "white",
		.led_num = 112,
//		.led_type = GPIO_LED,

	}
/*
	{
		//green led	
		.name = "green",
		.led_num = 12,
		.led_type = GPIO_LED,
	},
	{
		//blue led	
		.name = "blue",
		.led_num = PM_MPP_8,
		.led_type = PMIC_LED,
	},
	{
		//red led	
		.name = "red",
		.led_num = PM_MPP_7,
		.led_type = PMIC_LED,
	},
	{
		//green and blue bind led	
		.name = "light_blue",
		.led_num = PM_MPP_8,
		.mix_led_num = 12,
		.mix_led_type = GPIO_LED,
		.led_type = PMIC_LED,
	},
*/
};
static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_on);
}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	struct blade_led_data *led_dat =
		container_of(led_cdev, struct blade_led_data,cdev);
	if(led_dat->is_blinking == false)
		return ret;
	if (isspace(*after))
		count++;

	if (count == size) {
		led_blink_set(led_cdev, &state, &led_cdev->blink_delay_off);
		led_cdev->blink_delay_on = state;
		ret = count;
	}

	return ret;
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_off);
}

static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	struct blade_led_data *led_dat =
		container_of(led_cdev, struct blade_led_data,cdev);
	if(led_dat->is_blinking == false)
		return ret;
	if (isspace(*after))
		count++;

	if (count == size) {
		led_blink_set(led_cdev, &led_cdev->blink_delay_on, &state);
		led_cdev->blink_delay_off = state;
		ret = count;
	}

	return ret;
}

static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);

static ssize_t led_blink_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	struct blade_led_data *led_dat =
		container_of(led_cdev, struct blade_led_data,cdev);
	return sprintf(buf, "%d\n", led_dat->led_blink);
}

static ssize_t led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blade_led_data *led_dat =
		container_of(led_cdev, struct blade_led_data,cdev);
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		led_dat->led_blink = state;
		ret = count;
	}
	if(led_dat->led_blink == 0&&led_dat->is_blinking==true)
	{
		led_dat->is_blinking = false;
			//timer_trig_deactivate(led_cdev);
//		led_brightness_set(&led_cdev, LED_OFF);
		if(wake_lock_active(&led_wake_lock))
			wake_unlock(&led_wake_lock);
	}else if(led_dat->led_blink!=0)
	{
		led_dat->is_blinking = true;
	//	timer_trig_activate(led_cdev);
		led_blink_set(led_cdev, &led_cdev->blink_delay_on,
				  &led_cdev->blink_delay_off);
		wake_lock(&led_wake_lock);
	}
	return ret;
}
static DEVICE_ATTR(blink, 0644, led_blink_show, led_blink_store);

static void blade_led_set_work(struct work_struct *work)
{
	struct blade_led_data *led_dat =
		container_of(work, struct blade_led_data,led_work.work);
	int level;
	int value = led_dat->led_brightness;

	if (value == LED_OFF)
		level = 0;
	else
		level = 1;
//	printk("%s:led num is %d,type is %d,level is %d\n",__func__,led_dat->led_num,led_dat->led_type,level);
//	if(led_dat->led_type == PMIC_LED)
//	{
//		if(level!=0)	
//		{
//			pmic_secure_mpp_config_i_sink(led_dat->led_num,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_ENA);
//		}
//		else
//		{
//			pmic_secure_mpp_config_i_sink(led_dat->led_num,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
//		}
//	}else if(led_dat->led_type == GPIO_LED)	
		gpio_set_value(led_dat->led_num, level);
//	if(led_dat->mix_led_num != 0)
//	{
//		
//		if(led_dat->mix_led_type == GPIO_LED)	
//			gpio_set_value(led_dat->mix_led_num, level);
//	}
	if(led_dat->led_blink==0&&led_dat->is_blinking==true)
	{		
		led_dat->is_blinking=false;
	//	timer_trig_deactivate(&led_dat->cdev);
//		led_brightness_set(&led_dat->cdev, LED_OFF);
		if(wake_lock_active(&led_wake_lock))
			wake_unlock(&led_wake_lock);
	}
}



static void blade_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{

	struct blade_led_data *led_dat =
		container_of(led_cdev, struct blade_led_data,cdev);
	led_dat->led_brightness = value;
	if(led_dat->led_blink < MAX_BLINK_TIMES&&led_dat->is_blinking==true && value == 0)
		led_dat->led_blink--;
	schedule_delayed_work(&led_dat->led_work,0);
}
static int blade_led_probe(struct platform_device *pdev)
{
	int i,rc;
	struct blade_led_data* led_dat;
//	for(i = 0;i < LED_NUMS;i++)
//	{
		led_dat = &led_array[0];
//		if(led_dat->led_type == GPIO_LED)
//		{
			rc = gpio_request(led_dat->led_num, "breath led");
			if (rc < 0)
				pr_err("%s: gpio_request---green led failed!",__func__);
//			rc = gpio_tlmm_config(GPIO_CFG(led_dat->led_num, 0,
//									GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//			if (rc < 0) 
//			{
//				pr_err("%s: unable to enable green led\n", __func__);
//				gpio_free(led_dat->led_num);
//			}o
			gpio_direction_output(led_dat->led_num, 0);
			gpio_set_value(led_dat->led_num,0);
//		}else if(led_dat->led_type == PMIC_LED)
//		{
//			pmic_secure_mpp_config_i_sink(led_dat->led_num,PM_MPP__I_SINK__LEVEL_5mA,PM_MPP__I_SINK__SWITCH_DIS);
//		}
		INIT_DELAYED_WORK(&(led_dat->led_work),blade_led_set_work);
		led_dat->cdev.brightness = 0;
		led_dat->cdev.name = led_dat->name;	
		led_dat->cdev.brightness_set = blade_led_set;
		led_classdev_register(NULL,&led_dat->cdev);
		rc = device_create_file(led_dat->cdev.dev, &dev_attr_blink);
		if (rc)
			return -1;
		rc = device_create_file(led_dat->cdev.dev, &dev_attr_delay_on);
		if (rc)
			return -1;
		rc = device_create_file(led_dat->cdev.dev, &dev_attr_delay_off);
		if (rc)
			return -1;
//	}
	wake_lock_init(&led_wake_lock,WAKE_LOCK_SUSPEND,"led_wake_lock");
	return 0;
}

static int blade_led_remove(struct platform_device *pdev)
{

	int i;
	struct blade_led_data* led_dat;
	for(i = 0;i < LED_NUMS;i++)
	{
		led_dat = &led_array[i];
		led_dat->cdev.name = led_dat->name;	
		led_dat->cdev.brightness_set = blade_led_set;
		led_classdev_unregister(&led_dat->cdev);
	}
	return 0;
}

static struct platform_driver blade_led_driver = {
	.probe		= blade_led_probe,
	.remove		= blade_led_remove,
	.driver		= {
		.name	= "leds-blade",
		.owner	= THIS_MODULE,
	},
};
static struct platform_device led_pdev = {
	.name= "leds-blade",
	.id= -1,
};
MODULE_ALIAS("platform:leds-blade");

static int __init gpio_led_init(void)
{
	platform_device_register(&led_pdev);
	return platform_driver_register(&blade_led_driver);
}

static void __exit gpio_led_exit(void)
{
	platform_device_unregister(&led_pdev);
	platform_driver_unregister(&blade_led_driver);
}

module_init(gpio_led_init);
module_exit(gpio_led_exit);

MODULE_AUTHOR("Raphael Assenat <raph@8d.com>, Trent Piepho <tpiepho@freescale.com>");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_LICENSE("GPL");
