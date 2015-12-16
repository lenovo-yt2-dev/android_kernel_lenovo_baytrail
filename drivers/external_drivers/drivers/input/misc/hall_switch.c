#include <linux/module.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/leds.h>

#include <linux/platform_device.h>
//#include <linux/sysdev.h>

#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kernel.h>

#include "hall_switch.h"


#define HALL_SWITCH

static volatile int key_debug = 5;

#ifdef HALL_SWITCH
#define HALL_GPIO 156
#endif


static struct hall_switch_data *this_data;
static void hall_irq_work(struct work_struct *work)
{
    int gpio_value;


    gpio_value = gpio_get_value(HALL_GPIO);

    if(gpio_value){
        /*----hall far----*/
        if(key_debug == 5)
		printk("hall-switch report:____________________________far!!!\n");
	input_event(this_data->input_dev, EV_KEY, KEY_HALL_FAR, 1);
        input_sync(this_data->input_dev);
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_FAR, 0);
        input_sync(this_data->input_dev);
    }else{
        /*----hall near----*/
        if(key_debug == 5)
            printk("hall-switch report:_____________________________near!!!\n");
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_NEAR, 1);
        input_sync(this_data->input_dev);
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_NEAR, 0);
        input_sync(this_data->input_dev);
    }

}
static irqreturn_t misc_hall_irq(int irq, void *data)
{
	struct hall_switch_data *hall_data = data;
	//this_data = data;
	if(!work_pending(&hall_data->hall_work)){
		queue_work(hall_data->hall_workqueue, &hall_data->hall_work);
            printk("hall-switch:_____________________________irq!!!\n");
	}
	return IRQ_HANDLED;
}
/*
//gpio value: /sys/devices/system/hall-switch/hall_int_gpio
static ssize_t hall_int_gpio_show(struct sysdev_class *class, struct sysdev_class_attribute * attr, char *buf)
{
	int tmp = gpio_get_value(HALL_GPIO);

        return sprintf(buf, "%s\n", tmp==0?"0":"1");
}

static SYSDEV_CLASS_ATTR(hall_int_gpio, 0444, hall_int_gpio_show, NULL);

static struct sysdev_class_attribute *mhall_int_attributes[] = {
        &attr_hall_int_gpio,
        NULL
};

static struct sysdev_class module_hall_class = {
        .name = "hall-switch",
};

*/
static int hall_probe(struct platform_device *pdev)
{
        int retval = 0;
        int err = 0;
        struct hall_switch_data *hall_data;
        hall_data = kzalloc(sizeof(struct hall_switch_data), GFP_KERNEL);
            printk("hall-switch: ____________________________probe!!! \n");
        if (!hall_data){
            err = -ENOMEM;
            goto exit;
        }

        /*----Register to Input Device----*/
        hall_data->input_dev = input_allocate_device();
        if (hall_data->input_dev == NULL){
            err = -ENOMEM;
            printk("hall-switch: ____________________________Failed to allocate input device!!! \n");
            goto exit_kfree;
        }

        hall_data->input_dev->name = "hall-switch";

        set_bit(EV_SYN, hall_data->input_dev->evbit);
        set_bit(EV_KEY, hall_data->input_dev->evbit);
        set_bit(EV_ABS, hall_data->input_dev->evbit);

        set_bit(KEY_HALL_NEAR, hall_data->input_dev->keybit);
        input_set_capability(hall_data->input_dev, EV_KEY, KEY_HALL_NEAR);
        set_bit(KEY_HALL_FAR, hall_data->input_dev->keybit);
        input_set_capability(hall_data->input_dev, EV_KEY, KEY_HALL_FAR);

        retval = input_register_device(hall_data->input_dev);
        if(retval){
            printk("hall-switch:____________________________Failed to register input device!!!\n");
            goto exit_register_input;
        }

        /*----hall irq request----*/
	retval = gpio_request(HALL_GPIO, "hall_switch");
        if(retval){
            printk("hall-switch:____________________________Failed to request irq gpio!!!\n");
            goto exit;
        }

	retval = gpio_direction_input(HALL_GPIO);
        if(retval){
            printk("hall-switch:____________________________Failed to set gpio direction!!!\n");
            goto exit_request_irq;
        }
        hall_data->hall_irq = gpio_to_irq(HALL_GPIO);

	INIT_WORK(&hall_data->hall_work, hall_irq_work);
	hall_data->hall_workqueue = create_singlethread_workqueue("msm_hall_switch");
	retval = request_irq(hall_data->hall_irq, misc_hall_irq, IRQF_TRIGGER_MASK, "misc_hall_irq", hall_data);
//        retval = request_threaded_irq(hall_data->hall_irq, NULL, misc_hall_irq, IRQF_TRIGGER_MASK, "misc_hall_irq", hall_data);
        if(retval < 0){
            printk("hall-switch:____________________________Failed to create hall irq thread!!!\n");
            goto exit_enable_irq;
        }

        enable_irq_wake(hall_data->hall_irq);
        this_data = hall_data;
        return retval;
exit_request_irq:
	gpio_free(HALL_GPIO);
exit_enable_irq:
     input_unregister_device(hall_data->input_dev);

exit_register_input:
     input_free_device(hall_data->input_dev);
     hall_data->input_dev = NULL;

exit_kfree:
     kfree(hall_data);
exit:
     return err;
}

static int hall_switch_suspend(struct device *dev)
{
        enable_irq_wake(HALL_GPIO);
	return 0;
}

static int hall_switch_resume(struct device *dev)
{
    int gpio_value;


    gpio_value = gpio_get_value(HALL_GPIO);

    if(gpio_value){
        /*----hall far----*/
        if(key_debug == 5)
		printk("hall-switch report:____________________________far!!!\n");
	input_event(this_data->input_dev, EV_KEY, KEY_HALL_FAR, 1);
        input_sync(this_data->input_dev);
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_FAR, 0);
        input_sync(this_data->input_dev);
    }else{
        /*----hall near----*/
        if(key_debug == 5)
            printk("hall-switch report:_____________________________near!!!\n");
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_NEAR, 1);
        input_sync(this_data->input_dev);
        input_event(this_data->input_dev, EV_KEY, KEY_HALL_NEAR, 0);
        input_sync(this_data->input_dev);
    }
	
    return 0;

}

static const struct dev_pm_ops hall_switch_dev_pm_ops = {
	        .suspend = hall_switch_suspend,
		.resume  = hall_switch_resume,
};


static struct platform_driver msm_hall_driver = {
	.probe = hall_probe,
	.driver = {
		.name = "msm_hall_switch",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &hall_switch_dev_pm_ops,
#endif
	},
};

static struct platform_device msm_hall_switch = {
	       .name      = "msm_hall_switch",
	};


static struct platform_device *hall_devices_evb[] __initdata = {
	        &msm_hall_switch,
	};


static int __init hall_init(void)
{
 /*       struct sysdev_class_attribute **attr;
        int res;

        res = sysdev_class_register(&module_hall_class);
        if (unlikely(res)) {
                return res;
        }

        for (attr = mhall_int_attributes; *attr; attr++) {
                res = sysdev_class_create_file(&module_hall_class, *attr);
                if (res)
                        goto out_unreg;
        }
*/
	
  	platform_add_devices(hall_devices_evb, ARRAY_SIZE(hall_devices_evb));
	return platform_driver_register(&msm_hall_driver);
/*
out_unreg:
        for (; attr >= mhall_int_attributes; attr--)
                sysdev_class_remove_file(&module_hall_class, *attr);
        sysdev_class_unregister(&module_hall_class);

        return res;
*/

}

fs_initcall(hall_init);
MODULE_DESCRIPTION("Hall switch sensor driver");
MODULE_LICENSE("GPL");



