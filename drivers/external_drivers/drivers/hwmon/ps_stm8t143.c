/*
 * STM8T143 proximity sensor driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/lnw_gpio.h>


/*
* gpio pin number of output data
* ctrl input pin is always active after power on
*
* Byt-cr RVP board,94 for Baylake board
* Get from ACPI if BIOS provided
*/
#define	SARSENSOR_OUT_GPIO 	66
#define   SARSENSOR_MOV_OUT      68

#define SARSENSOR_DRIVER_NAME "sarsensor"
#define SARSENSOR_INPUT_NAME 	"sarsensor"
#define CONFIG_SARSENSOR_DEBUG
#ifdef CONFIG_SARSENSOR_DEBUG
static unsigned int debug_level = 4;
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define SENSOR_DBG(level, ...)				\
do {							\
	if (level <= debug_level)			\
		printk(KERN_DEBUG "<sarsensor>[%d]%s"	\
			 "\n",				\
			__LINE__, __func__,		\
			##__VA_ARGS__);			\
} while (0)

#else
#define SENSOR_DBG(level, ...)
#endif

struct sarsensor_data {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct mutex lock;
	int enabled;
	int gpio_data;
	int gpio_irq;
};

static int sarsensor_init(struct sarsensor_data *sarsensor)
{
	return 0;
}

static void sarsensor_get_data(struct sarsensor_data *sarsensor, s32 *data)
{
	*data = gpio_get_value(SARSENSOR_OUT_GPIO);
	SENSOR_DBG(DBG_LEVEL2, "liumiao:data=%d\n", *data);
}

/*since hw is always in work, so enable&disable in sw itself*/

static irqreturn_t sarsensor_irq(int irq, void *dev_id)
{      SENSOR_DBG(DBG_LEVEL3);
	struct sarsensor_data *sarsensor = (struct sarsensor_data*)dev_id;
	int data;

	sarsensor_get_data(sarsensor, &data);
	input_report_abs(sarsensor->input_dev, ABS_X, data);
	input_sync(sarsensor->input_dev);

	return IRQ_HANDLED;
}

static int sarsensor_input_init(struct sarsensor_data *sarsensor)
{
	int ret;
	struct input_dev *input;

	SENSOR_DBG(DBG_LEVEL3);

	input = input_allocate_device();
	if (!input) {
		dev_err(&sarsensor->pdev->dev, "input device allocate failed\n");
		return -ENOMEM;
	}
	input->name = SARSENSOR_INPUT_NAME;
	input->dev.parent = &sarsensor->pdev->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&sarsensor->pdev->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		goto err;
	}

	sarsensor->input_dev = input;
	return 0;
err:
	input_free_device(input);
	return ret;
}

static int sarsensor_get_data_init(struct sarsensor_data *sarsensor)
{
	int ret;
	int irq;
	int init_gpio_value;
       SENSOR_DBG(DBG_LEVEL3);
       sarsensor->gpio_irq=SARSENSOR_OUT_GPIO;
	if(gpio_request(sarsensor->gpio_irq, SARSENSOR_DRIVER_NAME))
		{printk("sarsensor:Failed to request GPIO !\n");
	};
	lnw_gpio_set_pininfo(sarsensor->gpio_irq,0x5,"pullup");
	gpio_direction_input(sarsensor->gpio_irq);

	init_gpio_value=gpio_get_value(sarsensor->gpio_irq);//lm for test
	printk("liumiao_init_gpio=%d\n",init_gpio_value);
	
	
	irq = gpio_to_irq(sarsensor->gpio_irq);
       //printk("liumiao:%d\n",sarsensor->gpio_irq);
       
	ret = request_threaded_irq(irq, NULL, sarsensor_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			SARSENSOR_DRIVER_NAME, sarsensor);
	if (ret < 0) {printk("liumiao request err");
		gpio_free(sarsensor->gpio_irq);
		dev_err(&sarsensor->pdev->dev,
			"Fail to request irq:%d ret=%d\n", irq, ret);
	}
	return ret;
}

static ssize_t sarsensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sarsensor_data *sarsensor = dev_get_drvdata(dev);
	int enabled;

	enabled = sarsensor->enabled;
	return sprintf(buf, "%d\n", sarsensor->enabled);
}


/*Avoid file operation error in HAL layer*/
static ssize_t sarsensor_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int delay = -1;
	return sprintf(buf, "%d\n", delay);
}

static ssize_t sarsensor_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, sarsensor_delay_show,
		sarsensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, sarsensor_enable_show,
		NULL);

static struct attribute *sarsensor_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group sarsensor_attribute_group = {
	.attrs = sarsensor_attributes
};

static int sarsensor_probe(struct platform_device *pdev)
{     printk("%s:start\n",__func__);
	int err;
	struct sarsensor_data *sarsensor;

	SENSOR_DBG(DBG_LEVEL3);

	sarsensor = kzalloc(sizeof(struct sarsensor_data), GFP_KERNEL);
	if (!sarsensor) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, sarsensor);

	sarsensor->pdev = pdev;
	mutex_init(&sarsensor->lock);
	
	sarsensor->gpio_data = SARSENSOR_OUT_GPIO;

	SENSOR_DBG(DBG_LEVEL3, "data gpio:%d\n", sarsensor->gpio_data);

	err = sarsensor_init(sarsensor);
	if (err < 0) {
		dev_err(&pdev->dev, "sarsensor_initchip failed with %d\n", err);
		goto err_init;
	}

	err = sarsensor_input_init(sarsensor);
	if (err < 0) {
		dev_err(&pdev->dev, "input init error\n");
		goto err_init;
	}

	err = sarsensor_get_data_init(sarsensor);
	if (err < 0) {
		dev_err(&pdev->dev, "input init error\n");
		goto err_data_init;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &sarsensor_attribute_group);
	if (err) {
		dev_err(&pdev->dev, "sysfs can not create group\n");
		goto err_sys_init;
	}

	return 0;

err_sys_init:
	free_irq(sarsensor->gpio_irq, sarsensor);
	gpio_free(sarsensor->gpio_irq);
err_data_init:
	input_unregister_device(sarsensor->input_dev);
err_init:
	kfree(sarsensor);
	return err;
}

static int sarsensor_remove(struct platform_device *pdev)
{
	struct sarsensor_data *sarsensor = platform_get_drvdata(pdev);
	int irq = gpio_to_irq(sarsensor->gpio_irq);

	sysfs_remove_group(&pdev->dev.kobj, &sarsensor_attribute_group);
	input_unregister_device(sarsensor->input_dev);
	free_irq(irq, sarsensor);
	gpio_free(sarsensor->gpio_irq);
	kfree(sarsensor);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sarsensor_suspend(struct device *dev)
{
	struct sarsensor_data *sarsensor = dev_get_drvdata(dev);

	
	return 0;
}

static int sarsensor_resume(struct device *dev)
{
	struct sarsensor_data *sarsensor = dev_get_drvdata(dev);

	
	return 0;
}
static SIMPLE_DEV_PM_OPS(sarsensor_pm, sarsensor_suspend, sarsensor_resume);
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id ps_sarsensor_acpi_ids[] = {
	{ "SRCL0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ps_sarsensor_acpi_ids);
#endif
static struct platform_device sarsensor_device = {
         .name    = "sarsensor",
};

static struct platform_driver sarsensor_driver = {
	.probe		= sarsensor_probe,
	.remove		= sarsensor_remove,
	.driver	= {
		.name	= SARSENSOR_DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &sarsensor_pm,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(ps_sarsensor_acpi_ids),
#endif
	},
};

static int __init sar_sensor_init(void)
{
        printk("%s:start\n",__func__);
        
	int ret;
       ret=platform_device_register(&sarsensor_device);
       if(ret < 0)
       	printk(KERN_ERR "Fail to register  sarsensor device\n");
	ret = platform_driver_register(&sarsensor_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register  sarsensor driver\n");

	return ret;
}

static void __exit sar_sensor_exit(void)
{
	platform_driver_unregister(&sarsensor_driver);
}

module_init(sar_sensor_init);
module_exit(sar_sensor_exit);

MODULE_AUTHOR("LIUMIAO");
MODULE_DESCRIPTION("SAR SENSOR");
MODULE_LICENSE("GPL V2");
