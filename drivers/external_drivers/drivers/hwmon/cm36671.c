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
#define PSENSOR_SYSFS_INTERFACE  //wqf add for proximity sensor macro
#define D(x...) pr_info(x)
#if 1
#define PS_DBG(format,...) do{ printk("[CM36671]");\
						printk(format, ## __VA_ARGS__);	\
			}while(0)
#else
#define PS_DBG(format,...) do{ 
			}while(0)
#endif
#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_PS                    0x01

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

struct cm36671_info {
	struct class *cm36671_class;
	struct device *ps_dev;

	struct input_dev *ps_input_dev;

	struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int ps_enable;
	int ps_irq_flag;

	int irq;
	
	int (*power)(int, uint8_t); /* power to the chip */

	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int psensor_enabled;//add by wqf
	uint8_t slave_addr;

	uint8_t ps_close_thd_set;
	uint8_t ps_away_thd_set;	
	uint8_t inte_cancel_set;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint8_t record_clear_int_fail;
};
struct cm36671_info *lp_info;
static uint8_t ps_cancel_set;
static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex CM36671_control_mutex;
static int initial_cm36671(struct cm36671_info *lpi);
static void psensor_initial_cmd(struct cm36671_info *lpi);

static int control_and_report(struct cm36671_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36671_info *lpi = lp_info;
		
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &cmd,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },		 
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36671 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM36671 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36671_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36671 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM36671 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm36671_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM36671 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM36671] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm36671_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM36671] %s: _cm36671_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36671 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct cm36671_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	

	ret = _cm36671_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);
	
	(*data) &= 0xFF;
	
	if (ret < 0) {
		pr_err(
			"[PS][CM36671 error]%s: _cm36671_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	} else {
		pr_err(
			"[PS][CM36671 OK]%s: _cm36671_I2C_Read_Word OK 0x%x\n",
			__func__, *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct cm36671_info *lpi = lp_info;

	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0) {
			msleep(10);
			wait_count++;
			if (wait_count > 12) {
				pr_err("[PS_ERR][CM36671 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][CM36671 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	*ps_adc = (mid_val & 0xFF);

	return 0;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm36671_info *lpi = lp_info;
	uint16_t intFlag;
  _cm36671_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag);  
	  
	enable_irq(lpi->irq);
}

static irqreturn_t cm36671_irq_handler(int irq, void *data)
{
	struct cm36671_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static void psensor_initial_cmd(struct cm36671_info *lpi)
{
	/*must disable p-sensor interrupt befrore IST create*//*disable PS func*/		
  lpi->ps_conf1_val |= CM36671_PS_SD;
  lpi->ps_conf1_val &= CM36671_PS_INT_MASK;  
  _cm36671_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);   
  _cm36671_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
  _cm36671_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);

	D("[PS][CM36671] %s, finish\n", __func__);	
}

static int psensor_enable(struct cm36671_info *lpi)
{
	int ret = -EIO;
	PS_DBG("%s:configure upspace call\n",__func__);
	mutex_lock(&ps_enable_mutex);
	
	if ( lpi->ps_enable ) {
		D("[PS][CM36671] %s: already enabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS, 1);
	
	mutex_unlock(&ps_enable_mutex);
	return ret;
}

static int psensor_disable(struct cm36671_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_disable_mutex);
	D("[PS][CM36671] %s\n", __func__);

	if ( lpi->ps_enable == 0 ) {
		D("[PS][CM36671] %s: already disabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS,0);
	
	mutex_unlock(&ps_disable_mutex);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct cm36671_info *lpi = lp_info;

	D("[PS][CM36671] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct cm36671_info *lpi = lp_info;

	D("[PS][CM36671] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct cm36671_info *lpi = lp_info;
	PS_DBG("%s:called by hal",__func__);
	D("[PS][CM36671] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case PROXIMITYSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		PS_DBG("the value get from hal layer is %d",val);
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[PS][CM36671 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &psensor_fops
};

static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct cm36671_info *lpi = lp_info;
	int intr_val;

	intr_val = gpio_get_value(lpi->intr_pin);

	get_ps_adc_value(&value);

	ret = sprintf(buf, "ADC[0x%04X], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct cm36671_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	if (ps_en) {
		D("[PS][CM36671] %s: ps_en=%d\n",
			__func__, ps_en);
		psensor_enable(lpi);
	} else
		psensor_disable(lpi);

	D("[PS][CM36671] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

unsigned PS_cmd_test_value;
static ssize_t ps_parameters_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36671_info *lpi = lp_info;

	ret = sprintf(buf, "PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct cm36671_info *lpi = lp_info;
	char *token[10];
	int i;

	printk(KERN_INFO "[PS][CM36671] %s\n", buf);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	lpi->ps_close_thd_set = simple_strtoul(token[0], NULL, 16);
	lpi->ps_away_thd_set = simple_strtoul(token[1], NULL, 16);	
	PS_cmd_test_value = simple_strtoul(token[2], NULL, 16);
	printk(KERN_INFO
		"[PS][CM36671]Set PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	D("[PS][CM36671] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_parameters, 0664,
	ps_parameters_show, ps_parameters_store);


static ssize_t ps_conf_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cm36671_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%04x, PS_CONF3 = 0x%04x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct cm36671_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	D("[PS]%s: store value PS conf1 reg = 0x%04x PS conf3 reg = 0x%04x\n", __func__, code1, code2);

  lpi->ps_conf1_val = code1;
  lpi->ps_conf3_val = code2;

	_cm36671_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val );  
	_cm36671_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val );

	return count;
}
static DEVICE_ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);

static ssize_t ps_thd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36671_info *lpi = lp_info;
  ret = sprintf(buf, "[PS][CM36671]PS Hi/Low THD ps_close_thd_set = 0x%02x, ps_away_thd_set = 0x%02x\n", lpi->ps_close_thd_set, lpi->ps_away_thd_set);
  return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	struct cm36671_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	lpi->ps_away_thd_set = code &0xFF;
	lpi->ps_close_thd_set = (code &0xFF00)>>8;	
	
    _cm36671_I2C_Write_Word(lpi->slave_addr, PS_THD, code );  
	
	D("[PS][CM36671]%s: ps_close_thd_set = 0x%02x, ps_away_thd_set = 0x%02x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);
    
	return count;
}
static DEVICE_ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);

static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36671_info *lpi = lp_info;

	ret = sprintf(buf, "[PS][CM36671]PS_CANC = 0x%02x\n", lpi->inte_cancel_set);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	struct cm36671_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS][CM36671]PS_CANC: store value = 0x%02x\n", code);
	lpi->inte_cancel_set = (code &0x00FF);	
	_cm36671_I2C_Write_Word(lpi->slave_addr, PS_CANC, lpi->inte_cancel_set );
	
	return count;
}
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static ssize_t ps_hw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36671_info *lpi = lp_info;

	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
		lpi->ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return ret;
}
static ssize_t ps_hw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
//	struct cm36671_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS]%s: store value = 0x%x\n", __func__, code);

	return count;
}
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static int psensor_setup(struct cm36671_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][CM36671 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "cm36671-ps";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][CM36671 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][CM36671 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}


static int initial_cm36671(struct cm36671_info *lpi)
{
	int val, ret;
	uint16_t idReg;

	val = gpio_get_value(lpi->intr_pin);
	D("[PS][CM36671] %s, INTERRUPT GPIO val = %d\n", __func__, val);

	ret = _cm36671_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
	idReg &= 0xFF;
	D("[PS][CM36671] Check Device ID = 0x%04x\n", idReg);
	if ((ret < 0) || ((idReg != 0x0071) && (idReg != 0x0083))) {   //0x0083 for ES chip, 0x71 for MP chip
  		if (record_init_fail == 0)
  			record_init_fail = 1;
		PS_DBG(" %s:%d the cm36671 ID check error\n",__func__,__LINE__);
  		return -ENOMEM;/*If devices without cm36671 chip and did not probe driver*/	
  }
  
	return 0;
}

static int cm36671_setup(struct cm36671_info *lpi)
{
	int ret = 0;
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_cm36671_intr");
	if (ret < 0) {
		pr_err("[PS][CM36671 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][CM36671 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
	//wqf add 
	lpi->irq = gpio_to_irq(lpi->intr_pin);

	ret = initial_cm36671(lpi);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM36671 error]%s: fail to initial cm36671 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}
	
	/*Default disable P sensor*/
  	psensor_initial_cmd(lpi);

	ret = request_any_context_irq(lpi->irq,
			cm36671_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm36671",
			lpi);
	if (ret < 0) {
		pr_err(
			"[PS][CM36671 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static void cm36671_early_suspend(struct early_suspend *h)
{
	struct cm36671_info *lpi = lp_info;

	D("[PS][CM36671] %s\n", __func__);
	//if(lpi->psensor_enabled){   /*add by wqf*/
	//if (lpi->ps_enable)
		//psensor_disable(lpi);
	//}
}

static void cm36671_late_resume(struct early_suspend *h)
{
	struct cm36671_info *lpi = lp_info;

	D("[PS][CM36671] %s\n", __func__);
	//if(lpi->psensor_enabled){   /*add by wqf*/
		//if (!lpi->ps_enable)     
		//psensor_enable(lpi);
	//}
}
//wqf add for proximity sensor sysfs interface called by hal
#ifdef PSENSOR_SYSFS_INTERFACE
static ssize_t cm36671_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36671_info *cm36671_pi = lp_info;
	int enabled;
	enabled = cm36671_pi->ps_enable;
	return sprintf(buf, "%d\n", cm36671_pi->ps_enable);
}

static ssize_t cm36671_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm36671_info *cm36671_pi = lp_info;
	unsigned long val;
	//PS_DBG("%s:called by hal-%s\n",__func__,*buf);
	PS_DBG("%s:called by hal-%s\n",__func__,buf);
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	PS_DBG("%s:called by hal the value:%ld\n",__func__,val);
	if (val)
	{
		PS_DBG("[PS][CM36671] %s: val=%ld\n",
			__func__, val);
		psensor_enable(cm36671_pi);
		cm36671_pi->psensor_enabled=1;//add by wqf

	}
	else{
		psensor_disable(cm36671_pi);
		cm36671_pi->psensor_enabled=0;//add by wqf
	}
	
	PS_DBG("[PS][CM36671] %s psensor_enabled=%d\n", __func__,cm36671_pi->psensor_enabled);//add by wqf
	PS_DBG("[PS][CM36671] %s\n", __func__);
	return count;
}
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, cm36671_enable_show,
		cm36671_enable_store);

static struct attribute *cm36671_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group cm36671_attribute_group = {
	.attrs = cm36671_attributes
};
#endif
//wqf add end
static int cm36671_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret=0 ;
	struct cm36671_info *lpi;
	struct cm36671_platform_data *pdata;

	D("[PS][CM36671] %s\n", __func__);
	PS_DBG("enter %s\n",__func__);

	lpi = kzalloc(sizeof(struct cm36671_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("[CM36671] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[PS][CM36671 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	PS_DBG("irq = 0x%2x, addr = 0x%x,ps_close_thd_set=0x%x\n",                
                pdata->intr,
                pdata->slave_addr,
                pdata->ps_close_thd_set);

	//lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	
	lpi->intr_pin = pdata->intr;
	lpi->power = pdata->power;
	
	lpi->slave_addr = pdata->slave_addr;
	
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	PS_DBG("ps_conf1_val=0x%x\n",lpi->ps_conf1_val);
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	ps_cancel_set = lpi->inte_cancel_set;
	lpi->psensor_enabled=0;//add by wqf	
	lpi->record_clear_int_fail=0;

	
	lp_info = lpi;

	mutex_init(&CM36671_control_mutex);
	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);
	mutex_init(&ps_get_adc_mutex);
	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][CM36671 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

  	lpi->lp_wq = create_singlethread_workqueue("cm36671_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][CM36671 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	ret = cm36671_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36671 error]%s: cm36671_setup error!\n", __func__);
		goto err_cm36671_setup;
	}
	lpi->cm36671_class = class_create(THIS_MODULE, "capella_sensors");
	if (IS_ERR(lpi->cm36671_class)) {
		ret = PTR_ERR(lpi->cm36671_class);
		lpi->cm36671_class = NULL;
		goto err_create_class;
	}

	lpi->ps_dev = device_create(lpi->cm36671_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ps_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev,
		&dev_attr_ps_parameters);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);
	if (ret)
		goto err_create_ps_device;
		
	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_canc);
	if (ret)
		goto err_create_ps_device;
		
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);
	if (ret)
		goto err_create_ps_device;
	// wqf add start
	#ifdef PSENSOR_SYSFS_INTERFACE
	ret = sysfs_create_group(&client->dev.kobj, &cm36671_attribute_group);
	if (ret) {
		dev_err(&client->dev, "sysfs can not create group\n");
		goto err_sys_init;
	}
	#endif
	//wqf add end
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm36671_early_suspend;
	lpi->early_suspend.resume = cm36671_late_resume;
	register_early_suspend(&lpi->early_suspend);
	D("[PS][CM36671] %s: Probe success!\n", __func__);

	return ret;
#ifdef PSENSOR_SYSFS_INTERFACE
err_sys_init:	
#endif
err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_class:
	class_destroy(lpi->cm36671_class);
err_cm36671_setup:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ps_wake_lock));
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
	misc_deregister(&psensor_misc);
err_psensor_setup:
	mutex_destroy(&CM36671_control_mutex);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	mutex_destroy(&ps_get_adc_mutex);
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm36671_info *lpi, uint8_t mode, uint16_t param ) {
	int ret=0;
	uint16_t ps_data = 0;
	int val;
	
  mutex_lock(&CM36671_control_mutex);
  
 if( mode == CONTROL_PS ){
    if(param){ 
      lpi->ps_conf1_val &= CM36671_PS_SD_MASK;
      lpi->ps_conf1_val |= CM36671_PS_INT_IN_AND_OUT;      
    } else {
      lpi->ps_conf1_val |= CM36671_PS_SD;
      lpi->ps_conf1_val &= CM36671_PS_INT_MASK;
    }
    _cm36671_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    
    lpi->ps_enable=param;  
  }
#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY
   if(lpi->ps_enable){
    int ps_status = 0;
    if( mode == CONTROL_PS )
      ps_status = PS_CLOSE_AND_AWAY;   
    else if(mode == CONTROL_INT_ISR_REPORT ){  
      if ( param & INT_FLAG_PS_IF_CLOSE )
        ps_status |= PS_CLOSE;      
      if ( param & INT_FLAG_PS_IF_AWAY )
        ps_status |= PS_AWAY;
    }
      
    if (ps_status!=0){
      switch(ps_status){
        case PS_CLOSE_AND_AWAY:
		  get_stable_ps_adc_value(&ps_data);
          val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
          break;
        case PS_AWAY:
          val = 1;
		  D("[PS][CM36671] proximity detected object away\n");
		  break;
        case PS_CLOSE:
          val = 0;
		  D("[PS][CM36671] proximity detected object close\n");
          break;
        };
      input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);      
      input_sync(lpi->ps_input_dev);        
    }
  }

  mutex_unlock(&CM36671_control_mutex);
  return ret;
}


static const struct i2c_device_id cm36671_i2c_id[] = {
	{CM36671_I2C_NAME, 0},
	{}
};

static struct i2c_driver cm36671_driver = {
	.id_table = cm36671_i2c_id,
	.probe = cm36671_probe,
	.driver = {
		.name = CM36671_I2C_NAME,
		.owner = THIS_MODULE,
	},
};
//wqf add start-- define proximity sensor platform data
#define INTEL_CM36671_PS_INT_N 6
static struct cm36671_platform_data cm36671_pdata = {
	
		.intr = INTEL_CM36671_PS_INT_N,
                  
        .power = NULL,
        .slave_addr = CM36671_slave_add,
        .ps_close_thd_set = 0x10,        
        .ps_away_thd_set = 0x05,
        .ps_conf1_val = CM36671_PS_ITB_1| CM36671_PS_INIT_BITS |CM36671_PS_DR_1_320 | CM36671_PS_IT_2T | CM36671_PS_PERS_2 | CM36671_PS_RES_1,
        .ps_conf3_val = CM36671_PS_MS_NORMAL | CM36671_PS_PROL_255 | CM36671_PS_SMART_PERS_ENABLE,      
};
//wqf add end
static int __init cm36671_init(void)
{
	//wqf add start--register cm36671 i2c device
    int i2c_busnum = 5;
    struct i2c_board_info i2c_info;
   
    PS_DBG("enter %s\n",__func__);

    memset(&i2c_info, 0, sizeof(i2c_info));
    strlcpy(i2c_info.type, CM36671_I2C_NAME, sizeof(CM36671_I2C_NAME));

    i2c_info.addr = CM36671_slave_add;
	i2c_info.platform_data=&cm36671_pdata;
	//i2c_info.irq = gpio_to_irq(OMAP3_CM36283_PS_INT_N);//wqf chang care
    pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);
	PS_DBG("I2C bus = %d, name = %16.16s, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.addr);
    i2c_register_board_info(i2c_busnum, &i2c_info, 1);
	//wqf add end
	return i2c_add_driver(&cm36671_driver);
}

static void __exit cm36671_exit(void)
{
	i2c_del_driver(&cm36671_driver);
}

module_init(cm36671_init);
module_exit(cm36671_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36671 Driver");
