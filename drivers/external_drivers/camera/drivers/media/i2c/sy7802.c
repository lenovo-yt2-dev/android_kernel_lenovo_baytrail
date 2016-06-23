/*
 * LED flash driver for sy7802
 *
 * Copyright (c) 2010-2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */


#define DEBUG


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <media/sy7802.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#include <linux/atomisp.h>


#define FEATURE_SY7802_I2C		/* Must Define*/
#define FEATURE_SY7802_SELF	/* Must Define*/
#define FEATURE_SY7802_V4L2	/* Must Define*/


/* Flash modes */
#define SY7802_MODE_SHUTDOWN		0
#define SY7802_MODE_TORCH			2
#define SY7802_MODE_FLASH			3
#define SY7802_MODE_SHIFT			3

/* Registers address */
#define SY7802_ENABLE_REG				0x10
#define SY7802_TORCH_BRIGHTNESS_REG	0xa0
#define SY7802_FLASH_BRIGHTNESS_REG	0xb0
#define SY7802_FLASH_DURATION_REG	0xc0
#define SY7802_FLAGS_REG				0xd0
#define SY7802_CONFIG_REG_1			0xe0
#define SY7802_CONFIG_REG_2			0xf0
#define SY7802_LAST_FLASH_REG			0x81
#define SY7802_VLED_MONITOR_REG		0x30
#define SY7802_ADC_DELAY_REG			0x31
#define SY7802_VIN_MONITOR_REG		0x80
#define SY7802_DEV_ID_REG				0xff


#define SY7802_FLAG_TIMEOUT				(1 << 0)
#define SY7802_FLAG_THERMAL_SHUTDOWN	(1 << 1)
#define SY7802_FLAG_LED_FAULT				(1 << 2)
#define SY7802_FLAG_TX1_INTERRUPT			(1 << 3)
#define SY7802_FLAG_TX2_INTERRUPT			(1 << 4)
#define SY7802_FLAG_LED_THERMAL_FAULT	(1 << 5)
#define SY7802_FLAG_FLASH_INPUT_VOLTAGE_LOW	(1 << 6)
#define SY7802_FLAG_INPUT_VOLTAGE_LOW	(1 << 7)

#define SY7802_FLASH_LED1_CURRENT_SHIFT 0
#define SY7802_FLASH_LED2_CURRENT_SHIFT 4

#define SY7802_TORCH_LED1_CURRENT_SHIFT 0
#define SY7802_TORCH_LED2_CURRENT_SHIFT 3

#define SY7802_FLASH_MAX_CURRENT		15
/***********add by wdy*************/
#define SY7802_FLASH_TIMEOUT_SHIFT	0
#define SY7802_CURRENT_LIMIT_SHIFT	5
#define SY7802_ID   0X51
 /***********add by wdy************/
#ifdef FEATURE_SY7802_SELF
struct sy7802 {
	struct v4l2_subdev sd;
	struct mutex power_lock;
	int power_count;
	unsigned int mode;
	int timeout;
	u8 torch_current;
	u8 flash_current;
	struct timer_list flash_off_delay;
	struct sy7802_platform_data *pdata;
};

#define to_sy7802(p_sd)	container_of(p_sd, struct sy7802, sd)

#endif /* FEATURE_SY7802_SELF */


#ifdef FEATURE_SY7802_V4L2
struct sy7802_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl) (struct v4l2_subdev *sd, __u32 val);
	int (*g_ctrl) (struct v4l2_subdev *sd, __s32 *val);
};
#endif /* FEATURE_SY7802_V4L2 */



#ifdef FEATURE_SY7802_I2C
static int sy7802_write(struct sy7802 *flash, u8 addr, u8 val);
static int sy7802_read(struct sy7802 *flash, u8 addr);
static int sy7802_i2c_read(struct sy7802 *flash,unsigned char u_addr, unsigned char *pu_data);
#endif /* FEATURE_SY7802_I2C */



#ifdef FEATURE_SY7802_SELF

/* -----------------------------------------------------------------------------
 * Hardware trigger
 */
static void sy7802_flash_off_delay(long unsigned int arg)
{
	
}

static int sy7802_gpio_init(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(sd);
	struct sy7802_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_request(pdata->gpio_strobe, "flash");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(pdata->gpio_strobe, 0);
	if (ret < 0)
		goto err_gpio_flash;

	ret = gpio_request(pdata->gpio_torch, "torch");
	if (ret < 0)
		goto err_gpio_flash;

	ret = gpio_direction_output(pdata->gpio_torch, 0);
	if (ret < 0)
		goto err_gpio_torch;
	
	ret = gpio_request(pdata->gpio_hold, "hold");
	if (ret < 0)
		goto err_gpio_torch;
	
	ret = gpio_direction_output(pdata->gpio_hold, 0);
	if (ret < 0)
		goto err_gpio_hold;
	
	return 0;

err_gpio_hold:
	gpio_free(pdata->gpio_hold);
err_gpio_torch:
	gpio_free(pdata->gpio_torch);
err_gpio_flash:
	gpio_free(pdata->gpio_strobe);
	return ret;
}

static int sy7802_gpio_uninit(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(sd);
	struct sy7802_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_direction_output(pdata->gpio_torch, 0);
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(pdata->gpio_strobe, 0);
	if (ret < 0)
		return ret;
	
	ret = gpio_direction_output(pdata->gpio_hold, 0);
	if (ret < 0)
		return ret;
	
	gpio_free(pdata->gpio_torch);

	gpio_free(pdata->gpio_strobe);

	gpio_free(pdata->gpio_hold);
	
	return 0;
}


/* Put device into known state. */
static int sy7802_setup(struct sy7802 *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	unsigned int flash_current_limit = flash->pdata->flash_current_limit;
	int ret = 0;
	int data = 0;

	dev_dbg(&client->dev, "%s %d\n", __func__, __LINE__ );
	/* Read chip id  */
	//ret = sy7802_i2c_read(flash,SY7802_DEV_ID_REG,data);
	ret = sy7802_read(flash, SY7802_DEV_ID_REG);
	/********************add by wdy********************/
	printk("@function  %s   @line%d   @SY7802_DEV_ID =0X %02x ",__FUNCTION__,__LINE__,(int)ret);
	if((int)ret != SY7802_ID)
		return -1;
	/*******************add by wdy*********************/
	dev_dbg(&client->dev, "%s: %d data = 0x%x\n", __func__, __LINE__, ret );

	if (ret < 0) {
		dev_err(&client->dev, "%s: %d ret = 0x%x\n", __func__, __LINE__, ret);
		//return ret;
	}

	ret = 0;
	ret = sy7802_write(flash, SY7802_TORCH_BRIGHTNESS_REG,	0x24);
	ret = sy7802_write(flash, SY7802_FLASH_BRIGHTNESS_REG,	0x88);
	ret = sy7802_write(flash, SY7802_FLASH_DURATION_REG,		0x7f);
	ret = sy7802_write(flash, SY7802_CONFIG_REG_1,			0xec);

	if (ret < 0) {
		dev_err(&client->dev, "%s: %d ret = 0x%x\n", __func__, __LINE__, ret);
		//return ret;
	}

	return ret ? -EIO : 0;
}

static int __sy7802_s_power(struct sy7802 *flash, int power)
{
	struct sy7802_platform_data *pdata = flash->pdata;
	int ret = 0;

	ret = gpio_request(pdata->gpio_reset, "flash reset");

	if (ret < 0) {
		printk("%s ,L%d gpio_request faild! \n",__func__,__LINE__);
		return ret; 
	}

	gpio_set_value(pdata->gpio_reset, power);
	printk(" %s %d reset %d , power %d \n",__func__, __LINE__,pdata->gpio_reset, power);
	
	gpio_free(pdata->gpio_reset);
	mdelay(50);

	return ret;
}


static int sy7802_hw_strobe(struct i2c_client *client, bool strobe)
{
	int ret, timer_pending;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(sd);
	struct sy7802_platform_data *pdata = flash->pdata;

	/* Strobe on Flash */
	gpio_set_value(pdata->gpio_strobe, 1);

	return 0;
}
/*********************add by wdy*********************/
static int sy7802_set_duration(struct sy7802 *flash)
{
	//printk("@function:%s    @line:%d\n",__FUNCTION__,__LINE__);
	u8 val;

	val = (flash->timeout << SY7802_FLASH_TIMEOUT_SHIFT) |
	      (flash->pdata->current_limit << SY7802_CURRENT_LIMIT_SHIFT);
	//printk("@function:%s    @line:%d  @flash->timeout:%d \n",__FUNCTION__,__LINE__,flash->timeout);
	//printk("@function:%s    @line:%d  @current_limit:%d \n",__FUNCTION__,__LINE__,flash->pdata->current_limit);
	return sy7802_write(flash, SY7802_FLASH_DURATION_REG, val);
}

static int sy7802_set_flash(struct sy7802 *flash)
{
	u8 val;
	//printk("@function:%s    @line:%d    @flash_current:%d\n",__FUNCTION__,__LINE__,flash->flash_current);
	val = (flash->flash_current << SY7802_FLASH_LED1_CURRENT_SHIFT) |
	      (flash->flash_current << SY7802_FLASH_LED2_CURRENT_SHIFT);
	 printk("@function:%s    @line:%d    @flash_doble_LED->flash_current:0X%02x\n",__FUNCTION__,__LINE__,(int)val);
	return sy7802_write(flash, SY7802_FLASH_BRIGHTNESS_REG, val);
}


static int sy7802_set_torch(struct sy7802 *flash)
{
	u8 val;
	//printk("@function:%s    @line:%d    @torch_current:%d\n",__FUNCTION__,__LINE__,flash->torch_current);
	val = (flash->torch_current << SY7802_TORCH_LED1_CURRENT_SHIFT) |
	      (flash->torch_current << SY7802_TORCH_LED2_CURRENT_SHIFT);
	 printk("@function:%s    @line:%d    @flash_double_LED->torch_current:0X%02x\n",__FUNCTION__,__LINE__,(int)val);
	return sy7802_write(flash, SY7802_TORCH_BRIGHTNESS_REG, val);
}
/*******************add by wdy*********************/
static int sy7802_set_mode(struct sy7802 *flash, unsigned int mode)
{
	u8 val = 0;
	int ret = 0;

	if (mode > SY7802_MODE_SHIFT) {
		return -1;
	}

	if ( mode == flash->mode ) {
		return 0;
	}

	val = sy7802_read(flash, SY7802_ENABLE_REG);

	val &=~  SY7802_MODE_SHIFT;//add by wdy 201410610
	
	val = ( mode & SY7802_MODE_SHIFT ) | val;

	ret = sy7802_write(flash, SY7802_ENABLE_REG, val);
	
	if (ret == 0) {
		flash->mode = mode;
	}
	return ret;
}

static int sy7802_read_status(struct sy7802 *flash)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);

	/* NOTE: reading register clear fault status */
	ret = sy7802_read(flash, SY7802_FLAGS_REG);
	if (ret < 0)
		return ret;

	/* remove monitor status bits */

	ret &= ~(SY7802_FLAG_FLASH_INPUT_VOLTAGE_LOW |
				SY7802_FLAG_INPUT_VOLTAGE_LOW);

	/*
	 * Do not take TX1/TX2 signal as an error
	 * because MSIC will not turn off flash, but turn to
	 * torch mode according to gsm modem signal by hardware.
	 */
	 
	ret &= ~(SY7802_FLAG_TX1_INTERRUPT | SY7802_FLAG_TX2_INTERRUPT);

	if (ret > 0)
		dev_dbg(&client->dev, "SY7802 flag status: %02x\n", ret);

	return ret;
}

#endif /* FEATURE_SY7802_SELF */

#ifdef FEATURE_SY7802_V4L2
/**************add by wdy**********************/
static int sy7802_s_flash_timeout(struct v4l2_subdev *sd, u32 val)
{

	struct sy7802 *flash = to_sy7802(sd);
	//printk("@function:%s    @line:%d\n   @SY7802_TIMEOUT_STEPSIZE_val1: %d\n",__FUNCTION__,__LINE__,val);
	val = clamp(val, SY7802_MIN_TIMEOUT, SY7802_MAX_TIMEOUT);
	//printk("@function:%s    @line:%d\n   @SY7802_TIMEOUT_STEPSIZE_val2: %d\n",__FUNCTION__,__LINE__,val);
	val = val / SY7802_TIMEOUT_STEPSIZE - 1;
	//printk("@function:%s    @line:%d\n   @SY7802_TIMEOUT_STEPSIZE_val3: %d\n",__FUNCTION__,__LINE__,val);
	//printk("@function:%s    @line:%d\n   @SY7802_TIMEOUT_STEPSIZE: %d\n",__FUNCTION__,__LINE__,SY7802_TIMEOUT_STEPSIZE);
	flash->timeout = val;
	//printk("@function:%s    @line:%d\n   @timeout: %d\n",__FUNCTION__,__LINE__,flash->timeout);
	return sy7802_set_duration(flash);
}

static int sy7802_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	struct sy7802 *flash = to_sy7802(sd);

	*val = (u32)(flash->timeout + 1) * SY7802_TIMEOUT_STEPSIZE;
	//printk("@function:%s    @line:%d\n",__FUNCTION__,__LINE__);
	return 0;
}




static int sy7802_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct sy7802 *flash = to_sy7802(sd);
	printk("@function:%s    @line:%d   @SY7802_FLASH_DEFAULT_BRIGHTNESS: %d\n",__FUNCTION__,__LINE__,SY7802_FLASH_DEFAULT_BRIGHTNESS);
	printk("@function:%s    @line:%d   @SY7802_TORCH_DEFAULT_BRIGHTNESS: %d\n",__FUNCTION__,__LINE__,SY7802_TORCH_DEFAULT_BRIGHTNESS);


	printk("@function:%s    @line:%d   @SY7802_FLASH_STEP: %d\n",__FUNCTION__,__LINE__,SY7802_FLASH_STEP);
	printk("@function:%s    @line:%d   @flash_intensity1: %d\n",__FUNCTION__,__LINE__,(int)intensity);
	intensity = SY7802_CLAMP_PERCENTAGE(intensity);
	printk("@function:%s    @line:%d   @flash_intensity2: %d\n",__FUNCTION__,__LINE__,(int)intensity);
	intensity = SY7802_PERCENT_TO_VALUE(intensity, SY7802_FLASH_STEP);
	printk("@function:%s    @line:%d   @flash_intensity3: 0X%02x\n",__FUNCTION__,__LINE__,(int)intensity);
	flash->flash_current = intensity;
	printk("@function:%s    @line:%d   @flash_current:0X%02x\n",__FUNCTION__,__LINE__,flash->flash_current);
	return sy7802_set_flash(flash);
}

static int sy7802_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct sy7802 *flash = to_sy7802(sd);

	*val = SY7802_VALUE_TO_PERCENT((u32)flash->flash_current,
			SY7802_FLASH_STEP);
	//printk("@function:%s    @line:%d\n",__FUNCTION__,__LINE__);
	return 0;
}




static int sy7802_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct sy7802 *flash = to_sy7802(sd);
	printk("@function:%s    @line:%d   @SY7802_TORCH_STEP: %d\n",__FUNCTION__,__LINE__,SY7802_TORCH_STEP);
	printk("@function:%s    @line:%d   @torch_intensity1: %d\n",__FUNCTION__,__LINE__,(int)intensity);
	intensity = SY7802_CLAMP_PERCENTAGE(intensity);
	printk("@function:%s    @line:%d   @torch_intensity2: %d\n",__FUNCTION__,__LINE__,(int)intensity);
	intensity = SY7802_PERCENT_TO_VALUE(intensity, SY7802_TORCH_STEP);
	printk("@function:%s    @line:%d   @torch_intensity3:0X %02x\n",__FUNCTION__,__LINE__,(int)intensity);
	flash->torch_current = intensity;
	printk("@function:%s    @line:%d   @torch_current:0X%02x\n",__FUNCTION__,__LINE__,flash->torch_current);
	return sy7802_set_torch(flash);
}

static int sy7802_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct sy7802 *flash = to_sy7802(sd);

	*val = SY7802_VALUE_TO_PERCENT((u32)flash->torch_current,
			SY7802_TORCH_STEP);
	//printk("@function:%s    @line:%d\n",__FUNCTION__,__LINE__);
	return 0;
}
/***************add by wdy******************/
static int sy7802_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return sy7802_hw_strobe(client, val);
}

static int sy7802_s_flash_mode(struct v4l2_subdev *sd, u32 new_mode)
{
	struct sy7802 *flash = to_sy7802(sd);
	unsigned int mode;

	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		mode = SY7802_MODE_SHUTDOWN;
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		mode = SY7802_MODE_FLASH;
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		mode = SY7802_MODE_TORCH;
		break;
	default:
		return -EINVAL;
	}

	return sy7802_set_mode(flash, mode);
}

static int sy7802_g_flash_mode(struct v4l2_subdev *sd, s32 * val)
{
	struct sy7802 *flash = to_sy7802(sd);
	*val = flash->mode;
	return 0;
}

static int sy7802_g_flash_status(struct v4l2_subdev *sd, s32 *val)
{
	struct sy7802 *flash = to_sy7802(sd);
	int value;

	value = sy7802_read_status(flash);
	
	if (value < 0)
		return value;

	if (value & SY7802_FLAG_TIMEOUT)
		*val = ATOMISP_FLASH_STATUS_TIMEOUT;
	else if (value > 0)
		*val = ATOMISP_FLASH_STATUS_HW_ERROR;
	else
		*val = ATOMISP_FLASH_STATUS_OK;

	return 0;
}

static const struct sy7802_ctrl_id sy7802_ctrls[] = {
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TIMEOUT,
				"Flash Timeout",
				0,
				SY7802_MAX_TIMEOUT,
				1,
				SY7802_DEFAULT_TIMEOUT,
				0,
				sy7802_s_flash_timeout,
				sy7802_g_flash_timeout),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INTENSITY,
				"Flash Intensity",
				SY7802_MIN_PERCENT,
				SY7802_MAX_PERCENT,
				1,
				SY7802_FLASH_DEFAULT_BRIGHTNESS,
				0,
				sy7802_s_flash_intensity,
				sy7802_g_flash_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TORCH_INTENSITY,
				"Torch Intensity",
				SY7802_MIN_PERCENT,
				SY7802_MAX_PERCENT,
				1,
				SY7802_TORCH_DEFAULT_BRIGHTNESS,
				0,
				sy7802_s_torch_intensity,
				sy7802_g_torch_intensity),		//add by wdy 20140606
	s_ctrl_id_entry_boolean(V4L2_CID_FLASH_STROBE,
				"Flash Strobe",
				0,
				0,
				sy7802_s_flash_strobe,
				NULL),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_MODE,
				"Flash Mode",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_MODE_OFF,
				0,
				sy7802_s_flash_mode,
				sy7802_g_flash_mode),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS,
				"Flash Status",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_STATUS_OK,
				0,
				NULL,
				sy7802_g_flash_status),
};

static const struct sy7802_ctrl_id *find_ctrl_id(unsigned int id)
{
	int i;
	int num;

	num = ARRAY_SIZE(sy7802_ctrls);
	for (i = 0; i < num; i++) {
		if (sy7802_ctrls[i].qc.id == id)
			return &sy7802_ctrls[i];
	}

	return NULL;
}

static int sy7802_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int num;

	if (!qc)
		return -EINVAL;

	num = ARRAY_SIZE(sy7802_ctrls);
	if (qc->id >= num)
		return -EINVAL;

	*qc = sy7802_ctrls[qc->id].qc;

	return 0;
}

static int sy7802_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sy7802_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (!s_ctrl)
		return -EINVAL;

	return s_ctrl->s_ctrl(sd, ctrl->value);
}

static int sy7802_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sy7802_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (s_ctrl == NULL)
		return -EINVAL;

	return s_ctrl->g_ctrl(sd, &ctrl->value);
}

static int sy7802_s_power(struct v4l2_subdev *sd, int power)
{
	struct sy7802 *flash = to_sy7802(sd);
	int ret = 0;

	mutex_lock(&flash->power_lock);

	if (flash->power_count != power) {
		ret = __sy7802_s_power(flash, power);
		if (ret < 0)
			goto done;
	}

	flash->power_count += power ? 1 : -1;
	WARN_ON(flash->power_count < 0);

done:
	mutex_unlock(&flash->power_lock);
	return ret;
}

static int sy7802_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	struct sy7802 *flash = to_sy7802(sd);
	int ret;

	dev_dbg(&client->dev, "%s , L%d \n",__func__, __LINE__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "sy7802_detect i2c error\n");
		return -ENODEV;
	}

	/* Power up the flash driver and reset it */
	ret = sy7802_s_power(&flash->sd, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s , L%d sy7802_s_power failed\n",__func__, __LINE__);
		return ret;
	}

	/* Setup default values. This makes sure that the chip is in a known
	 * state.
	 */
	ret = sy7802_setup(flash);
	
	if (ret < 0) {
		dev_err(&client->dev, "%s , L%d sy7802_setup failed\n",__func__, __LINE__);
		goto fail;
	}
	dev_dbg(&client->dev, "Successfully detected sy7802 LED flash\n");
	sy7802_s_power(&flash->sd, 0);
	return 0;

fail:
	sy7802_s_power(&flash->sd, 0);
	return ret;
}

static int sy7802_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return sy7802_s_power(sd, 1);
}

static int sy7802_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return sy7802_s_power(sd, 0);
}

static const struct v4l2_subdev_internal_ops sy7802_internal_ops = {
	.registered = sy7802_detect,
	.open = sy7802_open,
	.close = sy7802_close,
};

static const struct v4l2_subdev_core_ops sy7802_core_ops = {
	.queryctrl = sy7802_queryctrl,
	.g_ctrl = sy7802_g_ctrl,
	.s_ctrl = sy7802_s_ctrl,
	.s_power = sy7802_s_power,
};

static const struct v4l2_subdev_ops sy7802_ops = {
	.core = &sy7802_core_ops,
};

#endif /* FEATURE_SY7802_V4L2 */


#ifdef FEATURE_SY7802_I2C

/* Return negative errno else zero on success */
static int sy7802_write(struct sy7802 *flash, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	int ret;

	ret = i2c_smbus_write_byte_data(client, addr, val);

	dev_dbg(&client->dev, "Write Addr:%02X Val:%02X %s\n", addr, val,
		ret < 0 ? "fail" : "ok");

	return ret;
}

/* Return negative errno else a data byte received from the device. */
static int sy7802_read(struct sy7802 *flash, u8 addr)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	int ret;

	ret = i2c_smbus_read_byte_data(client, addr);

	dev_dbg(&client->dev, "Read Addr:%02X Val:%02X %s\n", addr, ret,
		ret < 0 ? "fail" : "ok");

	return ret;
}

static int sy7802_i2c_rx_data(struct sy7802 *flash,char* rxData, int length)
{
	int rc;
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(client->adapter, msgs, 2);
	if (rc < 0)
	{
		printk("sy7802_i2c_rx_data error %d\n", rc);
		return rc;
	}
	return 0;
}

static int sy7802_i2c_read(struct sy7802 *flash,unsigned char u_addr, unsigned char *pu_data)
{
	int rc;
	unsigned char buf[1];

	buf[0] = (u_addr & 0xFF);
	rc = sy7802_i2c_rx_data(flash,buf, 1);
	if (!rc)
		*pu_data = buf[0];
	else
		printk("i2c read failed\n");
	return rc;
}

#ifdef CONFIG_PM
static int sy7802_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(subdev);
	int rval;

	if (flash->power_count == 0)
		return 0;

	rval = __sy7802_s_power(flash, 0);

	dev_dbg(&client->dev, "Suspend %s\n", rval < 0 ? "failed" : "ok");

	return rval;
}

static int sy7802_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(subdev);
	int rval;
	dev_err(&client->dev, "%s %d\n", __func__, __LINE__ );
	if (flash->power_count == 0)
		return 0;

	rval = __sy7802_s_power(flash, 1);

	dev_dbg(&client->dev, "Resume %s\n", rval < 0 ? "fail" : "ok");

	return rval;
}

static const struct dev_pm_ops sy7802_pm_ops = {
	.suspend = sy7802_suspend,
	.resume = sy7802_resume,
};
#endif /* CONFIG_PM */

static int sy7802_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct sy7802 *flash;

	dev_dbg(&client->dev, "%s %d\n", __func__, __LINE__ );

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	flash->pdata = client->dev.platform_data;

	v4l2_i2c_subdev_init(&flash->sd, client, &sy7802_ops);
	flash->sd.internal_ops = &sy7802_internal_ops;
	flash->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->mode = ATOMISP_FLASH_MODE_OFF;

	err = media_entity_init(&flash->sd.entity, 0, NULL, 0);
	if (err) {
		dev_err(&client->dev, "error initialize a media entity.\n");
		goto fail1;
	}

	flash->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	mutex_init(&flash->power_lock);

	setup_timer(&flash->flash_off_delay, sy7802_flash_off_delay,
		    (unsigned long)client);

	err = sy7802_gpio_init(client);
	if (err) {
		dev_err(&client->dev, "gpio request/direction_output fail");
		goto fail2;
	}

	if (flash->pdata->flash_current_limit == 0 ||
	     flash->pdata->flash_current_limit > SY7802_FLASH_MAX_CURRENT) 
	{
		flash->pdata->flash_current_limit = SY7802_FLASH_MAX_CURRENT;
	}

	return 0;
fail2:
	media_entity_cleanup(&flash->sd.entity);
fail1:
	v4l2_device_unregister_subdev(&flash->sd);
	kfree(flash);

	return err;
}

static int sy7802_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sy7802 *flash = to_sy7802(sd);
	int ret;

	media_entity_cleanup(&flash->sd.entity);
	v4l2_device_unregister_subdev(sd);

	del_timer_sync(&flash->flash_off_delay);

	ret = sy7802_gpio_uninit(client);
	if (ret < 0)
		goto fail;

	kfree(flash);

	return 0;
fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}


MODULE_DEVICE_TABLE(i2c, sy7802_id);
static const struct i2c_device_id sy7802_id[] = {
	{ SY7802_NAME, 0 },
	{ },
};

static struct i2c_driver sy7802_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SY7802_NAME,
#ifdef CONFIG_PM
		.pm   = &sy7802_pm_ops,
#endif /* CONFIG_PM */
	},
	.probe = sy7802_probe,
	.remove = sy7802_remove,
	.id_table = sy7802_id,
};

static __init int init_sy7802(void)
{
	return i2c_add_driver(&sy7802_driver);
}

static __exit void exit_sy7802(void)
{
	i2c_del_driver(&sy7802_driver);
}
#endif /* FEATURE_SY7802_I2C */

module_init(init_sy7802);
module_exit(exit_sy7802);
MODULE_AUTHOR("qiang zuo<qiang.zuo0@gmail.com>");
MODULE_DESCRIPTION("LED flash driver for SY7802");
MODULE_LICENSE("GPL");

