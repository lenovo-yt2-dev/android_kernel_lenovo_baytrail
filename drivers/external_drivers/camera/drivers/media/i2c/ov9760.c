/*
 * Support for OV9760 720P camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <linux/firmware.h>

#include "ov9760.h"
#ifndef __KERNEL__
#define __KERNEL__
#endif

#include "ov9760_bld_otp.c"


#define DEBUG_VERBOSE	(1<<0)
#define DEBUG_GAIN_EXP	(1<<1)
#define DEBUG_INTG_FACT	(1<<2)
#define DEBUG_OTP	(1<<4)

#define OV9760_OTP_NAME "ov9760_otp.bin"

u8 ov9760_data[DATA_BUF_SIZE];
u8 otp_data[DATA_BUF_SIZE];
u32 ov9760_size = 0;
u32 otp_size = DATA_BUF_SIZE;

static unsigned int debug = 0x00;
module_param(debug, int, S_IRUGO|S_IWUSR);
#define OV9760_DEFAULT_LOG_LEVEL 1

static unsigned int log_level = OV9760_DEFAULT_LOG_LEVEL;
module_param(log_level, int, 0644);

#define OV9760_LOG(level, a, ...) \
	do { \
		if (level < log_level) \
			printk(a,## __VA_ARGS__); \
	} while (0)

unsigned int cmd_data, gpio_no_value, op_dump_otp;
struct i2c_client *gclient = NULL;
static int ov9760_read_i2c(const char *val, struct kernel_param *kp);
static int ov9760_set_gpio(const char *val, struct kernel_param *kp);
static int ov9760_get_gpio(const char *val, struct kernel_param *kp);
static int ov9760_dump_otp(const char *val, struct kernel_param *kp);

module_param_call(read_i2c, ov9760_read_i2c, param_get_uint,
				&cmd_data, S_IRUGO | S_IWUSR);
module_param_call(gpio_set, ov9760_set_gpio, param_get_uint,
				&gpio_no_value, S_IRUGO | S_IWUSR);
module_param_call(gpio_get, ov9760_get_gpio, param_get_uint,
				&gpio_no_value, S_IRUGO | S_IWUSR);

module_param_call(dump_otp, ov9760_dump_otp, param_get_uint,
				&op_dump_otp, S_IRUGO | S_IWUSR);

static int ov9760_dump_otp(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = ov9760_otp_save(ov9760_data,ov9760_size,OV9760_SAVE_RAW_DATA);
	if(ret != 0)
		OV9760_LOG(2,"Fail to save ov9760 RAW data\n");

	ret = ov9760_otp_save(otp_data,otp_size,OV9760_SAVE_OTP_DATA);
	if(ret != 0)
		OV9760_LOG(2,"Fail to save ov9760 OTP data\n");
	return 0;
}

static int ov9760_set_gpio(const char *val, struct kernel_param *kp)
{
	int gpio_number, gpio_value;

	int rv = param_set_int(val, kp);
	if (rv)
		return rv;
	gpio_number = gpio_no_value >> 16;
	gpio_value = gpio_no_value & 0x01;

	OV9760_LOG(1,"%s set gpio %d to %d\n", __func__, gpio_number, gpio_value);
	gpio_set_value(gpio_number, gpio_value);

	return 0;
}

static int ov9760_get_gpio(const char *val, struct kernel_param *kp)
{
	int gpio_number = 0;

	int rv = param_set_int(val, kp);
	if (rv)
		return rv;
	gpio_number = gpio_no_value >> 16;

	OV9760_LOG(1,"%s get gpio %d value:%d\n", __func__, gpio_number, gpio_get_value(gpio_number) & 0x01);

	return 0;
}

static int ov9760_read_i2c(const char *val, struct kernel_param *kp)
{
	u16 data;
	int i, len;

	int rv = param_set_int(val, kp);
	if (rv)
		return rv;

	len = (cmd_data & 0xffff)%17;
	if (gclient != NULL)
		for (i = 0; i < len; i++) {
			ov9760_read_reg(gclient, 1, (cmd_data >> 16) + i, &data);
			OV9760_LOG(1,"addr:0x%x data:0x%x\n", (cmd_data >> 16) + i, data);
		}

	return 0;
}

/* the bayer order mapping table
 *          hflip=0                  hflip=1
 * vflip=0  atomisp_bayer_order_bggr atomisp_bayer_order_gbrg
 * vflip=1  atomisp_bayer_order_grbg atomisp_bayer_order_rggb
 *
 * usage: ov9760_bayer_order_mapping[vflip][hflip]
 */
static const int ov9760_bayer_order_mapping[2][2] = {
	{atomisp_bayer_order_bggr, atomisp_bayer_order_gbrg},
	{atomisp_bayer_order_grbg, atomisp_bayer_order_rggb}
};

static int
ov9760_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[OV9760_SHORT_MAX] = {0};
	int err, i;

	if (len > OV9760_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == OV9760_8BIT)
		*val = (u8)data[0];
	else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int ov9760_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int
ov9760_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV9760_8BIT && data_length != OV9760_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV9760_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV9760_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov9760_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * ov9760_write_reg_array - Initializes a list of OV9760 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ov9760_flush_reg_array, __ov9760_buf_reg_array() and
 * __ov9760_write_reg_is_consecutive() are internal functions to
 * ov9760_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ov9760_flush_reg_array(struct i2c_client *client,
				 struct ov9760_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov9760_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ov9760_buf_reg_array(struct i2c_client *client,
				struct ov9760_write_ctrl *ctrl,
				const struct ov9760_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case OV9760_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case OV9760_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = (u16)next->val;
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= OV9760_MAX_WRITE_BUF_SIZE)
		return __ov9760_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ov9760_write_reg_is_consecutive(struct i2c_client *client,
				struct ov9760_write_ctrl *ctrl,
				const struct ov9760_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ov9760_write_reg_array(struct i2c_client *client,
				const struct ov9760_reg *reglist)
{
	const struct ov9760_reg *next = reglist;
	struct ov9760_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != OV9760_TOK_TERM; next++) {
		switch (next->type & OV9760_TOK_MASK) {
		case OV9760_TOK_DELAY:
			err = __ov9760_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ov9760_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ov9760_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ov9760_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ov9760_flush_reg_array(client, &ctrl);
}

static int __ov9760_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = ov9760_write_reg_array(client, ov9760_init_config);
	if (ret)
		return ret;

	/* restore settings */
	ov9760_res = ov9760_res_preview;
	N_RES = N_RES_PREVIEW;

	return 0;
}

static int ov9760_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);
	ret = __ov9760_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static void ov9760_uninit(struct v4l2_subdev *sd)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	dev->coarse_itg = 0;
	dev->fine_itg   = 0;
	dev->gain       = 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "gpio failed 1\n");
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int __ov9760_s_power(struct v4l2_subdev *sd, int power)
{
	int ret = 0;
	if (power == 0) {
		ov9760_uninit(sd);
		return power_down(sd);
	} else {
		ret = power_up(sd);
		if (ret)
			return -EINVAL;
		ret = __ov9760_init(sd, 0);
		if (ret) {
			power_down(sd);
			return ret;
		}
	}
	return 0;
}

static int ov9760_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	mutex_lock(&dev->input_lock);
	ret = __ov9760_s_power(sd, on);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov9760_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 tmp;
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = ov9760_read_reg(client, OV9760_8BIT,
			OV9760_COARSE_INTEGRATION_TIME_H, &tmp);

	/* the fine integration time is currently not calculated */
	ret = ov9760_read_reg(client, OV9760_16BIT,
			OV9760_COARSE_INTEGRATION_TIME_M, &coarse);

	coarse = (coarse >> 4) | ((tmp & 0xf) << 12);
	*value = coarse;

	return ret;
}

static int ov9760_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	int ret;
	u32	vt_pix_clk_div;
	u32	vt_sys_clk_div;
	u32	pll_pll1_pre_clk_div;
	u32	pll_multiplier;
	u32 mipi_div;


	const int ext_clk_freq_hz = 19200000;
	struct atomisp_sensor_mode_data *buf = &info->data;
	int vt_pix_clk_freq_mhz;
	u16 data[OV9760_INTG_BUF_COUNT];
	u16 reg_0x30b0, reg_0x30b1, reg_0x30b2, reg_0x30b3, reg_0x30b4;
	u16 reg_0x3010;

	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 fine_integration_time_min;
	u32 fine_integration_time_max_margin;
	u32 frame_length_lines;
	u32 line_length_pck;
	u32 read_mode;
	u32 div;

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, OV9760_INTG_BUF_COUNT * sizeof(u16));

	ret = ov9760_read_reg(client, 1, OV9760_VT_PIX_CLK_DIV, &reg_0x30b0);
	if (ret)
		return ret;
	vt_pix_clk_div = reg_0x30b0 & 0x0f;

	ret = ov9760_read_reg(client, 1, OV9760_VT_SYS_CLK_DIV, &reg_0x30b1);
	if (ret)
		return ret;
	vt_sys_clk_div = reg_0x30b1 & 0x1f;

	ret = ov9760_read_reg(client, 1, OV9760_PLL_MULTIPLIER_H, &reg_0x30b2);
	if (ret)
		return ret;

	ret = ov9760_read_reg(client, 1, OV9760_PLL_MULTIPLIER_L, &reg_0x30b3);
	if (ret)
		return ret;
	pll_multiplier = ((reg_0x30b2 & 0x01) << 8) | reg_0x30b3;

	ret = ov9760_read_reg(client, 1, OV9760_PLL_PLL1_PRE_DIV, &reg_0x30b4);
	if (ret)
		return ret;
	pll_pll1_pre_clk_div = reg_0x30b4 & 0x0f;

	ret = ov9760_read_reg(client, 1, OV9760_MIPI_DIV, &reg_0x3010);
	if (ret)
		return ret;

	mipi_div = reg_0x3010 & 0x07;

	OV9760_LOG(1,"reg[0x30b0..30b4]: 0x%x 0x%x 0x%x 0x%x 0x%x\n", reg_0x30b0,
		reg_0x30b1, reg_0x30b2, reg_0x30b3, reg_0x30b4);
	OV9760_LOG(1,"vt_pix_clk_div:0x%x vt_sys_clk_div:0x%x pll_multiplier:0x%x \
			pll_pll1_pre_clk_div:0x%x mipi_div:0x%x", vt_pix_clk_div,
			vt_sys_clk_div, pll_multiplier, pll_pll1_pre_clk_div,
			mipi_div);

	div = pll_pll1_pre_clk_div * vt_sys_clk_div * vt_pix_clk_div * mipi_div;
	if (div == 0)
		return -EINVAL;
	vt_pix_clk_freq_mhz = ext_clk_freq_hz*pll_multiplier/div;

	memset(data, 0, OV9760_INTG_BUF_COUNT * sizeof(u16));
	ret = ov9760_read_reg(client, 4, OV9760_FRAME_LENGTH_LINES, data);
	if (ret)
		return ret;

	frame_length_lines = data[0];
	line_length_pck = data[1];

	coarse_integration_time_min = 4;
	coarse_integration_time_max_margin = 4;

	fine_integration_time_min = 4;
	fine_integration_time_max_margin = 4;

	dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf->coarse_integration_time_min = coarse_integration_time_min;
	buf->coarse_integration_time_max_margin
		= coarse_integration_time_max_margin;
	buf->fine_integration_time_min = fine_integration_time_min;
	buf->fine_integration_time_max_margin =
					fine_integration_time_max_margin;
	buf->fine_integration_time_def = fine_integration_time_max_margin;
	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf->line_length_pck = line_length_pck;
	buf->frame_length_lines = frame_length_lines;

	buf->binning_factor_x = ov9760_res[dev->fmt_idx].bin_factor_x ? 2 : 1;
	buf->binning_factor_y = ov9760_res[dev->fmt_idx].bin_factor_y ? 2 : 1;
	read_mode = ov9760_res[dev->fmt_idx].bin_factor_x ? 0x800 : 0;
	buf->read_mode = read_mode;

	/* Get the cropping and output resolution to ISP for this mode. */
	ret = ov9760_read_reg(client, 2, OV9760_HORIZONTAL_START_H, data);
	if (ret)
		return ret;
	buf->crop_horizontal_start = data[0];

	ret = ov9760_read_reg(client, 2, OV9760_VERTICAL_START_H, data);
	if (ret)
		return ret;
	buf->crop_vertical_start = data[0];

	ret = ov9760_read_reg(client, 2, OV9760_HORIZONTAL_END_H, data);
	if (ret)
		return ret;
	buf->crop_horizontal_end = data[0];

	ret = ov9760_read_reg(client, 2, OV9760_VERTICAL_END_H, data);
	if (ret)
		return ret;
	buf->crop_vertical_end = data[0];

	ret = ov9760_read_reg(client, 2, OV9760_HORIZONTAL_OUTPUT_SIZE_H, data);
	if (ret)
		return ret;
	buf->output_width = data[0] - ISP_PADDING_W;

	ret = ov9760_read_reg(client, 2, OV9760_VERTICAL_OUTPUT_SIZE_H, data);
	if (ret)
		return ret;
	buf->output_height = data[0] - ISP_PADDING_H;

	/* OV9760 is upper-left crop. but this configuration has mirror setting
	 * So, the crop coordinate for upper-right.
	 */
	buf->crop_horizontal_start = OV9760_ISP_MAX_WIDTH - buf->output_width;
	buf->crop_horizontal_end = OV9760_ISP_MAX_WIDTH - 1;

	buf->crop_vertical_start = 0;
	buf->crop_vertical_end = buf->output_height - 1;

	if(debug & DEBUG_INTG_FACT) {
		OV9760_LOG(1,"%s %d vt_pix_clk_freq_mhz:%d line_length_pck:%d frame_length_lines:%d\n", __func__, __LINE__,
				buf->vt_pix_clk_freq_mhz, buf->line_length_pck,	buf->frame_length_lines);
		OV9760_LOG(1,"%s %d coarse_intg_min:%d coarse_intg_max_margin:%d fine_intg_min:%d fine_intg_max_margin:%d\n",
				__func__, __LINE__,
				buf->coarse_integration_time_min,buf->coarse_integration_time_max_margin,
				buf->fine_integration_time_min, buf->fine_integration_time_max_margin);
		OV9760_LOG(1,"%s %d crop_x_start:%d crop_y_start:%d crop_x_end:%d crop_y_end:%d \n", __func__, __LINE__,
				buf->crop_horizontal_start, buf->crop_vertical_start, buf->crop_horizontal_end, buf->crop_vertical_end);
		OV9760_LOG(1,"%s %d output_width:%d output_height:%d\n", __func__, __LINE__, buf->output_width, buf->output_height);
	}
	return 0;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 140
static int distance(struct ov9760_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13) / w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -1;
	h_ratio = ((res->height << 13) / h);
	if (h_ratio == 0)
		return -1;
	match = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192) ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ov9760_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov9760_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov9760_res[i].width)
			continue;
		if (h != ov9760_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int ov9760_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if ((fmt->width > OV9760_RES_WIDTH_MAX)
		|| (fmt->height > OV9760_RES_HEIGHT_MAX)) {
		fmt->width = OV9760_RES_WIDTH_MAX;
		fmt->height = OV9760_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 * resolutions. If it fails, it means the requested
		 * resolution is higher than wecan support. Fallback
		 * to highest possible resolution in this case.
		 */
		if (idx == -1) {
			idx = N_RES - 1;
			OV9760_LOG(1,"%s %d idx:%d w:%d h:%d \n", __func__, __LINE__, idx, ov9760_res[idx].width, ov9760_res[idx].height);
		}

		fmt->width = ov9760_res[idx].width;
		fmt->height = ov9760_res[idx].height;
	}


	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;


	return 0;
}

static int ov9760_set_mbus_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *fmt)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	const struct ov9760_reg *ov9760_def_reg;
	struct camera_mipi_info *ov9760_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ov9760_info = v4l2_get_subdev_hostdata(sd);
	if (ov9760_info == NULL)
		return -EINVAL;

	ret = ov9760_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}

	mutex_lock(&dev->input_lock);
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);

	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	OV9760_LOG(1,"%s %d name:%s width:%d height:%d\n", __func__, __LINE__, ov9760_res[dev->fmt_idx].desc, ov9760_res[dev->fmt_idx].width, ov9760_res[dev->fmt_idx].height);
	ov9760_def_reg = ov9760_res[dev->fmt_idx].regs;
	/* enable group hold */
	ret = ov9760_write_reg_array(client, ov9760_param_hold);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = ov9760_write_reg_array(client, ov9760_def_reg);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	/* disable group hold */
	ret = ov9760_write_reg_array(client, ov9760_param_update);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}
	dev->fps = ov9760_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ov9760_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ov9760_res[dev->fmt_idx].lines_per_frame;
	dev->coarse_itg = 0;
	dev->fine_itg = 0;
	dev->gain = 0;

	ret = ov9760_get_intg_factor(client, ov9760_info);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}
	return 0;
}

static int ov9760_g_mbus_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *fmt)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	if (!fmt)
		return -EINVAL;

	OV9760_LOG(1,"%s %d idx:%d w:%d h:%d\n", __func__, __LINE__, dev->fmt_idx, ov9760_res[dev->fmt_idx].width, ov9760_res[dev->fmt_idx].height);
	fmt->width = ov9760_res[dev->fmt_idx].width;
	fmt->height = ov9760_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov9760_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV9760_FOCAL_LENGTH_NUM << 16) | OV9760_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov9760_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ov9760*/
	*val = (OV9760_F_NUMBER_DEFAULT_NUM << 16) | OV9760_F_NUMBER_DEM;
	return 0;
}

static int ov9760_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV9760_F_NUMBER_DEFAULT_NUM << 24) |
		(OV9760_F_NUMBER_DEM << 16) |
		(OV9760_F_NUMBER_DEFAULT_NUM << 8) | OV9760_F_NUMBER_DEM;
	return 0;
}

/* Horizontal flip the image. */
static int ov9760_t_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

/* Vertically flip the image */
static int ov9760_t_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov9760_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov9760_write_reg(client, OV9760_8BIT,
			OV9760_TEST_PATTERN_MODE, value);
}

static int ov9760_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	*val = ov9760_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int ov9760_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	*val = ov9760_res[dev->fmt_idx].bin_factor_y;

	return 0;
}
static int ov9760_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	if (IS_ERR(dev->otp_data)) {
		dev_err(&client->dev, "OTP data not available");
		return PTR_ERR(dev->otp_data);
	}
	/* Correct read_size value only if bigger than maximum */
	if (read_size > DATA_BUF_SIZE)
		read_size = DATA_BUF_SIZE;

	OV9760_LOG(1,"%s %d read:%d\n", __func__, __LINE__, read_size);
	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		dev_err(&client->dev, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}
out:

	/* Return correct size */
	priv->size = DATA_BUF_SIZE;
	OV9760_LOG(1,"%s %d qurry size:%d\n", __func__, __LINE__, priv->size);

	return 0;
}


static struct ov9760_control ov9760_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov9760_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image v-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov9760_t_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image h-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov9760_t_hflip,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov9760_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV9760_FOCAL_LENGTH_DEFAULT,
			.maximum = OV9760_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV9760_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov9760_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV9760_F_NUMBER_DEFAULT,
			.maximum = OV9760_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV9760_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov9760_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV9760_F_NUMBER_RANGE,
			.maximum = OV9760_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV9760_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov9760_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = OV9760_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov9760_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = OV9760_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov9760_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov9760_controls))

static long __ov9760_set_exposure(struct v4l2_subdev *sd, u16 coarse_itg,
				 u16 gain)

{
	/* shunyong, disable before the format from IQ tool confirmed */
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	/* enable group hold */
	ret = ov9760_write_reg_array(client, ov9760_param_hold);
	if (ret)
		goto out;

	/* set coarse integration time */
	if (coarse_itg > (dev->lines_per_frame - 4))
		ov9760_write_reg(client, OV9760_16BIT,
			OV9760_FRAME_LENGTH_LINES, coarse_itg + 4);
	else
		ov9760_write_reg(client, OV9760_16BIT,
			OV9760_FRAME_LENGTH_LINES, dev->lines_per_frame);
	/* bits [3..0]-->coarse exp[15..12] */
	ret = ov9760_write_reg(client, OV9760_8BIT,
			OV9760_COARSE_INTEGRATION_TIME_H, coarse_itg>>12);
	if (ret)
		goto out_disable;
	/* bits [15..4]-->coarse exp[11..0] */
	ret = ov9760_write_reg(client, OV9760_16BIT,
			OV9760_COARSE_INTEGRATION_TIME_M, (coarse_itg & 0xfff)<<4);
	if (ret)
		goto out_disable;

	/* set global gain */
	gain = clamp(gain, 0, OV9760_MAX_GAIN_VALUE);
	ret = ov9760_write_reg(client, OV9760_16BIT,
			OV9760_GLOBAL_GAIN, (gain & 0x7ff));
	if (ret)
		goto out_disable;
	dev->gain = gain;
	dev->coarse_itg = coarse_itg;

out_disable:
	/* disable group hold */
	ov9760_write_reg_array(client, ov9760_param_update);
	if (debug & DEBUG_GAIN_EXP) {
		u16 reg_global_gain, reg_frame_length, reg_coarse_intg_h, reg_coarse_intg_m;
		ov9760_read_reg(client, OV9760_16BIT, OV9760_GLOBAL_GAIN, &reg_global_gain);
		ov9760_read_reg(client, OV9760_16BIT, OV9760_FRAME_LENGTH_LINES, &reg_frame_length);
		ov9760_read_reg(client, OV9760_8BIT, OV9760_COARSE_INTEGRATION_TIME_H, &reg_coarse_intg_h);
		ov9760_read_reg(client, OV9760_16BIT, OV9760_COARSE_INTEGRATION_TIME_M, &reg_coarse_intg_m);

		OV9760_LOG(1,"%s %d gain:%d coarse_itg:%d\n", __func__, __LINE__, gain, coarse_itg);
		OV9760_LOG(1,"%s %d register:0x%x--0x%x 0x%x--0x%x 0x%x--0x%x 0x%x--0x%x\n",
				__func__, __LINE__,
				OV9760_GLOBAL_GAIN, reg_global_gain,
				OV9760_FRAME_LENGTH_LINES, reg_frame_length,
				OV9760_COARSE_INTEGRATION_TIME_H, reg_coarse_intg_h,
				OV9760_COARSE_INTEGRATION_TIME_M, reg_coarse_intg_m
				);
	}
out:
	return ret;
}

static int ov9760_set_exposure(struct v4l2_subdev *sd, u16 exposure, u16 gain)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ov9760_set_exposure(sd, exposure, gain);
	mutex_unlock(&dev->input_lock);

	return ret;
}
static long ov9760_s_exposure(struct v4l2_subdev *sd,
			 struct atomisp_exposure *exposure)
{
	u16 coarse_itg, gain;

	coarse_itg = exposure->integration_time[0];
	gain = exposure->gain[0];

	return ov9760_set_exposure(sd, coarse_itg, gain);
}

static long ov9760_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov9760_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ov9760_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static struct ov9760_control *ov9760_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (ov9760_controls[i].qc.id == id)
			return &ov9760_controls[i];
	}
	return NULL;
}

static int ov9760_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low, rev;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (ov9760_read_reg(client, OV9760_8BIT, OV9760_PID_HIGH,
			&high)) {
		v4l2_err(client, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}

	if (ov9760_read_reg(client, OV9760_8BIT, OV9760_PID_LOW,
			&low)) {
		v4l2_err(client, "sensor_id_low = 0x%x\n", low);
		return -ENODEV;
	}

	*id = (((u8) high) << 8) | (u8) low;
	v4l2_err(client, "sensor_id = 0x%x\n", *id);

	if (*id != OV9760_MOD_ID) {
		v4l2_err(client, "sensor ID error\n");
		return -ENODEV;
	}

	v4l2_err(client, "detect ov9760 success\n");

	if (ov9760_read_reg(client, OV9760_8BIT, OV9760_REV,
			&rev)) {
		v4l2_err(client, "sensor_id_low = 0x%x\n", rev);
		return -ENODEV;
	}

	/* TODO - need to be updated */
	*revision = rev;

	return 0;
}

static int
ov9760_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct firmware *fw;
	u8 sensor_revision;
	u16 sensor_id;
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
		(struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			dev_err(&client->dev, "imx platform init err\n");
			return ret;
		}
	}

	ret = __ov9760_s_power(sd, 1);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "ov9760 power-up err");
		return ret;
	}

	/* config & detect sensor */
	ret = ov9760_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ov9760_detect err s_config.\n");
		goto fail_detect;
	}
	memset(ov9760_data, 0, DATA_BUF_SIZE);
	memset(otp_data, 0, DATA_BUF_SIZE);

	dev->otp_data = NULL;
	/* try to load otp from user-space first */
	ret = request_firmware(&fw, OV9760_OTP_NAME, &client->dev);
	if(ret) {
		OV9760_LOG(1,"ov9760 load from user-space failed, load from sensor\n");
		ov9760_write_reg_array(client, ov9760_streaming);
		ret = ov9760_otp_read(client, ov9760_data, &ov9760_size);
		ov9760_write_reg_array(client, ov9760_suspend);
		if (!ret)
		{
			OV9760_LOG(1,"ov9760 otp read done\n");
			ret = ov9760_otp_trans(ov9760_data, ov9760_size, otp_data, &otp_size);
			if (!ret)
			{
				OV9760_LOG(1,"ov9760 otp trans done\n");
				dev->otp_data = otp_data;
			}
		}
	} else {
		OV9760_LOG(1,"ov9760 load from user-space success size:0x%x\n", fw->size);
		memcpy(ov9760_data, fw->data, fw->size);
		ov9760_size = fw->size;
		ret = ov9760_otp_trans(ov9760_data, ov9760_size, otp_data, &otp_size);
		if (!ret)
		{
			OV9760_LOG(1,"ov9760 otp trans done\n");
			dev->otp_data = otp_data;
		}
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = __ov9760_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(client, "ov9760 power down err");
		return ret;
	}

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	__ov9760_s_power(sd, 0);

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int ov9760_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov9760_control *ctrl = ov9760_find_control(qc->id);
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ov9760_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov9760_control *octrl = ov9760_find_control(ctrl->id);
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov9760_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov9760_control *octrl = ov9760_find_control(ctrl->id);
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;
	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov9760_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	mutex_lock(&dev->input_lock);

	if (enable) {
		usleep_range(10000, 10100);
		ret = ov9760_write_reg_array(client, ov9760_streaming);
		dev->streaming = 1;
	} else {
		ret = ov9760_write_reg_array(client, ov9760_suspend);
		if (ret != 0) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "write_reg_array err\n");
			return ret;
		}
		dev->streaming = 0;
	}

	mutex_unlock(&dev->input_lock);
	return 0;
}

/*
 * ov9760 enum frame size, frame intervals
 */
static int ov9760_enum_framesizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov9760_res[index].width;
	fsize->discrete.height = ov9760_res[index].height;
	fsize->reserved[0] = ov9760_res[index].used;

	return 0;
}

static int ov9760_enum_frameintervals(struct v4l2_subdev *sd,
				 struct v4l2_frmivalenum *fival)
{
	int i;

	/* since the isp will donwscale the resolution to the right size,
	  * find the nearest one that will allow the isp to do so
	  * important to ensure that the resolution requested is padded
	  * correctly by the requester, which is the atomisp driver in
	  * this case.
	  */
	i = nearest_resolution_index(fival->width, fival->height);

	if (i == -1)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = ov9760_res[i].width;
	fival->height = ov9760_res[i].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov9760_res[i].fps;

	return 0;
}

static int ov9760_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	if (index >= MAX_FMTS)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}


static int
ov9760_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV9760, 0);
}

static int
ov9760_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int
ov9760_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov9760_res[index].width;
	fse->min_height = ov9760_res[index].height;
	fse->max_width = ov9760_res[index].width;
	fse->max_height = ov9760_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov9760_get_pad_format(struct ov9760_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
ov9760_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov9760_get_pad_format(dev, fh, fmt->pad, fmt->which);

	fmt->format = *format;

	return 0;
}

static int
ov9760_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
ov9760_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	OV9760_LOG(1,"%s %d mode:%d\n", __func__, __LINE__, dev->run_mode);
	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		OV9760_LOG(1,"%s %d\n", __func__, __LINE__);
		ov9760_res = ov9760_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		OV9760_LOG(1,"%s %d\n", __func__, __LINE__);
		ov9760_res = ov9760_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		OV9760_LOG(1,"%s %d\n", __func__, __LINE__);
		ov9760_res = ov9760_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

int
ov9760_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;
	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->coarse_itg > dev->lines_per_frame) {
		if (dev->coarse_itg == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal coarse_itg:0x%x\n",
				__func__, dev->coarse_itg);
			lines_per_frame = dev->coarse_itg;
		} else {
			lines_per_frame = dev->coarse_itg + 1;
		}
	} else {
		lines_per_frame = dev->lines_per_frame;
	}

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = dev->vt_pix_clk_freq_mhz;

	return 0;
}

static int ov9760_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = ov9760_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops ov9760_sensor_ops = {
	.g_skip_frames = ov9760_g_skip_frames,
};

static const struct v4l2_subdev_video_ops ov9760_video_ops = {
	.try_mbus_fmt = ov9760_try_mbus_fmt,
	.s_mbus_fmt = ov9760_set_mbus_fmt,
	.s_stream = ov9760_s_stream,
	.enum_framesizes = ov9760_enum_framesizes,
	.enum_frameintervals = ov9760_enum_frameintervals,
	.s_parm = ov9760_s_parm,
	.g_mbus_fmt = ov9760_g_mbus_fmt,
	.enum_mbus_fmt = ov9760_enum_mbus_fmt,
	.g_frame_interval = ov9760_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov9760_core_ops = {
	.g_chip_ident = ov9760_g_chip_ident,
	.queryctrl = ov9760_queryctrl,
	.g_ctrl = ov9760_g_ctrl,
	.s_ctrl = ov9760_s_ctrl,
	.ioctl = ov9760_ioctl,
	.s_power = ov9760_s_power,
	.init = ov9760_init,
};

static const struct v4l2_subdev_pad_ops ov9760_pad_ops = {
	.enum_mbus_code = ov9760_enum_mbus_code,
	.enum_frame_size = ov9760_enum_frame_size,
	.get_fmt = ov9760_get_pad_format,
	.set_fmt = ov9760_set_pad_format,
};

static const struct v4l2_subdev_ops ov9760_ops = {
	.core = &ov9760_core_ops,
	.video = &ov9760_video_ops,
	.pad = &ov9760_pad_ops,
	.sensor = &ov9760_sensor_ops,
};

static const struct media_entity_operations ov9760_entity_ops = {
	.link_setup = NULL,
};

static int ov9760_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9760_device *dev = to_ov9760_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int ov9760_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ov9760_device *dev;
	int ret;

	gclient = client;
	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ov9760_ops);

	if (client->dev.platform_data) {
		ret = ov9760_s_config(&dev->sd, client->irq,
				client->dev.platform_data);
		if (ret)
			goto out_free;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
	dev->sd.entity.ops = &ov9760_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		ov9760_remove(client);

	return ret;
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}


static const struct i2c_device_id ov9760_id[] = {
	{OV9760_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov9760_id);

static struct i2c_driver ov9760_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV9760_NAME,
	},
	.probe = ov9760_probe,
	.remove = ov9760_remove,
	.id_table = ov9760_id,
};


static __init int init_ov9760(void)
{
	OV9760_LOG(1,"%s %d\n", __func__, __LINE__);
	return i2c_add_driver(&ov9760_driver);
}

static __exit void exit_ov9760(void)
{
	OV9760_LOG(1,"%s %d\n", __func__, __LINE__);
	i2c_del_driver(&ov9760_driver);
}

module_init(init_ov9760);
module_exit(exit_ov9760);

MODULE_DESCRIPTION("A low-level driver for OV9760 sensor");
MODULE_AUTHOR("Yang shunyong <shunyong.yang@intel.com>");
MODULE_LICENSE("GPL");
