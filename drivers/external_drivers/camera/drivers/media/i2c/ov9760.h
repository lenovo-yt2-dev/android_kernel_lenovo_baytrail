/*
 * Support for Sony OV9760 camera sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
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

#ifndef __OV9760_H__
#define __OV9760_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define OV9760_NAME	"ov9760"
#define V4L2_IDENT_OV9760 8245

/* Defines for register writes and register array processing */
#define OV9760_BYTE_MAX	32
#define OV9760_SHORT_MAX	16
#define I2C_MSG_LENGTH		0x2
#define I2C_RETRY_COUNT		5

#define OV9760_READ_MODE	0x3820
#define OV9760_TEST_PATTERN_MODE			0x0601

#define OV9760_HFLIP_BIT	0x1
#define OV9760_VFLIP_BIT	0x2
#define OV9760_VFLIP_OFFSET	1
#define OV9760_IMG_ORIENTATION	0x0101

#define I2C_RETRY_COUNT		5
#define MAX_FMTS		1

#define OV9760_PID_LOW		0x1
#define OV9760_PID_HIGH		0x0
#define OV9760_REV		0x2
#define OV9760_MOD_ID		0x9760

#define OV9760_RES_WIDTH_MAX	1456
#define OV9760_RES_HEIGHT_MAX	1096

#define ISP_PADDING_W 16
#define ISP_PADDING_H 16
#define OV9760_ISP_MAX_WIDTH	(OV9760_RES_WIDTH_MAX - ISP_PADDING_W)
#define OV9760_ISP_MAX_HEIGHT	(OV9760_RES_HEIGHT_MAX - ISP_PADDING_H)

#define OV9760_MAX_GAIN_VALUE 512

#define OV9760_FOCAL_LENGTH_NUM	235	/*2.35mm*/
#define OV9760_FOCAL_LENGTH_DEM	100
#define OV9760_F_NUMBER_DEFAULT_NUM	240
#define OV9760_F_NUMBER_DEM	100

#define OV9760_COARSE_INTEGRATION_TIME_H		0x3500
#define OV9760_COARSE_INTEGRATION_TIME_M		0x3501
#define OV9760_COARSE_INTEGRATION_TIME_L		0x3502
#define OV9760_GLOBAL_GAIN			0x350A

#define OV9760_INTG_BUF_COUNT		2

#define OV9760_VT_PIX_CLK_DIV			0x30b0
#define OV9760_VT_SYS_CLK_DIV			0x30b1
#define OV9760_PRE_PLL_CLK_DIV			0x0304
#define OV9760_PLL_MULTIPLIER_H			0x30b2
#define OV9760_PLL_MULTIPLIER_L 		0x30b3
#define OV9760_PLL_PLL1_PRE_DIV			0x30b4
#define OV9760_MIPI_DIV					0x3010

#define OV9760_OP_PIX_DIV			0x0300
#define OV9760_OP_SYS_DIV			0x0302
#define OV9760_FRAME_LENGTH_LINES		0x0340
#define OV9760_COARSE_INTG_TIME_MIN		0x1004
#define OV9760_FINE_INTG_TIME_MIN		0x1008

#define OV9760_BIN_FACTOR_MAX			1
#define OV9760_MCLK		192

#define OV9760_HORIZONTAL_START_H 0x0344
#define OV9760_VERTICAL_START_H 0x0346
#define OV9760_HORIZONTAL_END_H 0x0348
#define OV9760_VERTICAL_END_H 0x034a
#define OV9760_HORIZONTAL_OUTPUT_SIZE_H 0x034c
#define OV9760_VERTICAL_OUTPUT_SIZE_H 0x034e

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV9760_FOCAL_LENGTH_DEFAULT 0xA60064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV9760_F_NUMBER_DEFAULT 0x1200064

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV9760_F_NUMBER_RANGE 0x1D0a1D0a

#define	v4l2_format_capture_type_entry(_width, _height, \
		_pixelformat, _bytesperline, _colorspace) \
	{\
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,\
		.fmt.pix.width = (_width),\
		.fmt.pix.height = (_height),\
		.fmt.pix.pixelformat = (_pixelformat),\
		.fmt.pix.bytesperline = (_bytesperline),\
		.fmt.pix.colorspace = (_colorspace),\
		.fmt.pix.sizeimage = (_height)*(_bytesperline),\
	}

#define	s_output_format_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps) \
	{\
		.v4l2_fmt = v4l2_format_capture_type_entry(_width, \
			_height, _pixelformat, _bytesperline, \
				_colorspace),\
		.fps = (_fps),\
	}

#define	s_output_format_reg_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps, _reg_setting) \
	{\
		.s_fmt = s_output_format_entry(_width, _height,\
				_pixelformat, _bytesperline, \
				_colorspace, _fps),\
		.reg_setting = (_reg_setting),\
	}

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

enum ov9760_tok_type {
	OV9760_8BIT  = 0x0001,
	OV9760_16BIT = 0x0002,
	OV9760_RMW   = 0x0010,
	OV9760_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	OV9760_TOK_DELAY  = 0xfe00, /* delay token for reg list */
	OV9760_TOK_MASK = 0xfff0
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */

struct s_register_setting {
	u32 reg;
	u32 val;
};

struct s_output_format {
	struct v4l2_format v4l2_fmt;
	int fps;
};

/**
 * struct ov9760_fwreg - Fisare burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ov9760_fwreg {
	enum ov9760_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ov9760_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ov9760_reg {
	enum ov9760_tok_type type;
	union {
		u16 sreg;
		struct ov9760_fwreg *fwreg;
	} reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_ov9760_sensor(x) container_of(x, struct ov9760_device, sd)

#define OV9760_MAX_WRITE_BUF_SIZE	30
struct ov9760_write_buffer {
	u16 addr;
	u8 data[OV9760_MAX_WRITE_BUF_SIZE];
};

struct ov9760_write_ctrl {
	int index;
	struct ov9760_write_buffer buffer;
};


struct ov9760_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	u8 *otp_data;
	int fmt_idx;
	int status;
	int streaming;
	int power;
	int run_mode;
	int vt_pix_clk_freq_mhz;
	u16 sensor_id;
	u16 coarse_itg;
	u16 fine_itg;
	u16 gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	u8 res;
	u8 type;
	u8 sensor_revision;
	s32 odm_exposure_value;
};

struct ov9760_format_struct {
	u8 *desc;
	struct regval_list *regs;
	u32 pixelformat;
};

struct ov9760_resolution {
	u8 *desc;
	const struct ov9760_reg *regs;
	int res;
	int width;
	int height;
	int fps;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	bool used;
	u32 skip_frames;
};

struct ov9760_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};


/************************** settings for ov9760 *************************/
static struct ov9760_reg const ov9760_1456x1096_30fps[] = {
	//1456x1096 30fps
	
	{OV9760_8BIT, {0x0340}, 0x04}, //	;VTS
	{OV9760_8BIT, {0x0341}, 0x7c}, //   ;VTS, 03/05/2012"
	{OV9760_8BIT, {0x0342}, 0x06}, //   ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0343}, 0xDC}, //   ;;c8 ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0344}, 0x00}, //	;x_addr_start
	{OV9760_8BIT, {0x0345}, 0x08}, //   ;x_addr_start, 03/01/2012"
	{OV9760_8BIT, {0x0346}, 0x00}, //	;y_addr_start
	{OV9760_8BIT, {0x0347}, 0x02}, // 	;y_addr_start
	{OV9760_8BIT, {0x0348}, 0x05}, //	;x_addr_end
	{OV9760_8BIT, {0x0349}, 0xdf}, //  	;x_addr_end,    03/01/2012"
	{OV9760_8BIT, {0x034a}, 0x04}, //  	;y_addr_end
	{OV9760_8BIT, {0x034b}, 0x50}, // 	;y_addr_end
	{OV9760_8BIT, {0x3811}, 0x04}, //	;x_offset
	{OV9760_8BIT, {0x3813}, 0x04}, //	;y_offset
	{OV9760_8BIT, {0x034c}, 0x05}, //	;x_output_size
	{OV9760_8BIT, {0x034d}, 0xb0}, // 	;x_output_size
	{OV9760_8BIT, {0x034e}, 0x04}, // 	;y_output_size
	{OV9760_8BIT, {0x034f}, 0x48}, //  	;y_output_size
	{OV9760_8BIT, {0x0383}, 0x01}, //	;x_odd_inc
	{OV9760_8BIT, {0x0387}, 0x01}, //	;y_odd_inc
	{OV9760_8BIT, {0x3820}, 0x00}, //	;V bin
	{OV9760_8BIT, {0x3821}, 0x00}, //	;H bin
	
	{OV9760_8BIT, {0x30b0}, 0x0a}, // 	;v05
	{OV9760_8BIT, {0x30b1}, 0x01}, //
	{OV9760_8BIT, {0x30b2}, 0x00}, //
	{OV9760_8BIT, {0x30b3}, 0x3F}, // 	;PLL control
	{OV9760_8BIT, {0x30b4}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3010}, 0x81}, //
	
	{OV9760_8BIT, {0x3090}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3091}, 0x32}, // 	;PLL control
	{OV9760_8BIT, {0x3092}, 0x02}, //	;
	{OV9760_8BIT, {0x3093}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3094}, 0x00}, //	;PLL control
	{OV9760_8BIT, {0x5781}, 0x17}, //	add by wdy for spark
	{OV9760_8BIT, {0x5792}, 0x00}, //	add by wdy for spark
	{OV9760_TOK_TERM, {0}, 0}
};
static struct ov9760_reg const ov9760_1296x736_30fps[] = {
	// 1296x736 30fps
	
	{OV9760_8BIT, {0x0340}, 0x04}, //	;VTS
	{OV9760_8BIT, {0x0341}, 0x7c}, //  ;VTS, 03/05/2012"
	{OV9760_8BIT, {0x0342}, 0x06}, //  ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0343}, 0xDC}, //  ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0344}, 0x00}, //	;x_addr_start
	{OV9760_8BIT, {0x0345}, 0x08}, //  ;x_addr_start, 03/01/2012"
	{OV9760_8BIT, {0x0346}, 0x00}, //	;y_addr_start
	{OV9760_8BIT, {0x0347}, 0x02}, // 	;y_addr_start
	{OV9760_8BIT, {0x0348}, 0x05}, //	;x_addr_end
	{OV9760_8BIT, {0x0349}, 0xdf}, //  ;x_addr_end,    03/01/2012"
	{OV9760_8BIT, {0x034a}, 0x04}, //  ;y_addr_end
	{OV9760_8BIT, {0x034b}, 0x50}, // 	;y_addr_end
	{OV9760_8BIT, {0x3811}, 0x04}, //	;x_offset
	{OV9760_8BIT, {0x3813}, 0x04}, //	;y_offset
	{OV9760_8BIT, {0x034c}, 0x05}, //	;x_output_size
	{OV9760_8BIT, {0x034d}, 0x10}, // 	;x_output_size
	{OV9760_8BIT, {0x034e}, 0x02}, // 	;y_output_size
	{OV9760_8BIT, {0x034f}, 0xe0}, //  ;y_output_size
	{OV9760_8BIT, {0x0383}, 0x01}, //	;x_odd_inc
	{OV9760_8BIT, {0x0387}, 0x01}, //	;y_odd_inc
	{OV9760_8BIT, {0x3820}, 0x00}, //	;V bin
	{OV9760_8BIT, {0x3821}, 0x00}, //	;H bin
								
	{OV9760_8BIT, {0x30b0}, 0x0a}, // 	;v05
	{OV9760_8BIT, {0x30b1}, 0x01}, //
	{OV9760_8BIT, {0x30b2}, 0x00}, //
	{OV9760_8BIT, {0x30b3}, 0x3F}, // 	;PLL control
	{OV9760_8BIT, {0x30b4}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3010}, 0x81}, //
								
	{OV9760_8BIT, {0x3090}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3091}, 0x32}, // 	;PLL control
	{OV9760_8BIT, {0x3092}, 0x02}, //	;
	{OV9760_8BIT, {0x3093}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3094}, 0x00}, //	;PLL control
	{OV9760_8BIT, {0x5781}, 0x17}, //	add by wdy for spark
	{OV9760_8BIT, {0x5792}, 0x00}, //	add by wdy for spark
	{OV9760_TOK_TERM, {0}, 0}
};

static struct ov9760_reg const ov9760_1216x736_30fps[] = {
	// 1216x736 30fps
	
	{OV9760_8BIT, {0x0340}, 0x04}, //	;VTS
	{OV9760_8BIT, {0x0341}, 0x7c}, //  ;VTS, 03/05/2012"
	{OV9760_8BIT, {0x0342}, 0x06}, //  ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0343}, 0xDC}, //  ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0344}, 0x00}, //	;x_addr_start
	{OV9760_8BIT, {0x0345}, 0x08}, //  ;x_addr_start, 03/01/2012"
	{OV9760_8BIT, {0x0346}, 0x00}, //	;y_addr_start
	{OV9760_8BIT, {0x0347}, 0x02}, // 	;y_addr_start
	{OV9760_8BIT, {0x0348}, 0x05}, //	;x_addr_end
	{OV9760_8BIT, {0x0349}, 0xdf}, //  ;x_addr_end,    03/01/2012"
	{OV9760_8BIT, {0x034a}, 0x04}, //  ;y_addr_end
	{OV9760_8BIT, {0x034b}, 0x50}, // 	;y_addr_end
	{OV9760_8BIT, {0x3811}, 0x04}, //	;x_offset
	{OV9760_8BIT, {0x3813}, 0x04}, //	;y_offset
	{OV9760_8BIT, {0x034c}, 0x04}, //	;x_output_size
	{OV9760_8BIT, {0x034d}, 0xc0}, // 	;x_output_size
	{OV9760_8BIT, {0x034e}, 0x02}, // 	;y_output_size
	{OV9760_8BIT, {0x034f}, 0xe0}, //  ;y_output_size
	{OV9760_8BIT, {0x0383}, 0x01}, //	;x_odd_inc
	{OV9760_8BIT, {0x0387}, 0x01}, //	;y_odd_inc
	{OV9760_8BIT, {0x3820}, 0x00}, //	;V bin
	{OV9760_8BIT, {0x3821}, 0x00}, //	;H bin
								
	{OV9760_8BIT, {0x30b0}, 0x0a}, // 	;v05
	{OV9760_8BIT, {0x30b1}, 0x01}, //
	{OV9760_8BIT, {0x30b2}, 0x00}, //
	{OV9760_8BIT, {0x30b3}, 0x3F}, // 	;PLL control
	{OV9760_8BIT, {0x30b4}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3010}, 0x81}, //
								
	{OV9760_8BIT, {0x3090}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3091}, 0x32}, // 	;PLL control
	{OV9760_8BIT, {0x3092}, 0x02}, //	;
	{OV9760_8BIT, {0x3093}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3094}, 0x00}, //	;PLL control
	{OV9760_8BIT, {0x5781}, 0x17}, //	add by wdy for spark
	{OV9760_8BIT, {0x5792}, 0x00}, //	add by wdy for spark
	{OV9760_TOK_TERM, {0}, 0}
};

static struct ov9760_reg const ov9760_896x736_30fps[] = {
	//896x736 30fps
	
	{OV9760_8BIT, {0x0340}, 0x04}, //	;VTS
	{OV9760_8BIT, {0x0341}, 0x7c}, //   ;VTS, 03/05/2012"
	{OV9760_8BIT, {0x0342}, 0x06}, //   ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0343}, 0xDC}, //   ;HTS, 03/05/2012"
	{OV9760_8BIT, {0x0344}, 0x00}, //	;x_addr_start
	{OV9760_8BIT, {0x0345}, 0x08}, //   ;x_addr_start, 03/01/2012"
	{OV9760_8BIT, {0x0346}, 0x00}, //	;y_addr_start
	{OV9760_8BIT, {0x0347}, 0x02}, // 	;y_addr_start
	{OV9760_8BIT, {0x0348}, 0x05}, //	;x_addr_end
	{OV9760_8BIT, {0x0349}, 0xdf}, //  	;x_addr_end,    03/01/2012"
	{OV9760_8BIT, {0x034a}, 0x04}, //  	;y_addr_end
	{OV9760_8BIT, {0x034b}, 0x50}, // 	;y_addr_end
	{OV9760_8BIT, {0x3811}, 0x04}, //	;x_offset
	{OV9760_8BIT, {0x3813}, 0x04}, //	;y_offset
	{OV9760_8BIT, {0x034c}, 0x03}, //	;x_output_size
	{OV9760_8BIT, {0x034d}, 0x80}, // 	;x_output_size
	{OV9760_8BIT, {0x034e}, 0x02}, // 	;y_output_size
	{OV9760_8BIT, {0x034f}, 0xe0}, //  	;y_output_size
	{OV9760_8BIT, {0x0383}, 0x01}, //	;x_odd_inc
	{OV9760_8BIT, {0x0387}, 0x01}, //	;y_odd_inc
	{OV9760_8BIT, {0x3820}, 0x00}, //	;V bin
	{OV9760_8BIT, {0x3821}, 0x00}, //	;H bin
								
	{OV9760_8BIT, {0x30b0}, 0x0a}, // 	;v05
	{OV9760_8BIT, {0x30b1}, 0x01}, //
	{OV9760_8BIT, {0x30b2}, 0x00}, //
	{OV9760_8BIT, {0x30b3}, 0x3F}, // 	;PLL control
	{OV9760_8BIT, {0x30b4}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3010}, 0x81}, //
								
	{OV9760_8BIT, {0x3090}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3091}, 0x32}, // 	;PLL control
	{OV9760_8BIT, {0x3092}, 0x02}, //	;
	{OV9760_8BIT, {0x3093}, 0x02}, //	;PLL control
	{OV9760_8BIT, {0x3094}, 0x00}, //	;PLL control
	{OV9760_8BIT, {0x5781}, 0x17}, //	add by wdy for spark
	{OV9760_8BIT, {0x5792}, 0x00}, //	add by wdy for spark
	{OV9760_TOK_TERM, {0}, 0}
};

/* TODO settings of preview/still/video will be updated with new use case */
struct ov9760_resolution ov9760_res_preview[] = {
	{
		.desc = "ov9760_1456x1096_30fps",
		.regs = ov9760_1456x1096_30fps,
		.width = 1456,
		.height = 1096,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_1296x736_30fps",
		.regs = ov9760_1296x736_30fps,
		.width = 1296,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_1216x736_30fps",
		.regs = ov9760_1216x736_30fps,
		.width = 1216,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_896x736_30fps",
		.regs = ov9760_896x736_30fps,
		.width = 896,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
};
#define N_RES_PREVIEW (ARRAY_SIZE(ov9760_res_preview))

struct ov9760_resolution ov9760_res_still[] = {
	{
		.desc = "ov9760_1456x1096_30fps",
		.regs = ov9760_1456x1096_30fps,
		.width = 1456,
		.height = 1096,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_1296x736_30fps",
		.regs = ov9760_1296x736_30fps,
		.width = 1296,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_1216x736_30fps",
		.regs = ov9760_1216x736_30fps,
		.width = 1216,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		.desc = "ov9760_896x736_30fps",
		.regs = ov9760_896x736_30fps,
		.width = 896,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
};
#define N_RES_STILL (ARRAY_SIZE(ov9760_res_still))

struct ov9760_resolution ov9760_res_video[] = {
	{
		.desc = "ov9760_1456x1096_30fps",
		.regs = ov9760_1456x1096_30fps,
		.width = 1456,
		.height = 1096,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 2,
	},
	{
		.desc = "ov9760_1296x736_30fps",
		.regs = ov9760_1296x736_30fps,
		.width = 1296,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 2,
	},
	{
		.desc = "ov9760_1216x736_30fps",
		.regs = ov9760_1216x736_30fps,
		.width = 1216,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 2,
	},
	{
		.desc = "ov9760_896x736_30fps",
		.regs = ov9760_896x736_30fps,
		.width = 896,
		.height = 736,
		.fps = 30,
		.pixels_per_line = 0x06dc, /* consistent with regs arrays */
		.lines_per_frame = 0x047c, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 2,
	},
};
#define N_RES_VIDEO (ARRAY_SIZE(ov9760_res_video))

struct ov9760_resolution *ov9760_res = ov9760_res_preview;
static int N_RES = N_RES_PREVIEW;


static struct ov9760_reg const ov9760_suspend[] = {
	 {OV9760_8BIT,  {0x0100}, 0x0},
	 {OV9760_TOK_TERM, {0}, 0}
};

static struct ov9760_reg const ov9760_streaming[] = {
	 {OV9760_8BIT,  {0x0100}, 0x1},
	 {OV9760_TOK_TERM, {0}, 0}
};

static struct ov9760_reg const ov9760_param_hold[] = {
	{OV9760_8BIT,  {0x3208}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{OV9760_TOK_TERM, {0}, 0}
};

static struct ov9760_reg const ov9760_param_update[] = {
	{OV9760_8BIT,  {0x3208}, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{OV9760_TOK_TERM, {0}, 0}
};

/* init settings */
static struct ov9760_reg const ov9760_init_config[] = {
	//Common settings for all test items			
	{OV9760_8BIT, {0x0103}, 0x01}, //S/W reset
	// insert 10ms delay here    
	{OV9760_TOK_DELAY, {0}, 0x0a}, //delay 10ms
	{OV9760_8BIT, {0x0340}, 0x04}, //VTS
	{OV9760_8BIT, {0x0341}, 0x7C}, //VTS, 03/05/2012"
	{OV9760_8BIT, {0x0342}, 0x06}, //HTS, 03/05/2012"
	{OV9760_8BIT, {0x0343}, 0xDC}, //HTS, 03/05/2012"
	{OV9760_8BIT, {0x0344}, 0x00}, //x_addr_start
	{OV9760_8BIT, {0x0345}, 0x08}, //x_addr_start, 03/01/2012"
	{OV9760_8BIT, {0x0346}, 0x00}, //y_addr_start
	{OV9760_8BIT, {0x0347}, 0x02}, //y_addr_start
	{OV9760_8BIT, {0x0348}, 0x05}, //x_addr_end
	{OV9760_8BIT, {0x0349}, 0xdf}, //x_addr_end, 03/01/2012"
	{OV9760_8BIT, {0x034a}, 0x04}, //y_addr_end
	{OV9760_8BIT, {0x034b}, 0x50}, //y_addr_end
	{OV9760_8BIT, {0x3811}, 0x04}, //x_offset
	{OV9760_8BIT, {0x3813}, 0x04}, //y_offset
	{OV9760_8BIT, {0x034c}, 0x05}, //x_output_size
	{OV9760_8BIT, {0x034d}, 0xb0}, //x_output_size
	{OV9760_8BIT, {0x034e}, 0x04}, //y_output_size
	{OV9760_8BIT, {0x034f}, 0x48}, //y_output_size
	{OV9760_8BIT, {0x0383}, 0x01}, //x_odd_inc
	{OV9760_8BIT, {0x0387}, 0x01}, //y_odd_inc
	{OV9760_8BIT, {0x3820}, 0x00}, //V bin
	{OV9760_8BIT, {0x3821}, 0x00}, //H bin
	{OV9760_8BIT, {0x3660}, 0x80}, //Analog control, 03/01/2012
	{OV9760_8BIT, {0x3680}, 0xf4}, //Analog control, 03/01/2012
	{OV9760_8BIT, {0x0100}, 0x00}, //Mode select - stop streaming
	{OV9760_8BIT, {0x0101}, 0x01}, //Orientation
	{OV9760_8BIT, {0x3002}, 0x80}, //IO control
	{OV9760_8BIT, {0x3012}, 0x08}, //MIPI control
	{OV9760_8BIT, {0x3014}, 0x04}, //MIPI control
	{OV9760_8BIT, {0x3022}, 0x02}, //Analog control
	{OV9760_8BIT, {0x3023}, 0x0f}, //Analog control
	{OV9760_8BIT, {0x3080}, 0x00}, //PLL control
	{OV9760_8BIT, {0x3090}, 0x02}, //PLL control
	{OV9760_8BIT, {0x3091}, 0x32}, //PLL control
	{OV9760_8BIT, {0x3092}, 0x02}, //
	{OV9760_8BIT, {0x3093}, 0x02}, //PLL control
	{OV9760_8BIT, {0x3094}, 0x00}, //PLL control
	{OV9760_8BIT, {0x3095}, 0x00}, //PLL control
	{OV9760_8BIT, {0x3096}, 0x01}, //PLL control
	{OV9760_8BIT, {0x3097}, 0x00}, //PLL control
	{OV9760_8BIT, {0x3098}, 0x04}, //PLL control
	{OV9760_8BIT, {0x3099}, 0x14}, //PLL control
	{OV9760_8BIT, {0x309a}, 0x03}, //PLL control
	{OV9760_8BIT, {0x309c}, 0x00}, //PLL control
	{OV9760_8BIT, {0x309d}, 0x00}, //PLL control
	{OV9760_8BIT, {0x309e}, 0x01}, //PLL control
	{OV9760_8BIT, {0x309f}, 0x00}, //PLL control
	{OV9760_8BIT, {0x30a2}, 0x01}, //PLL control
	{OV9760_8BIT, {0x30b0}, 0x0a}, //v05
	{OV9760_8BIT, {0x30b3}, 0x3F}, //PLL control
	{OV9760_8BIT, {0x30b4}, 0x02}, //PLL control
	{OV9760_8BIT, {0x30b5}, 0x00}, //PLL control
	{OV9760_8BIT, {0x3503}, 0x27}, //Auto gain/exposure, //set as 0x17 become manual mode"
	{OV9760_8BIT, {0x3509}, 0x10}, //AEC control
	{OV9760_8BIT, {0x3600}, 0x7c}, //Analog control
	{OV9760_8BIT, {0x3621}, 0xb8}, //v04
	{OV9760_8BIT, {0x3622}, 0x23}, //Analog control
	{OV9760_8BIT, {0x3631}, 0xe2}, //Analog control
	{OV9760_8BIT, {0x3634}, 0x03}, //Analog control
	{OV9760_8BIT, {0x3662}, 0x14}, //Analog control
	{OV9760_8BIT, {0x366b}, 0x03}, //Analog control
	{OV9760_8BIT, {0x3682}, 0x82}, //Analog control
	{OV9760_8BIT, {0x3705}, 0x20}, //
	{OV9760_8BIT, {0x3708}, 0x64}, //
	{OV9760_8BIT, {0x371b}, 0x60}, //Sensor control
	{OV9760_8BIT, {0x3732}, 0x40}, //Sensor control
	{OV9760_8BIT, {0x3745}, 0x00}, //Sensor control
	{OV9760_8BIT, {0x3746}, 0x18}, //Sensor control
	{OV9760_8BIT, {0x3780}, 0x2a}, //Sensor control
	{OV9760_8BIT, {0x3781}, 0x8c}, //Sensor control
	{OV9760_8BIT, {0x378f}, 0xf5}, //Sensor control
	{OV9760_8BIT, {0x3823}, 0x37}, //Internal timing control
	{OV9760_8BIT, {0x383d}, 0x88}, //Adjust starting black row for BLC calibration to avoid FIFO empty condition, 03/01/2012"
	{OV9760_8BIT, {0x4000}, 0x23}, //BLC control, 03/06/2012, disable DCBLC for production test"
	{OV9760_8BIT, {0x4001}, 0x04}, //BLC control
	{OV9760_8BIT, {0x4002}, 0x45}, //BLC control
	{OV9760_8BIT, {0x4004}, 0x08}, //BLC control
	{OV9760_8BIT, {0x4005}, 0x40}, //BLC for flashing
	{OV9760_8BIT, {0x4006}, 0x40}, //BLC control
	{OV9760_8BIT, {0x4009}, 0x40}, //BLC
	{OV9760_8BIT, {0x404F}, 0x8F}, //BLC control to improve black level fluctuation, 03/01/2012"
	{OV9760_8BIT, {0x4058}, 0x44}, //BLC control
	{OV9760_8BIT, {0x4101}, 0x32}, //BLC control
	{OV9760_8BIT, {0x4102}, 0xa4}, //BLC control
	{OV9760_8BIT, {0x4520}, 0xb0}, //For full res
	{OV9760_8BIT, {0x4580}, 0x08}, //Bypassing HDR gain latch, 03/01/2012"
	{OV9760_8BIT, {0x4582}, 0x00}, //Bypassing HDR gain latch, 03/01/2012"
	{OV9760_8BIT, {0x4307}, 0x30}, //MIPI control
	{OV9760_8BIT, {0x4605}, 0x00}, //VFIFO control v04 updated
	{OV9760_8BIT, {0x4608}, 0x02}, //VFIFO control v04 updated
	{OV9760_8BIT, {0x4609}, 0x00}, //VFIFO control v04 updated
	{OV9760_8BIT, {0x4801}, 0x0f}, //MIPI control
	{OV9760_8BIT, {0x4819}, 0xB6}, //MIPI control v05 updated
	{OV9760_8BIT, {0x4837}, 0x21}, //MIPI control
	{OV9760_8BIT, {0x4906}, 0xff}, //Internal timing control
	{OV9760_8BIT, {0x4d00}, 0x04}, //Temperature sensor
	{OV9760_8BIT, {0x4d01}, 0x4b}, //Temperature sensor
	{OV9760_8BIT, {0x4d02}, 0xfe}, //Temperature sensor
	{OV9760_8BIT, {0x4d03}, 0x09}, //Temperature sensor
	{OV9760_8BIT, {0x4d04}, 0x1e}, //Temperature sensor
	{OV9760_8BIT, {0x4d05}, 0xb7}, //Temperature sensor

	{OV9760_8BIT, {0x5000}, 0x06}, //Turn off LSC
	{OV9760_8BIT, {0x5180}, 0x04}, //Manual White Balance
	{OV9760_8BIT, {0x5181}, 0x00},
	{OV9760_8BIT, {0x5182}, 0x04},
	{OV9760_8BIT, {0x5183}, 0x00},
	{OV9760_8BIT, {0x5184}, 0x04},
	{OV9760_8BIT, {0x5185}, 0x00},
	{OV9760_8BIT, {0x5186}, 0x01},
	{OV9760_8BIT, {0x5002}, 0x41},
	{OV9760_8BIT, {0x5781}, 0x17}, //	add by wdy for spark
	{OV9760_8BIT, {0x5792}, 0x00}, //	add by wdy for spark
	{OV9760_TOK_TERM, {0}, 0}
};


static int
ov9760_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val);
static int ov9760_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val);
static int ov9760_s_stream(struct v4l2_subdev *sd, int enable);
#endif

