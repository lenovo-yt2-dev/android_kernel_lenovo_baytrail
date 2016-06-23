/*
 * include/media/sy7802.h
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
#ifndef _SY7802_H_
#define _SY7802_H_

#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>

#define SY7802_NAME    "sy7802"

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
/***************add by wdy*******************************/
#define SY7802_DEFAULT_TIMEOUT          512U
#define SY7802_MIN_TIMEOUT              32U
#define SY7802_MAX_TIMEOUT              1024U
#define SY7802_TIMEOUT_STEPSIZE         32U

#define SY7802_MIN_PERCENT                   0U
#define SY7802_MAX_PERCENT                   100U
#define SY7802_FLASH_MAX_LVL   				 0x0F 		//900mA * 2 MAX
#define SY7802_TORCH_MAX_LVL   				 0x07 		//225mA * 2 MAX
#define SY7802_CLAMP_PERCENTAGE(val) \
	clamp(val, SY7802_MIN_PERCENT, SY7802_MAX_PERCENT)

#define SY7802_FLASH_STEP	\
	((100ul*(SY7802_MAX_PERCENT)+((SY7802_FLASH_MAX_LVL)>>1))/((SY7802_FLASH_MAX_LVL)))

#define SY7802_TORCH_STEP	\
	((100ul*(SY7802_MAX_PERCENT)+((SY7802_TORCH_MAX_LVL)>>1))/((SY7802_TORCH_MAX_LVL)))

#define SY7802_FLASH_DEFAULT_BRIGHTNESS \
	SY7802_VALUE_TO_PERCENT(13, SY7802_FLASH_STEP)

#define SY7802_TORCH_DEFAULT_BRIGHTNESS \
	SY7802_VALUE_TO_PERCENT(2, SY7802_TORCH_STEP)

#define SY7802_VALUE_TO_PERCENT(v, step)     (((((unsigned long)(v))*(step))+50)/100)
#define SY7802_PERCENT_TO_VALUE(p, step)     (((((unsigned long)(p))*100)+(step>>1))/(step))
/*************add by wdy*********************************/
/*
 * sy7802_platform_data - Flash controller platform data
 */
struct sy7802_platform_data {
	int gpio_torch;
	int gpio_strobe;
	int gpio_reset;
	int gpio_hold;
	
	unsigned int current_limit;
	unsigned int envm_tx2;
	unsigned int tx2_polarity;
	unsigned int flash_current_limit;
	unsigned int disable_tx2;
};


#endif /* _SY7802_H_ */
