/*
 * DLP3430 Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _DLP3430_H
#define _DLP3430_H

extern struct dlp3430 *dlpdata;
int dlp3430_ext_write_byte(u8 reg, u8 data);
int dlp3430_ext_read_byte(u8 reg);
struct dlp3430_int_data {
	u32  data;
};

#define DLP3430_IOC_MAGIC          0x92

#define DLP3430_SET_HIGH          _IO(DLP3430_IOC_MAGIC, 1)
#define DLP3430_SET_LOW           _IO(DLP3430_IOC_MAGIC, 2)

#define DLP3430_WRITE_STRUCT   _IOW(DLP3430_IOC_MAGIC, 3,struct dlp3430_rom_data)
#define DLP3430_WRITE_INT         _IOW(DLP3430_IOC_MAGIC, 4,struct dlp3430_int_data)

#define DLP3430_READ_STRUCT     _IOR(DLP3430_IOC_MAGIC, 5,struct dlp3430_rom_data)
#define DLP3430_READ_INT          _IOR(DLP3430_IOC_MAGIC, 6,struct dlp3430_int_data)
#define DLP3430_SET_BRIGHTNESS         _IOW(DLP3430_IOC_MAGIC, 7,struct dlp3430_int_data)
#define DLP3430_GET_BRIGHTNESS          _IOR(DLP3430_IOC_MAGIC, 8,struct dlp3430_int_data)
#define DLP3430_SET_WB         _IOW(DLP3430_IOC_MAGIC, 9,struct dlp3430_int_data)
#define DLP3430_GET_WB          _IOR(DLP3430_IOC_MAGIC, 10,struct dlp3430_int_data)
#define DLP3430_SET_ANGLE         _IOW(DLP3430_IOC_MAGIC, 11,struct dlp3430_int_data)
#define DLP3430_GET_ANGLE          _IOR(DLP3430_IOC_MAGIC, 12,struct dlp3430_int_data)
#define DLP3430_SET_RESOLUTION         _IOW(DLP3430_IOC_MAGIC, 13,struct dlp3430_int_data)
#define DLP3430_GET_RESOLUTION          _IOR(DLP3430_IOC_MAGIC, 14,struct dlp3430_int_data)
#define DLP3430_SET_ROTATION         _IOW(DLP3430_IOC_MAGIC, 15,struct dlp3430_int_data)
#define DLP3430_GET_ROTATION          _IOR(DLP3430_IOC_MAGIC, 16,struct dlp3430_int_data)
#define DLP3430_GET_TEMPERATURE          _IOR(DLP3430_IOC_MAGIC, 18,struct dlp3430_int_data)
#define DLP3430_GET_DLP_STATUS          _IOR(DLP3430_IOC_MAGIC, 20,struct dlp3430_int_data)
#define DLP3430_GET_LOCAL_BRIGHTNESS          _IOR(DLP3430_IOC_MAGIC, 22,struct dlp3430_int_data)
#define DLP3430_SET_LOCAL_BRIGHTNESS         _IOW(DLP3430_IOC_MAGIC, 21,struct dlp3430_int_data)
#define DLP3430_GET_EDP_STATUS          _IOR(DLP3430_IOC_MAGIC, 24,struct dlp3430_int_data)
#define DLP3430_SET_EDP_ON           _IO(DLP3430_IOC_MAGIC, 25)
struct dlp3430_rom_data {
	u8 addr;
	u8 val;
};


/**
 * struct lp855x_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @device_control : value of DEVICE CONTROL register
 * @initial_brightness : initial value of backlight brightness
 * @period_ns : platform specific pwm period value. unit is nano.
		Only valid when mode is PWM_BASED.
 * @size_program : total size of lp855x_rom_data
 * @rom_data : list of new eeprom/eprom registers
 */
struct dlp3430_platform_data {
	const char *name;
	u8 device_control;
	u8 initial_brightness;
	unsigned int period_ns;
	int size_program;
	struct dlp3430_rom_data *rom_data;
};

#endif
