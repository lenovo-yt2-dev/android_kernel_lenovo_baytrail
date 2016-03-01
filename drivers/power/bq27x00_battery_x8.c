/* BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <linux/switch.h>
#include <linux/interrupt.h>

#include <linux/wakelock.h>
#include <linux/power/bq27x00_battery_x8.h>

#if 1
#define pr_axs(format, args...) printk("<bat_fg> %s: "format, __func__, ##args)
#define fg_dbg(format, args...) printk("<fg> %s: "format, __func__, ##args)
#else
#define pr_axs(format, args...) NULL
#define fg_dbg(format, args...) NULL
#endif
#define DRIVER_VERSION			"1.2.0"

#define G3_FW_VERSION			0x0324
#define L1_600_FW_VERSION		0x0600
#define L1_604_FW_VERSION		0x0604

#define BQ27541_DEV_TYPE 		0x0541
#define BQ27541_FW_VER	 		0x0219
#define BQ27541_HW_VER	 		0x0060

#define  CMD_CONTROL			0x00
/* Subcommands of Control() */
#define CMD_CONTROL_STATUS	0x0000
#define CMD_CONTROL_DEV_TYPE	0x0001
#define CMD_CONTROL_FW_VER		0x0002
#define CMD_CONTROL_HW_VER		0x0003
#define CMD_CONTROL_RESET_DATA		0x0005
#define CMD_CONTROL_PREV_W		0x0007
#define CMD_CONTROL_CHEM_ID		0x0008
#define CMD_CONTROL_BD_OFF		0x0009
#define CMD_CONTROL_CC_OFF		0x000A
#define CMD_CONTROL_CC_OFF_SAVE	0x000B
#define CMD_CONTROL_DF_VER		0x000C
#define CMD_CONTROL_FSLEEP		0x0010
#define CMD_CONTROL_HIBERNATE	0x0011
#define CMD_CONTROL_CLR_HIBER	0x0012
#define CMD_CONTROL_SHUTDOWN	0x0013
#define CMD_CONTROL_CLR_SHUT	0x0014
#define CMD_CONTROL_HDQ_INT		0x0015
#define CMD_CONTROL_CLR_HDQE	0x0016
#define CMD_CONTROL_DF_CHKSUM	0x0017
#define CMD_CONTROL_SEALED		0x0020
#define CMD_CONTROL_IT_EN		0x0021
#define CMD_CONTROL_RESET		0x0041
#define CMD_CONTROL_EXIT_CAL	0x0080
#define CMD_CONTROL_ENTER_CAL	0x0081
#define CMD_CONTROL_OFF_CAL		0x0082

#define CMD_ATRATE				0x02
#define CMD_UF_SOC				0x04
#define CMD_TEMP				0x06
#define CMD_VOLT				0x08
#define CMD_FLAGS				0x0A
#define CMD_NOMINAL_CAP			0x0C
#define CMD_FULL_CAP			0x0E
#define CMD_REMAIN_CAP			0x10
#define CMD_FULL_CHRG_CAP  		0x12
#define CMD_AVER_CURR			0x14
#define CMD_TIME_EMPTY			0x16
#define CMD_FILT_FCC			0x18
#define CMD_STDBY_CURR			0x1A
#define CMD_UNFILT_FCC			0x1C
#define CMD_MAX_LD_CURR			0x1E
#define CMD_UNFILT_REMAIN		0x20
#define CMD_FILT_REMAIN			0x22
#define CMD_AVERAGE_POWER		0x24
#define CMD_INTER_TEMP			0x28
#define CMD_CYCLE_CNT			0x2A
#define CMD_SOC					0x2C
#define CMD_SOH					0x2E
#define CMD_PASS_CHRGE			0x34
#define CMD_DOD0				0x36
#define CMD_SELF_DISCH_CURR		0x38

#define CMD_EXT_PACK_CONFIG		0x3A
#define CMD_EXT_DESIGN_CAP		0x3C
#define CMD_EXT_DAT_FLASH_CLASS	0x3E
#define CMD_EXT_DAT_FLASH_BLK	0x3E
#define CMD_EXT_PACK_CONFIG		0x3A

#define CMD_INVALID		0xFF

#define DEBUG_1HZ_MAX_COUNT			00

#define SYSDOWN_BIT             (1<<1)

static char debug_1hz_buffer[500] = {0,};
static char subclass_buffer[500] = {0,};


enum bq27x00_reg_index {
	BQ27x00_REG_TEMP = 0,
	BQ27x00_REG_INT_TEMP = 1,
	BQ27x00_REG_VOLT = 2,
	BQ27x00_REG_AI = 3,
	BQ27x00_REG_FLAGS = 4,
	BQ27x00_REG_TTE = 5,
	BQ27x00_REG_TTF = 6,
	BQ27x00_REG_TTES = 7,
	BQ27x00_REG_TTECP = 8,
	BQ27x00_REG_NAC = 9,
	BQ27x00_REG_LMD = 10,
	BQ27x00_REG_CC = 11,
	BQ27x00_REG_AE = 12,
	BQ27x00_REG_RSOC = 13,
	BQ27x00_REG_ILMD = 14,
	BQ27x00_REG_SOC = 15,
	BQ27x00_REG_DCAP = 16,
	BQ27x00_REG_CTRL = 17,
	BQ27x00_REG_AR = 18,
	BQ27x00_REG_ARTE = 19,
	BQ27x00_REG_FAC = 20,
	BQ27x00_REG_RM = 21,
	BQ27x00_REG_FCC = 22,
	BQ27x00_REG_STBYI = 23,
	BQ27x00_REG_SOH = 24,
	BQ27x00_REG_INSTI = 25,
	BQ27x00_REG_RSCLE = 26,
	BQ27x00_REG_OC = 27,
	BQ27x00_REG_TRUECAP = 28,
	BQ27x00_REG_TRUEFCC = 29,
	BQ27x00_REG_TRUESOC = 30,
	
	/* TI L1 firmware (v6.03) extra registers */
	BQ27x00_REG_MAX_CURRENT = 31,
	BQ27x00_REG_QPASSED_HIRES_INT = 32,
	BQ27x00_REG_QPASSED_HIRES_FRACTION = 33,

	/*BQ27541*/
	BQ27541_UFSOC = 34,
	BQ27541_FFCC = 35,
	BQ27541_SI = 36,
	BQ27541_UFFCC = 37,
	BQ27541_MLI = 38,
	BQ27541_UFRM = 39,
	BQ27541_FRM = 40,
	BQ27541_AP = 41,
	BQ27541_PCHG = 42,
	BQ27541_DOD0 = 43,
	BQ27541_SDSG = 44,
};

/* BQ27541 register */
static u8 bq27541_regs[] = {
	0x06,  // TEMP 
	0x28,  // INTTEMP
	0x08,  // VOLT
	0x14,  // AI
	0x0A,  // FLAGS
	0x16,  // TTE
	0xFF,  // TTF *
	0xFF,  // TTES *
	0xFF,  // TTECP *
	0x0C,  // NAC
	0xFF,  // LMD *
	0x2A,  // CC
	0xFF,  // AE *
	0xFF,  // RSOC *
	0xFF,  // ILMD *
	0x2C,  // SOC
	0x3C,  // DCAP
	0x00,  // CTRL
	0x02,  // AR
	0xFF,  // ARTE *
	0x0E,  // FAC 
	0x10,  // RM
	0x12,  // FCC
	0xFF,  // STBYI*
	0x2E,  // SOH
	0xFF,  // INSTI *

	0xFF,  // RSCLE *
	0xFF,  // OC *
	0xFF,  // TRUECAP,
	0xFF,  // TRUEFCC,
	0xFF,  // TRUESOC,
	0xFF,  // CURRENT,
	0xFF,  // QPASSED_HIRES_INT,
	0xFF,  // QPASSED_HIRES_FRACTION,

	/*BQ27541*/
	0x04,  // BQ27541_UFSOC,
	0x18,  // BQ27541_FFCC,
	0x1A,  // BQ27541_SI,
	0x1C,  // BQ27541_UFFCC,
	0x1E,  // BQ27541_MLI,
	0x20,  // BQ27541_UFRM,
	0x22,  // BQ27541_FRM,
	0x24,  // BQ27541_AP,
	0x34,  // BQ27541_PCHG,
	0x36,  // BQ27541_DOD0,
	0x38,  // BQ27541_SDSG,
};

#define BQ27000_FLAG_CHGS		BIT(7)
#define BQ27000_FLAG_FC			BIT(5)

#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_FC			BIT(9)

#define BQ27000_RS			20 /* Resistor sense */
extern int chrgState2FuelGauge;
//extern bool ovp_stat;
static int soc_prev = 0,soc_jump = 0,chrg_full_cap = 0;
volatile int rd_count = 0,rd_loop = 1,low_soc_count = 0;
#define CHRG_DONE			(3 << 4)
static int btpsoc_set = 0x32; /*50mAh*/
static int btpsoc_clr = 0x37; /*55mAh*/
static int shutdown_soc = 2;

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27x00_device_info *di, u8 reg, int value,
			bool single);
};

static int bq27x00_dump_dataflash(struct bq27x00_device_info *di);
static int bq27x00_dump_partial_dataflash(struct bq27x00_device_info *di);
static int bq27x00_control_cmd(struct bq27x00_device_info *di, u16 cmd);
static void bq27x00_reset_registers(struct bq27x00_device_info *di);
static int bq27x00_battery_read_control_reg(struct bq27x00_device_info *di);
static int bq27x00_read_block_i2c(struct bq27x00_device_info *di, u8 reg,
	unsigned char *buf, size_t len);
static void bq27x00_btpsoc_set(struct bq27x00_device_info *di);
static void bq27x00_btpsoc_clr(struct bq27x00_device_info *di);
static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single);
static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, int value, bool single);


enum bq27x00_chip { BQ27000, BQ27500 };

struct bq27x00_debug_info {
	int voltage;
	int avg_current;
	int temperature;
	struct timespec timestamp;
};

struct bq27x00_reg_cache {
	int temperature;
	int internal_temp;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int raw_capacity;
	int flags;
	int control;
	int voltage;
	int full_avail_cap;
	int remain_cap;
	int full_charge_cap;
	int average_i;
	int state_of_health;
	int state_of_charge;
	int instant_i;
	int r_scale;
	int true_cap;
	int true_fcc;
	int true_soc;
	int nom_avail_cap;
	int current_now;
	short q_max;
	short q_passed;
	unsigned short DOD0;
	short q_start;
	struct timespec timestamp;
	unsigned short DODfinal;
	short delta_v;
	unsigned short max_current;
	unsigned short q_passed_hires_int;
	unsigned short q_passed_hires_fraction;
	short max_dod_diff;
	short ambient_temp;
	unsigned short regr_dod;
	short regr_res;
	short rnew;
	short dod_diff;
	unsigned short sleeptime;
	short sim_temp;

};

struct bq27x00_partial_data_flash {
	struct timespec timestamp;
	char data_ram[200];
	char subclass_0x52[150];
	char subclass_0x57[120];
	char subclass_0x58[120];
	char subclass_0x5b[120];
	char subclass_0x5c[120];
	char subclass_0x5d[120];
	char subclass_0x5e[120];
};


struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;
	int fake_battery;

	int (*translate_temp)(int temperature);

	unsigned long last_update;
	unsigned long data_flash_update_time;
	struct delayed_work work;
	struct delayed_work debug_work;
	struct delayed_work soc_low_work;

	struct power_supply	bat;

	struct bq27x00_access_methods bus;

	struct bq27x00_debug_info debug_info[DEBUG_1HZ_MAX_COUNT];
	int debug_index;
	int debug_print_interval;

	struct bq27x00_partial_data_flash partial_df;

	struct mutex lock;

	u8 *regs;
	int fw_ver;
	int df_ver;

	struct switch_dev sdev;
	struct wake_lock wake_lock;
	struct wake_lock low_power_lock;

};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW
};

static unsigned int debug_dataflash_interval = 20*60*1000;

static unsigned int poll_interval = 60;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");
/*
 * Common code for BQ27x00 devices
 */

static inline int bq27x00_read(struct bq27x00_device_info *di, int reg_index,
		bool single)
{
	int val;

	/* Reports 0 for invalid/missing registers */
	if (!di || !di->regs || di->regs[reg_index] == CMD_INVALID)
		return 0;

	val = di->bus.read(di, di->regs[reg_index], single);

	return val;
}

static inline int bq27x00_read_reg(struct bq27x00_device_info *di, int reg, bool single)
{
	int val;

	if(reg == CMD_INVALID)
		fg_dbg("BQ27541: CMD_INVALID(0x%04X)\n", reg);
	
	/* Reports 0 for invalid/missing registers */
	if (!di || !di->regs || reg == CMD_INVALID)
		return 0;

	val = di->bus.read(di, reg, single);

	return val;
}

static inline int bq27x00_write(struct bq27x00_device_info *di, int reg_index,
		int value, bool single)
{
	if (!di || !di->regs || di->regs[reg_index] == CMD_INVALID)
		return -1;

	return di->bus.write(di, di->regs[reg_index], value, single);
}

/*
 * Return the battery Raw State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_raw_soc(struct bq27x00_device_info *di)
{
	int rsoc;

	rsoc = bq27x00_read(di, BQ27x00_REG_TRUESOC, false);

	if (rsoc < 0)
		dev_err(di->dev, "error reading raw State-of-Charge\n");

	return rsoc;
}
static int soc_remain_calc (int remin_cap,int chrg_cap_full,int real_soc)
{
	int filter_soc=0;

	if(!chrg_cap_full)
		return real_soc;

	remin_cap*=100; /*change cap from mAh to uAh*/
	filter_soc = remin_cap/chrg_cap_full;
/*
	if(filter_soc == real_soc){
		soc_flag = 0;
		return real_soc;
*/
	if(filter_soc > 100)
		filter_soc = 100;
	printk("%s:%d,%d,%d,%d\n",__func__,
			filter_soc,remin_cap/100,chrg_cap_full,real_soc);
	if(filter_soc == real_soc)
		soc_jump = 0;
	return filter_soc;
}

extern int get_chrg_done(void);
/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc,remin_chrg_cap = 0;
	int chrg_done =0;
	int volt;

	if (di->chip == BQ27500)
		rsoc = bq27x00_read(di, BQ27x00_REG_SOC, false);
	else
		rsoc = bq27x00_read(di, BQ27x00_REG_RSOC, true);

	chrg_done = get_chrg_done();
	chrg_done &= CHRG_DONE;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if(volt > 0 && volt < 4321){
		chrg_done = 0;
	}
	if((di->cache.flags & BQ27500_FLAG_FC) || (chrg_done == CHRG_DONE)){
		if(rsoc != 100){
			chrg_full_cap = di->cache.remain_cap;
			dev_info(di->dev,"remain cap = %d and rsoc = %d\n",chrg_full_cap,rsoc);
			soc_jump = 1;
			//rsoc = 100;
		}else
			soc_jump = 0;
	}

	if(soc_jump == 1){
		remin_chrg_cap = di->cache.remain_cap;
		rsoc = soc_remain_calc(remin_chrg_cap,chrg_full_cap,rsoc);
	}
	if(rsoc > 0){
		soc_prev = rsoc;
		rd_count = 0;
		dev_info(di->dev,"prev soc %d\n",soc_prev);
	}else{
		if(soc_prev > 2)
			rd_loop = 5;
		else
			rd_loop = 0;
		if(rd_count >= rd_loop){
			rd_count = 0;
			dev_info(di->dev,"Low soc %d detect!\n",rsoc);
		}else{
			rd_count++;
			dev_info(di->dev,"Low soc %d prev %d loop %d!\n",rsoc,soc_prev,rd_count);
			rsoc = soc_prev;
		}
	}
	if (rsoc < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_err(di->dev, "error reading nominal available capacity\n");
		return charge;
	}

	if (di->chip == BQ27500)
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (di->chip == BQ27500)
		ilmd = bq27x00_read(di, BQ27x00_REG_DCAP, false);
	else
		ilmd = bq27x00_read(di, BQ27x00_REG_ILMD, true);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

	if (di->chip == BQ27500)
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CC, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading register %02x: %d\n", reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}



static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27500 = di->chip == BQ27500;
	unsigned char block_data[26];
	unsigned char block_addr;
	int block_len;
	struct timespec ts;

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	//dev_info(di->dev,"FLAGS = 0x%04X\n", cache.flags);
	di->cache.flags = cache.flags; //liulc1 add
	di->cache.remain_cap = bq27x00_read(di, BQ27x00_REG_RM, false); //liulc1 add

	if (cache.flags >= 0) {
		getnstimeofday(&ts);
		cache.timestamp = ts;
		cache.capacity = bq27x00_battery_read_rsoc(di);
		cache.raw_capacity = bq27x00_battery_read_raw_soc(di);
		cache.temperature = bq27x00_read(di, BQ27x00_REG_TEMP, false);
		cache.internal_temp = bq27x00_read(di, BQ27x00_REG_INT_TEMP, false);
		cache.time_to_empty = bq27x00_battery_read_time(di, BQ27x00_REG_TTE);
		cache.time_to_empty_avg = bq27x00_battery_read_time(di, BQ27x00_REG_TTES);
		cache.time_to_full = bq27x00_battery_read_time(di, BQ27x00_REG_TTF);
		cache.charge_full = bq27x00_battery_read_lmd(di);
		cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.control = bq27x00_battery_read_control_reg(di);
		cache.voltage = bq27x00_read(di, BQ27x00_REG_VOLT, false);
		cache.nom_avail_cap = bq27x00_read(di, BQ27x00_REG_NAC, false);
		cache.full_avail_cap = bq27x00_read(di, BQ27x00_REG_FAC, false);
		cache.full_charge_cap = bq27x00_read(di, BQ27x00_REG_FCC, false);
		cache.average_i = bq27x00_read(di, BQ27x00_REG_AI, false);
		cache.remain_cap = bq27x00_read(di, BQ27x00_REG_RM, false);
		cache.state_of_health = bq27x00_read(di, BQ27x00_REG_SOH, false);
		cache.instant_i = bq27x00_read(di, BQ27x00_REG_INSTI, false);
		cache.r_scale = bq27x00_read(di, BQ27x00_REG_RSCLE, false);
		cache.true_cap = bq27x00_read(di, BQ27x00_REG_TRUECAP, false);
		cache.true_fcc = bq27x00_read(di, BQ27x00_REG_TRUEFCC, false);
		cache.true_soc = bq27x00_read(di, BQ27x00_REG_TRUESOC, false);

		cache.DOD0 = bq27x00_read(di, BQ27541_DOD0, false);

		block_addr = 0x61;
		block_len  = 11;
		if (bq27x00_read_block_i2c(di, block_addr, block_data, block_len) < 0) {
			dev_err(di->dev,
				"error block reading debug registers @ 0x%x\n",block_addr);
		} else {
			cache.q_max = (short) get_unaligned_le16(block_data+1);
			cache.q_passed = (short) get_unaligned_le16(block_data+3);
			// cache.DOD0 = get_unaligned_le16(block_data+5);
			cache.q_start = (short) get_unaligned_le16(block_data+7);
			cache.DODfinal = get_unaligned_le16(block_data+9);
		}

		if ( di->fw_ver >= L1_604_FW_VERSION ) {
			cache.max_current = bq27x00_read(di, BQ27x00_REG_MAX_CURRENT, false);

			block_addr = 0x24;
			block_len  = 26;
			if (bq27x00_read_block_i2c(di, block_addr, block_data, block_len) < 0) {
				dev_err(di->dev,
					"error block reading debug registers @ 0x%x\n",block_addr);
			} else {
				cache.q_passed_hires_int = get_unaligned_le16(block_data);
				cache.q_passed_hires_fraction = get_unaligned_le16(block_data+3);
				cache.max_dod_diff = (short) get_unaligned_le16(block_data+8);
				cache.ambient_temp = (short) get_unaligned_le16(block_data+10);
				cache.delta_v = (short) get_unaligned_le16(block_data+12);
				cache.regr_dod = get_unaligned_le16(block_data+14);
				cache.regr_res = (short) get_unaligned_le16(block_data+16);
				cache.rnew = (short) get_unaligned_le16(block_data+18);
				cache.dod_diff = (short) get_unaligned_le16(block_data+20);
				cache.sleeptime = get_unaligned_le16(block_data+22);
				cache.sim_temp = (short) get_unaligned_le16(block_data+24);
			}
		}

		if (!is_bq27500)
			cache.current_now = bq27x00_read(di, BQ27x00_REG_AI, false);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27x00_battery_read_ilmd(di);
	dev_info(di->dev,"Battery %d,%d,%d,0x%04X\n",
				cache.remain_cap,
				cache.full_charge_cap,
				(int)((s16)cache.average_i),
				cache.flags);
	}

	/*
	 * For debugging use if debug_print_interval is non zero.
	 */
	if (di->debug_print_interval > 0) {
		int count;
		char dr_buf[200];

		/* Dumps out Data Ram Info */
		count = scnprintf(dr_buf,sizeof(dr_buf),
		       "bq27x00 Ex DR: %ld.%ld,"
		       "0x%04x,%d,%d,0x%04x,%d,%d,%d,%d,%d,%d%%,"
		       "0x%02x,%d,%d%%,%d,%d,%d,%d,%d,%d%%,%d,%d,%u,%d",
		       cache.timestamp.tv_sec,
		       cache.timestamp.tv_nsec/100000000,
		       cache.control,
		       cache.temperature-2732,
		       cache.voltage,
		       cache.flags,
		       cache.nom_avail_cap,
		       cache.full_avail_cap,
		       cache.remain_cap,
		       cache.full_charge_cap,
		       (short)cache.average_i,
		       (cache.state_of_health & 0x00FF),
		       (cache.state_of_health & 0xFF00) >> 8,
		       cache.cycle_count,
		       cache.capacity,
		       (short)cache.instant_i,
		       cache.internal_temp-2732,
		       cache.r_scale,
		       cache.true_cap,
		       cache.true_fcc,
		       cache.true_soc,
		       cache.q_max,
		       cache.q_passed,
		       cache.DOD0,
		       cache.q_start);

		/* For Version 0x604, there is some extra info */
		if ( di->fw_ver >= L1_604_FW_VERSION ) {
			scnprintf(dr_buf+count, sizeof(dr_buf)-count,
				",%d,%u,%u,0x%04x,0x%04x,%d,%d,%u,%d,%d,%d,%u,%d",
				cache.delta_v,
				cache.DODfinal,
				cache.max_current,
				cache.q_passed_hires_int,
				cache.q_passed_hires_fraction,
				cache.max_dod_diff,
				cache.ambient_temp,
				cache.regr_dod,
				cache.regr_res,
				cache.rnew,
				cache.dod_diff,
				cache.sleeptime,
				cache.sim_temp);

		}



		/*
		 * Selectively records Data Flash and Data Ram periodically.
		 * These cached values are dumped out on sysfs read.
		 */
		if(time_after_eq(jiffies,di->data_flash_update_time)) {
			count = scnprintf(di->partial_df.data_ram,
				sizeof(di->partial_df.data_ram),
				"0x%04x %d %d 0x%04x %d %d %d %d %d %d "
				"0x%02x %d %d %d %d %d %d %d %d %d %d %u %d",
				cache.control,
				cache.temperature-2732,
				cache.voltage,
				cache.flags,
				cache.nom_avail_cap,
				cache.full_avail_cap,
				cache.remain_cap,
				cache.full_charge_cap,
				(short)cache.average_i,
				(cache.state_of_health & 0x00FF),
				(cache.state_of_health & 0xFF00) >> 8,
				cache.cycle_count,
				cache.capacity,
				(short)cache.instant_i,
				cache.internal_temp-2732,
				cache.r_scale,
				cache.true_cap,
				cache.true_fcc,
				cache.true_soc,
				cache.q_max,
				cache.q_passed,
				cache.DOD0,
				cache.q_start);

			if ( di->fw_ver >= L1_604_FW_VERSION ) {
				scnprintf(di->partial_df.data_ram+count,
					sizeof(di->partial_df.data_ram)-count,
					" %d %u %u 0x%04x 0x%04x %d %d %u %d %d %d %u %d",
					cache.delta_v,
					cache.DODfinal,
					cache.max_current,
					cache.q_passed_hires_int,
					cache.q_passed_hires_fraction,
					cache.max_dod_diff,
					cache.ambient_temp,
					cache.regr_dod,
					cache.regr_res,
					cache.rnew,
					cache.dod_diff,
					cache.sleeptime,
					cache.sim_temp);

			}

			bq27x00_dump_partial_dataflash(di);
			di->data_flash_update_time =
				jiffies + msecs_to_jiffies(debug_dataflash_interval);
		}
	}

	/* Ignore current_now which is a snapshot of the current battery state
	 * and is likely to be different even between two consecutive reads */
	if (memcmp(&di->cache, &cache, sizeof(cache) - sizeof(int)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	if (wake_lock_active(&di->wake_lock))
		wake_unlock(&di->wake_lock);
	di->last_update = jiffies;

}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);
	int rsoc;

	rsoc = bq27x00_read(di, BQ27x00_REG_SOC, false);
	if(chrgState2FuelGauge < 0 )
		chrgState2FuelGauge = 0;
	if(chrgState2FuelGauge == 0){
		if(rsoc >= 0 && rsoc <= 5){
			bq27x00_btpsoc_set(di);
			poll_interval = 10;
		}
	}else
		poll_interval = 60;

	if (poll_interval > 0) {
		bq27x00_update(di);
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}

/*
 *When detect the battery voltage < 3100 shutdown system.
 *
 */
static void bq27x00_low_shutdown(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, soc_low_work.work);
	int batv,rsoc;
	batv = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	rsoc = bq27x00_read(di, BQ27x00_REG_SOC, false);

	if(chrgState2FuelGauge ==0){
		if(batv <= 3100 || (rsoc >= 0 && rsoc <= shutdown_soc)){
			if(low_soc_count < 10){ /*6s x 10*/
				schedule_delayed_work(&di->soc_low_work, 6 * HZ);
				printk(KERN_ERR "WARNING: Low batv %d mV and Shutdown count %d\n",
							batv,low_soc_count);
				low_soc_count++;
			}else{
				low_soc_count = 0;
				printk(KERN_ERR "WARNING: Low batv %d mV and Shutdown system\n",batv);
				kernel_power_off();
			}
		}
	}else{
		low_soc_count = 0;
		if (wake_lock_active(&di->low_power_lock))
			wake_unlock(&di->low_power_lock);
	}
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int temperature;

	if (di->cache.temperature < 0)
		return di->cache.temperature;

	if (di->chip == BQ27500)
		temperature = di->cache.temperature - 2731;
	else
		temperature = ((di->cache.temperature * 5) - 5463) / 2;

	/* let the board translate the thermistor reading if necessary */
	if (di->translate_temp)
		temperature = di->translate_temp(temperature);

	/*
	 * If the reading indicates missing/malfunctioning battery thermistor,
	 * fall back on the internal temperature reading.
	 */
	if (temperature < -350) {
		static int once = 0;

		if (!once) {
			dev_warn(di->dev, "Battery thermistor missing or malfunctioning, falling back to "
					"gas gauge internal temp\n");
			once = 1;
		}

		if (di->chip == BQ27500)
			temperature = di->cache.internal_temp - 2731;
		else
			temperature = ((di->cache.internal_temp * 5) - 5463) / 2;

		/*
		 * Offset by 20 C since the board will run hotter than the battery.
		 */
		temperature -= 200;
		di->fake_battery = 1;
	} else {
		/* if we ever get a valid reading we must not have a fake battery */
		di->fake_battery = 0;
	}

	val->intval = temperature;

	return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;

	if (di->chip == BQ27500)
	    curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	else
	    curr = di->cache.current_now;

#if 0
	if (curr < 0)
		return curr;
#endif

	if (di->chip == BQ27500) {
		/* bq27500 returns signed value */
		val->intval = (int)((s16)curr) * (0 - 1000);
	} else {
		if (di->cache.flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}

	return 0;
}
static int bq27x00_battery_status(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int status;
/*
	if(ovp_stat){
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		val->intval = status;
		return 0;
	}
*/
	fg_dbg("chrgState2FuelGauge= %d\n",chrgState2FuelGauge);
	if (di->chip == BQ27500) {
		if (di->cache.capacity == 100 && chrgState2FuelGauge == 1)
			status = POWER_SUPPLY_STATUS_FULL;
		else{
			if(chrgState2FuelGauge == 1)
				status = POWER_SUPPLY_STATUS_CHARGING;
			else
				status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_energy(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int ae;

	ae = bq27x00_read(di, BQ27x00_REG_AE, false);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27500)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	val->intval = ae;

	return 0;
}

/*
 * Return the coulumb counter in mAh
 * Positive value means charge is going out of the battery
 * Negative value means charge is going into the battery
 */
static int bq27x00_battery_qpassed(struct bq27x00_device_info *di,
	union power_supply_propval *val) {

	unsigned char q_data[10];
	size_t q_size = 10;

	/* This block of data must be read in one shot.
	 * Even though we are only interested 2 registers.
	 */
	bq27x00_read_block_i2c(di, 0x61, q_data, q_size);
	val->intval = (int) get_unaligned_le16(q_data+3);

	return 0;
}

static int bq27x00_battery_hires_qpassed(struct bq27x00_device_info *di,
	unsigned short *i, unsigned short *f) {

	unsigned short last_i;

	if (!i || !f)
		return -1;

	last_i = bq27x00_read(di, BQ27x00_REG_QPASSED_HIRES_INT, false);

	while (1) {
		*i = bq27x00_read(di, BQ27x00_REG_QPASSED_HIRES_INT, false);
		*f = bq27x00_read(di, BQ27x00_REG_QPASSED_HIRES_FRACTION, false);
		if (*i == last_i)
			break;
		last_i = *i;
	}

	return 0;
}


static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	fg_dbg("psy_name %s, psp %d, val %d\n", psy->name, psp, val->intval);
	fg_dbg("NULL Function\n");
}

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);
/*
	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);
*/
	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:	
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27x00_simple_value(di->cache.raw_capacity, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = bq27x00_battery_qpassed(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->fake_battery) {
			val->intval = 96;
			ret = 0;
		} else {
			ret = bq27x00_simple_value(di->cache.capacity, val);
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27x00_battery_temperature(di, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(1000 * di->cache.true_cap, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(1000 * di->cache.true_fcc, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_battery_energy(di, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27x00_read(di, BQ27x00_REG_SOH, false);
		
		if(ret > 0)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = 0;
					
		break;
	
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

/*
 * Work to record voltage and current at 1Hz.
 */
static void bq27x00_battery_debug_poll(struct work_struct *work)
{
	union power_supply_propval val;
	struct timespec ts;
	int i;
	struct timespec first_ts,time_diff;

	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, debug_work.work);

	mutex_lock(&di->lock);

	getnstimeofday(&ts);

	/* Get Voltage */
	bq27x00_battery_voltage(di, &val);
	di->debug_info[di->debug_index].voltage = val.intval;

	/* Get Current */
	bq27x00_battery_current(di, &val);
	di->debug_info[di->debug_index].avg_current = val.intval;

	/* Get Temperature */
	bq27x00_battery_temperature(di, &val);
	di->debug_info[di->debug_index].temperature= val.intval;

	/* Record Time */
	di->debug_info[di->debug_index].timestamp = ts;

	if (di->debug_print_interval > 0)
		schedule_delayed_work(&di->debug_work, HZ);

	/* Dumps out 1HZ recording of V, I and T at fixed interval */
	if (di->debug_print_interval > 0 &&
	    di->debug_index >= di->debug_print_interval-1) {
		int buffer_used = 0;
		const int num_samples = di->debug_index + 1;
		bool first_sample = true;
		for (i = 0; i < num_samples; i++) {
			if (first_sample) {
				buffer_used += scnprintf(
				    debug_1hz_buffer+buffer_used,
				    sizeof(debug_1hz_buffer) - buffer_used,
				    "bq27x00 1-HZ: %ld.%ld,",
				    di->debug_info[i].timestamp.tv_sec,
				    di->debug_info[i].timestamp.tv_nsec/100000000);
				first_sample = false;
				first_ts = di->debug_info[i].timestamp;
			} else {
				time_diff =
					timespec_sub(di->debug_info[i].timestamp,first_ts);
				buffer_used += scnprintf(
				    debug_1hz_buffer+buffer_used,
				    sizeof(debug_1hz_buffer) - buffer_used,
				    "%ld.%ld,",
				    time_diff.tv_sec,
				    time_diff.tv_nsec/100000000);
			}
			buffer_used += scnprintf(
			    debug_1hz_buffer+buffer_used,
			    sizeof(debug_1hz_buffer) - buffer_used,
			    "%d,%d,%d;",
			    di->debug_info[i].voltage/1000,
			    di->debug_info[i].avg_current/1000,
			    di->debug_info[i].temperature);
		}
		fg_dbg("%s\n", debug_1hz_buffer);
		di->debug_index = 0;
	} else {
		di->debug_index++;
	}

	mutex_unlock(&di->lock);

	return;
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.set_property = bq27x00_battery_set_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	INIT_DELAYED_WORK(&di->debug_work, bq27x00_battery_debug_poll);
	INIT_DELAYED_WORK(&di->soc_low_work, bq27x00_low_shutdown);
	mutex_init(&di->lock);

	/*
	 * Read the battery temp now to prevent races between userspace reading
	 * properties and battery "detection" logic.
	 */
	di->cache.temperature = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	di->cache.internal_temp = bq27x00_read(di, BQ27x00_REG_INT_TEMP, false);

	/*
	 * NOTE: Properties can be read as soon as we register the power supply.
	 */
	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	/* If debug is enabled, force a DR and DF dump on boot */
	if (di->debug_print_interval > 0)
		di->data_flash_update_time = jiffies;

	bq27x00_update(di);
	bq27x00_btpsoc_clr(di);
	bq27x00_btpsoc_set(di);


	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}


/* i2c specific code */
#ifndef CONFIG_BATTERY_BQ27X00_I2C_X8

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, int value, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (!single)
		put_unaligned_le16(value, data);
	else
		data[0] = value;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

static int bq27x00_control_cmd(struct bq27x00_device_info *di, u16 cmd)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[3];
	unsigned char cmd_write[3];
	unsigned char cmd_read[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	cmd_write[0] = 0x0;
	put_unaligned_le16(cmd, cmd_write + 1);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = cmd_write;
	msg[0].len = sizeof(cmd_write);
	msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].buf = cmd_write;
	msg[1].len = 1;
	msg[2].addr = client->addr;
	msg[2].flags = I2C_M_RD;
	msg[2].buf = cmd_read;
	msg[2].len = sizeof(cmd_read);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	ret = get_unaligned_le16(cmd_read);

	return ret;
}

static int bq27x00_read_block_i2c(struct bq27x00_device_info *di, u8 reg,
		unsigned char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

static int bq27x00_battery_reset(struct bq27x00_device_info *di)
{
	dev_info(di->dev, "Gas Gauge Reset\n");

	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_RESET, false);

	msleep(10);

	bq27x00_read_i2c(di, CMD_CONTROL, false);

	msleep(10);

	/* Reset register map based on fw version */
	bq27x00_reset_registers(di);

	return 0;
}


static int bq27x00_battery_read_fw_version(struct bq27x00_device_info *di)
{
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_FW_VER, false);

	msleep(10);

	return bq27x00_read_i2c(di, CMD_CONTROL, false);
}

static int bq27x00_battery_read_control_reg(struct bq27x00_device_info *di)
{
        bq27x00_write_i2c(di, CMD_CONTROL, 0 , false);

        msleep(10);

        return bq27x00_read_i2c(di, CMD_CONTROL, false);
}

static int bq27x00_battery_read_control(struct bq27x00_device_info *di, int sub_cmd)
{
	bq27x00_write_i2c(di, CMD_CONTROL, sub_cmd, false);

	msleep(10);

	return bq27x00_read_i2c(di, CMD_CONTROL, false);
}

static int bq27x00_battery_read_device_type(struct bq27x00_device_info *di)
{
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_DEV_TYPE, false);

	msleep(10);

	return bq27x00_read_i2c(di, CMD_CONTROL, false);
}

static int bq27x00_battery_read_dataflash_version(struct bq27x00_device_info *di)
{
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_DF_VER, false);

	msleep(10);

	return bq27x00_read_i2c(di, CMD_CONTROL, false);
}
#define SLAVE_LATENCY_DELAY 100

static int dump_and_store_subclass(struct bq27x00_device_info *di, u8 subclass, size_t len, char * buf)
{
	int ret;
	size_t i, offset, remaining;
	unsigned char data[64];
	int buffer_used = 0;
	int header_used = 0;
	struct timespec ts;

	getnstimeofday(&ts);

	memset(data, 0x00, sizeof(data));

	//printk("%s: enter subclass=%02x len=%u\n", __func__, subclass, len);

	/* enable block flash control */
	ret = bq27x00_write_i2c(di, 0x61, 0x00, true);
	if (ret) {
		dev_warn(di->dev, "Failed to write (enable block flash control): %d\n", ret);
		goto error;
	}

	msleep(SLAVE_LATENCY_DELAY);

	/* set subclass */
	ret = bq27x00_write_i2c(di, 0x3e, subclass, true);
	if (ret) {
		dev_warn(di->dev, "Failed to write (set subclass 0x%02x): %d\n", subclass, ret);
		goto error;
	}

	offset = 0;
	remaining = len;

	while (remaining > 0) {
		size_t count = remaining < 32 ? remaining : 32;
		buffer_used = 0;

		msleep(SLAVE_LATENCY_DELAY);

		/* set sebclass offset 0x00 */
		ret = bq27x00_write_i2c(di, 0x3f, offset, true);
		if (ret) {
			dev_warn(di->dev, "Failed to write (set subclass offset %d): %d\n", offset, ret);
			goto error;
		}

		msleep(SLAVE_LATENCY_DELAY);

		/* read in subclass block */
		ret = bq27x00_read_block_i2c(di, 0x40, data, count);
		if (ret) {
			dev_warn(di->dev, "Failed to read block count=%d: %d\n", count, ret);
			goto error;
		}

		header_used = scnprintf(
			subclass_buffer+buffer_used,
			sizeof(subclass_buffer) - buffer_used,
			"bq27x00 DF: %ld.%ld subclass=0x%02x len=%02u blk=%u count=%02u: ",
			ts.tv_sec, ts.tv_nsec/100000000,
			subclass, len, offset, count);
		buffer_used += header_used;

		for (i=0; i < count; i++) {
			buffer_used += scnprintf(
				subclass_buffer+buffer_used,
				sizeof(subclass_buffer) - buffer_used,
				"0x%02x ", data[i]);
		}
		printk("%s\n", subclass_buffer);

		remaining -= count;
		offset++;
	}

	/* Record the df dump for a subclass */
	if (buffer_used > 0 && buf != NULL && len < 32) {
		memcpy(buf, subclass_buffer+header_used, buffer_used-header_used);
	}

	return 0;

error:
	return ret;
}

static int dump_subclass(struct bq27x00_device_info *di, u8 subclass, size_t len) {
	return dump_and_store_subclass(di, subclass, len, 0);
}

#define dump_value(name, reg_index) do { \
	int value = bq27x00_read(di, reg_index, false); \
	printk("bq27x00: %s=0x%04x\n", #name, value); \
} while(0)

static int bq27x00_dump_partial_dataflash(struct bq27x00_device_info *di)
{
	int ret;
	struct timespec ts;

	printk("bq27x00: fw version 0x%04x; df version 0x%04x\n",
		di->fw_ver, di->df_ver);

	if(di->fw_ver == L1_600_FW_VERSION || di->fw_ver == L1_604_FW_VERSION) {


		getnstimeofday(&ts);
		di->partial_df.timestamp = ts;

		ret = dump_and_store_subclass(
			di, 0x52, 28, di->partial_df.subclass_0x52);
		ret = dump_and_store_subclass(
			di, 0x57, 20, di->partial_df.subclass_0x57);
		ret = dump_and_store_subclass(
			di, 0x58, 20, di->partial_df.subclass_0x58);
		ret = dump_and_store_subclass(
			di, 0x5b, 20, di->partial_df.subclass_0x5b);
		ret = dump_and_store_subclass(
			di, 0x5c, 20, di->partial_df.subclass_0x5c);
		ret = dump_and_store_subclass(
			di, 0x5d, 20, di->partial_df.subclass_0x5d);
		ret = dump_and_store_subclass(
			di, 0x5e, 20, di->partial_df.subclass_0x5e);

	}

	return ret;

}

static int bq27x00_dump_dataflash(struct bq27x00_device_info *di)
{
	int ret;

	printk("bq27x00: Control=0x%04x\n", bq27x00_control_cmd(di, 0x0000));
	dump_value(Temperature, BQ27x00_REG_TEMP);
	dump_value(Voltage, BQ27x00_REG_VOLT);
	dump_value(Flags, BQ27x00_REG_FLAGS);
	dump_value(NominalAvailableCapacity, BQ27x00_REG_NAC);
	dump_value(FullAvailableCapacity, BQ27x00_REG_FAC);
	dump_value(RemainingCapacity, BQ27x00_REG_RM);
	dump_value(FullChargeCapacity, BQ27x00_REG_FCC);
	dump_value(AverageCurrent, BQ27x00_REG_AI);
	dump_value(StateOfHealth, BQ27x00_REG_SOH);
	dump_value(CycleCount, BQ27x00_REG_CC);
	dump_value(StateOfCharge, BQ27x00_REG_SOC);
	dump_value(OperationConfiguration, BQ27x00_REG_OC);

	/* unseal device */
	ret = bq27x00_write_i2c(di, 0x00, 0x0414, false);
	if (ret) {
		dev_err(di->dev, "Failed to write (unseal part 1): %d\n", ret);
		goto error;
	}

	msleep(SLAVE_LATENCY_DELAY);

	ret = bq27x00_write_i2c(di, 0x00, 0x3672, false);
	if (ret) {
		dev_err(di->dev, "Failed to write (unseal part 2): %d\n", ret);
		goto error;
	}

	msleep(SLAVE_LATENCY_DELAY);

#if 0
	ret = bq27x00_write_i2c(di, 0x00, 0xffff, false);
	if (ret) {
		dev_err(di->dev, "Failed to write (unseal part 3): %d\n", ret);
		goto error;
	}

	ret = bq27x00_write_i2c(di, 0x00, 0xffff, false);
	if (ret) {
		dev_err(di->dev, "Failed to write (unseal part 4): %d\n", ret);
		goto error;
	}
#endif

	ret = dump_subclass(di, 0x40, 4);
	ret = dump_subclass(di, 0x31, 4);
	if (di->fw_ver == G3_FW_VERSION) {
		ret = dump_subclass(di, 0x02, 9);
		ret = dump_subclass(di, 0x20, 5);
		ret = dump_subclass(di, 0x22, 9);
		ret = dump_subclass(di, 0x24, 14);
		ret = dump_subclass(di, 0x30, 26);
		ret = dump_subclass(di, 0x31, 24);
		ret = dump_subclass(di, 0x38, 9);
		ret = dump_subclass(di, 0x40, 17);
		ret = dump_subclass(di, 0x44, 16);
		ret = dump_subclass(di, 0x50, 90);
		ret = dump_subclass(di, 0x51, 13);
		ret = dump_subclass(di, 0x52, 27);
		ret = dump_subclass(di, 0x53, 45);
		ret = dump_subclass(di, 0x54, 45);
		ret = dump_subclass(di, 0x55, 65);
		ret = dump_subclass(di, 0x56, 65);
		ret = dump_subclass(di, 0x57, 19);
		ret = dump_subclass(di, 0x58, 19);
		ret = dump_subclass(di, 0x59, 19);
		ret = dump_subclass(di, 0x5a, 19);
		ret = dump_subclass(di, 0x5b, 19);
		ret = dump_subclass(di, 0x5c, 19);
		ret = dump_subclass(di, 0x5d, 19);
		ret = dump_subclass(di, 0x5e, 19);
		ret = dump_subclass(di, 0x68, 15);
		ret = dump_subclass(di, 0x69, 18);
		ret = dump_subclass(di, 0x6a, 44);
		ret = dump_subclass(di, 0x6b, 18);
		ret = dump_subclass(di, 0x6c, 19);
		ret = dump_subclass(di, 0x6d, 19);
	} else if(di->fw_ver == L1_600_FW_VERSION || di->fw_ver == L1_604_FW_VERSION) {

		ret = dump_subclass(di, 0x02, 10);
		ret = dump_subclass(di, 0x20, 6);
		ret = dump_subclass(di, 0x22, 10);
		ret = dump_subclass(di, 0x24, 15);
		ret = dump_subclass(di, 0x30, 26);
		ret = dump_subclass(di, 0x31, 25);
		ret = dump_subclass(di, 0x38, 10);
		ret = dump_subclass(di, 0x40, 14);
		ret = dump_subclass(di, 0x44, 17);
		ret = dump_subclass(di, 0x50, 79);
		ret = dump_subclass(di, 0x51, 14);
		ret = dump_subclass(di, 0x52, 28);
		ret = dump_subclass(di, 0x53, 46);
		ret = dump_subclass(di, 0x54, 46);
		ret = dump_subclass(di, 0x55, 66);
		ret = dump_subclass(di, 0x56, 66);
		ret = dump_subclass(di, 0x57, 20);
		ret = dump_subclass(di, 0x58, 20);
		ret = dump_subclass(di, 0x59, 20);
		ret = dump_subclass(di, 0x5a, 20);
		ret = dump_subclass(di, 0x5b, 20);
		ret = dump_subclass(di, 0x5c, 20);
		ret = dump_subclass(di, 0x5d, 20);
		ret = dump_subclass(di, 0x5e, 20);
		ret = dump_subclass(di, 0x68, 16);
		ret = dump_subclass(di, 0x69, 19);
		ret = dump_subclass(di, 0x6a, 45);
		ret = dump_subclass(di, 0x6b, 19);
		ret = dump_subclass(di, 0x6c, 20);
		ret = dump_subclass(di, 0x6d, 20);
	}

#if 0
	/* seal device */
	ret = bq27x00_write_i2c(di, 0x00, 0x0020, false);
	if (ret) {
		dev_err(di->dev, "Failed to write (seal part): %d\n", ret);
		goto error;
	}
#endif

	return 0;

error:
	return ret;
}
/*
 *BTSOC set 50mAh and clear was 55mAh.
 *SET_HDQINTEN.
 */
static void bq27x00_btpsoc_set(struct bq27x00_device_info *di)
{
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_HDQ_INT, false);
	msleep(5);
	bq27x00_write_i2c(di, 0x24, btpsoc_set ,false);
	printk(KERN_ERR "<fg>:[0x24] = 0x%04X\n",
			bq27x00_read_i2c(di, 0x24, false));
}
static void bq27x00_btpsoc_clr(struct bq27x00_device_info *di)
{
	bq27x00_write_i2c(di, 0x26, btpsoc_clr ,false);
	printk(KERN_ERR "<fg>:[0x26] = 0x%04X\n",
			bq27x00_read_i2c(di, 0x26, false));
}
static ssize_t show_dump_partial_data_flash(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int count = 0;

	mutex_lock(&di->lock);

	if (di->partial_df.timestamp.tv_sec != 0) {
		count = sprintf(buf, "%d,%d\n%ld.%ld\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
			di->fw_ver,
			di->df_ver,
			di->partial_df.timestamp.tv_sec,
			di->partial_df.timestamp.tv_nsec/100000000,
			di->partial_df.data_ram,
			di->partial_df.subclass_0x52,
			di->partial_df.subclass_0x57,
			di->partial_df.subclass_0x58,
			di->partial_df.subclass_0x5b,
			di->partial_df.subclass_0x5c,
			di->partial_df.subclass_0x5d,
			di->partial_df.subclass_0x5e );
	} else {
		count = sprintf(buf, "none\n");
	}

	mutex_unlock(&di->lock);

	return count;
}

static irqreturn_t soc_int_irq_threaded_handler(int irq, void *arg)
{
	struct bq27x00_device_info *di = arg;
	int flags,rsoc,batv;

	dev_info(di->dev, "soc_int\n");

	/* the actual SysDown event is processed in the normal update path */

	mutex_lock(&di->lock);

	flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	rsoc = bq27x00_read(di, BQ27x00_REG_SOC, false);
	batv = bq27x00_read(di, BQ27x00_REG_VOLT, false);

	if(chrgState2FuelGauge < 0 )
		chrgState2FuelGauge = 0;

	if(chrgState2FuelGauge  == 0 && rsoc <= shutdown_soc ){
		wake_lock(&di->low_power_lock);
		schedule_delayed_work(&di->soc_low_work, 0);
	}
/*
	if (flags & SYSDOWN_BIT) {
		dev_warn(di->dev, "detected SYSDOWN condition, pulsing poweroff switch\n");
		wake_lock(&di->wake_lock);
		switch_set_state(&di->sdev, 0);
		switch_set_state(&di->sdev, 1);
	} else {
		dev_warn(di->dev, "SYSDOWN condition not detected\n");
		switch_set_state(&di->sdev, 0);
		wake_unlock(&di->wake_lock);
	}
*/
	mutex_unlock(&di->lock);
	printk(KERN_ERR "%s:0x%x,%d,%d\n",__func__,flags,rsoc,batv);

	return IRQ_HANDLED;
}

static ssize_t show_dump_data_flash(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	dev_warn(di->dev, "Dump data flash:\n");
	bq27x00_dump_dataflash(di);

	return sprintf(buf, "okay\n");
}

static ssize_t show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int i;

	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_STATUS, false);
	printk("CMD_CONTROL_STATUS		[0x%04X] = 0x%04X\n", CMD_CONTROL_STATUS, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_DEV_TYPE, false);
	printk("CMD_CONTROL_DEV_TYPE	[0x%04X] = 0x%04X\n", CMD_CONTROL_DEV_TYPE, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_FW_VER, false);
	printk("CMD_CONTROL_FW_VER		[0x%04X] = 0x%04X\n", CMD_CONTROL_FW_VER, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_HW_VER, false);
	printk("CMD_CONTROL_HW_VER		[0x%04X] = 0x%04X\n", CMD_CONTROL_HW_VER, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_PREV_W, false);
	printk("CMD_CONTROL_PREV_W		[0x%04X] = 0x%04X\n", CMD_CONTROL_PREV_W, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_CHEM_ID, false);
	printk("CMD_CONTROL_CHEM_ID		[0x%04X] = 0x%04X\n", CMD_CONTROL_CHEM_ID, bq27x00_read_i2c(di, CMD_CONTROL, false));
	bq27x00_write_i2c(di, CMD_CONTROL, CMD_CONTROL_DF_VER, false);
	printk("CMD_CONTROL_DF_VER		[0x%04X] = 0x%04X\n", CMD_CONTROL_DF_VER, bq27x00_read_i2c(di, CMD_CONTROL, false));
	printk("\n");
/*	
	fg_dbg("CMD_ATRATE				[0x%02X] = %d\n", CMD_ATRATE, bq27x00_read_reg(di, CMD_ATRATE, false));
	fg_dbg("CMD_UF_SOC				[0x%02X] = %d\n", CMD_UF_SOC, bq27x00_read_reg(di, CMD_UF_SOC, false));
	fg_dbg("CMD_TEMP				[0x%02X] = %d\n", CMD_TEMP, bq27x00_read_reg(di, CMD_TEMP, false));
	fg_dbg("CMD_VOLT				[0x%02X] = %d\n", CMD_VOLT, bq27x00_read_reg(di, CMD_VOLT, false));
	fg_dbg("CMD_FLAGS				[0x%02X] = %04X\n", CMD_FLAGS, bq27x00_read_reg(di, CMD_FLAGS, false));
	fg_dbg("CMD_NOMINAL_CAP			[0x%02X] = %d\n", CMD_NOMINAL_CAP, bq27x00_read_reg(di, CMD_NOMINAL_CAP, false));
	fg_dbg("CMD_FULL_CAP			[0x%02X] = %d\n", CMD_FULL_CAP, bq27x00_read_reg(di, CMD_FULL_CAP, false));
	fg_dbg("CMD_FULL_CHRG_CAP		[0x%02X] = %d\n", CMD_FULL_CHRG_CAP, bq27x00_read_reg(di, CMD_FULL_CHRG_CAP, false));
	fg_dbg("CMD_AVER_CURR			[0x%02X] = %d\n", CMD_AVER_CURR, bq27x00_read_reg(di, CMD_AVER_CURR, false));
	fg_dbg("CMD_TIME_EMPTY			[0x%02X] = %d\n", CMD_TIME_EMPTY, bq27x00_read_reg(di, CMD_TIME_EMPTY, false));
	fg_dbg("CMD_FILT_FCC			[0x%02X] = %d\n", CMD_FILT_FCC, bq27x00_read_reg(di, CMD_FILT_FCC, false));
	fg_dbg("CMD_STDBY_CURR			[0x%02X] = %d\n", CMD_STDBY_CURR, bq27x00_read_reg(di, CMD_STDBY_CURR, false));
	fg_dbg("CMD_UNFILT_FCC			[0x%02X] = %d\n", CMD_UNFILT_FCC, bq27x00_read_reg(di, CMD_UNFILT_FCC, false));
	fg_dbg("CMD_MAX_LD_CURR			[0x%02X] = %d\n", CMD_MAX_LD_CURR, bq27x00_read_reg(di, CMD_MAX_LD_CURR, false));
	fg_dbg("CMD_UNFILT_REMAIN		[0x%02X] = %d\n", CMD_UNFILT_REMAIN, bq27x00_read_reg(di, CMD_UNFILT_REMAIN, false));
	fg_dbg("CMD_FILT_REMAIN			[0x%02X] = %d\n", CMD_FILT_REMAIN, bq27x00_read_reg(di, CMD_FILT_REMAIN, false));

	fg_dbg("CMD_AVERAGE_POWER		[0x%02X] = %d\n", CMD_AVERAGE_POWER, bq27x00_read_reg(di, CMD_AVERAGE_POWER, false));
	fg_dbg("CMD_INTER_TEMP			[0x%02X] = %d\n", CMD_INTER_TEMP, bq27x00_read_reg(di, CMD_INTER_TEMP, false));
	fg_dbg("CMD_CYCLE_CNT			[0x%02X] = %d\n", CMD_CYCLE_CNT, bq27x00_read_reg(di, CMD_CYCLE_CNT, false));
	fg_dbg("CMD_SOC					[0x%02X] = %d\n", CMD_SOC, bq27x00_read_reg(di, CMD_SOC, false));
	fg_dbg("CMD_SOH					[0x%02X] = %d\n", CMD_SOH, bq27x00_read_reg(di, CMD_SOH, false));
	fg_dbg("CMD_PASS_CHRGE			[0x%02X] = %d\n", CMD_PASS_CHRGE, bq27x00_read_reg(di, CMD_PASS_CHRGE, false));
	fg_dbg("CMD_DOD0				[0x%02X] = %d\n", CMD_DOD0, bq27x00_read_reg(di, CMD_DOD0, false));
	fg_dbg("CMD_SELF_DISCH_CURR		[0x%02X] = %d\n", CMD_SELF_DISCH_CURR, bq27x00_read_reg(di, CMD_SELF_DISCH_CURR, false));
*/
	for(i=0;i<45;i++){
		if(bq27541_regs[i] == 0x0A)
			printk("<fg>:	[0x%02X] = 0x%04X\n",
				bq27541_regs[i],
				bq27x00_read_reg(di, bq27541_regs[i], false));
		else
			printk("<fg>:	[0x%02X] = %d\n",
				bq27541_regs[i],
				bq27x00_read_reg(di, bq27541_regs[i], false));
	}
	printk("\n");

	printk("CMD_EXT_PACK_CONFIG		[0x%02X] = %d\n", CMD_EXT_PACK_CONFIG, bq27x00_read_reg(di, CMD_EXT_PACK_CONFIG, true));
	printk("CMD_EXT_DESIGN_CAP		[0x%02X] = %d\n", CMD_EXT_DESIGN_CAP, bq27x00_read_reg(di, CMD_EXT_DESIGN_CAP, true));
	printk("CMD_EXT_DAT_FLASH_CLASS	[0x%02X] = %d\n", CMD_EXT_DAT_FLASH_CLASS, bq27x00_read_reg(di, CMD_EXT_DAT_FLASH_CLASS, true));
	printk("CMD_EXT_DAT_FLASH_BLK	[0x%02X] = %d\n", CMD_EXT_DAT_FLASH_BLK, bq27x00_read_reg(di, CMD_EXT_DAT_FLASH_BLK, true));
	printk("CMD_EXT_PACK_CONFIG		[0x%02X] = %d\n", CMD_EXT_PACK_CONFIG, bq27x00_read_reg(di, CMD_EXT_PACK_CONFIG, true));
	printk("\n");

	return 1;
}

static ssize_t show_firmware_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_fw_version(di);

	return sprintf(buf, "%d\n", ver);
}

static ssize_t show_dataflash_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_dataflash_version(di);

	return sprintf(buf, "%d\n", ver);
}

static ssize_t show_device_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int dev_type;

	dev_type = bq27x00_battery_read_device_type(di);

	return sprintf(buf, "%d\n", dev_type);
}

static ssize_t show_reset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	bq27x00_battery_reset(di);

	return sprintf(buf, "okay\n");
}

static ssize_t show_qpassed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int count;

	mutex_lock(&di->lock);

	count = sprintf(buf, "%ld.%ld,%d\n",
		di->cache.timestamp.tv_sec,
		di->cache.timestamp.tv_nsec/100000000,
		di->cache.q_passed);

	mutex_unlock(&di->lock);

	return count;
}

static ssize_t show_hires_qpassed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int count;
	int rc;
	unsigned short i, f;

	mutex_lock(&di->lock);
	rc = bq27x00_battery_hires_qpassed(di, &i, &f);
	if (rc == -1) {
		count = sprintf(buf, "error\n");
	} else {
		count = sprintf(buf, "%ld.%ld,%04x,%04x\n",
				di->cache.timestamp.tv_sec,
				di->cache.timestamp.tv_nsec/100000000,
				i,f);
	}
	mutex_unlock(&di->lock);
	return count;
}

static ssize_t show_battery_details(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int count;
	unsigned short DOD0;
	short q_max;
	short q_passed;
	int DODfinal;
	int true_fcc;
	int true_cap;
	struct timespec timestamp;

	/* Read a bunch of data from the cache */
	mutex_lock(&di->lock);
	timestamp = di->cache.timestamp;
	DOD0 = di->cache.DOD0;
	DODfinal = di->cache.DODfinal;
	q_max = di->cache.q_max;
	q_passed = di->cache.q_passed;
	true_fcc = di->cache.true_fcc;
	true_cap = di->cache.true_cap;
	mutex_unlock(&di->lock);

	count = sprintf(buf, "%ld.%ld,%u,%u,%d,%d,%d,%d\n",
			timestamp.tv_sec,
			timestamp.tv_nsec/100000000,
			DOD0, DODfinal, q_max, q_passed,
			true_fcc, true_cap);
	return count;
}

static ssize_t show_debug_print_interval(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",di->debug_print_interval);
}

static ssize_t set_debug_print_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long val;
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	if (strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (val < 0 || val > DEBUG_1HZ_MAX_COUNT) return -EINVAL;
	di->debug_print_interval = val;
	di->debug_index = 0;

	if (di->debug_print_interval)
		schedule_delayed_work(&di->debug_work, 0);

	return count;
}

static DEVICE_ATTR(dump_partial_data_flash, S_IRUGO,show_dump_partial_data_flash, NULL);
static DEVICE_ATTR(dump_data_flash, S_IRUGO, show_dump_data_flash, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(df_version, S_IRUGO, show_dataflash_version, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, show_device_type, NULL);
static DEVICE_ATTR(reset, S_IRUGO, show_reset, NULL);
static DEVICE_ATTR(qpassed, S_IRUGO, show_qpassed, NULL);
static DEVICE_ATTR(qpassed_hires, S_IRUGO, show_hires_qpassed, NULL);
static DEVICE_ATTR(battery_details, S_IRUGO, show_battery_details, NULL);
static DEVICE_ATTR(debug_print_interval, S_IWUSR|S_IRUGO,
		   show_debug_print_interval, set_debug_print_interval);
static DEVICE_ATTR(registers, S_IRUGO, show_registers, NULL);

static struct attribute *bq27x00_attributes[] = {
	&dev_attr_dump_partial_data_flash.attr,
	&dev_attr_dump_data_flash.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_df_version.attr,
	&dev_attr_device_type.attr,
	&dev_attr_reset.attr,
	&dev_attr_qpassed.attr,
	&dev_attr_qpassed_hires.attr,
	&dev_attr_battery_details.attr,
	&dev_attr_debug_print_interval.attr,	
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group bq27x00_attr_group = {
	.name = "bq27x00_group_attri",
	.attrs = bq27x00_attributes,
};

static void bq27x00_reset_registers(struct bq27x00_device_info *di)
{
	/* Get the fw version to determine the register mapping */
	di->fw_ver = bq27x00_battery_read_fw_version(di);
	di->df_ver = bq27x00_battery_read_dataflash_version(di);
	dev_info(di->dev, "Gas Gauge fw version 0x%04x; df version 0x%04x\n",
						di->fw_ver, di->df_ver);

	di->regs = bq27541_regs;
}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	int retval = 0;
	struct bq27x00_platform_data *pdata = client->dev.platform_data;

	fg_dbg("++\n");

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	//name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	name = kasprintf(GFP_KERNEL, "battery");
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27x00_read_i2c;
	di->bus.write = &bq27x00_write_i2c;
	di->debug_print_interval = DEBUG_1HZ_MAX_COUNT;
	di->debug_index = 0;

	if (pdata && pdata->translate_temp)
		di->translate_temp = pdata->translate_temp;
	else
		dev_warn(&client->dev, "fixup func not set, using default thermistor behavior\n");

	bq27x00_reset_registers(di);

	wake_lock_init(&di->wake_lock, WAKE_LOCK_SUSPEND, "battery_wake_lock");
	wake_lock_init(&di->low_power_lock, WAKE_LOCK_SUSPEND, "batt_low_power_wake_lock");

	/* use switch dev reporting to tell userspace to poweroff gracefully */
	di->sdev.name = "poweroff";
	retval = switch_dev_register(&di->sdev);
	if (retval) {
		dev_err(&client->dev, "error registering switch device: %d\n", retval);
		goto batt_failed_3;
	}

	if (bq27x00_powersupply_init(di))
		goto batt_failed_3;

	i2c_set_clientdata(client, di);

	if (pdata->soc_int_irq >= 0) {
		retval = request_threaded_irq(pdata->soc_int_irq, NULL,
				soc_int_irq_threaded_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "soc_int_irq", di);
		if (retval) {
			dev_err(&client->dev, "failed to request threaded irq for soc_int: %d\n", retval);
			goto batt_failed_3;
		}
	}

	retval = sysfs_create_group(&client->dev.kobj, &bq27x00_attr_group);
	if (retval)
		dev_err(&client->dev, "could not create sysfs files\n");

	/* For Debugging:
	 * 1) 1Hz sampling of V, I and T.
	 * 2) Dump partial dataflash at debug_dataflash_interval
	 * 3) Dump additional data ram
	 */
	if (di->debug_print_interval) {
		schedule_delayed_work(&di->debug_work, 0);
		di->data_flash_update_time =
			jiffies + msecs_to_jiffies(debug_dataflash_interval);
	}
	
	fg_dbg("--\n");

	return 0;

batt_failed_3:
	wake_lock_destroy(&di->wake_lock);
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	switch_dev_unregister(&di->sdev);
	wake_lock_destroy(&di->wake_lock);

	kfree(di);

	return 0;
}

static int bq27x00_battery_dump_qpassed(struct bq27x00_device_info *di, char* buf, size_t size)
{
	union power_supply_propval val;
	unsigned short i, f;
	int cnt;

	if (!di || !buf || size == 0)
		return -1;

	bq27x00_battery_qpassed(di,&val);

	cnt = scnprintf(buf, size, "%d mAh", val.intval);

	/* Read hi resolution version if available */
	if (di->fw_ver >= L1_604_FW_VERSION) {
		bq27x00_battery_hires_qpassed(di, &i, &f);
		scnprintf(buf+cnt, size-cnt, " (0x%04x,0x%04x)",i,f);
	}

	return 0;
}

static char *SUSPEND_STR = "suspend";
static char *RESUME_STR = "resume";
static int bq27x00_battery_suspend_resume(struct i2c_client *client, const char *suspend_resume)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);
	char buf[100];
	int ret;

	if (!di) {
		dev_err(di->dev,"Missing device info\n");
		return -EINVAL;
	}

	mutex_lock(&di->lock);

	ret = bq27x00_battery_dump_qpassed(di, buf, sizeof(buf));

	if (di->debug_print_interval > 0) {
		if (suspend_resume == SUSPEND_STR)
			cancel_delayed_work_sync(&di->work);
		else if (suspend_resume == RESUME_STR)
			schedule_delayed_work(&di->debug_work, HZ);
	}

	mutex_unlock(&di->lock);

	if (ret < 0) {
		dev_err(di->dev,"Failed to get Qpassed value!\n");
		return -EIO;
	}
	if (suspend_resume == RESUME_STR){
		wake_lock(&di->wake_lock);
		schedule_delayed_work(&di->work, 0);
	}

	dev_info(di->dev,"Qpassed @%s: %s\n", suspend_resume, buf);

	return 0;
}

static int bq27x00_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return bq27x00_battery_suspend_resume(client, SUSPEND_STR);
}

static int bq27x00_battery_resume(struct i2c_client *client)
{
	return bq27x00_battery_suspend_resume(client, RESUME_STR);
}


static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
	{ "bq27520", BQ27500 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00-battery",
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.suspend = bq27x00_battery_suspend,
	.resume = bq27x00_battery_resume,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#else

static inline int bq27x00_battery_i2c_init(void) { return 0; }
static inline void bq27x00_battery_i2c_exit(void) {};

#endif

/* platform specific code */
#ifndef CONFIG_BATTERY_BQ27X00_PLATFORM_X8
static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
			bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	
	fg_dbg("++\n");

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27000;

	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

static int bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static struct platform_driver bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = bq27000_battery_remove,
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void) { return 0; }
static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;

	fg_dbg("++\n");
	
	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
