/*
 *  m1120.c - Linux kernel modules for hall switch 
 *
 *  Copyright (C) 2013 Seunghwan Park <seunghwan.park@magnachip.com>
 *  Copyright (C) 2014 MagnaChip Semiconductor.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/m1120.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

///////////
//
// struct
/*
typedef sturct {
	int type;
	int min;
	int max;
} m1120_sts_cfg_t;
stscfg
stscfg_init
*/

/* ********************************************************* */
/* customer config */ 
/* ********************************************************* */
//#define M1120_DBG_ENABLE					// for debugging
#define M1120_DETECTION_MODE				M1120_DETECTION_MODE_INTERRUPT // M1120_DETECTION_MODE_POLLING
#define M1120_INTERRUPT_TYPE				M1120_VAL_INTSRS_INTTYPE_BESIDE
#define M1120_SENSITIVITY_TYPE				M1120_VAL_INTSRS_SRS_10BIT_0_017mT
#define M1120_PERSISTENCE_COUNT				M1120_VAL_PERSINT_COUNT(5)
#define M1120_OPERATION_FREQUENCY			M1120_VAL_OPF_FREQ_10HZ
#define M1120_OPERATION_RESOLUTION			M1120_VAL_OPF_BIT_10
#define M1120_DETECT_RANGE_HIGH				(60)
#define M1120_DETECT_RANGE_LOW				(50)
#define M1120_RESULT_STATUS_NUM				(3)
#define M1120_RESULT_STATUS_A				(0x01)	// result status A	close register(0)
#define M1120_RESULT_STATUS_B				(0x02)	// result status B	stand
#define M1120_RESULT_STATUS_C				(0x03)	// result status C	lift
#define M1120_RESULT_STATUS_D				(0x04)	// result status D	camera1
#define M1120_RESULT_STATUS_E				(0x05)	// result status E	open1
#define M1120_RESULT_STATUS_F				(0x06)	// result status F	camera2
#define M1120_RESULT_STATUS_G				(0x07)	// result status G	open2
#define M1120_RESULT_STATUS_UNKNOWN			(0xFF)	// result status Unknown
#define M1120_EVENT_TYPE				EV_KEY //EV_ABS	// EV_KEY
#define M1120_EVENT_CODE					ABS_X	// KEY_F1
#define M1120_EVENT_DATA_CAPABILITY_MIN		(-32768)
#define M1120_EVENT_DATA_CAPABILITY_MAX		(32767)


/* ********************************************************* */
/* debug macro */
/* ********************************************************* */
#ifdef M1120_DBG_ENABLE
#define dbg(fmt, args...)  printk("[M1120-DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)   
#define dbgn(fmt, args...)  
#endif // M1120_DBG_ENABLE
#define dbg_func_in()       dbg("[M1120-DBG-F.IN] %s", __func__)
#define dbg_func_out()      dbg("[M1120-DBG-F.OUT] %s", __func__)
#define dbg_line()          dbg("[LINE] %d(%s)", __LINE__, __func__)
/* ********************************************************* */


/* ********************************************************* */
/* error display macro */
/* ********************************************************* */
#define mxerr(pdev, fmt, args...)			\
	dev_err(pdev, "[M1120-ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args) 
#define mxinfo(pdev, fmt, args...)			\
	dev_info(pdev, "[M1120-INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args) 
/* ********************************************************* */

//extern void *m1120_platform_data(void *info);


/* ********************************************************* */
/* static variable */
/* ********************************************************* */
static m1120_data_t *p_m1120_data;
/* ********************************************************* */


/* ********************************************************* */
/* function protyps */
/* ********************************************************* */
/* i2c interface */
static int	m1120_i2c_read(struct i2c_client *client, u8 reg, u8* rdata, u8 len);
static int	m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8* rdata);
static int	m1120_i2c_write(struct i2c_client *client, u8 reg, u8* wdata, u8 len);
static int	m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata);
/* vdd / vid power control */
static int m1120_set_power(struct device *dev, bool on);
/* scheduled work */
static void m1120_work_func(struct work_struct *work);
/* interrupt handler */
static irqreturn_t m1120_irq_handler(int irq, void *dev_id);
/* configuring or getting configured status */
static void m1120_get_reg(struct device *dev, int* regdata);
static void m1120_set_reg(struct device *dev, int* regdata);
static int	m1120_get_enable(struct device *dev);
static void	m1120_set_enable(struct device *dev, int enable);
static int	m1120_get_delay(struct device *dev);
static void	m1120_set_delay(struct device *dev, int delay);
static int	m1120_get_debug(struct device *dev);
static void	m1120_set_debug(struct device *dev, int debug);
static int	m1120_clear_interrupt(struct device *dev);
static int	m1120_update_interrupt_threshold(struct device *dev, short raw);
static int	m1120_set_operation_mode(struct device *dev, int mode);
static int	m1120_set_detection_mode(struct device *dev, u8 mode);
static int	m1120_init_device(struct device *dev);
static int	m1120_reset_device(struct device *dev);
static int	m1120_set_calibration(struct device *dev);
static int	m1120_get_calibrated_data(struct device *dev, int* data);
static int	m1120_measure(m1120_data_t *p_data, short *raw);
static int	m1120_get_result_status(m1120_data_t* p_data, int raw);
/* ********************************************************* */

static int irq_status = 2;
/******/
static int close;
static int stand;
static int lift;
static int open_a;
static int open_b;
static int cal_data;

/*****/

static int last_state = 0, current_state;

/* ********************************************************* */
/* functions for i2c interface */
/* ********************************************************* */
#define M1120_I2C_BUF_SIZE					(17)
static int m1120_i2c_read(struct i2c_client* client, u8 reg, u8* rdata, u8 len)
{
	int rc;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = rdata,
		},
	};

	if ( client == NULL ) {
		mxerr(&client->dev, "client is NULL");
		return -ENODEV;
	}

	rc = i2c_transfer(client->adapter, msg, 2);
	if(rc<0) {
		mxerr(&client->dev, "i2c_transfer was failed(%d)", rc);
		return rc;
	}

	return 0;
}

static int	m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8* rdata)
{
	return m1120_i2c_read(client, reg, rdata, 1);
}

static int m1120_i2c_write(struct i2c_client* client, u8 reg, u8* wdata, u8 len)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8  buf[M1120_I2C_BUF_SIZE];
	int rc;
	int i;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len+1,
			.buf = buf,
		},
	};

	if ( client == NULL ) {
		printk("[ERROR] %s : i2c client is NULL.\n", __func__);
		return -ENODEV;
	}
	
	buf[0] = reg;
	if (len > M1120_I2C_BUF_SIZE) {
		mxerr(&client->dev, "i2c buffer size must be less than %d", M1120_I2C_BUF_SIZE);
		return -EIO;
	}
	for( i=0 ; i<len; i++ ) buf[i+1] = wdata[i];

	rc = i2c_transfer(client->adapter, msg, 1);
	if(rc< 0) {
		mxerr(&client->dev, "i2c_transfer was failed (%d)", rc);
		return rc;
	}

	if(len==1) {
		switch(reg){
		case M1120_REG_PERSINT:
			p_data->reg.map.persint = wdata[0];
			break;
		case M1120_REG_INTSRS:
			p_data->reg.map.intsrs = wdata[0];
			break;
		case M1120_REG_LTHL:
			p_data->reg.map.lthl = wdata[0];
			break;
		case M1120_REG_LTHH:
			p_data->reg.map.lthh = wdata[0];
			break;
		case M1120_REG_HTHL:
			p_data->reg.map.hthl = wdata[0];
			break;
		case M1120_REG_HTHH:
			p_data->reg.map.hthh = wdata[0];
			break;
		case M1120_REG_I2CDIS:
			p_data->reg.map.i2cdis = wdata[0];
			break;
		case M1120_REG_SRST:
			p_data->reg.map.srst = wdata[0];
			msleep(1);
			break;
		case M1120_REG_OPF:
			p_data->reg.map.opf = wdata[0];
			break;
		}
	}

	for(i=0; i<len; i++) dbg("reg=0x%02X data=0x%02X", buf[0]+(u8)i, buf[i+1]);

	return 0;
}

static int m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata)
{
	return m1120_i2c_write(client, reg, &wdata, sizeof(wdata));
}

/* ********************************************************* */



/* ********************************************************* */
/* vdd / vid power control */
/* ********************************************************* */
static int m1120_set_power(struct device *dev, bool on)
{
	struct i2c_client *client = to_i2c_client(dev);

	if(on) {
		// to do for vdd power up
		mxinfo(&client->dev, "vdd power up");

		msleep(5); // wait 5ms
		dbg("waiting 5ms after vdd power up");

		// to do vid power up
		mxinfo(&client->dev, "vid power up");

		msleep(10); // wait 10ms
		dbg("waiting 10ms after vid power up");
	} else {
		// to do for vid power down
		mxinfo(&client->dev, "vid power down");

		// to do for vdd power down
		mxinfo(&client->dev, "vdd power down");
	}

	return 0;
}
/* ********************************************************* */

/* ********************************************************* */
/* functions for scheduling */
/* ********************************************************* */
static void m1120_work_func(struct work_struct *work)
{
	m1120_data_t* p_data = container_of((struct delayed_work *)work, m1120_data_t, work);
	unsigned long delay = msecs_to_jiffies(m1120_get_delay(&p_data->client->dev));
	short raw = 0;
	int err = 0;

	dbg_func_in();
	err = m1120_measure(p_data, &raw);

	if(!err) {

		if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
			p_data->last_data = m1120_get_result_status(p_data, raw);
		} else {
			p_data->last_data = (int)raw;
		}

#if (M1120_EVENT_TYPE == EV_ABS)
		printk("-------------hall (%d)\n", p_data->last_data);
		input_report_abs(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
#elif (M1120_EVENT_TYPE == EV_KEY)
		input_report_key(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
		if(p_data->last_data == M1120_RESULT_STATUS_A){
			input_report_key(p_data->input_dev, KEY_ANGLE_0, 1);
			input_sync(p_data->input_dev);
			input_report_key(p_data->input_dev, KEY_ANGLE_0, 0);
			input_sync(p_data->input_dev);
		printk("-------------hall (%d)\n", p_data->last_data);
		}else if(p_data->last_data == M1120_RESULT_STATUS_D){
			input_report_key(p_data->input_dev, KEY_ANGLE_90, 1);
			input_sync(p_data->input_dev);
			input_report_key(p_data->input_dev, KEY_ANGLE_90, 0);
			input_sync(p_data->input_dev);
		printk("-------------hall (%d)\n", p_data->last_data);
		}else if(p_data->last_data == M1120_RESULT_STATUS_G){
			input_report_key(p_data->input_dev, KEY_ANGLE_180, 1);
			input_sync(p_data->input_dev);
			input_report_key(p_data->input_dev, KEY_ANGLE_180, 0);
			input_sync(p_data->input_dev);
		printk("-------------hall (%d)\n", p_data->last_data);
		}else if((p_data->last_data == M1120_RESULT_STATUS_C) || (p_data->last_data == M1120_RESULT_STATUS_E)){
			input_report_key(p_data->input_dev, KEY_ANGLE_CAMERA, 1);
			input_sync(p_data->input_dev);
			input_report_key(p_data->input_dev, KEY_ANGLE_CAMERA, 0);
			input_sync(p_data->input_dev);
		printk("-------------hall (%d)\n", p_data->last_data);
		}
		current_state = p_data->last_data;
		if((last_state == M1120_RESULT_STATUS_A)&&(current_state != M1120_RESULT_STATUS_A)){
			input_report_key(p_data->input_dev, KEY_ANGLE_OPEN, 1);
			input_sync(p_data->input_dev);
			input_report_key(p_data->input_dev, KEY_ANGLE_OPEN, 0);
			input_sync(p_data->input_dev);
		printk("-------------hall open\n");
		}
		last_state = p_data->last_data;


#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

		input_sync(p_data->input_dev);
	}

	if( p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
		dbg("run update_interrupt_threshold");
		m1120_update_interrupt_threshold(&p_data->client->dev, raw);
	} else {
		schedule_delayed_work(&p_data->work, delay);
		dbg("run schedule_delayed_work");
	}
}
/* ********************************************************* */


/* ********************************************************* */
/* functions for interrupt handler */
/* ********************************************************* */
static irqreturn_t m1120_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();
		dbg("run schedule_delayed_work");
	if(p_m1120_data != NULL) {
		dbg("run schedule_delayed_work");
		schedule_delayed_work(&p_m1120_data->work, 0);
	}
	return IRQ_HANDLED;
}
/* ********************************************************* */


/* ********************************************************* */
/* functions for configuring or getting configured status */
/* ********************************************************* */

static void m1120_get_reg(struct device *dev, int* regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = 0;
	err = m1120_i2c_get_reg(client, rega, &regd);

	*regdata = 0;
	*regdata |= (err==0) ? 0x0000 : 0xFF00;
	*regdata |= regd;
}

static void m1120_set_reg(struct device *dev, int* regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = *regdata&0xFF;
	err = m1120_i2c_set_reg(client, rega, regd);

	*regdata = 0;
	*regdata |= (err==0) ? 0x0000 : 0xFF00;
}


static int m1120_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.enable);
}

static void m1120_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int delay = m1120_get_delay(dev);

	mutex_lock(&p_data->mtx.enable);

	if (enable) {                   /* enable if state will be changed */
		if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
			m1120_set_detection_mode(dev, p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT);
			m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_MEASUREMENT);
			if( ! (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)) {
				schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
			}
		}
	} else {                        /* disable if state will be changed */
		if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
			cancel_delayed_work_sync(&p_data->work);
			m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_POWERDOWN);
		}
	}
	atomic_set(&p_data->atm.enable, enable);

	mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	int delay = 0;

	delay = atomic_read(&p_data->atm.delay);

	return delay;
}

static void m1120_set_delay(struct device *dev, int delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	if(delay<M1120_DELAY_MIN) delay = M1120_DELAY_MIN;
	atomic_set(&p_data->atm.delay, delay);

	mutex_lock(&p_data->mtx.enable);

	if (m1120_get_enable(dev)) {
		if( ! (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)) {
			cancel_delayed_work_sync(&p_data->work);
			schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
		}
	}

	mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_debug(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.debug);
}

static void m1120_set_debug(struct device *dev, int debug)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	atomic_set(&p_data->atm.debug, debug);
}

static int m1120_clear_interrupt(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int ret = 0;

	ret = m1120_i2c_set_reg(p_data->client, M1120_REG_PERSINT, p_data->reg.map.persint | 0x01);

	return ret;
}

void m1120_convdata_short_to_2byte(u8 opf, short x, unsigned char *hbyte, unsigned char *lbyte)
{
	if( (opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		if(x<-128) x=-128;
		else if(x>127) x=127;

		if(x>=0) {
			*lbyte = x & 0x7F;
		} else {
			*lbyte = ( (0x80 - (x*(-1))) & 0x7F ) | 0x80;
		}
		*hbyte = 0x00;
	} else {
		/* 10 bit resolution */
		if(x<-512) x=-512;
		else if(x>511) x=511;

		if(x>=0) {
			*lbyte = x & 0xFF;
			*hbyte = (((x&0x100)>>8)&0x01) << 6;
		} else {
			*lbyte = (0x0200 - (x*(-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x*(-1))) & 0x100)>>8)<<6) | 0x80;
		}
	}
}

short m1120_convdata_2byte_to_short(u8 opf, unsigned char hbyte, unsigned char lbyte)
{
	short x;

	if( (opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		x = lbyte & 0x7F;
		if(lbyte & 0x80) {
			x -= 0x80;
		}
	} else {
		/* 10 bit resolution */
		x = ( ( (hbyte & 0x40) >> 6) << 8 ) | lbyte;
		if(hbyte&0x80) {
			x -= 0x200;
		}
	}

	return x;
}

static int m1120_update_interrupt_threshold(struct device *dev, short raw)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 lthh=0, lthl=0, hthh=0, hthl=0;
	int err = -1;
	int h_status;		//digital hall current status.

	if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {

		dbg("reg.map.intsrs = 0x%02X", p_data->reg.map.intsrs);
		if(p_data->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_WITHIN) {
			// to do another condition
		} else {
			dbg("BESIDE raw = %d", raw);
#if (M1120_RESULT_STATUS_NUM==3)
			if(p_data->stscfg_init) {
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, 510, &lthh, &lthl);
				p_data->stscfg_init = 0;
/*	
 *	} else {
				if (m1120_get_result_status(p_data, (int)raw) == p_data->stscfg[0].type) {
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[1].max, &lthh, &lthl);
				} else if (m1120_get_result_status(p_data, (int)raw) == p_data->stscfg[1].type) {
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[0].min, &hthh, &hthl);
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[2].max, &lthh, &lthl);
				} else if (m1120_get_result_status(p_data, (int)raw) == p_data->stscfg[2].type) {
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[1].min, &hthh, &hthl);
					m1120_convdata_short_to_2byte(p_data->reg.map.opf, -511, &lthh, &lthl);
				}
			}
*/
/**************1 Added for lenovo requirement--liulf2***************/
			} else {
				h_status = m1120_get_result_status(p_data, (int)raw) - 1;
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[h_status].max, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, (short)p_data->stscfg[h_status].min, &lthh, &lthl);
			}


/**************1 Added for lenovo requirement--liulf2***************/
#else
			if( (raw >=-512) && (raw < p_data->thrhigh) ) {
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrhigh, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, -512, &lthh, &lthl);
			} else if ( (raw >= p_data->thrlow) && (raw <= 511)) {
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrlow, &lthh, &lthl);
			}
#endif
		}

		err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHH, hthh);
		if(err) return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHL, hthl);
		if(err) return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHH, lthh);
		if(err) return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHL, lthl);
		if(err) return err;

		dbg("threshold : (0x%02X%02X[%d], 0x%02X%02X[%d])",				\
			hthh, hthl, 								\
			m1120_convdata_2byte_to_short(p_data->reg.map.opf, hthh, hthl),		\
			lthh, lthl,								\
			m1120_convdata_2byte_to_short(p_data->reg.map.opf, lthh, lthl)		\
		);

		err = m1120_clear_interrupt(dev);
		if(err) return err;
	}

	return err;
}

static int m1120_set_operation_mode(struct device *dev, int mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 opf = p_data->reg.map.opf;
	int err = -1;

	switch(mode) {
		case OPERATION_MODE_POWERDOWN:

			if((irq_status == 1) && (irq_status != 2)){	/* disable irq */
				disable_irq(p_data->irq);
				irq_status = 0;
			}
			opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
			err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
			mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_POWERDOWN");
			break;
		case OPERATION_MODE_MEASUREMENT:
			opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
			opf |= M1120_VAL_OPF_HSSON_ON;
			err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
			if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
				if((irq_status == 0) && (irq_status != 2)){
					enable_irq(p_data->irq);
					irq_status = 1;
				}
			}
			mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_MEASUREMENT");
			break;
		case OPERATION_MODE_FUSEROMACCESS:
			opf |= M1120_VAL_OPF_EFRD_ON;
			opf |= M1120_VAL_OPF_HSSON_ON;
			err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
			mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_FUSEROMACCESS");
			break;
	}

	return err;
}

static int m1120_set_detection_mode(struct device *dev, u8 mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 data;
	int err = 0;

	if(mode & M1120_DETECTION_MODE_INTERRUPT) {

		/* config threshold */
		m1120_update_interrupt_threshold(dev, p_data->last_data);

		/* write intsrs */
		data = p_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);

		p_data->stscfg_init = 1;
		if(err) return err;

	} else {

		/* write intsrs */
		data = p_data->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
		if(err) return err;
	}

	return err;
}


static int m1120_init_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int err = -1;

	/* (1) vdd and vid power up */
	err = m1120_set_power(dev, 1);
	if(err) {
		mxerr(&client->dev, "m1120 power-on was failed (%d)", err);
		return err;
	}

	/* (2) init variables */
	atomic_set(&p_data->atm.enable, 0);
	atomic_set(&p_data->atm.delay, M1120_DELAY_MIN);
#ifdef M1120_DBG_ENABLE
	atomic_set(&p_data->atm.debug, 1);
#else
	atomic_set(&p_data->atm.debug, 0);
#endif
	p_data->calibrated_data = 0;
	p_data->last_data = 0;
//	p_data->irq_enabled = 0;
	p_data->irq_first = 1;

	close = 70;
	stand = 10;
	lift = -100;
	open_a = (close + stand)/4 + stand;
	open_b = (stand + lift)/2;

#if (M1120_RESULT_STATUS_NUM==3)
	/* init stscfg */
/*	p_data->stscfg[2].type = M1120_RESULT_STATUS_C; //180 degree
	p_data->stscfg[2].min = -110;
	p_data->stscfg[2].max = -90;

	p_data->stscfg[1].type = M1120_RESULT_STATUS_B; //90 degree
	p_data->stscfg[1].min = 0;
	p_data->stscfg[1].max = 20;

	p_data->stscfg[0].type = M1120_RESULT_STATUS_A; //0 degree
	p_data->stscfg[0].min = 120;
	p_data->stscfg[0].max = 140;
	*/
	p_data->stscfg[6].type = M1120_RESULT_STATUS_G; //180 degree
	p_data->stscfg[6].min = -511;
	p_data->stscfg[6].max = lift + 10;

	p_data->stscfg[5].type = M1120_RESULT_STATUS_F; //135~180 area
	p_data->stscfg[5].min = lift + 10;
	p_data->stscfg[5].max = open_b;

	p_data->stscfg[4].type = M1120_RESULT_STATUS_E; //90~135 area
	p_data->stscfg[4].min = open_b;
	p_data->stscfg[4].max = stand - 10;

	p_data->stscfg[3].type = M1120_RESULT_STATUS_D; //90 degree
	p_data->stscfg[3].min = stand - 10;
	p_data->stscfg[3].max = stand + 10;

	p_data->stscfg[2].type = M1120_RESULT_STATUS_C; //45~90 area
	p_data->stscfg[2].min = stand + 10;
	p_data->stscfg[2].max = open_a;

	p_data->stscfg[1].type = M1120_RESULT_STATUS_B; //0~45 area
	p_data->stscfg[1].min = open_a;
	p_data->stscfg[1].max = close -10;

	p_data->stscfg[0].type = M1120_RESULT_STATUS_A; //0 degree
	p_data->stscfg[0].min = close - 10;
	p_data->stscfg[0].max = 511;

#else
	p_data->thrhigh = M1120_DETECT_RANGE_HIGH;
	p_data->thrlow = M1120_DETECT_RANGE_LOW;
#endif
	m1120_set_delay(&client->dev, M1120_DELAY_MAX);
	m1120_set_debug(&client->dev, 0);

	/* (3) reset registers */
	err = m1120_reset_device(dev);
	if(err) {
		mxerr(&client->dev, "m1120_reset_device was failed (%d)", err);
		return err;
	}

	mxinfo(&client->dev, "initializing device was success");

	return 0;
}

static int m1120_reset_device(struct device *dev)
{
	int	err = 0;
	u8	id = 0xFF, data = 0x00;

	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	if( (p_data == NULL) || (p_data->client == NULL) ) return -ENODEV;

	/* (1) sw reset */
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_SRST, M1120_VAL_SRST_RESET);
	if(err) {
		mxerr(&client->dev, "sw-reset was failed(%d)", err);
		return err;
	}
	msleep(5); // wait 5ms
	dbg("wait 5ms after vdd power up");

	/* (2) check id */
	err = m1120_i2c_get_reg(p_data->client, M1120_REG_DID, &id);
	if (err < 0) return err;
	if (id != M1120_VAL_DID) {
		mxerr(&client->dev, "current device id(0x%02X) is not M1120 device id(0x%02X)", id, M1120_VAL_DID);
		return -ENXIO;
	}

	/* (3) init variables */
	/* (3-1) persint */
	data = M1120_PERSISTENCE_COUNT;
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_PERSINT, data);
	/* (3-2) intsrs */
	data = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
	if(data & M1120_DETECTION_MODE_INTERRUPT) {
		data |= M1120_INTERRUPT_TYPE;
	}
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
	/* (3-3) opf */
	data = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_OPF, data);

	/* (4) write variable to register */
	err = m1120_set_detection_mode(dev, M1120_DETECTION_MODE);
	if(err) {
		mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
		return err;
	}

	/* (5) set power-down mode */
	err = m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);
	if(err) {
		mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
		return err;
	}
#if 0
	err = m1120_i2c_set_reg(p_data->client, 0x6F, 0x6E);
	err = m1120_i2c_set_reg(p_data->client, 0x50, 0x60);
	err = m1120_i2c_set_reg(p_data->client, 0x51, 0x08);
	err = m1120_i2c_set_reg(p_data->client, 0x52, 0x19);
	err = m1120_i2c_set_reg(p_data->client, 0x53, 0x19);
	err = m1120_i2c_set_reg(p_data->client, 0x54, 0x03);
	err = m1120_i2c_set_reg(p_data->client, 0x55, 0x01);
	err = m1120_i2c_set_reg(p_data->client, 0x56, 0x25);
	err = m1120_i2c_set_reg(p_data->client, 0x5C, 0x06);
	err = m1120_i2c_set_reg(p_data->client, 0x5D, 0x7B);
	err = m1120_i2c_set_reg(p_data->client, 0x5E, 0x9C);
	err = m1120_i2c_set_reg(p_data->client, 0x5F, 0x15);
	err = m1120_i2c_set_reg(p_data->client, 0x60, 0xEF);
	err = m1120_i2c_set_reg(p_data->client, 0x67, 0x50);
	err = m1120_i2c_set_reg(p_data->client, 0x68, 0x60);
	err = m1120_i2c_set_reg(p_data->client, 0x65, 0x10);
	msleep(10);
#endif
	return err;
}


static int m1120_set_calibration(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int retrycnt = 10, cnt = 0;

	short raw = 0;
	int err;
	int cal_store = 0, cnt_s = 0;

	m1120_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);
	for(cnt=0; cnt<retrycnt; cnt++) {
		msleep(M1120_DELAY_FOR_READY);
		err = m1120_measure(p_data, &raw);
		if(!err) {
			cal_store += raw;
			cnt_s++;
		}
	}
	cal_store /= cnt_s; 
	p_data->calibrated_data = cal_store;
	cal_data = cal_store;
	if(!m1120_get_enable(dev)) m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);

	return err;
}

static int m1120_get_calibrated_data(struct device *dev, int* data)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	int err = 0;

	if(p_data == NULL) err = -ENODEV;
	else *data = p_data->calibrated_data;

	return err;
}

static int m1120_measure(m1120_data_t *p_data, short *raw)
{
	struct i2c_client *client = p_data->client;
	int err;
	u8 buf[3];
	int st1_is_ok = 0;

	// (1) read data
	err = m1120_i2c_read(client, M1120_REG_ST1, buf, sizeof(buf));
	if(err) return err;

	// (2) collect data
	if(p_data->reg.map.intsrs & M1120_VAL_INTSRS_INT_ON) {
		// check st1 at interrupt mode
		if( ! (buf[0] & 0x10) ) {
			st1_is_ok = 1;
		}
	} else {
		// check st1 at polling mode
		if(buf[0] & 0x01) {
			st1_is_ok = 1;
		}
	}

			st1_is_ok = 1;
	if(st1_is_ok) {
		*raw = m1120_convdata_2byte_to_short(p_data->reg.map.opf, buf[2], buf[1]);
	} else {
		mxerr(&client->dev, "st1(0x%02X) is not DRDY", buf[0]);
		err = -1;
	}

//	if(m1120_get_debug(&client->dev)) {
		dbg("raw data (%d)", *raw);
//	}

	return err;
}


static int m1120_get_result_status(m1120_data_t* p_data, int raw)
{
	int status, status_loop;

#if (M1120_RESULT_STATUS_NUM==3)
/**	if( (raw<p_data->stscfg[2].max) ) status = p_data->stscfg[2].type;
	else if( (raw > p_data->stscfg[2].max) && (raw<p_data->stscfg[0].min) ) status = p_data->stscfg[1].type;
	else if( (raw >= p_data->stscfg[0].min) ) status = p_data->stscfg[0].type;
	else status = M1120_RESULT_STATUS_UNKNOWN;
	**/
	for(status_loop = 0;status_loop < 7; status_loop++){
		if((raw<p_data->stscfg[status_loop].max)&&(raw>p_data->stscfg[status_loop].min)){
			status = p_data->stscfg[status_loop].type;
			break;
		}
	}
	if(status_loop > 6)
		status = M1120_RESULT_STATUS_UNKNOWN;
		
	dbg("Result is status [0x%02X]", status);
#else
	if(p_data->thrhigh <= raw) {
		status = M1120_RESULT_STATUS_B;
	} else if(p_data->thrlow >= raw) {
		status = M1120_RESULT_STATUS_A;
	} else {
		status = p_data->last_data;
	}



	if(p_data->thrhigh <= raw) {
		status = M1120_RESULT_STATUS_B;
	} else if(p_data->thrlow >= raw) {
		status = M1120_RESULT_STATUS_A;
	} else {
		status = p_data->last_data;
	}

	switch(status) {
		case M1120_RESULT_STATUS_A:
			dbg("Result is status [A]\n");
			break;
		case M1120_RESULT_STATUS_B:
			dbg("Result is status [B]\n");
			break;
	}
#endif
	return status;
}






/* *************************************************
   input device interface
   ************************************************* */

static int m1120_input_dev_init(m1120_data_t *p_data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev) {
		return -ENOMEM;
	}
	dev->name = M1120_DRIVER_NAME;
	dev->id.bustype = BUS_I2C;

#if (M1120_EVENT_TYPE == EV_ABS)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, M1120_EVENT_TYPE, ABS_MISC);
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	input_set_abs_params(dev, M1120_EVENT_CODE, M1120_EVENT_DATA_CAPABILITY_MIN, M1120_EVENT_DATA_CAPABILITY_MAX, 0, 0);
#elif (M1120_EVENT_TYPE == EV_KEY)
	input_set_drvdata(dev, p_data);
	set_bit(EV_KEY, dev->evbit);
	set_bit(KEY_ANGLE_0, dev->keybit);
	set_bit(KEY_ANGLE_90, dev->keybit);
	set_bit(KEY_ANGLE_180, dev->keybit);
	set_bit(KEY_ANGLE_CAMERA, dev->keybit);
	set_bit(KEY_ANGLE_OPEN, dev->keybit);
	input_set_capability(dev, M1120_EVENT_TYPE, M1120_EVENT_CODE);
#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	p_data->input_dev = dev;

	return 0;
}

static void m1120_input_dev_terminate(m1120_data_t *p_data)
{
	struct input_dev *dev = p_data->input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}





/* *************************************************
   misc device interface
   ************************************************* */

static int m1120_misc_dev_open( struct inode*, struct file* );
static int m1120_misc_dev_release( struct inode*, struct file* );
static long m1120_misc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
static ssize_t m1120_misc_dev_read( struct file *filp, char *buf, size_t count, loff_t *ofs );
static ssize_t m1120_misc_dev_write( struct file *filp, const char *buf, size_t count, loff_t *ofs );
static unsigned int m1120_misc_dev_poll( struct file *filp, struct poll_table_struct *pwait );

static struct file_operations m1120_misc_dev_fops =
{
	.owner = THIS_MODULE,
	.open = m1120_misc_dev_open,
	.unlocked_ioctl = m1120_misc_dev_ioctl,
	.release = m1120_misc_dev_release,
	.read = m1120_misc_dev_read,
	.write = m1120_misc_dev_write,
	.poll = m1120_misc_dev_poll,
};

static struct miscdevice m1120_misc_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = M1120_DRIVER_NAME,
	.fops = &m1120_misc_dev_fops,
};
/* m1120 misc device file operation */
static int m1120_misc_dev_open( struct inode* inode, struct file* file)
{
	return 0;
}

static int m1120_misc_dev_release( struct inode* inode, struct file* file)
{
	return 0;
}

static long m1120_misc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	void __user *argp = (void __user *)arg;
	int kbuf = 0;
	int caldata = 0;

	switch( cmd ) {
	case M1120_IOCTL_SET_ENABLE:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_ENABLE(%d)", kbuf);
		m1120_set_enable(&p_m1120_data->client->dev, kbuf);
		break;
	case M1120_IOCTL_GET_ENABLE:
		kbuf = m1120_get_enable(&p_m1120_data->client->dev);
		dbg("M1120_IOCTL_GET_ENABLE(%d)", kbuf);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
		break;
	case M1120_IOCTL_SET_DELAY:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_DELAY(%d)", kbuf);
		m1120_set_delay(&p_m1120_data->client->dev, kbuf);
		break;
	case M1120_IOCTL_GET_DELAY:
		kbuf = m1120_get_delay(&p_m1120_data->client->dev);
		dbg("M1120_IOCTL_GET_DELAY(%d)", kbuf);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
		break;
	case M1120_IOCTL_SET_CALIBRATION:
		dbg("M1120_IOCTL_SET_CALIBRATION");
		kbuf = m1120_set_calibration(&p_m1120_data->client->dev);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
		break;
	case M1120_IOCTL_GET_CALIBRATED_DATA:
		dbg("M1120_IOCTL_GET_CALIBRATED_DATA");
		kbuf = m1120_get_calibrated_data(&p_m1120_data->client->dev, &caldata);
		if(copy_to_user(argp, &caldata, sizeof(caldata))) return -EFAULT;
		dbg("calibrated data (%d)", caldata);
		break;
	case M1120_IOCTL_SET_REG:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_REG([0x%02X] %02X", (u8)((kbuf>>8)&0xFF), (u8)(kbuf&0xFF));
		m1120_set_reg(&p_m1120_data->client->dev, &kbuf);
		dbgn(" (%s))\n", (kbuf&0xFF00)?"Not Ok":"Ok");
		if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
		break;
	case M1120_IOCTL_GET_REG:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_GET_REG([0x%02X]", (u8)((kbuf>>8)&0xFF) );
		m1120_get_reg(&p_m1120_data->client->dev, &kbuf);
		dbgn(" 0x%02X (%s))\n", (u8)(kbuf&0xFF), (kbuf&0xFF00)?"Not Ok":"Ok");
		if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
		break;
	case M1120_IOCTL_SET_INTERRUPT:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_INTERRUPT(%d)", kbuf);
		if(kbuf) {
			m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_INTERRUPT);
		} else {
			m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_POLLING);
		}
		break;
	case M1120_IOCTL_GET_INTERRUPT:
		kbuf = (p_m1120_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) ? 1 : 0 ;
		dbg("M1120_IOCTL_GET_INTERRUPT(%d)", kbuf);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
		break;
	case M1120_IOCTL_SET_THRESHOLD_HIGH:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_THRESHOLD_HIGH(%d)", kbuf);
		p_m1120_data->thrhigh = kbuf;
		break;
	case M1120_IOCTL_GET_THRESHOLD_HIGH:
		kbuf = p_m1120_data->thrhigh;
		dbg("M1120_IOCTL_GET_THRESHOLD_HIGH(%d)", kbuf);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
		break;
	case M1120_IOCTL_SET_THRESHOLD_LOW:
		if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
		dbg("M1120_IOCTL_SET_THRESHOLD_LOW(%d)", kbuf);
		p_m1120_data->thrlow = kbuf;
		break;
	case M1120_IOCTL_GET_THRESHOLD_LOW:
		kbuf = p_m1120_data->thrlow;
		dbg("M1120_IOCTL_GET_THRESHOLD_LOW(%d)", kbuf);
		if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static ssize_t m1120_misc_dev_read( struct file *filp, char *buf, size_t count, loff_t *ofs )
{
	return 0;
}

static ssize_t m1120_misc_dev_write( struct file *filp, const char *buf, size_t count, loff_t *ofs )
{
	return 0;
}

static unsigned int m1120_misc_dev_poll( struct file *filp, struct poll_table_struct *pwait )
{
	return 0;
}






/* *************************************************
   sysfs attributes
   ************************************************* */
static ssize_t m1120_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", m1120_get_enable(dev));
}

static ssize_t m1120_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);
/***********get calibration data from factory/hcal_data1********************/
	char cal_data[14];	//calibration data readed from factory/hcal_data.
	char s_data[5];		//buffer for storing each rawdata.
	static int c_data[3];		//storing each rawdata.
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int loop1, loop2, loop3;
/***********get calibration data from factory/hcal_data1********************/

	if ((enable == 0) || (enable == 1)) {
		m1120_set_enable(dev, enable);
	}
/***********get calibration data from factory/hcal_data2********************/
	mdelay(10);
	fp = filp_open("/factory/hcal_data", O_RDWR|O_CREAT, 0644);
	if(IS_ERR(fp)){
		printk("open, file error.\n");
	}else{
		
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(fp, cal_data, sizeof(cal_data), &pos);
	printk("read: %s\n", cal_data);
	filp_close(fp, NULL);
	set_fs(fs);
	
	if(cal_data != NULL){
	loop3 = 0;
	for(loop1 = 0;(cal_data[loop1] != '\0');loop1++){
		for(loop2 = 0;(cal_data[loop1] != '/');loop2++){
				s_data[loop2] = cal_data[loop1];
			if(cal_data[loop1] == '\0')
				break;
			loop1++;		
		}
		s_data[loop2] = '\0';
		sscanf(s_data,"%d", &(c_data[loop3++]));
	}
	if((c_data[0] != 0)&&(c_data[1] != 0)&&(c_data[2] != 0)){
		close = c_data[0];
		stand = c_data[1];
		lift = c_data[2];
	}else 
		return count;
	
	printk("---------------- %d %d %d\n", close, stand, lift);
	}
}
	open_a = (close + stand)/4 + stand;
	open_b = (stand + lift)/2;

	p_m1120_data->stscfg[0].max = 511;
	p_m1120_data->stscfg[0].min = close - 10;
	p_m1120_data->stscfg[1].max = close - 10;
	p_m1120_data->stscfg[1].min = open_a;
	p_m1120_data->stscfg[2].max = open_a;
	p_m1120_data->stscfg[2].min = stand + 10;
	p_m1120_data->stscfg[3].max = stand + 10;
	p_m1120_data->stscfg[3].min = stand - 10;
	p_m1120_data->stscfg[4].max = stand - 10;
	p_m1120_data->stscfg[4].min = open_b;
	p_m1120_data->stscfg[5].max = open_b;
	p_m1120_data->stscfg[5].min = lift + 10;
	p_m1120_data->stscfg[6].max = lift + 10;
	p_m1120_data->stscfg[6].min = -511;
	printk("read: %d %d %d \n", close, stand, lift);
/***********get calibration data from factory/hcal_data2********************/

	return count;
}

static ssize_t m1120_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", m1120_get_delay(dev));
}

static ssize_t m1120_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > M1120_DELAY_MAX) {
		delay = M1120_DELAY_MAX;
	}

	m1120_set_delay(dev, delay);

	return count;
}

static ssize_t m1120_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", m1120_get_debug(dev));
}

static ssize_t m1120_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long debug = simple_strtoul(buf, NULL, 10);

	m1120_set_debug(dev, debug);

	return count;
}

static ssize_t m1120_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return 0;
}

/************calibration************/
static ssize_t m1120_calibration_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "data[0] = %d, data[90] = %d, data[180] = %d.\n", close, stand, lift);
}

static ssize_t m1120_calibration_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long cal_angle = simple_strtoul(buf, NULL, 10);
/***********store calibration data in factory/hcal_data3********************/
	static char buf1[14];
	char buf2[14];
	char flag1[] = "/";
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
/***********store calibration data in factory/hcal_data3********************/

	if(cal_angle == 0){
		m1120_set_calibration(&p_m1120_data->client->dev);
		close = cal_data;
		open_a = (close + stand)/2;
		p_m1120_data->stscfg[0].max = 511;
		p_m1120_data->stscfg[0].min = close - 10;
		p_m1120_data->stscfg[1].max = close - 10;
		p_m1120_data->stscfg[1].min = open_a;
		p_m1120_data->stscfg[2].max = open_a;
	}else if(cal_angle == 1){
		m1120_set_calibration(&p_m1120_data->client->dev);
		stand = cal_data;
		open_a = (close + stand)/2;
		open_b = (stand + lift)/2;
		p_m1120_data->stscfg[1].min = open_a;
		p_m1120_data->stscfg[2].max = open_a;
		p_m1120_data->stscfg[2].min = stand + 10;
		p_m1120_data->stscfg[3].max = stand + 10;
		p_m1120_data->stscfg[3].min = stand - 10;
		p_m1120_data->stscfg[4].max = stand - 10;
		p_m1120_data->stscfg[4].min = open_b;
		p_m1120_data->stscfg[5].max = open_b;
	}else if(cal_angle == 2){
		m1120_set_calibration(&p_m1120_data->client->dev);
		lift = cal_data;
		open_b = (stand + lift)/2;
		p_m1120_data->stscfg[4].min = open_b;
		p_m1120_data->stscfg[5].max = open_b;
		p_m1120_data->stscfg[5].min = lift + 10;
		p_m1120_data->stscfg[6].max = lift + 10;
		p_m1120_data->stscfg[6].min = -511;

	}else
		printk("------");

/***********store calibration data in factory/hcal_data4********************/

	sprintf(buf2, "%d", close);
	strcat(buf2, flag1);
	strcpy(buf1, buf2);
	sprintf(buf2, "%d", stand);
	strcat(buf2, flag1);
	strcat(buf1, buf2);
	sprintf(buf2, "%d", lift);
	strcat(buf1, buf2);


	fp = filp_open("/factory/hcal_data", O_RDWR|O_CREAT, 0644);
	if(IS_ERR(fp)){
		printk("creat file error.\n");
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(fp, buf1,sizeof(buf1), &pos);
	pos = 0;
	vfs_read(fp, buf1, sizeof(buf1), &pos);
	printk("read: %s\n", buf1);
	filp_close(fp, NULL);
	set_fs(fs);
/***********store calibration data in factory/hcal_data4********************/

	return count;
}

static ssize_t m1120_cal_result_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int cal_result = 0;
		
	if((close - stand) < 45)
		cal_result += 1;
	if((stand - lift) < 80)
		cal_result += 2;

	
	return sprintf(buf, "%d\n", cal_result);

}
static ssize_t m1120_hall_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", p_m1120_data->last_data);
}
static DEVICE_ATTR(calibration,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_calibration_show, m1120_calibration_store);
static DEVICE_ATTR(result,	S_IRUGO, m1120_cal_result_show, NULL);
static DEVICE_ATTR(hall_status,	S_IRUGO, m1120_hall_status_show, NULL);
/************calibration***********/
static DEVICE_ATTR(enable,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_enable_show, m1120_enable_store);
static DEVICE_ATTR(delay,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_delay_show,	m1120_delay_store);
static DEVICE_ATTR(debug,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_debug_show,	m1120_debug_store);
static DEVICE_ATTR(wake,	S_IWUSR|S_IWGRP,		 NULL,				m1120_wake_store);

static struct attribute *m1120_attributes[] = {
	&dev_attr_calibration.attr,
	&dev_attr_result.attr,
	&dev_attr_hall_status.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_debug.attr,
	&dev_attr_wake.attr,
	NULL
};

static struct attribute_group m1120_attribute_group = {
	.attrs = m1120_attributes
};


/* *************************************************
   i2c client
   ************************************************* */

int m1120_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	m1120_platform_data_t	*p_platform;
	m1120_data_t			*p_data;
	int						err = 0;

	dbg_func_in();

	/* (1) allocation memory for p_m1120_data */
	p_data = kzalloc(sizeof(m1120_data_t), GFP_KERNEL);
	if (!p_data) {
		mxerr(&client->dev, "kernel memory alocation was failed");
		err = -ENOMEM;
		goto error_0;
	}

	/* (2) init mutex variable */
	mutex_init(&p_data->mtx.enable);
	mutex_init(&p_data->mtx.data);

	/* (3) config i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		mxerr(&client->dev, "i2c_check_functionality was failed");
		err = -ENODEV;
		goto error_1;
	}
	i2c_set_clientdata(client, p_data);
	p_data->client = client;

	/* (4) get platform data */
	p_platform = client->dev.platform_data;
	if(p_platform) {
		p_data->power_vi2c		= p_platform->power_vi2c;
		p_data->power_vdd		= p_platform->power_vdd;
		p_data->igpio			= p_platform->interrupt_gpio;
		p_data->irq				= p_platform->interrupt_irq;
	}
	else {
		p_data->power_vi2c = -1;
		p_data->power_vdd = -1;
		p_data->igpio = 155;
		p_data->irq = gpio_to_irq(155);
	}

	/* (5) setup interrupt gpio */
	if(p_data->igpio != -1) {
		err = gpio_request(p_data->igpio, "m1120_irq");
		if (err){
			mxerr(&client->dev, "gpio_request was failed(%d)", err); 
			goto error_1;
		}
		mxinfo(&client->dev, "gpio_request was success");
		err = gpio_direction_input(p_data->igpio);
		if (err < 0) {
			mxerr(&client->dev, "gpio_direction_input was failed(%d)", err);
			goto error_2;
		}
		mxinfo(&client->dev, "gpio_direction_input was success");
		err = request_irq(p_data->irq, &m1120_irq_handler, IRQF_TRIGGER_FALLING, M1120_IRQ_NAME, 0);
		irq_status = 1;
	}
	

	/* (6) reset and init device */
	err = m1120_init_device(&p_data->client->dev);
	if(err) {
		mxerr(&client->dev, "m1120_init_device was failed(%d)", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was found", id->name);

	/* (7) config work function */
	INIT_DELAYED_WORK(&p_data->work, m1120_work_func);

	/* (8) init input device */
	err = m1120_input_dev_init(p_data);
	if(err) {
		mxerr(&client->dev, "m1120_input_dev_init was failed(%d)", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was initialized", M1120_DRIVER_NAME);

	/* (9) create sysfs group */
	err = sysfs_create_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
	if(err) {
		mxerr(&client->dev, "sysfs_create_group was failed(%d)", err);
		goto error_3;
	}

	/* (10) register misc device */
	err = misc_register(&m1120_misc_dev);
	if(err) {
		mxerr(&client->dev, "misc_register was failed(%d)", err);
		goto error_4;
	}

	/* (11) imigrate p_data to p_m1120_data */
	p_m1120_data = p_data;
	dbg("%s : %s was probed.", __func__, M1120_DRIVER_NAME);

	return 0;

error_4:
	sysfs_remove_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);

error_3:
	m1120_input_dev_terminate(p_data);

error_2:
	if(p_data->igpio != -1) {
		gpio_free(p_data->igpio);
	}

error_1:
	kfree(p_data);

error_0:

	return err;
}

static int m1120_i2c_drv_remove(struct i2c_client *client)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);

			free_irq(p_data->irq, NULL);
	m1120_set_enable(&client->dev, 0);
	misc_deregister(&m1120_misc_dev);
	sysfs_remove_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
	m1120_input_dev_terminate(p_data);
	if(p_data->igpio!= -1) {
		gpio_free(p_data->igpio);
	}
	kfree(p_data);

	return 0;
}
/*
static int m1120_i2c_drv_suspend(struct i2c_client *client, pm_message_t mesg)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);

	dbg_func_in();

	mutex_lock(&p_data->mtx.enable);

	if (m1120_get_enable(&client->dev)) {
		if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
			m1120_set_operation_mode(&client->dev, OPERATION_MODE_MEASUREMENT);
		} else {
			cancel_delayed_work_sync(&p_data->work);
			m1120_set_detection_mode(&client->dev, M1120_DETECTION_MODE_INTERRUPT);
		}
	}

	mutex_unlock(&p_data->mtx.enable);

	dbg_func_out();

	return 0;
}

static int m1120_i2c_drv_resume(struct i2c_client *client)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);

	dbg_func_in();

	mutex_lock(&p_data->mtx.enable);

	if (m1120_get_enable(&client->dev)) {
		if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
			m1120_set_detection_mode(&client->dev, M1120_DETECTION_MODE_POLLING);
			schedule_delayed_work(&p_data->work, msecs_to_jiffies(m1120_get_delay(&client->dev)));
		}
	}

	mutex_unlock(&p_data->mtx.enable);

	dbg_func_out();

	return 0;
}
*/
static const struct i2c_device_id m1120_i2c_drv_id_table[] = {
	{M1120_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver m1120_driver = {
	.driver = {
		.name	= M1120_DRIVER_NAME,
	},
	.probe		= m1120_i2c_drv_probe,
	.remove		= m1120_i2c_drv_remove,
	.id_table	= m1120_i2c_drv_id_table,
//	.suspend	= m1120_i2c_drv_suspend,
//	.resume		= m1120_i2c_drv_resume,
};

static int __init m1120_platform_init(void)
{
    int i2c_busnum = 5;
    struct i2c_board_info i2c_info;
    void *pdata = NULL;

    //intel_scu_ipc_msic_vprog3(1);

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, M1120_DRIVER_NAME, strlen(M1120_DRIVER_NAME));

    i2c_info.addr = 0x0C;
//    i2c_info.interrupt_gpio = 25;
    i2c_info.irq = gpio_to_irq(155);

    pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

 //  pdata = m1120_platform_data(&i2c_info);

    if(pdata != NULL)
        i2c_info.platform_data = pdata;
    else
        printk("%s, pdata is NULL\n", __func__);

    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
}

fs_initcall(m1120_platform_init);

static int __init m1120_driver_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&m1120_driver);
}
module_init(m1120_driver_init);

static void __exit m1120_driver_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
	i2c_del_driver(&m1120_driver);
}
module_exit(m1120_driver_exit);

MODULE_AUTHOR("shpark <seunghwan.park@magnachip.com>");
MODULE_VERSION(M1120_DRIVER_VERSION);
MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");

