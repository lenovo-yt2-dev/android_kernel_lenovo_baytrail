/*
 * extcon-fsa9285.c - FSA9285 extcon driver
 *
 * Copyright (C) 2013 Intel Corporation
 * Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/fs.h>
#include <linux/serial_core.h>
#include <linux/lnw_gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/usb/phy.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/power_supply.h>
#include <linux/extcon/extcon-fsa9285.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h> 
/* FSA9285 I2C registers */
#define FSA9285_REG_DEVID		0x01
#define FSA9285_REG_CTRL		0x02
#define FSA9285_REG_INTR		0x03
#define FSA9285_REG_INTR_MASK		0x05
#define FSA9285_REG_OHM_CODE		0x07
#define FSA9285_REG_TIMING		0x08
#define FSA9285_REG_STATUS		0x09
#define FSA9285_REG_DEVTYPE		0x0A
#define FSA9285_REG_DACSAR		0x0B
#define FSA9285_REG_MAN_SW		0x13
#define FSA9285_REG_MAN_CHGCTRL		0x14

/* device ID value */
#define DEVID_VALUE		0x10

/* Control */
#define CTRL_EN_DCD_TOUT	(1 << 7)
#define CTRL_EM_RESETB		(1 << 6)
#define CTRL_EM_ID_DIS		(1 << 5)
#define CTRL_EM_MAN_SW		(1 << 4)
#define CTRL_INT_MASK		(1 << 0)

/* Interrupts */
#define INTR_OCP_CHANGE		(1 << 6)
#define INTR_OVP_CHANGE		(1 << 5)
#define INTR_MIC_OVP		(1 << 4)
#define INTR_OHM_CHANGE		(1 << 3)
#define INTR_VBUS_CHANGE	(1 << 2)
#define INTR_BC12_DONE		(1 << 1)

/* resistance codes */
#define OHM_CODE_USB_SLAVE	0x16
#define OHM_CODE_UART		0x0C
#define OHM_CODE_USB_ACA	0x0A

/* Timing */
#define TIMING_500MS		0x6

/* Status */
#define STATUS_ID_SHORT		(1 << 7)
#define STATUS_OCP		(1 << 6)
#define STATUS_OVP		(1 << 5)
#define STATUS_MIC_OVP		(1 << 4)
#define STATUS_ID_NO_FLOAT	(1 << 3)
#define STATUS_VBUS_VALID	(1 << 2)
#define STATUS_DCD		(1 << 0)

/* Device Type */
#define DEVTYPE_DOCK		(1 << 5)
#define DEVTYPE_DCP		(1 << 2)
#define DEVTYPE_CDP		(1 << 1)
#define DEVTYPE_SDP		(1 << 0)
#define DEVTYPE_HCP		(1 << 3)
/*
 * Manual Switch
 * D- [7:5] / D+ [4:2]
 * 000: Open all / 001: HOST USB1 / 010: AUDIO / 011: HOST USB2 / 100: MIC
 * VBUS_IN SW[1:0]
 * 00: Open all/ 01: N/A / 10: VBUS_IN to MIC / 11: VBUS_IN to VBUS_OUT
 */
#define MAN_SW_DPDM_MIC			((4 << 5) | (4 << 2))
#define MAN_SW_DPDM_HOST2		((3 << 5) | (3 << 2))
/* same code can be used for AUDIO/UART */
#define MAN_SW_DPDM_UART		((2 << 5) | (2 << 2))
#define MAN_SW_DPDM_HOST1		((1 << 5) | (1 << 2))
#define MAN_SW_DPDP_AUTO		((0 << 5) | (0 << 2))
#define MAN_SW_VBUSIN_MASK		(3 << 0)
#define MAN_SW_VBUSIN_VOUT		(3 << 0)
#define MAN_SW_VBUSIN_MIC		(2 << 0)
#define MAN_SW_VBUSIN_AUTO		(0 << 0)

/* Manual Charge Control */
#define CHGCTRL_ASSERT_CHG_DETB		(1 << 4)
#define CHGCTRL_MIC_OVP_EN		(1 << 3)
#define CHGCTRL_ASSERT_DP		(1 << 2)

#define FSA_CHARGE_CUR_DCP		2000
#define FSA_CHARGE_CUR_ACA		2000
#define FSA_CHARGE_CUR_CDP		1500
#define FSA_CHARGE_CUR_SDP		500

#define FSA9285_EXTCON_SDP		"CHARGER_USB_SDP"
#define FSA9285_EXTCON_DCP		"CHARGER_USB_DCP"
#define FSA9285_EXTCON_CDP		"CHARGER_USB_CDP"
#define FSA9285_EXTCON_ACA		"CHARGER_USB_ACA"
#define FSA9285_EXTCON_DOCK		"Dock"
#define FSA9285_EXTCON_USB_HOST		"USB-Host"

#define MAX_RETRY			3

#define LC8x_SWITCH_SET_MODE (0)
#define SOC_USB_SWITCH_INT (131)
int  flag =0; //liulc1
int  mousetype=0; //liulc1 add
int fs_ok = 0;
int lenovo_charger_flag = 0;
int lenovo_h_charger_kthread_running =0; 
//extern int console_flag;
int swmode = 0; 
static struct file* lenovo_h_charger_uart_file = 0;
static int saved_loglevel=0; 
static const char *fsa9285_extcon_cable[] = {
	FSA9285_EXTCON_SDP,
	FSA9285_EXTCON_DCP,
	FSA9285_EXTCON_CDP,
	FSA9285_EXTCON_ACA,
	FSA9285_EXTCON_DOCK,
	FSA9285_EXTCON_USB_HOST,
	NULL,
};

struct fsa9285_chip {
	struct i2c_client	*client;
	struct fsa9285_pdata	*pdata;
	struct usb_phy		*otg;
	struct extcon_dev	*edev;

	/* reg data */
	u8	cntl;
	u8	man_sw;
	u8	man_chg_cntl;

	bool	vbus_drive;
	bool	a_bus_drop;
	struct delayed_work fsa9285_wrkr; //liulc1
	struct wake_lock wakelock;
};

static struct fsa9285_chip *chip_ptr;
extern void *fsa9285_platform_data(void);
extern  unsigned int read_vbus(); //liulc1
//void lenovo_charger_log_on(void);
static int lenovo_h_charger_uart_config(int baud_level);
static void lenovo_h_charger_uart_close(void);
static int lenovo_charger_detection(struct fsa9285_chip *chip);

static int fsa9285_write_reg(struct i2c_client *client,
		int reg, int value)
{
	int ret;
	int retry;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		else
			break;
	}

	return ret;
}

static int fsa9285_read_reg(struct i2c_client *client, int reg)
{
	int ret;
	int retry;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		else
			break;
	}

	return ret;
}

static void fsa9285_vbus_cntl_state(struct usb_phy *phy)
{
	struct fsa9285_chip *chip = chip_ptr;
	int ret = 0;

	if (!chip)
		return;

	if (phy->vbus_state == VBUS_DISABLED) {
		dev_info(&chip->client->dev,
			"a_bus_drop event(true)\n");
		chip->a_bus_drop = true;
		if (chip->vbus_drive)
			ret = chip->pdata->disable_vbus();
	} else {
		dev_info(&chip->client->dev,
			"a_bus_drop event(false)\n");
		chip->a_bus_drop = false;
		if (chip->vbus_drive)
			ret = chip->pdata->enable_vbus();
	}

	if (ret < 0)
		dev_warn(&chip->client->dev,
			"pmic vbus control failed\n");
}

static void usb_otg(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    int otg_vbus_mask = 1 ; //liulc1
    atomic_notifier_call_chain(&chip->otg->notifier,USB_EVENT_DRIVE_VBUS, &otg_vbus_mask); //liulc1
    mdelay(10); //liulc1

    printk("%s, USB-OTG, DP/DN switch to HOST\n", __func__);
    fsa9285_write_reg(client, 0x2, 0xEC);
}
//liulc1  add
static void usb_plug_out_otg_vbus(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
   int otg_vbus_mask = 0 ; //liulc1
   atomic_notifier_call_chain(&chip->otg->notifier,USB_EVENT_DRIVE_VBUS, &otg_vbus_mask); //liulc1
   mdelay(10); //liulc1

    fsa9285_write_reg(client, 0x2, 0xF8);
    fsa9285_write_reg(client, 0x5, 0x7F);
    fsa9285_write_reg(client, 0x5, 0x00);

    fsa9285_write_reg(client, 0x6, 0x6C); //liulc1 add

}

//liulc1 end
static void get_id(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    int id;
    
    printk("%s\n", __func__);
    
    // ADC always ON
    fsa9285_write_reg(client, 0x7, 0x44); //liulc1 add
    // ID read
    id = fsa9285_read_reg(client, 0x3);
    // ADC normal
    fsa9285_write_reg(client, 0x7, 0x40); //liulc1 add
    
    if(id == 0x10)
    {
        usb_otg(chip);
	flag=1;
    }
    
}

static void charger_detect_retry(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    
    printk("%s\n", __func__);
    
    // interrupt mask
    fsa9285_write_reg(client, 0x6, 0x7F);
    // charger re-detection OFF
    fsa9285_write_reg(client, 0x8, 0x0);
    // OVP, VBUS, CHG, ID mask off
    fsa9285_write_reg(client, 0x6, 0x6C); //liulc1 add
    //charger re-detection ON
    fsa9285_write_reg(client, 0x8, 0x1);
}

static void usb_plug_out(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    
    printk("%s\n", __func__);
   
if(swmode==0)  
{ 
    fsa9285_write_reg(client, 0x2, 0xF8);
    fsa9285_write_reg(client, 0x5, 0x7F);
    fsa9285_write_reg(client, 0x5, 0x00);
    
    fsa9285_write_reg(client, 0x6, 0x6C); //liulc1 add
    if (lenovo_charger_flag == 1)
    {
        lnw_gpio_set_alt(57, 1);
        lnw_gpio_set_alt(61, 1);
        //lenovo_h_charger_uart_config(115200);
        //lenovo_charger_log_on();
        //console_flag = 0;
        //lenovo_h_charger_uart_close();
        lenovo_charger_flag = 0;
	 //cancel_delayed_work_sync(&chip->fsa9285_wrkr); //liulc1
    }

}

}

static int lenovo_h_charger_uart_config(int baud_level)
{
        struct termios settings;
        mm_segment_t oldfs;
        printk("%s\n", __func__);
        struct file* temp=0;
        lenovo_h_charger_uart_file = filp_open("/dev/ttyS0", O_RDWR|O_NDELAY, 0664); //open file
        if(lenovo_h_charger_uart_file <= 0)
        {
                printk("%s Open /dev/console failed\n", __func__);
                return -1;
        }
        printk("%s Open /dev/ttyS0 sucessed\n", __func__);
        oldfs = get_fs();
        set_fs(KERNEL_DS);

        printk("Begin to set baudrate.\n");
        //set baud test

        lenovo_h_charger_uart_file->f_op->unlocked_ioctl(lenovo_h_charger_uart_file, TCGETS, (unsigned long)&settings);
        settings.c_cflag &= ~CBAUD;
        switch(baud_level)
        {
                case 300:
                        settings.c_cflag |= B300;
                        break;
                case 600:
                        settings.c_cflag |= B600;
                        break;
                case 0:
                case 921600:
                        settings.c_cflag |= B921600;
                        break;
                default:
                        settings.c_cflag |= B115200;
                        break;
        }

//	settings.c_cflag &= ~PARENB;
//	settings.c_cflag &= ~CSTOPB;
//	settings.c_cflag &= CSIZE;
//	settings.c_cflag |= CS8;
        settings.c_lflag &= ~ECHO;
        settings.c_lflag &= ~ICANON;
//	settings.c_lflag &= ~ECHOE;
//	settings.c_lflag &= ~ISIG;
//	settings.c_oflag &= ~OPOST;
        lenovo_h_charger_uart_file->f_op->unlocked_ioctl(lenovo_h_charger_uart_file, TCSETS, (unsigned long)&settings);
        set_fs(oldfs);
        return 1;

}

static int lenovo_h_charger_uart_write(unsigned char *data, int len)
{
        int num;
        mm_segment_t oldfs;
        {
                oldfs = get_fs();
                set_fs(KERNEL_DS);
                //write to TX
                num = lenovo_h_charger_uart_file->f_op->write(lenovo_h_charger_uart_file, data, len, &lenovo_h_charger_uart_file->f_pos);
                set_fs(oldfs);
                if(num != len)
                {
                        printk("write_byte_test error: len =%d, num = %d", len,num);
                }
        }

        return num;
}

static int lenovo_h_charger_uart_read(unsigned char *data)
{
        int len;
        mm_segment_t oldfs;
        struct termios settings;
        {
                oldfs = get_fs();
                set_fs(KERNEL_DS);
                len = lenovo_h_charger_uart_file->f_op->read(lenovo_h_charger_uart_file, data, 1024, &lenovo_h_charger_uart_file->f_pos);
                set_fs(oldfs);
        }
        return len;
}
#if 0
void lenovo_charger_log_on(void)
{
	console_flag = 0;
	return;
}
void lenovo_charger_log_off(void)
{
	console_flag = 1;
        return;
}

#endif

static void lenovo_h_charger_uart_close()
{
        if(lenovo_h_charger_uart_file >0)
        {
                filp_close(lenovo_h_charger_uart_file, NULL);
                lenovo_h_charger_uart_file = 0 ;
        }
}
static int lenovo_h_charger_switch(int val,struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;

    printk("%s\n", __func__);
    if (val ==0)
    {
	fsa9285_write_reg(client,0x0,0x1);
	fsa9285_write_reg(client, 0x2, 0xf8);
    }
    else
    {
	fsa9285_write_reg(client,0x0,0x1);
	fsa9285_write_reg(client, 0x2, 0xC8);
    }
    return 0;
}

static int lenovo_h_charger_config()
{
    printk("%s\n", __func__);
    return 0;
}

static int lenovo_h_charger_kthread(void *x)
{
    static char *cable;
    static struct power_supply_cable_props cable_props;
	lenovo_h_charger_kthread_running =1;
        while (fs_ok == 0)
        {
                msleep(100);
                printk("waiting for the user space file system......\n");
        }
        {
            if(lenovo_charger_detection(chip_ptr))
            {
		/*
                cable = FSA9285_EXTCON_DCP;
                cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
                cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
                cable_props.ma = FSA_CHARGE_CUR_DCP;
                if (!wake_lock_active(&chip_ptr->wakelock))
                        wake_lock(&chip_ptr->wakelock);
                atomic_notifier_call_chain(&power_supply_notifier,
                                        POWER_SUPPLY_CABLE_EVENT, &cable_props);
		*/
            }
        }
	

	lenovo_h_charger_kthread_running =0;
    return 1;
}

static int lenovo_charger_detection(struct fsa9285_chip *chip)
{
    int cnt = 0;
    int i;
    struct i2c_client *client = chip->client;
    int devtype =0;
    printk("%s\n", __func__);

    //Mute the debug infomations
    //lenovo_charger_log_off(); 
    //msleep(100);
    lenovo_h_charger_switch(1,chip); //switch the USB path to MIC-ON
    if(lenovo_h_charger_uart_config(600)==-1)
	return 1;
    mdelay(10);
            int len = 0;
            unsigned char buf[1024];
            int ret = 0;
            printk("Before ww_debug tx %d cnt=%d\n", len, i);
            len = lenovo_h_charger_uart_write("SC", 2);
	    mdelay(50);
	    lenovo_h_charger_uart_close();
            lnw_gpio_set_alt(57, 0);
            lnw_gpio_set_alt(61, 0);
            gpio_direction_output(57, 1);
            gpio_direction_output(61, 1);
	    lenovo_charger_flag =1;
	    return 1;
    
}

static int normal_charger_detection(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    int charger_type;
    int cdp_det;
    int devtype;
    charger_type = fsa9285_read_reg(client, 0x9);
    mousetype=charger_type; 
    printk("%s, charger_type=0x%x\n", __func__, charger_type);
    if(charger_type==0x0)   return 0;
    if(charger_type == 0x4)
    {
        cdp_det = fsa9285_read_reg(client, 0x8);
        cdp_det = (cdp_det & 0xFD) | 0x2; // read back -> clear bit[1] -> set bit[1] to 1 
        printk("%s, [W]cdp_det=0x%x\n", __func__, cdp_det);
        fsa9285_write_reg(client, 0x8, cdp_det);
        
        mdelay(100);
        cdp_det = fsa9285_read_reg(client, 0x8);
        
        printk("%s, [R]cdp_det=0x%x\n", __func__, cdp_det);
        
        if( (cdp_det & 0x04) == 0x04)
        {
            printk("%s, CDP detection\n", __func__);
	    devtype = DEVTYPE_CDP;
        }
        else
        {
            printk("%s, SDP detection\n", __func__);
	    devtype = DEVTYPE_SDP;
        }
        // CDP detection OFF
        cdp_det = fsa9285_read_reg(client, 0x8);
        cdp_det = (cdp_det & 0xFD); // 0b1111_1101 
        printk("%s, [W]cdp_det=0x%x\n", __func__, cdp_det);
        fsa9285_write_reg(client, 0x8, cdp_det);
        
        //Switch USB1-ON
        printk("%s DP/DM switch to USB-OTG-DEVICE\n", __func__);
	if(swmode!=1)
        fsa9285_write_reg(client, 0x2, 0xFC);
    }
    
    if (charger_type == 0x1)
    {
        printk("%s, DCP detection\n", __func__);
	devtype = DEVTYPE_DCP;
	//if (lenovo_charger_detection(chip)!=0)
	//	devtype = DEVTYPE_DCP;
	//if (lenovo_h_charger_kthread_running ==0)
	//	kthread_run(lenovo_h_charger_kthread, NULL, "lenovo_h_charger_kthread");
	
	schedule_delayed_work(&chip->fsa9285_wrkr,HZ*0); //liulc1
    }

    return devtype; 
}

static int charger_detect(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    int charger_type;
    
    charger_type = fsa9285_read_reg(client, 0x9);
  
    printk("%s, charger_type=0x%x\n", __func__, charger_type);
    charger_type = normal_charger_detection(chip);
 
    return charger_type;
    
}



static void ovp_accessory(struct fsa9285_chip *chip)
{

}

void dcp_plug_out(struct fsa9285_chip *chip)
{
    static struct power_supply_cable_props cable_props;


    cable_props.ma = 0;
    cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
    cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;

    usb_plug_out(chip);
    atomic_notifier_call_chain(&power_supply_notifier,POWER_SUPPLY_CABLE_EVENT, &cable_props);

}

static int fsa9285_detect_dev(struct fsa9285_chip *chip)
{
    struct i2c_client *client = chip->client;
    static bool notify_otg, notify_charger; 
    static char *cable;
    static struct power_supply_cable_props cable_props;
    int stat, devtype, ohm_code, cntl, intmask, ret;
    u8 w_man_sw, w_man_chg_cntl;
    bool discon_evt = false, drive_vbus = false;
    int vbus_mask = 0;
    int usb_switch = 1;
    unsigned int intr_status;
    unsigned int switch_status;
    
    // interrupt status read
    intr_status = fsa9285_read_reg(client, 0x4);
    
    // interrupt clear
    //fsa9285_write_reg(client, 0x5, intr_status);
    fsa9285_write_reg(client, 0x5, 0x7f); //liulc1 modify
    fsa9285_write_reg(client, 0x5, 0);
    
    //switch status read
    switch_status = fsa9285_read_reg(client, 0x1);
  
    printk("%s, intr_status=0x%x, switch_status=0x%x \n", __func__, 
                                                        intr_status, 
                                                        switch_status);
    devtype = 0;


    if(switch_status == 0xF8)
    {
		 /* disconnect event */
                discon_evt = true;
                /* usb switch off per nothing attached */
                usb_switch = 0;
                cable_props.ma = 0;
                cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
    	 if(flag==1)
  	 {
		flag=0;
		usb_plug_out_otg_vbus(chip);
	}
	else
	{	
        	usb_plug_out(chip);
	} 
   }
    else if(switch_status == 0xF9)
    {
        printk("OVP accessory\n", __func__);
        ovp_accessory(chip);
    }
    else if(switch_status == 0x80)
    {
        //Note: Reserve USB_ID pin to PHY, OTG handle device enumerate, LC8x handle USB DP/DM switch
        usb_otg(chip);
  	flag=1;
    }
    else
    {
        printk("%s, charge=0x%x, 0x%x\n", __func__, (switch_status>>3), (switch_status &0x4) );
        if( (switch_status >> 3) == 0x1F)
        {
            if( (switch_status & 0x05) == 0x4) //liulc1 add 
            {
                devtype = charger_detect(chip);
		if(mousetype==0x06)
               {
                       mousetype==0;
                       flag=0;
                       usb_plug_out_otg_vbus(chip);
                       return;
               }

            }
        }
    }



    if (devtype & DEVTYPE_SDP) {
		dev_info(&chip->client->dev,
				"SDP cable connecetd\n");
		/* select Host2 */
		notify_otg = true;
		vbus_mask = 1;
		notify_charger = true;
	        cable = FSA9285_EXTCON_SDP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
		cable_props.ma = FSA_CHARGE_CUR_SDP;
	} else if (devtype & DEVTYPE_CDP) {
		dev_info(&chip->client->dev,
				"CDP cable connecetd\n");
		/* select Host2 */
		notify_otg = true;
		vbus_mask = 1;
		notify_charger = true;
		cable = FSA9285_EXTCON_CDP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		cable_props.ma = FSA_CHARGE_CUR_CDP;
	} else if (devtype & DEVTYPE_DCP) {
		dev_info(&chip->client->dev,
				"DCP cable connecetd\n");
		notify_charger = true;
		cable = FSA9285_EXTCON_DCP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		cable_props.ma = FSA_CHARGE_CUR_DCP;
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
	} else if (devtype & DEVTYPE_HCP){
                dev_info(&chip->client->dev,
                              "HCP cable connecetd\n");
                notify_charger = true;
                cable = FSA9285_EXTCON_DCP;
                cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
                cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_HCP;
                cable_props.ma = FSA_CHARGE_CUR_DCP;
                if (!wake_lock_active(&chip->wakelock))
                        wake_lock(&chip->wakelock);

	}

        printk("%s, discon_evt=%d, notify_otg=%d, notify_charger=%d, vbus_mask=%d\n", 
                                                                   __func__, 
                                                                   discon_evt, 
                                                                   notify_otg, 
                                                                   notify_charger,
                                                                   vbus_mask);
    
    if (discon_evt) {
		if (notify_otg) {
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
			notify_otg = false;
		}
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
			notify_charger = false;
			cable = NULL;
		}
		if (wake_lock_active(&chip->wakelock))
			wake_unlock(&chip->wakelock);
	} else {
		if (notify_otg)
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
		}
	}

 
#if 0
	/* read status registers */
	ret = fsa9285_read_reg(client, FSA9285_REG_CTRL);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		cntl = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_DEVTYPE);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		devtype = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_STATUS);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		stat = ret;

	ret = fsa9285_read_reg(client, FSA9285_REG_OHM_CODE);
	if (ret < 0)
		goto dev_det_i2c_failed;
	else
		ohm_code = ret;

	dev_info(&client->dev, "devtype:%x, Stat:%x, ohm:%x cntl:%x\n",
				devtype, stat, ohm_code, cntl);

	/* set default register setting */
	w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST1;
	w_man_chg_cntl = chip->man_chg_cntl & ~CHGCTRL_ASSERT_CHG_DETB;

	if (stat & STATUS_ID_SHORT) {
		if (ohm_code == OHM_CODE_USB_SLAVE) {
			dev_info(&chip->client->dev,
				"USB slave device connecetd\n");
			drive_vbus = true;
		}
	} else if ((stat & STATUS_ID_NO_FLOAT) && (stat & STATUS_VBUS_VALID)) {
		if (ohm_code == OHM_CODE_UART) {
			dev_info(&chip->client->dev,
				"UART device connecetd\n");
			/* select UART */
			w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_UART;
		} else if (ohm_code == OHM_CODE_USB_ACA) {
			dev_info(&chip->client->dev,
				"ACA device connecetd\n");
			notify_charger = true;
			cable = FSA9285_EXTCON_ACA;
			cable_props.chrg_evt =
					POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			cable_props.chrg_type =
					POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
			cable_props.ma = FSA_CHARGE_CUR_ACA;
			if (!wake_lock_active(&chip->wakelock))
				wake_lock(&chip->wakelock);
		} else {
			/* unknown device */
			dev_warn(&chip->client->dev, "unknown ID detceted\n");
		}
	} else if (devtype & DEVTYPE_SDP) {
		dev_info(&chip->client->dev,
				"SDP cable connecetd\n");
		/* select Host2 */
		w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST2;
		w_man_chg_cntl = chip->man_chg_cntl | CHGCTRL_ASSERT_CHG_DETB;
		notify_otg = true;
		vbus_mask = 1;
	} else if (devtype & DEVTYPE_CDP) {
		dev_info(&chip->client->dev,
				"CDP cable connecetd\n");
		/* select Host2 */
		w_man_sw = (chip->man_sw & 0x3) | MAN_SW_DPDM_HOST2;
		notify_otg = true;
		vbus_mask = 1;
		notify_charger = true;
		cable = FSA9285_EXTCON_CDP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		cable_props.ma = FSA_CHARGE_CUR_CDP;
	} else if (devtype & DEVTYPE_DCP) {
		dev_info(&chip->client->dev,
				"DCP cable connecetd\n");
		notify_charger = true;
		cable = FSA9285_EXTCON_DCP;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		cable_props.ma = FSA_CHARGE_CUR_DCP;
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
	} else if (devtype & DEVTYPE_DOCK) {
		dev_info(&chip->client->dev,
				"Dock connecetd\n");
		notify_charger = true;
		cable = FSA9285_EXTCON_DOCK;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK;
		cable_props.ma = FSA_CHARGE_CUR_ACA;
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
	} else {
		dev_warn(&chip->client->dev,
			"ID or VBUS change event\n");
		if (stat & STATUS_VBUS_VALID)
			chip->cntl = cntl | CTRL_EN_DCD_TOUT;
		else
			chip->cntl = cntl & ~CTRL_EN_DCD_TOUT;

		ret = fsa9285_write_reg(client, FSA9285_REG_CTRL, chip->cntl);
		if (ret < 0)
			dev_warn(&chip->client->dev, "i2c write failed\n");
		/* disconnect event */
		discon_evt = true;
		/* usb switch off per nothing attached */
		usb_switch = 0;
		cable_props.ma = 0;
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
	}

	/* VBUS control */
	if (drive_vbus && !chip->a_bus_drop) {
		intmask = fsa9285_read_reg(client, FSA9285_REG_INTR_MASK);
		if (intmask < 0)
			goto dev_det_i2c_failed;

		/* disable VBUS interrupt */
		ret = fsa9285_write_reg(client, FSA9285_REG_INTR_MASK,
						intmask | INTR_VBUS_CHANGE);
		if (ret < 0)
			goto dev_det_i2c_failed;

		ret = chip->pdata->enable_vbus();

		/* clear interrupt */
		ret = fsa9285_read_reg(client, FSA9285_REG_INTR);
		if (ret < 0)
			dev_err(&chip->client->dev,
				"i2c read failed:%d\n", ret);

		/* enable VBUS interrupt */
		ret = fsa9285_write_reg(client, FSA9285_REG_INTR_MASK,
						intmask);
		if (ret < 0)
			goto dev_det_i2c_failed;

	} else
		ret = chip->pdata->disable_vbus();
	if (ret < 0)
		dev_warn(&chip->client->dev,
			"pmic vbus control failed\n");
	chip->vbus_drive = drive_vbus;

	/* handle SDP case before enabling CHG_DETB */
	if (w_man_chg_cntl & CHGCTRL_ASSERT_CHG_DETB)
		ret = chip->pdata->sdp_pre_setup();
	else if (!discon_evt)
		ret = chip->pdata->sdp_post_setup();

	if (ret < 0)
		dev_warn(&chip->client->dev,
			"sdp cable control failed\n");

	if (chip->pdata->xsd_gpio != -1) {
		if (usb_switch)
			gpio_direction_output(chip->pdata->xsd_gpio, 0);
		else
			gpio_direction_output(chip->pdata->xsd_gpio, 1);
	}

	if (chip->pdata->mux_gpio != -1) {
		if (vbus_mask)
			gpio_direction_output(chip->pdata->mux_gpio, 1);
		else
			gpio_direction_output(chip->pdata->mux_gpio, 0);
	}

	/* enable manual charge detection */
	ret = fsa9285_write_reg(client,
			FSA9285_REG_MAN_CHGCTRL, w_man_chg_cntl);
	if (ret < 0)
		goto dev_det_i2c_failed;

	/* select the controller */
	ret = fsa9285_write_reg(client, FSA9285_REG_MAN_SW, w_man_sw);
	if (ret < 0)
		goto dev_det_i2c_failed;

	if (discon_evt) {
		if (notify_otg) {
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
			notify_otg = false;
		}
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
			notify_charger = false;
			cable = NULL;
		}
		if (wake_lock_active(&chip->wakelock))
			wake_unlock(&chip->wakelock);
	} else {
		if (notify_otg)
			atomic_notifier_call_chain(&chip->otg->notifier,
						USB_EVENT_VBUS, &vbus_mask);
		if (notify_charger) {
			atomic_notifier_call_chain(&power_supply_notifier,
					POWER_SUPPLY_CABLE_EVENT, &cable_props);
		}
	}
#endif 

	return 0;

dev_det_i2c_failed:
	dev_err(&chip->client->dev, "i2c read failed:%d\n", ret);
	return ret;
}

static irqreturn_t fsa9285_irq_handler(int irq, void *data)
{
	struct fsa9285_chip *chip = data;
	struct i2c_client *client = chip->client;
	int ret;
        int intr_status;
    
#if 1
	pm_runtime_get_sync(&chip->client->dev);
	
        /* clear interrupt */
	//ret = fsa9285_read_reg(client, FSA9285_REG_INTR);
        printk("%s\n", __func__);
        // interrupt factor: 0x4H
        intr_status = fsa9285_read_reg(client, 0x4);
    
        if(intr_status == 0)
        {
            printk("%s, false alarm\n", __func__);
            goto isr_ret;
        }
        else
        {
            printk("%s intr_status=0x%x\n", __func__, intr_status);
        }

	//mdelay(10);
    
	/* device detection */
	ret = fsa9285_detect_dev(chip);
    
	if (ret < 0)
		dev_err(&chip->client->dev,
				"fsa9285 detecting devices failed:%d\n", ret);
isr_ret:
	pm_runtime_put_sync(&chip->client->dev);
#endif 

	return IRQ_HANDLED;
}

static int fsa9285_irq_init(struct fsa9285_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret, gpio_num, cntl, man_sw, man_chg_cntl;
	struct acpi_gpio_info gpio_info;
	int irq_gpio;
	int irq;
	int intr_status;

	fsa9285_write_reg(client, 0x0, 0x01);
        fsa9285_write_reg(client, 0x2, 0xF8);
	fsa9285_write_reg(client, 0x10, 0x0); //liulc1 add
        mdelay(100);

    	fsa9285_write_reg(client, 0x5, 0x7F);
	fsa9285_write_reg(client, 0x5, 0x0);
	fsa9285_write_reg(client, 0x6, 0x6C); //liulc1 add
	
    // interrupt factor: 0x4H
    intr_status = fsa9285_read_reg(client, 0x4);

	printk("%s, clear and enable interrupt, intr_status=0x%x\n", __func__, intr_status);

	irq_gpio = SOC_USB_SWITCH_INT; 
	irq = gpio_to_irq(irq_gpio);
	client->irq = irq ;  //liulc1
	ret = request_threaded_irq(irq, NULL,
				fsa9285_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"lc8260204", chip);

	if (ret) {
			dev_err(&client->dev, "failed to reqeust IRQ\n");
			return ret;
	}
	
	enable_irq_wake(client->irq);

	return 0;

irq_i2c_failed:
	dev_err(&chip->client->dev, "i2c read failed:%d\n", ret);
	return ret;
}
//liulc1  add for  proc


static ssize_t
read_proc(struct file *file, char __user *buf,
                  size_t size, loff_t *ppos)
{     
         *ppos=1;
       printk("%d\n",size);
	size = 1 ;
	//*buf = '1';
	copy_to_user(buf,"1",1);
	printk("liulc1 enable  UART3  mode OK\n");
	return size;
}

static ssize_t
write_proc(struct file *file, const char __user * buffer,
                   size_t count, loff_t *ppos)
{
        struct fsa9285_chip *chip = PDE_DATA(file_inode(file));
        struct i2c_client *client = chip->client;

        if(*buffer=='1')
	{
	   swmode = 1;
	   fsa9285_write_reg(client,0x0,0x1);
	   fsa9285_write_reg(client, 0x2, 0xC8);
           printk("liulc1 enable  UART3  mode\n");
	}
	else if(*buffer=='0')
	{  
	   swmode = 0;
	   fsa9285_write_reg(client, 0x2, 0xFC);
           printk("liulc1 enable  USB1  mode\n");
	}
        else if (*buffer =='3')
        {
                printk("FS OK\n");
                fs_ok = 1;
        }


    
        return count;
}

static const struct file_operations fops = {
        //.read = read_proc,
        .write = write_proc,
};



//liulc1 end


#if 1

static void fsa9285_detection_worker(struct work_struct *work)
{
        struct fsa9285_chip *chip =
            container_of(work, struct  fsa9285_chip, fsa9285_wrkr.work);
       struct i2c_client *client = chip->client;
       int i=0,ch;
      unsigned int sw_status;
       lenovo_charger_flag =1;
       if(lenovo_h_charger_kthread_running==1)
               return;
       else
               lenovo_h_charger_kthread_running=1;

       if(fs_ok == 0)
         {
               lenovo_h_charger_kthread_running=0;
                printk("waiting for the user space file system......\n");
               schedule_delayed_work(&chip->fsa9285_wrkr,HZ*1);
               return ;
         }


       if(read_vbus()<10500)
       {
               while(i<5)
               {
                       i++;
                       lnw_gpio_set_alt(57, 1);
                       lnw_gpio_set_alt(61, 1);
                       mdelay(10);
                       lenovo_h_charger_switch(1,chip); //switch the USB path to MIC-ON
                       if(lenovo_h_charger_uart_config(600)==-1)
                       {
				lenovo_h_charger_kthread_running=0;  
			        return;
			}
                       mdelay(10);
		
		        ch = fsa9285_read_reg(client, 0x08); //liulc1 add
			fsa9285_write_reg(client, 0x8, (ch & 0xfe)); //liulc1 add 
                       int len = 0;
                       len = lenovo_h_charger_uart_write("SC", 2);
                       printk("Before ww_debug tx %d len=%d\n", len, i);
                       mdelay(100);

                       lenovo_h_charger_uart_close();
                       lnw_gpio_set_alt(57, 0);
                       lnw_gpio_set_alt(61, 0);
                       gpio_direction_output(57, 1);
                       gpio_direction_output(61, 1);
                       msleep(1000);
			ch = fsa9285_read_reg(client, 0x08); //liulc1 add
                        fsa9285_write_reg(client, 0x8, (ch | 0x01)); //liulc1 add
			ch=read_vbus();
			if(ch < 4000)
			 {
				lenovo_h_charger_kthread_running=0;
				dcp_plug_out(chip);
                                return;				

			 }
                       if(ch > 10500)
                               break;
		       sw_status=fsa9285_read_reg(client, 0x01);
                       if(sw_status==0xf8)
			{
				lenovo_h_charger_kthread_running=0;
                                lenovo_h_charger_switch(0,chip); //switch the USB path to MIC-ON
				return;
			}
               }

               lenovo_h_charger_kthread_running=0;
               //schedule_delayed_work(&chip->fsa9285_wrkr,HZ*60);
       }
       else
       {
               printk("==liulc1===switch to 12V  OK\n");
               lenovo_h_charger_kthread_running=0;
               //schedule_delayed_work(&chip->fsa9285_wrkr,HZ*300);
       }

}


#endif


static int fsa9285_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct fsa9285_chip *chip;
	int ret = 0;
	int irq_gpio = 0;
	struct proc_dir_entry *entry; 

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	ret = fsa9285_read_reg(client, LC8x_SWITCH_SET_MODE);
        printk("%s, SWITCH_SET_MODE=0x%x\n", __func__, ret);

	chip = kzalloc(sizeof(struct fsa9285_chip), GFP_KERNEL);

	if (!chip) {
	    dev_err(&client->dev, "failed to allocate driver data\n");
	    return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = dev->platform_data;
	chip->pdata =   fsa9285_platform_data();
	i2c_set_clientdata(client, chip);
	chip_ptr = chip;

	/* register with extcon */
	chip->edev = kzalloc(sizeof(struct extcon_dev), GFP_KERNEL);

	if (!chip->edev) {
		dev_err(&client->dev, "mem alloc failed\n");
		ret = -ENOMEM;
		goto extcon_mem_failed;
	}
 
	chip->edev->name = "fsa9285";
	chip->edev->supported_cable = fsa9285_extcon_cable;
#if 1
	ret = extcon_dev_register(chip->edev, &client->dev);
	
        if (ret) {
		dev_err(&client->dev, "extcon registration failed!!\n");
		goto extcon_reg_failed;
	}

	/* OTG notification */
	chip->otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!chip->otg) {
		dev_warn(&client->dev, "Failed to get otg transceiver!!\n");
		goto otg_reg_failed;
	}
#endif 	
        //chip->otg->a_bus_drop = fsa9285_vbus_cntl_state;
	INIT_DELAYED_WORK(&chip->fsa9285_wrkr, fsa9285_detection_worker);  //liulc1 
	ret = fsa9285_irq_init(chip);
    
        get_id(chip);
        charger_detect_retry(chip);
    
	if (ret)
	    goto intr_reg_failed;

	wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND,
						"fsa_charger_wakelock");
	/* device detection */
        ret = fsa9285_detect_dev(chip);
    
	if (ret < 0)
		dev_warn(&client->dev, "probe: detection failed\n");

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);
//liulc1  add 
	entry = proc_create_data("driver/switch", S_IWUGO, NULL, &fops, chip);
        if (!entry) {
                printk("unable to create /proc entry-liulc2\n");
        }	
//liulc1 end
	return 0;

intr_reg_failed:
	if (client->irq)
		free_irq(client->irq, chip);
/* WA for FFRD8 */
//	if (chip->pdata->mux_gpio != -1)
//		gpio_free(chip->pdata->mux_gpio);
/* gpio_req_failed: */
	usb_put_phy(chip->otg);
otg_reg_failed:
	extcon_dev_unregister(chip->edev);
extcon_reg_failed:
	kfree(chip->edev);
extcon_mem_failed:
	kfree(chip);

	return ret;
}

static int fsa9285_remove(struct i2c_client *client)
{
	struct fsa9285_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->mux_gpio != -1)
		gpio_free(chip->pdata->mux_gpio);
	free_irq(client->irq, chip);
	usb_put_phy(chip->otg);
	extcon_dev_unregister(chip->edev);
	kfree(chip->edev);
	pm_runtime_get_noresume(&chip->client->dev);
	kfree(chip);
	return 0;
}

static void fsa9285_shutdown(struct i2c_client *client)
{
	dev_dbg(&client->dev, "fsa9285 shutdown\n");

	if (client->irq > 0)
		disable_irq(client->irq);
	return;
}

static int fsa9285_suspend(struct device *dev)
{
	struct fsa9285_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		disable_irq(chip->client->irq);
	}

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_resume(struct device *dev)
{
	struct fsa9285_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		enable_irq(chip->client->irq);
	}

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int fsa9285_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops fsa9285_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(fsa9285_suspend,
				fsa9285_resume)
		SET_RUNTIME_PM_OPS(fsa9285_runtime_suspend,
				fsa9285_runtime_resume,
				fsa9285_runtime_idle)
};

static const struct i2c_device_id fsa9285_id[] = {
	{"lc824206", 0},
//	{"SFSA9285", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fsa9285_id);

static const struct acpi_device_id acpi_fsa9285_id[] = {
//	{"SFSA9285", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, acpi_fsa9285_id);

static struct i2c_driver fsa9285_i2c_driver = {
	.driver = {
		.name = "lc824206",
		.owner	= THIS_MODULE,
		.pm	= &fsa9285_pm_ops,
		//.acpi_match_table = ACPI_PTR(acpi_fsa9285_id),
	},
	.probe = fsa9285_probe,
	.remove = fsa9285_remove,
	.id_table = fsa9285_id,
	.shutdown = fsa9285_shutdown,
};

/*
 * Module stuff
 */
static int __init fsa9285_extcon_early_init(void)
{
    int i2c_busnum = 3;
    struct i2c_board_info i2c_info;
    void *pdata = NULL;
    int ret;

    memset(&i2c_info, 0, sizeof(i2c_info));
    strlcpy(i2c_info.type, "lc824206", sizeof("lc824206"));

    i2c_info.addr = 0x48;

    pr_info("%s, I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                __func__, 
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    ret = i2c_register_board_info(i2c_busnum, &i2c_info, 1);

    printk("%s,ret = %d\n" ,__func__,ret );

    return ret;
}

static int __init fsa9285_extcon_late_init(void)
{
    int i2c_busnum = 3;
    struct i2c_board_info i2c_info;
    void *pdata = NULL;

    printk("-%s\n", __func__);
    
    int ret = i2c_add_driver(&fsa9285_i2c_driver);

    if (ret)
	printk(KERN_ERR "Unable to register ULPMC i2c driver\n");

#if 0    
    memset(&i2c_info, 0, sizeof(i2c_info));
    strlcpy(i2c_info.type, "lc824206", sizeof("lc824206"));

    i2c_info.addr = 0x48;

    pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    i2c_register_board_info(i2c_busnum, &i2c_info, 1);

#endif
    printk("-%s\n", __func__);
 
    return ret;

}

fs_initcall(fsa9285_extcon_early_init);
late_initcall(fsa9285_extcon_late_init);
//module_init(fsa9285_extcon_init);
static void __exit fsa9285_extcon_exit(void)
{
	i2c_del_driver(&fsa9285_i2c_driver);
}
module_exit(fsa9285_extcon_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("FSA9285 extcon driver");
MODULE_LICENSE("GPL");
