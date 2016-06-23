/* include/linux/cm36671.h
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

#include <linux/types.h>
#include <linux/ioctl.h>
 
#ifndef __LINUX_CM36671_H
#define __LINUX_CM36671_H


#define CM36671_I2C_NAME "cm36671"

/* Define Slave Address*/
#define	CM36671_slave_add	0xC0>>1


/*Define Command Code*/
#define		PS_CONF1      0x03
#define		PS_CONF3      0x04
#define		PS_CANC       0x05
#define		PS_THD        0x06

#define		PS_DATA       0x08
#define		INT_FLAG      0x0B
#define		ID_REG        0x0C

/*cm36671*/
/*for PS CONF1 command*/
#define CM36671_PS_ITB_1_2	 (0 << 14)
#define CM36671_PS_ITB_1     (1 << 14)
#define CM36671_PS_ITB_2     (2 << 14)
#define CM36671_PS_ITB_4     (3 << 14)
#define CM36671_PS_INIT_BITS     (3 << 12)
#define CM36671_PS_INT_OFF	       (0 << 8) /*enable/disable Interrupt*/
#define CM36671_PS_INT_IN          (1 << 8)
#define CM36671_PS_INT_OUT         (2 << 8)
#define CM36671_PS_INT_IN_AND_OUT  (3 << 8)

#define CM36671_PS_INT_MASK   0xFCFF

#define CM36671_PS_DR_1_40   (0 << 6)
#define CM36671_PS_DR_1_80   (1 << 6)
#define CM36671_PS_DR_1_160  (2 << 6)
#define CM36671_PS_DR_1_320  (3 << 6)
#define CM36671_PS_IT_1T 	   (0 << 4)
#define CM36671_PS_IT_1_3T   (1 << 4)
#define CM36671_PS_IT_1_6T 	 (2 << 4)
#define CM36671_PS_IT_2T 		 (3 << 4)
#define CM36671_PS_PERS_1 	 (0 << 2)
#define CM36671_PS_PERS_2 	 (1 << 2)
#define CM36671_PS_PERS_3 	 (2 << 2)
#define CM36671_PS_PERS_4 	 (3 << 2)
#define CM36671_PS_RES_1     (1 << 1)
#define CM36671_PS_SD	       (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36671_PS_SD_MASK	 0xFFFE

/*for PS CONF3 command*/
#define CM36671_PS_MS_NORMAL        (0 << 14)
#define CM36671_PS_MS_LOGIC_ENABLE  (1 << 14)
#define CM36671_PS_PROL_63 	     (0 << 12)
#define CM36671_PS_PROL_127      (1 << 12)
#define CM36671_PS_PROL_191 	   (2 << 12)
#define CM36671_PS_PROL_255 		 (3 << 12)
#define CM36671_PS_SMART_PERS_ENABLE  (1 << 4)
#define CM36671_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36671_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG           (1<<14)
#define INT_FLAG_ALS_IF_L            (1<<13)
#define INT_FLAG_ALS_IF_H            (1<<12)
#define INT_FLAG_PS_IF_CLOSE         (1<<9)
#define INT_FLAG_PS_IF_AWAY          (1<<8)  

#define PROXIMITYSENSOR_IOCTL_MAGIC 'c'

#define PROXIMITYSENSOR_IOCTL_GET_ENABLED _IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *)
#define PROXIMITYSENSOR_IOCTL_ENABLE _IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *)

#define PROXIMITYSENSOR "capella_ps"

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct cm36671_platform_data {
	int intr;
	uint16_t levels[10];
	uint16_t golden_adc;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr;
	uint8_t ps_close_thd_set;
	uint8_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;	
};
#ifdef __KERNEL__
#define PROXIMITYSENSOR "capella_ps"
struct capella_ps_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
	int (*enable)(uint8_t); /* enable to the chip */
	int p_en; /* proximity-sensor enable */
	int p_out; /* proximity-sensor outpu PROXIMITYSENSOR_IOCTL_ENABLE,t */
	int irq;
};
#endif /* __KERNEL__ */
#endif

