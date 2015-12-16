/*
 * =====================================================================================
 *
 *       Filename:  drivers/leds/leds-lp5560.h
 *
 *    Description:  
 *
 *        Version:  0.1
 *        Created:  06/07/2013 03:50:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  LENOVO
 *        Company:  LENOVO Inc.
 *
 * =====================================================================================
 */

/*
 *     TCON            |         | TCAL        TTOFF               |
 *     +--+  +--+      |         | +--+  +----+    +------------+  +--+  +--+  +--+
 *     |  |  |  |      |         | |  |  |    |    |            |  |  |  |  |  |  |
 *     |  |  |  |      |         | | C|  | I  |    |            |  |  |  |  |  |  |
 *  ---+  +--+  +------+---------+-+  +--+    +----+            +--+  +--+  +--+  +------
 *     |  TCOFF        |         |         TTON                    |
 *     |    TENTER     |  TBLANK |                                 |
 *
 *	TENTER + TBLANK < 1500us                                        
 *	TCAL 350 ~ 8000us
 *	ndelay unit (us)
 */

#ifndef _INC_LEDS_LP5560_H_
#define _INC_LEDS_LP5560_H_                                             

#include <linux/types.h>

#define MAX_COUNT		    18

#define LP5560_ATTR_RO(_name) \
        DEVICE_ATTR(_name, S_IRUGO, show_##_name, NULL)
#define LP5560_ATTR_RW(_name) \
        DEVICE_ATTR(_name, S_IRUGO | S_IWUSR , show_##_name, store_##_name)


typedef enum LED_CMD_TAG
{
	LED_RESET = 0,
    LED_TURN_ON,
    LED_TURN_OFF,
    LED_FLASH,
    LED_FLASH_ONCE,
    LED_SET_CURRENT,
}LED_CMD;

struct pluse {
	u32 high;
	u32 low;
};

struct seq {
    u32 rise;
    u32 on;
    u32 fall;
    u32 off;
};

struct full_train {
	/* reset */
	struct pluse reset_start[2]; 
	struct pluse reset_end[3];
	/* train */
	struct pluse train_start[2]; 
	struct pluse calib;
	struct pluse cur;
	struct seq train_seq[3];
	struct pluse train_end[3];
};

union u_full_train {
	struct full_train train;
	u32 train_array[MAX_COUNT * 2];
};

#endif //_INC_LEDS_LP5560_H_
