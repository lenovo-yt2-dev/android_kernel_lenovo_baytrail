/*
 * platform_ov9760.c: ov9760 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/mfd/intel_mid_pmic.h>

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_ov9760.h"

/* workround - pin defined for byt */
/* GP_CAMERASB09 is NO.15, the number is 102+15 = 117
 * So, for GP_CAMERASB09 = 117 + 9 = 126
 */
#define CAMERA_1_RESET 127	/* GP_CAMERASB10, Low to reset */
#define CAMERA_1_PWDN 124	/* GP_CAMERASB07, Low to power down*/
#define CAMERA_DOVDD_EN 0	/* GP_CAMERASB05, High to enable */
#define CAMERA_AVDD_EN 51	/* GPIO_S0_51, High to enable */
#define CAMERA_DVDD_EN 125	/* GP_CAMERASB08, High to enable */

#define CAMERA_1_RESET_CRV2 CAMERA_1_RESET

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
#endif
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66		/* +V2P85SX */
#define VPROG_1P8V 0x5D		/* +V1P8SX	*/
#define VPROG_3P3V 0x67		/* +V3P3A   */
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2
#endif
static int camera_reset = -1;
static int camera_power_down = -1;
static int camera_dovdd_en = -1;
static int camera_avdd_en = -1;
static int camera_dvdd_en = -1;

static int camera_vprog1_on = 0;
/*
 * MRFLD VV primary camera sensor - OV9760 platform data
 */
#ifdef OV8865_PLAT_DEBUG
	#define OV9760_PLAT_LOG OV9760_PLAT_LOG
#else
	static inline void OV9760_PLAT_LOG(const char *fmt, ...) {}
#endif
	
static void ov9760_verify_gpio_power(void)
{
	OV9760_PLAT_LOG("CAMERA: start check ov9760 gpio\n");
	OV9760_PLAT_LOG("CAMERA_1_RESET: %d\n", gpio_get_value(CAMERA_1_RESET));
	OV9760_PLAT_LOG("CAMERA_1_PWDN: %d\n", gpio_get_value(CAMERA_1_PWDN));
	OV9760_PLAT_LOG("CAMERA_DOVDD_EN: %d\n", gpio_get_value(CAMERA_DOVDD_EN));
	OV9760_PLAT_LOG("CAMERA_AVDD_EN: %d\n", gpio_get_value(CAMERA_AVDD_EN));
	OV9760_PLAT_LOG("CAMERA_DVDD_EN: %d\n", gpio_get_value(CAMERA_DVDD_EN));
	OV9760_PLAT_LOG("VPROG_3P3V  addr:0x%x value:%x\n", VPROG_3P3V, intel_mid_pmic_readb(VPROG_3P3V));
	OV9760_PLAT_LOG("VPROG_2P8V  addr:0x%x value:%x\n", VPROG_2P8V, intel_mid_pmic_readb(VPROG_2P8V));
	OV9760_PLAT_LOG("VPROG_1P8V  addr:0x%x value:%x\n", VPROG_1P8V, intel_mid_pmic_readb(VPROG_1P8V));
}
static int ov9760_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (!IS_BYT) {
		if (camera_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			camera_reset = ret;
		}
	} else {
		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently
		 */
		if (camera_reset < 0) {
			if (spid.hardware_id == BYT_TABLET_BLK_CRV2)
				camera_reset = CAMERA_1_RESET_CRV2;
			else
				camera_reset = CAMERA_1_RESET;

			ret = gpio_request(camera_reset, "camera_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep going\n",
				__func__, CAMERA_1_RESET);
				return -EINVAL;
			}
		}

		printk("CAMERA:%s %d camera_reset:%d\n", __func__, __LINE__, camera_reset);
		ret = gpio_direction_output(camera_reset, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, camera_reset);
			gpio_free(camera_reset);
		}
	}
	if (flag) {
		OV9760_PLAT_LOG("%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_set_value(camera_reset, 1); /* reset is active low */
		/* ov9760 reset pulse should be more than 2ms
		 */
		usleep_range(3500, 4000);

	} else {
		OV9760_PLAT_LOG("%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_set_value(camera_reset, 0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		usleep_range(2000, 2100);
	}

	return 0;
}

static int ov9760_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	ov9760_verify_gpio_power();
	OV9760_PLAT_LOG("%s %d flag:%d\n", __func__, __LINE__, flag);
#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		int ret;
		ret = vlv2_plat_set_clock_freq(OSC_CAM1_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	return vlv2_plat_configure_clock(OSC_CAM1_CLK, flag);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
			flag ? clock_khz : 0);
#else
	pr_err("ov9760 clock is not set.\n");
	return 0;
#endif
}

int blade_power_pins(void)
{
	int ret;

	if (IS_BYT) {
		if (camera_dovdd_en < 0) {
			camera_dovdd_en = CAMERA_DOVDD_EN;
			ret = gpio_request(camera_dovdd_en, "camera_dovdd_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep goign\n",
				__func__, camera_dovdd_en);
			}
			ret = gpio_direction_output(camera_dovdd_en, 0);
		}
		if (camera_avdd_en < 0) {
			camera_avdd_en = CAMERA_AVDD_EN;
			ret = gpio_request(camera_avdd_en, "camera_avdd_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, camera_avdd_en);
			}
			ret = gpio_direction_output(camera_avdd_en, 0);
		}
		if (camera_dvdd_en < 0) {
			camera_dvdd_en = CAMERA_DVDD_EN;
			ret = gpio_request(camera_dvdd_en, "camera_dvdd_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep going\n",
				__func__, camera_dvdd_en);
			}
			ret = gpio_direction_output(camera_dvdd_en, 0);
		}
	}

	return ret;
}
int ov9760_power_pins(void)
{
	int ret;

	if (IS_BYT) {
		//blade_power_pins();

		if (camera_avdd_en < 0) {
			camera_avdd_en = CAMERA_AVDD_EN;
			ret = gpio_request(camera_avdd_en, "camera_avdd_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, camera_avdd_en);
				return -EINVAL;
			}
			ret = gpio_direction_output(camera_avdd_en, 0);
		}


		if (camera_dvdd_en < 0) {
			camera_dvdd_en = CAMERA_DVDD_EN;
			ret = gpio_request(camera_dvdd_en, "camera_dvdd_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, camera_dvdd_en);
				return -EINVAL;
			}
			ret = gpio_direction_output(camera_dvdd_en, 0);
		}
		
		if (camera_power_down < 0) {
			camera_power_down = CAMERA_1_PWDN;
			ret = gpio_request(camera_power_down, "camera_power_down");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep going\n",
				__func__, camera_power_down);
			}
			ret = gpio_direction_output(camera_power_down, 1);
		}
	}
	OV9760_PLAT_LOG("%s %d camera_power_down:%d camera_dovdd_en:%d camera_avdd_en:%d camera_dvdd_en:%d\n", __func__, __LINE__,
		camera_power_down, camera_dovdd_en, camera_avdd_en, camera_dvdd_en);

	return 0;
}


static int ov9760_power_pins_free(void)
{
	int ret = -1;
	if(IS_BYT){
	if (camera_avdd_en >= 0) {
			gpio_free(camera_avdd_en);
			camera_avdd_en = -1;
			printk("free camera  gpio %s  %d ",__FUNCTION__,__LINE__);
		}
	
	if (camera_dvdd_en >= 0) {
			gpio_free(camera_dvdd_en);
			camera_dvdd_en = -1;
			printk("free camera  gpio %s  %d ",__FUNCTION__,__LINE__);
		}

	
	if (camera_power_down >= 0) {
			gpio_free(camera_power_down);
			camera_power_down = -1;
			printk("free camera  gpio %s  %d ",__FUNCTION__,__LINE__);
		}
	
		ret = 0;
	}
	
	return ret;
}



static int ov9760_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int value = 0;

	if(flag == camera_vprog1_on){
		printk("%s %d  flag== camera_vprog1_on,return and do nothing  flag= %d\n",__FUNCTION__,__LINE__,flag);
		return 0;
  	}
	
	ret = ov9760_power_pins();
	if(ret)
			goto free_gpio;
	
	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			/* AVDD on First */
			value = intel_mid_pmic_readb(VPROG_3P3V);
			value = value | VPROG_ENABLE;
			OV9760_PLAT_LOG("try to enable VPROG_3P3V: value=0x%x\n", value);
			ret = intel_mid_pmic_writeb(VPROG_3P3V, value);
			gpio_set_value(camera_avdd_en, 1);

			usleep_range(2000, 2100);
			/* DOVDD and DVDD On*/
			OV9760_PLAT_LOG("try to enable VPROG_1P8V\n");
			ret = intel_mid_pmic_writeb(VPROG_1P8V, VPROG_ENABLE);
			if(ret){
				printk("1p8v failed   %s  %d  ",__FUNCTION__,__LINE__);
				goto fail_1p8v;
			}
			usleep_range(2000, 2100);
			gpio_set_value(camera_dvdd_en, 1);
			
			if (ret)
				return ret;
			/* VCM 2P8 power on*/
			OV9760_PLAT_LOG("try to enable VPROG_2P8V\n");
			ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_ENABLE);
			if (ret)
				goto fail_1p8v;
			/* power up sequence control via GPIO*/

			gpio_set_value(camera_power_down, 0);


#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("ov9760 power is not set.\n");
#endif
			if (!ret) {
				/* ov9760 VDIG rise to XCLR release */
				usleep_range(1000, 1200);
				camera_vprog1_on = 1;
			}
			goto free_gpio;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			/* power down sequence control via GPIO*/
			gpio_set_value(camera_dvdd_en, 0);
			gpio_set_value(camera_avdd_en, 0);
			gpio_set_value(camera_power_down, 1);
			ret = intel_mid_pmic_writeb(VPROG_1P8V, VPROG_DISABLE);//active here to fix the current leak
			/* power down power rail*/

			/*
			ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE);
			if (ret)
				return ret;
			ret = intel_mid_pmic_writeb(VPROG_1P8V, VPROG_DISABLE);

			*/
			/* WA: We don't know previous status of VPROG_3P3V
			 * So keep it ON here
			 */

#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("ov9760 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 0;
				
			goto free_gpio;
		}
	}

fail_1p8v:
	intel_mid_pmic_writeb(VPROG_1P8V, VPROG_DISABLE);
	printk("VPROG_DISABLE  %s  %d  ret = %d\n",__FUNCTION__,__LINE__,ret);
	
free_gpio:	
	ret = ov9760_power_pins_free();
	printk("ov9760 gpio free %s  %d  ret = %d\n",__FUNCTION__,__LINE__,ret);
	return ret;
}

static int ov9760_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static struct camera_sensor_platform_data ov9760_sensor_platform_data = {
	.gpio_ctrl      = ov9760_gpio_ctrl,
	.flisclk_ctrl   = ov9760_flisclk_ctrl,
	.power_ctrl     = ov9760_power_ctrl,
	.csi_cfg        = ov9760_csi_configure,
};

void *ov9760_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &ov9760_sensor_platform_data;
}