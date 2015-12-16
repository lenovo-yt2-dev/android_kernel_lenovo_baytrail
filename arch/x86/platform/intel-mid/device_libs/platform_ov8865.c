/*
 * platform_ov8865.c: ov8865 platform data initilization file
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
#include <linux/moduleparam.h>
#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_ov8865.h"

/* workround - pin defined for byt */
/* GP_CAMERASB09 is NO.15, the number is 102+15 = 117
 * So, for GP_CAMERASB09 = 117 + 9 = 126
 */
#define CAMERA_0_RESET 126	/* GP_CAMERASB09, Low to reset */
#define CAMERA_0_PWDN 123	/* GP_CAMERASB06, Low to power down*/
#define CAMERA_DOVDD_EN 122 /* GP_CAMERASB05, High to enable */
#define CAMERA_VCM_EN 119	/* GP_CAMERASB02, High to enable */
#define CAMERA_AVDD_EN 118	/* GP_CAMERASB01, High to enable */
#define CAMERA_DVDD_EN 121	/* GP_CAMERASB04, High to enable */

#define CAMERA_0_RESET_CRV2 CAMERA_0_RESET

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x0
#define CLK_19P2MHz 0x1
#endif
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66		/* +V2P85SX */
#define VPROG_1P8V 0x5D		/* +V1P8SX	*/
#define VPROG_3P3V 0x67		/* +V3P3A   */
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2
#endif
static int camera_vprog1_on;
static int camera_reset = -1;
static int camera_power_down = -1;
static int camera_vcm_en = -1;

extern int camera_dovdd_en;
extern int camera_avdd_en;
extern int camera_dvdd_en;


/*
 * MRFLD VV primary camera sensor - OV8865 platform data
 */
extern int blade_power_pins(void);
/*
#define OV8865_PLAT_DEBUG 1

#ifdef OV8865_PLAT_DEBUG
	#define OV8865_PLAT_LOG printk
#else
	static inline void OV8865_PLAT_LOG(1,const char *fmt, ...) {}
#endif
*/
#define OV8865_PLAT_LOG_LEVEL 1

static unsigned int log_level = OV8865_PLAT_LOG_LEVEL;
module_param(log_level, int, 0644);

#define OV8865_PLAT_LOG(level, a, ...) \
	do { \
		if (level < log_level) \
			printk(a,## __VA_ARGS__); \
	} while (0)
static void ov8865_verify_gpio_power(void)
{
	OV8865_PLAT_LOG(1,"CAMERA: start check ov8865 gpio\n");
	OV8865_PLAT_LOG(1,"CAMERA_1_RESET: %d\n", gpio_get_value(CAMERA_0_RESET));
	OV8865_PLAT_LOG(1,"CAMERA_1_PWDN: %d\n", gpio_get_value(CAMERA_0_PWDN));
	OV8865_PLAT_LOG(1,"CAMERA_DOVDD_EN: %d\n", gpio_get_value(CAMERA_DOVDD_EN));
	OV8865_PLAT_LOG(1,"CAMERA_AVDD_EN: %d\n", gpio_get_value(CAMERA_AVDD_EN));
	OV8865_PLAT_LOG(1,"CAMERA_DVDD_EN: %d\n", gpio_get_value(CAMERA_DVDD_EN));
	OV8865_PLAT_LOG(1,"VPROG_3P3V  addr:0x%x value:%x\n", VPROG_3P3V, intel_mid_pmic_readb(VPROG_3P3V));
	OV8865_PLAT_LOG(1,"VPROG_2P8V  addr:0x%x value:%x\n", VPROG_2P8V, intel_mid_pmic_readb(VPROG_2P8V));
	OV8865_PLAT_LOG(1,"VPROG_1P8V  addr:0x%x value:%x\n", VPROG_1P8V, intel_mid_pmic_readb(VPROG_1P8V));
}
static int ov8865_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (flag) {
		OV8865_PLAT_LOG(1,"%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_set_value(camera_power_down, 1); /* reset is active low */
		/* ov8865 reset pulse should be more than 2ms
		 */
		usleep_range(3500, 4000);

	} else {
		OV8865_PLAT_LOG(1,"%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_set_value(camera_power_down, 0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		usleep_range(2000, 2100);
	}

	return 0;
}

static int ov8865_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	OV8865_PLAT_LOG(1,"%s %d flag:%d\n", __func__, __LINE__, flag);
	ov8865_verify_gpio_power();
#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		int ret;
		ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	usleep_range(2000, 2500);
	OV8865_PLAT_LOG(1,"%s %d VLV2 PLAT sleep 2ms for clock to stable\n", __func__, __LINE__);
	return vlv2_plat_configure_clock(OSC_CAM0_CLK, flag);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
			flag ? clock_khz : 0);
#else
	pr_err("ov8865 clock is not set.\n");
	return 0;
#endif
}
static int ov8865_power_pins(void)
{
	int ret;

	if (IS_BYT) {
		blade_power_pins();
		if (camera_power_down < 0) {
			camera_power_down = CAMERA_0_PWDN;
			ret = gpio_request(camera_power_down, "camera_power_down");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, camera_power_down);
				return -EINVAL;
			}
			ret = gpio_direction_output(camera_power_down, 0);
		}

		if (camera_vcm_en < 0) {
			camera_vcm_en = CAMERA_VCM_EN;
			ret = gpio_request(camera_vcm_en, "camera_vcm_en");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep going\n",
				__func__, camera_vcm_en);
				//return -EINVAL;
			}
			ret = gpio_direction_output(camera_vcm_en, 0);
		}
		OV8865_PLAT_LOG(1,"%s %d camera_power_down:%d camera_dovdd_en:%d camera_vcm_en:%d camera_avdd_en:%d camera_dvdd_en:%d\n", __func__, __LINE__,
				camera_power_down, camera_dovdd_en, camera_vcm_en, camera_avdd_en, camera_dvdd_en);
	}
	if (!IS_BYT) {
		if (camera_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
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
				camera_reset = CAMERA_0_RESET_CRV2;
			else
				camera_reset = CAMERA_0_RESET;

			ret = gpio_request(camera_reset, "camera_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d) keep going\n",
				__func__, CAMERA_0_RESET);
				//return -EINVAL;
			}
			OV8865_PLAT_LOG(1,"CAMERA:%s %d camera_reset:%d\n", __func__, __LINE__, camera_reset);
			ret = gpio_direction_output(camera_reset, 0);
			if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, camera_reset);
			gpio_free(camera_reset);
		        }
		}
	#if 0
		OV8865_PLAT_LOG(1,"CAMERA:%s %d camera_reset:%d\n", __func__, __LINE__, camera_reset);
		ret = gpio_direction_output(camera_reset, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, camera_reset);
			gpio_free(camera_reset);
		}
	#endif
	}
	return 0;
}
static int ov8865_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int value = 0;

	ov8865_power_pins();
	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			/* AVDD on First */
			value = intel_mid_pmic_readb(VPROG_3P3V);
			value = value | VPROG_ENABLE;
			OV8865_PLAT_LOG(1,"try to enable VPROG_3P3V: value=0x%x\n", value);
			ret = intel_mid_pmic_writeb(VPROG_3P3V, value);

			usleep_range(2000, 2100);
			/* power up sequence control via GPIO*/
			gpio_set_value(camera_avdd_en, 1);
			usleep_range(1000, 1500);
			/* DOVDD and DVDD On*/
			OV8865_PLAT_LOG(1,"try to enable VPROG_1P8V\n");
			ret = intel_mid_pmic_writeb(VPROG_1P8V, VPROG_ENABLE);
			if (ret)
				return ret;
			gpio_set_value(camera_reset, 1); /* reset is active low */
		      /* ov8865 reset pulse should be more than 2ms
		      */
		      usleep_range(3500, 4000);
			/* VCM 2P8 power on*/
			OV8865_PLAT_LOG(1,"try to enable VPROG_2P8V\n");
			ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_ENABLE);
			if (ret)
				return ret;

		#if 0
			gpio_set_value(camera_power_down, 1);
			//gpio_set_value(camera_dovdd_en, 1);
			usleep_range(2500, 3500);
		#endif

			gpio_set_value(camera_dvdd_en, 1);
			//gpio_set_value(camera_vcm_en, 1);
			usleep_range(1000, 1500);

			#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("ov8865 power is not set.\n");
#endif
			if (!ret) {
				/* ov8865 VDIG rise to XCLR release */
				usleep_range(1000, 1200);
				camera_vprog1_on = 1;
			}
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			/* power down sequence control via GPIO*/
			gpio_set_value(camera_dvdd_en, 0);
			//gpio_set_value(camera_vcm_en, 0);
			usleep_range(2500, 3500);
			//gpio_set_value(camera_power_down, 0);
			//gpio_set_value(camera_dovdd_en, 0);

			/* power down power rail*/
			ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE);
			if (ret)
				return ret;
			ret = intel_mid_pmic_writeb(VPROG_1P8V, VPROG_DISABLE);
			gpio_set_value(camera_reset, 0);
			gpio_set_value(camera_avdd_en, 0);

			
			/* WA: We don't know previous status of VPROG_3P3V
			 * So keep it ON here
			 */

#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("ov8865 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}
	return ret;
}

static int ov8865_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static struct camera_sensor_platform_data ov8865_sensor_platform_data = {
	.gpio_ctrl      = ov8865_gpio_ctrl,
	.flisclk_ctrl   = ov8865_flisclk_ctrl,
	.power_ctrl     = ov8865_power_ctrl,
	.csi_cfg        = ov8865_csi_configure,
};

void *ov8865_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &ov8865_sensor_platform_data;
}
