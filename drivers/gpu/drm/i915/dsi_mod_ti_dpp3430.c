/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_ti_dpp3430.h"
#ifdef CONFIG_DLP
extern int dlp3430_init(void);
extern int dlp3430_power_on(void);
extern int dlp3430_power_off(void);
#endif

void  dpp3430_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = 120;
		connector->display_info.height_mm = 192;
	}

	return;
}

bool dpp3430_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct mipi_phy_config config;
	bool ret = false;

	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */
	DRM_DEBUG_KMS("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 2;
	intel_dsi->eotp_pkt = 1;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 1;
	intel_dsi->bw_timer = 0x820;
	intel_dsi->clk_lp_to_hs_count = 0xa;
	intel_dsi->clk_hs_to_lp_count = 0x14;
	intel_dsi->video_frmt_cfg_bits = 0x8;
	intel_dsi->lp_rx_timeout = 0xffffff;

       //intel_dsi->hs_to_lp_count = 0x46;
       //intel_dsi->lp_byte_clk = 1;
       //intel_dsi->clk_lp_to_hs_count = 0xa;
       //intel_dsi->clk_hs_to_lp_count = 0x14;

	//intel_dsi->dphy_reg = 0x150c340f;
	//intel_dsi->dphy_reg = 0x280c340e;
	//intel_dsi->dphy_reg = 0x2c0c340e;
//	intel_dsi->dphy_reg = 0x3f0c3411;

 	config.tclk_prepare = 69;
       //config.tclk_trail = 80;
       config.tclk_trail = 69;
       config.tclk_prepare_clkzero = 400;
       //config.ths_prepare = 69;
       config.ths_prepare = 96; 
       config.ths_trail = 70;
       //config.ths_prepare_hszero = (255 + 69);
       config.ths_prepare_hszero = (365 + 96);//raw form ti 372+93; overflow ,after ti confirm 365 + 99; wait ti confirm;
       ret = intel_dsi_generate_phy_reg(intel_dsi, &config);
       printk("mipi_timming intel_dsi->dphy_reg:%x;ret :%d\n ",intel_dsi->dphy_reg,ret  );
       if (ret == false) {
               DRM_ERROR("intel_dsi_generate_phy_reg failed.\n");
               intel_dsi->hs_to_lp_count = 0x46;
               intel_dsi->lp_byte_clk = 1;
               intel_dsi->clk_lp_to_hs_count = 0xa;
               intel_dsi->clk_hs_to_lp_count = 0x14;
               //intel_dsi->dphy_reg = 0x2c0c340e;
	       intel_dsi->dphy_reg = 0x3f0c3511;
       }



	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;
       intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
        intel_dsi->port = 1; /* PORT_A by default */
	intel_dsi->burst_mode_ratio = 100;
	//intel_dsi->dsi_clock_freq = 300;

	return true;
}

void dpp3430_create_resources(struct intel_dsi_device *dsi) { }
void dpp3430_enable(struct intel_dsi_device *dsi)
{
	printk("====>dpp3430_enable\n");
	printk("====>dpp3430 debug version 0.01\n");
#ifdef CONFIG_DLP
	dlp3430_init();
#endif
}


void dpp3430_disable(struct intel_dsi_device *dsi)
{
	printk("====>dpp3430_disable\n");
#ifdef CONFIG_DLP
	dlp3430_power_off();
#endif
}

void dpp3430_reset(struct intel_dsi_device *dsi)
{
	printk("====>dpp3430_reset\n");
#ifdef CONFIG_DLP	
	dlp3430_power_on();
#endif
}

void dpp3430_dpms(struct intel_dsi_device *dsi, bool enable)
{

	DRM_DEBUG_KMS("\n");
#if 0
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	if (enable) {

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);
	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
#endif
}

int dpp3430_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool dpp3430_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode)
{
     struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
    intel_dsi->pclk = adjusted_mode->clock;
     DRM_DEBUG_KMS("pclk : %d\n", intel_dsi->pclk);
	return true;
}

enum drm_connector_status dpp3430_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

bool dpp3430_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *dpp3430_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("AUO B101UAN01E Panel: No memory\n");
		return NULL;
	}

	mode->hdisplay = 854;
	mode->hsync_start = mode->hdisplay + 64; //fp
	mode->hsync_end = mode->hsync_start + 10;	//26; //sync
	mode->htotal = mode->hsync_end + 34;  //bp

	mode->vdisplay = 480;
	mode->vsync_start = mode->vdisplay + 8;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 6;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	/* Configure */
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	printk("%s drm debug Loaded mode=%dx%d\n",__func__,
		mode->hdisplay, mode->vdisplay);

	return mode;
}

void dpp3430_dump_regs(struct intel_dsi_device *dsi) { }

void dpp3430_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops ti_dpp3430_dsi_display_ops = {
	.init = dpp3430_init,
	.get_info = dpp3430_vid_get_panel_info,
	.create_resources = dpp3430_create_resources,
	.dpms = dpp3430_dpms,
	.mode_valid = dpp3430_mode_valid,
	.mode_fixup = dpp3430_mode_fixup,
	.detect = dpp3430_detect,
	.get_hw_state = dpp3430_get_hw_state,
	.get_modes = dpp3430_get_modes,
	.destroy = dpp3430_destroy,
	.enable = dpp3430_enable,
	.dump_regs = dpp3430_dump_regs,
	.enable = dpp3430_enable,
	.disable = dpp3430_disable,
	.panel_reset = dpp3430_reset,
};
