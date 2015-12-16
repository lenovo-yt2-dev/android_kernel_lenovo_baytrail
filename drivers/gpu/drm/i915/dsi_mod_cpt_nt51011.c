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
#include "dsi_mod_cpt_nt51011.h"
#include "lenovo_lcd_panel.h"

static unsigned char ce_status = 0x55;
static void  cpt_nt51011_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = 135;
		connector->display_info.height_mm = 216;
	}

	return;
}
static int cpt_nt51011_get_effect_index(char *name)
{
	int i = 0;
	struct lcd_effect_data *effect_data= cpt_nt51011_data.lcd_effects;

	for(i = 0; i < effect_data->supported_effects; i++)
		if(!strcmp(name, effect_data->lcd_effects[i].name))
			return i;

	return -1;
}
static int cpt_nt51011_set_effect(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0, ret = 0;
	int effect_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_effect_data *effect_data= cpt_nt51011_data.lcd_effects;
	struct lcd_effect effect = effect_data->lcd_effects[effect_index];
	struct lcd_effect_cmd effect_cmd = effect.lcd_effect_cmds[level];
	int cmd_nums = effect_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = true;

	if(level < 0 || level > effect.max_level)
		return -EINVAL;

	for(i = 0; i < cmd_nums; i++){
		cmd = effect_cmd.lcd_cmds[i];
		dsi_vc_dcs_write(dsi, 0, cmd.cmds, cmd.len);
	}

	//store the effect level
	effect_data->lcd_effects[effect_index].current_level = level;
    if(effect_index == 1)
    {
        if(level == 0)
            ce_status = 0xAA;
        else
            ce_status = 0x55;
    }

    printk("[LCD]:==jinjt==%s line=%d effect_index=%d level=%d\n",__func__,__LINE__,effect_index,level);
	return ret;

}

static int cpt_nt51011_get_current_level(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= cpt_nt51011_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}

static struct lcd_panel_dev cpt_nt51011_panel_device = {
	.name = "cpt_nt51011_1920x1200_panel",
	.status = OFF,
	.set_effect = cpt_nt51011_set_effect,
	.get_current_level = cpt_nt51011_get_current_level,
	.get_effect_index_by_name = cpt_nt51011_get_effect_index,
};
static bool cpt_nt51011_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
    struct drm_device *dev = intel_dsi->base.base.dev;
    struct drm_i915_private *dev_priv = dev->dev_private;

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
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 0;
	intel_dsi->clock_stop =1;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SPULSE;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 1;
	intel_dsi->bw_timer = 0x820;
	intel_dsi->clk_lp_to_hs_count = 0xa;
	intel_dsi->clk_hs_to_lp_count = 0x14;
	intel_dsi->video_frmt_cfg_bits = 0;
	intel_dsi->panel_on_delay = 100;
	intel_dsi->dphy_reg = 0x3c1fc51f;

	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;

    dev_priv->mipi.panel_bpp = PIPE_24BPP;

    cpt_nt51011_panel_device.dsi = intel_dsi;
	lenovo_lcd_panel_register(&cpt_nt51011_panel_device);

	return true;
}

static void cpt_nt51011_create_resources(struct intel_dsi_device *dsi) { }

static void cpt_nt51011_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	if (enable) {

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

static int cpt_nt51011_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static bool cpt_nt51011_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode)
{
	return true;
}

static enum drm_connector_status cpt_nt51011_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

static bool cpt_nt51011_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static struct drm_display_mode *cpt_nt51011_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("AUO B101UAN01E Panel: No memory\n");
		return NULL;
	}

	mode->vdisplay = 1200;
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 4;

	mode->hdisplay = 1920;
	mode->hsync_start = mode->hdisplay + 48;
	mode->hsync_end = mode->hsync_start + 32;
	mode->htotal = mode->hsync_end + 40;

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

static void cpt_nt51011_dump_regs(struct intel_dsi_device *dsi) { }
static void cpt_enable(struct intel_dsi_device *dsi)
{

	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	printk("[LCD]:%s\n",__func__);
	mdelay(250);
	intel_dsi->hs=0;
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X15, 0X55);
	/*dsi_vc_dcs_write_1(intel_dsi, 0, 0X15, 0XAA);*/
	/*dsi_vc_dcs_write_1(intel_dsi, 0, 0X16, 0X55);*/
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X16, ce_status);

	cpt_nt51011_panel_device.status = ON;
}
static void cpt_disable(struct intel_dsi_device *dsi)
{
	cpt_nt51011_panel_device.status = OFF;
}
static void cpt_nt51011_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops cpt_nt51011_dsi_display_ops = {
	.init = cpt_nt51011_init,
	.get_info = cpt_nt51011_vid_get_panel_info,
	.create_resources = cpt_nt51011_create_resources,
	.dpms = cpt_nt51011_dpms,
	.mode_valid = cpt_nt51011_mode_valid,
	.mode_fixup = cpt_nt51011_mode_fixup,
	.detect = cpt_nt51011_detect,
	.get_hw_state = cpt_nt51011_get_hw_state,
	.get_modes = cpt_nt51011_get_modes,
	.destroy = cpt_nt51011_destroy,
	.dump_regs = cpt_nt51011_dump_regs,
	.enable = cpt_enable,
	.disable = cpt_disable,
};
