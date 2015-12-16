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
#include "dsi_mod_boe_nt51021.h"
#include "lenovo_lcd_panel.h"

static unsigned char ce_status = 0x1E;
static void  boe_nt51021_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = 108;
		connector->display_info.height_mm = 172;
	}

	return;
}
static int boe_nt51021_get_effect_index(char *name)
{
	int i = 0;
	struct lcd_effect_data *effect_data= boe_nt51021_data.lcd_effects;

	for(i = 0; i < effect_data->supported_effects; i++)
		if(!strcmp(name, effect_data->lcd_effects[i].name))
			return i;

	return -1;
}
static int boe_nt51021_set_effect(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0, ret = 0;
	int effect_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_effect_data *effect_data= boe_nt51021_data.lcd_effects;
	struct lcd_effect effect = effect_data->lcd_effects[effect_index];
	struct lcd_effect_cmd effect_cmd = effect.lcd_effect_cmds[level];
	int cmd_nums = effect_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = true;

	//printk("[LCD]:==jinjt==%s line=%d\n",__func__,__LINE__);
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
            ce_status = 0x11;
        else
            ce_status = 0x1E;
    }
	printk("[LCD]:==jinjt==%s line=%d effect_index=%d level=%d\n",__func__,__LINE__,effect_index,level);
	return ret;

}

static int boe_nt51021_get_current_level(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= boe_nt51021_data.lcd_effects;
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

static int boe_nt51021_set_mode(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0, ret = 0;
	int mode_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_mode_data *mode_data= boe_nt51021_data.lcd_modes;
	struct lcd_mode mode = mode_data->lcd_modes[mode_index];
	struct lcd_mode_cmd mode_cmd = mode.lcd_mode_cmds[level];
	int cmd_nums = mode_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = true;

	/*if(level < 0 || level > effect.max_level)*/
		/*return -EINVAL;*/

	for(i = 0; i < cmd_nums; i++){
		cmd = mode_cmd.lcd_cmds[i];
		dsi_vc_dcs_write(dsi, 0, cmd.cmds, cmd.len);
	}

	//store the effect level
	mode_data->lcd_modes[mode_index].mode_status = 1;

	return ret;

}
int boe_nt51021_get_supported_mode(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_mode_data *mode_data= boe_nt51021_data.lcd_modes;
	struct lcd_mode *mode ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_mode *hal_lcd_mode;

	if(index >= mode_data->supported_modes)
		return -EINVAL;

	mode= &mode_data->lcd_modes[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_mode = &hal_panel_data->mode[index];
	/*hal_lcd_mode->mode_status = mode->mode_status;*/

	return 0;
}
static int boe_nt51021_get_supported_effect(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= boe_nt51021_data.lcd_effects;
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
static int boe_nt51021_get_effect_levels(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= boe_nt51021_data.lcd_effects;
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
static struct lcd_panel_dev boe_nt51021_panel_device = {
	.name = "boe_nt51021_1200x1920_panel",
	.status = OFF,
	.set_effect = boe_nt51021_set_effect,
	.get_current_level = boe_nt51021_get_current_level,
	.get_effect_index_by_name = boe_nt51021_get_effect_index,
    .set_mode = boe_nt51021_set_mode,
    .get_supported_mode = boe_nt51021_get_supported_mode,
    .get_supported_effect = boe_nt51021_get_supported_effect,
    .get_effect_levels = boe_nt51021_get_effect_levels,
};
static bool boe_nt51021_init(struct intel_dsi_device *dsi)
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
	printk("[LCD]:%s\n",__func__);

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 0;
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
	intel_dsi->dphy_reg = 0x3c1fc51f;

	intel_dsi->backlight_off_delay = 20;
	intel_dsi->backlight_on_delay = 80;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;

	boe_nt51021_panel_device.dsi = intel_dsi;
	lenovo_lcd_panel_register(&boe_nt51021_panel_device);
	dev_priv->mipi.panel_bpp = PIPE_24BPP;

	return true;
}

static void boe_nt51021_create_resources(struct intel_dsi_device *dsi) { }
static void boe_nt51021_enable(struct intel_dsi_device *dsi)
{

	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	printk("[LCD]:%s\n",__func__);
	mdelay(20);
	intel_dsi->hs=0;
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X01, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9F, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X97, 0XFF);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0XAA);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X11);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA9, 0X4B);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0XBB);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X22);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X90, 0X00);
	//dsi_vc_dcs_write_1(intel_dsi, 0, 0X90, 0XC0);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X91, 0XA2);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X95, 0X20);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X94, 0X2A);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9B, 0X8C);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X96, 0X00);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0XCC);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X33);
    /*dsi_vc_dcs_write_1(intel_dsi, 0, 0X90, 0X1F);*/
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X90, ce_status);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X92, 0X0F);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X93, 0X0A);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X94, 0X07);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X95, 0X09);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X96, 0X0d);
    dsi_vc_dcs_write_1(intel_dsi, 0, 0X97, 0X06);

    dsi_vc_dcs_write_1(intel_dsi, 0, 0X98, 0XB5);
	boe_nt51021_panel_device.status = ON;
}

static void boe_nt51021_disable(struct intel_dsi_device *dsi)
{
	//struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	boe_nt51021_panel_device.status = OFF;
	printk("[LCD]:%s\n",__func__);
}
static void boe_nt51021_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	printk("[LCD]:%s\n",__func__);

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

static int boe_nt51021_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static bool boe_nt51021_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode)
{
	return true;
}

static enum drm_connector_status boe_nt51021_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

static bool boe_nt51021_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static struct drm_display_mode *boe_nt51021_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("AUO B101UAN01E Panel: No memory\n");
		return NULL;
	}

	mode->hdisplay = 1200;
	mode->hsync_start = mode->hdisplay + 110; //fp
	mode->hsync_end = mode->hsync_start + 1; //sync
	mode->htotal = mode->hsync_end + 32;  //bp

	mode->vdisplay = 1920;
	mode->vsync_start = mode->vdisplay + 11;
	mode->vsync_end = mode->vsync_start + 1;
	mode->vtotal = mode->vsync_end + 14;

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

static void boe_nt51021_dump_regs(struct intel_dsi_device *dsi) { }
static void boe_nt51021_late_operations(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	printk("[LCD]:%s\n",__func__);
	intel_dsi->hs = true;
	//dsi_vc_dcs_write_1(intel_dsi, 0, 0X93, 0XC0);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X94, 0XB7);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X95, 0X21);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9F, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0XAA);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X11);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA0, 0x36);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA1, 0x36);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA2, 0x36);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0XA3, 0x36);
}

static void boe_nt51021_set_backlight(struct intel_dsi_device *dsi, u8 level)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	intel_dsi->hs = true;
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X83, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X84, 0X00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0X9F, level);
}

static void boe_nt51021_destroy(struct intel_dsi_device *dsi) { }


/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops boe_nt51021_dsi_display_ops = {
	.init = boe_nt51021_init,
	.get_info = boe_nt51021_vid_get_panel_info,
	.create_resources = boe_nt51021_create_resources,
	.dpms = boe_nt51021_dpms,
	.mode_valid = boe_nt51021_mode_valid,
	.mode_fixup = boe_nt51021_mode_fixup,
	.detect = boe_nt51021_detect,
	.get_hw_state = boe_nt51021_get_hw_state,
	.get_modes = boe_nt51021_get_modes,
	.destroy = boe_nt51021_destroy,
	.enable = boe_nt51021_enable,
	.dump_regs = boe_nt51021_dump_regs,
	.enable = boe_nt51021_enable,
	.late_operations = boe_nt51021_late_operations,
	.set_backlight_level = boe_nt51021_set_backlight,
	.disable = boe_nt51021_disable,
};
