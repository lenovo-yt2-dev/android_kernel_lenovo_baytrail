/*
 * Copyright © 2006-2010 Intel Corporation
 * Copyright (c) 2006 Dave Airlie <airlied@linux.ie>
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
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *      Dave Airlie <airlied@linux.ie>
 *      Jesse Barnes <jesse.barnes@intel.com>
 *      Chris Wilson <chris@chris-wilson.co.uk>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/moduleparam.h>
#include "intel_drv.h"
#include "linux/mfd/intel_mid_pmic.h"
#include <linux/pwm.h>
#include <linux/platform_data/lp855x.h>
#include <asm/spid.h>
#include "intel_dsi.h"
#include <linux/gpio.h>

#define PCI_LBPC 0xf4 /* legacy/combination backlight modes */
#define GPIO_BOARD_ID 210
#define PMIC_GPIO1P0_BASE_ADDRESS 0x3b
#define PMIC_GPIO1P2_OFFSET 0x2
#define PMIC_GPIO1P2_ADDRESS PMIC_GPIO1P0_BASE_ADDRESS+PMIC_GPIO1P2_OFFSET
#define GPIO_INPUT_NO_DRV 0x0
static unsigned char backlight_buf[]={
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x02,
0x02,0x02,0x02,0x02,0x02,0x03,0x03,0x03,0x03,0x03,
0x04,0x04,0x04,0x04,0x05,0x05,0x06,0x06,0x07,0x07,
0x07,0x08,0x08,0x08,0x09,0x09,0x0A,0x0A,0x0B,0x0B,
0x0C,0x0C,0x0D,0x0D,0x0E,0x0E,0x0F,0x0F,0x10,0x10,
0x11,0x11,0x12,0x12,0x13,0x13,0x14,0x15,0x16,0x17,
0x18,0x19,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1E,0x1F,
0x1F,0x20,0x20,0x21,0x21,0x22,0x23,0x24,0x25,0x26,
0x27,0x27,0x28,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,
0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,
0x39,0x39,0x3A,0x3A,0x3B,0x3C,0x3D,0x3D,0x3F,0x3F,
0x41,0x41,0x43,0x43,0x45,0x45,0x47,0x47,0x48,0x48,
0x49,0x49,0x4A,0x4A,0x4B,0x4B,0x4D,0x4D,0x4E,0x4E,
0x4F,0x4F,0x51,0x51,0x53,0x54,0x55,0x56,0x56,0x58,
0x58,0x5B,0x5B,0x5D,0x5D,0x60,0x60,0x63,0x63,0x65,
0x65,0x66,0x67,0x68,0x68,0x69,0x6A,0x6B,0x6C,0x6D,
0x6D,0x6E,0x6E,0x6F,0x6F,0x70,0x70,0x71,0x71,0x74,
0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,
0x7F,0x80,0x81,0x82,0x84,0x85,0x86,0x88,0x89,0x8A,
0x8C,0x8D,0x8E,0x90,0x91,0x92,0x94,0x95,0x96,0x98,
0x99,0x9A,0x9D,0x9E,0x9F,0xA1,0xA2,0xA3,0xA6,0xA7,
0xA8,0xAA,0xAB,0xAC,0xAF,0xB0,0xB1,0xB4,0xB5,0xB6,
0xBA,0xBB,0xBC,0xBF,0xC0,0xC1,0xC2,0xC4,0xC6,0xCB,
0xCC,0xCE,0xCF,0xD1,0xD2,0xD4,0xD5,0xD7,0xD9,0xDA,
0xDD,0xDE,0xDF,0xE3,0xE4,0xE5,0xEA,0xEB,0xED,0xEE,
0xF0,0xF1,0xF7,0xF8,0xFE,0xFF
};

static bool backlight_level0 = false;
/*static bool power_on_init = true;*/
extern struct intel_dsi_device *intel_dsi_dev;
void
intel_fixed_panel_mode(struct drm_display_mode *fixed_mode,
		       struct drm_display_mode *adjusted_mode)
{
	adjusted_mode->hdisplay = fixed_mode->hdisplay;
	adjusted_mode->hsync_start = fixed_mode->hsync_start;
	adjusted_mode->hsync_end = fixed_mode->hsync_end;
	adjusted_mode->htotal = fixed_mode->htotal;

	adjusted_mode->vdisplay = fixed_mode->vdisplay;
	adjusted_mode->vsync_start = fixed_mode->vsync_start;
	adjusted_mode->vsync_end = fixed_mode->vsync_end;
	adjusted_mode->vtotal = fixed_mode->vtotal;

	adjusted_mode->clock = fixed_mode->clock;
}

/* adjusted_mode has been preset to be the panel's fixed mode */
void
intel_pch_panel_fitting(struct intel_crtc *intel_crtc,
			struct intel_crtc_config *pipe_config,
			int fitting_mode)
{
	struct drm_display_mode *mode, *adjusted_mode;
	int x, y, width, height;

	mode = &pipe_config->requested_mode;
	adjusted_mode = &pipe_config->adjusted_mode;

	x = y = width = height = 0;

	/* Native modes don't need fitting */
	if (adjusted_mode->hdisplay == mode->hdisplay &&
	    adjusted_mode->vdisplay == mode->vdisplay)
		goto done;

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		width = mode->hdisplay;
		height = mode->vdisplay;
		x = (adjusted_mode->hdisplay - width + 1)/2;
		y = (adjusted_mode->vdisplay - height + 1)/2;
		break;

	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		{
			u32 scaled_width = adjusted_mode->hdisplay * mode->vdisplay;
			u32 scaled_height = mode->hdisplay * adjusted_mode->vdisplay;
			if (scaled_width > scaled_height) { /* pillar */
				width = scaled_height / mode->vdisplay;
				if (width & 1)
					width++;
				x = (adjusted_mode->hdisplay - width + 1) / 2;
				y = 0;
				height = adjusted_mode->vdisplay;
			} else if (scaled_width < scaled_height) { /* letter */
				height = scaled_width / mode->hdisplay;
				if (height & 1)
				    height++;
				y = (adjusted_mode->vdisplay - height + 1) / 2;
				x = 0;
				width = adjusted_mode->hdisplay;
			} else {
				x = y = 0;
				width = adjusted_mode->hdisplay;
				height = adjusted_mode->vdisplay;
			}
		}
		break;

	case DRM_MODE_SCALE_FULLSCREEN:
		x = y = 0;
		width = adjusted_mode->hdisplay;
		height = adjusted_mode->vdisplay;
		break;

	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

done:
	pipe_config->pch_pfit.pos = (x << 16) | y;
	pipe_config->pch_pfit.size = (width << 16) | height;
}

static void
centre_horizontally(struct drm_display_mode *mode,
		    int width)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the hsync and hblank widths constant */
	sync_width = mode->crtc_hsync_end - mode->crtc_hsync_start;
	blank_width = mode->crtc_hblank_end - mode->crtc_hblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->hdisplay - width + 1) / 2;
	border += border & 1; /* make the border even */

	mode->crtc_hdisplay = width;
	mode->crtc_hblank_start = width + border;
	mode->crtc_hblank_end = mode->crtc_hblank_start + blank_width;

	mode->crtc_hsync_start = mode->crtc_hblank_start + sync_pos;
	mode->crtc_hsync_end = mode->crtc_hsync_start + sync_width;
}

static void
centre_vertically(struct drm_display_mode *mode,
		  int height)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the vsync and vblank widths constant */
	sync_width = mode->crtc_vsync_end - mode->crtc_vsync_start;
	blank_width = mode->crtc_vblank_end - mode->crtc_vblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->vdisplay - height + 1) / 2;

	mode->crtc_vdisplay = height;
	mode->crtc_vblank_start = height + border;
	mode->crtc_vblank_end = mode->crtc_vblank_start + blank_width;

	mode->crtc_vsync_start = mode->crtc_vblank_start + sync_pos;
	mode->crtc_vsync_end = mode->crtc_vsync_start + sync_width;
}

static inline u32 panel_fitter_scaling(u32 source, u32 target)
{
	/*
	 * Floating point operation is not supported. So the FACTOR
	 * is defined, which can avoid the floating point computation
	 * when calculating the panel ratio.
	 */
#define ACCURACY 12
#define FACTOR (1 << ACCURACY)
	u32 ratio = source * FACTOR / target;
	return (FACTOR * ratio + FACTOR/2) / FACTOR;
}

void intel_gmch_panel_fitting(struct intel_crtc *intel_crtc,
			      struct intel_crtc_config *pipe_config,
			      int fitting_mode)
{
	struct drm_device *dev = intel_crtc->base.dev;
	u32 pfit_control = 0, pfit_pgm_ratios = 0, border = 0;
	struct drm_display_mode *mode, *adjusted_mode;

	intel_crtc->base.panning_en = false;

	mode = &pipe_config->requested_mode;
	adjusted_mode = &pipe_config->adjusted_mode;
	//DRM_ERROR("fitting mode:%d\n",fitting_mode);
	if (IS_VALLEYVIEW(dev)) {
		/* The input src size should be < 2kx2k */
		if ((adjusted_mode->hdisplay > PFIT_SIZE_LIMIT) ||
			(adjusted_mode->vdisplay > PFIT_SIZE_LIMIT)) {
			DRM_ERROR("Wrong panel fitter input src conf");
			goto out;
		}

		if (fitting_mode == AUTOSCALE)
			pfit_control = PFIT_SCALING_AUTO;
		else if (fitting_mode == PILLARBOX)
			pfit_control = PFIT_SCALING_PILLAR;
		else if (fitting_mode == LETTERBOX)
			pfit_control = PFIT_SCALING_LETTER;
		else {
			pfit_control = 0;
			intel_crtc->base.panning_en = false;
			goto out;
		}
		pfit_control |= (PFIT_ENABLE | (intel_crtc->pipe
					<< PFIT_PIPE_SHIFT));
		intel_crtc->base.panning_en = true;
		goto out;
	}

	/* Native modes don't need fitting */
	if (adjusted_mode->hdisplay == mode->hdisplay &&
	    adjusted_mode->vdisplay == mode->vdisplay)
		goto out;

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		/*
		 * For centered modes, we have to calculate border widths &
		 * heights and modify the values programmed into the CRTC.
		 */
		centre_horizontally(adjusted_mode, mode->hdisplay);
		centre_vertically(adjusted_mode, mode->vdisplay);
		border = LVDS_BORDER_ENABLE;
		break;
	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		if (INTEL_INFO(dev)->gen >= 4) {
			u32 scaled_width = adjusted_mode->hdisplay *
				mode->vdisplay;
			u32 scaled_height = mode->hdisplay *
				adjusted_mode->vdisplay;

			/* 965+ is easy, it does everything in hw */
			if (scaled_width > scaled_height)
				pfit_control |= PFIT_ENABLE |
					PFIT_SCALING_PILLAR;
			else if (scaled_width < scaled_height)
				pfit_control |= PFIT_ENABLE |
					PFIT_SCALING_LETTER;
			else if (adjusted_mode->hdisplay != mode->hdisplay)
				pfit_control |= PFIT_ENABLE | PFIT_SCALING_AUTO;
		} else {
			u32 scaled_width = adjusted_mode->hdisplay *
				mode->vdisplay;
			u32 scaled_height = mode->hdisplay *
				adjusted_mode->vdisplay;
			/*
			 * For earlier chips we have to calculate the scaling
			 * ratio by hand and program it into the
			 * PFIT_PGM_RATIO register
			 */
			if (scaled_width > scaled_height) { /* pillar */
				centre_horizontally(adjusted_mode,
						    scaled_height /
						    mode->vdisplay);

				border = LVDS_BORDER_ENABLE;
				if (mode->vdisplay != adjusted_mode->vdisplay) {
					u32 bits = panel_fitter_scaling(mode->vdisplay, adjusted_mode->vdisplay);
					pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
							    bits << PFIT_VERT_SCALE_SHIFT);
					pfit_control |= (PFIT_ENABLE |
							 VERT_INTERP_BILINEAR |
							 HORIZ_INTERP_BILINEAR);
				}
			} else if (scaled_width < scaled_height) { /* letter */
				centre_vertically(adjusted_mode,
						  scaled_width /
						  mode->hdisplay);

				border = LVDS_BORDER_ENABLE;
				if (mode->hdisplay != adjusted_mode->hdisplay) {
					u32 bits = panel_fitter_scaling(mode->hdisplay, adjusted_mode->hdisplay);
					pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
							    bits << PFIT_VERT_SCALE_SHIFT);
					pfit_control |= (PFIT_ENABLE |
							 VERT_INTERP_BILINEAR |
							 HORIZ_INTERP_BILINEAR);
				}
			} else {
				/* Aspects match, Let hw scale both directions */
				pfit_control |= (PFIT_ENABLE |
						 VERT_AUTO_SCALE | HORIZ_AUTO_SCALE |
						 VERT_INTERP_BILINEAR |
						 HORIZ_INTERP_BILINEAR);
			}
		}
		break;
	case DRM_MODE_SCALE_FULLSCREEN:
		/*
		 * Full scaling, even if it changes the aspect ratio.
		 * Fortunately this is all done for us in hw.
		 */
		if (mode->vdisplay != adjusted_mode->vdisplay ||
		    mode->hdisplay != adjusted_mode->hdisplay) {
			pfit_control |= PFIT_ENABLE;
			if (INTEL_INFO(dev)->gen >= 4)
				pfit_control |= PFIT_SCALING_AUTO;
			else
				pfit_control |= (VERT_AUTO_SCALE |
						 VERT_INTERP_BILINEAR |
						 HORIZ_AUTO_SCALE |
						 HORIZ_INTERP_BILINEAR);
		}
		break;
	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

	/* 965+ wants fuzzy fitting */
	/* FIXME: handle multiple panels by failing gracefully */
	if (INTEL_INFO(dev)->gen >= 4)
		pfit_control |= ((intel_crtc->pipe << PFIT_PIPE_SHIFT) |
				 PFIT_FILTER_FUZZY);

out:
	if ((pfit_control & PFIT_ENABLE) == 0) {
		pfit_control = 0;
		pfit_pgm_ratios = 0;
		intel_crtc->scaling_src_size = 0;
	}

	/* Make sure pre-965 set dither correctly for 18bpp panels. */
	if (INTEL_INFO(dev)->gen < 4 && pipe_config->pipe_bpp == 18)
		pfit_control |= PANEL_8TO6_DITHER_ENABLE;

	pipe_config->gmch_pfit.control = pfit_control;
	pipe_config->gmch_pfit.pgm_ratios = pfit_pgm_ratios;
	pipe_config->gmch_pfit.lvds_border_bits = border;
}

static int is_backlight_combination_mode(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen >= 4)
		return I915_READ(BLC_PWM_CTL2) & BLM_COMBINATION_MODE;

	if (IS_GEN2(dev))
		return I915_READ(BLC_PWM_CTL) & BLM_LEGACY_MODE;

	return 0;
}

/* XXX: query mode clock or hardware clock and program max PWM appropriately
 * when it's 0.
 */
static u32 i915_read_blc_pwm_ctl(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val;

	WARN_ON_SMP(!spin_is_locked(&dev_priv->backlight.lock));

	/* Restore the CTL value if it lost, e.g. GPU reset */

	if (HAS_PCH_SPLIT(dev_priv->dev)) {
		val = I915_READ(BLC_PWM_PCH_CTL2);
		if (dev_priv->regfile.saveBLC_PWM_CTL2 == 0) {
			dev_priv->regfile.saveBLC_PWM_CTL2 = val;
		} else if (val == 0) {
			val = dev_priv->regfile.saveBLC_PWM_CTL2;
			I915_WRITE(BLC_PWM_PCH_CTL2, val);
		}
	} else {
		val = I915_READ(BLC_PWM_CTL);
		if (dev_priv->regfile.saveBLC_PWM_CTL == 0) {
			dev_priv->regfile.saveBLC_PWM_CTL = val;
			if (INTEL_INFO(dev)->gen >= 4)
				dev_priv->regfile.saveBLC_PWM_CTL2 =
					I915_READ(BLC_PWM_CTL2);
		} else if (val == 0) {
			val = dev_priv->regfile.saveBLC_PWM_CTL;
			I915_WRITE(BLC_PWM_CTL, val);
			if (INTEL_INFO(dev)->gen >= 4)
				I915_WRITE(BLC_PWM_CTL2,
					   dev_priv->regfile.saveBLC_PWM_CTL2);
		}
	}

	return val;
}

static u32 intel_panel_get_max_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 max;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi)
		return 0xff;

	max = i915_read_blc_pwm_ctl(dev);

	if (HAS_PCH_SPLIT(dev)) {
		max >>= 16;
	} else {
		if (INTEL_INFO(dev)->gen < 4)
			max >>= 17;
		else
			max >>= 16;

		if (is_backlight_combination_mode(dev))
			max *= 0xff;
	}

	DRM_DEBUG_DRIVER("max backlight PWM = %d\n", max);

	return max;
}

static int i915_panel_invert_brightness;
MODULE_PARM_DESC(invert_brightness, "Invert backlight brightness "
	"(-1 force normal, 0 machine defaults, 1 force inversion), please "
	"report PCI device ID, subsystem vendor and subsystem device ID "
	"to dri-devel@lists.freedesktop.org, if your machine needs it. "
	"It will then be included in an upcoming module version.");
module_param_named(invert_brightness, i915_panel_invert_brightness, int, 0600);
static u32 intel_panel_compute_brightness(struct drm_device *dev, u32 val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (i915_panel_invert_brightness < 0)
		return val;

	if (i915_panel_invert_brightness > 0 ||
	    dev_priv->quirks & QUIRK_INVERT_BRIGHTNESS) {
		u32 max = intel_panel_get_max_backlight(dev);
		if (max)
			return max - val;
	}

	return val;
}

static u32 intel_panel_get_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = 0;
	unsigned long flags;


	/*
	 * PMIC i2c write for backlight control is accessed only
	 * from intel_panel.c and need not be in spin_lock
	 * There are anyway mutex to protect the i2c read in the
	 * PMIC driver
	 *
	 * Was causing BUG as mutex was taken within spin_lock
	 */
	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
#ifdef CONFIG_CRYSTAL_COVE
		val = intel_mid_pmic_readb(0x4E);
#else
		DRM_ERROR("Backlight not supported yet\n");
#endif
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	if (HAS_PCH_SPLIT(dev)) {
		val = I915_READ(BLC_PWM_CPU_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
	} else if (!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		val = I915_READ(BLC_PWM_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
		if (INTEL_INFO(dev)->gen < 4)
			val >>= 1;

		if (is_backlight_combination_mode(dev)) {
			u8 lbpc = 0;

			pci_read_config_byte(dev->pdev, PCI_LBPC, &lbpc);
			val *= lbpc;
		}
	}

	/* When DPST is enabled, reading the backlight register will
	 - give the DPST adjusted backlight value. Since DPST works
	 * without user knowing a perceived difference in the backlight,
	 * the programmed backlight isn't the correct value to return.
	 * So, get the user perceived backlight level from DPST. */
	if (dev_priv->dpst.enabled)
		val = i915_dpst_get_brightness(dev);
	else
		val = intel_panel_compute_brightness(dev, val);

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	DRM_DEBUG_DRIVER("get backlight PWM = %d\n", val);
	return val;
}

static void intel_pch_panel_set_backlight(struct drm_device *dev, u32 level)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(BLC_PWM_CPU_CTL) & ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_CPU_CTL, val | level);
}

void intel_panel_actually_set_backlight(struct drm_device *dev, u32 level)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp;

	DRM_DEBUG_DRIVER("set backlight PWM = %d\n", level);
	level = intel_panel_compute_brightness(dev, level);

	if (HAS_PCH_SPLIT(dev))
		return intel_pch_panel_set_backlight(dev, level);

	if (is_backlight_combination_mode(dev)) {
		u32 max = intel_panel_get_max_backlight(dev);
		u8 lbpc;

		/* we're screwed, but keep behaviour backwards compatible */
		if (!max)
			max = 1;

		lbpc = level * 0xfe / max + 1;
		level /= lbpc;
		pci_write_config_byte(dev->pdev, PCI_LBPC, lbpc);
	}

	tmp = I915_READ(BLC_PWM_CTL);
	if (INTEL_INFO(dev)->gen < 4)
		level <<= 1;
	tmp &= ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_CTL, tmp | level);
}

void intel_panel_actually_set_mipi_backlight(struct drm_device *dev, u32 level)
{
#ifdef CONFIG_CRYSTAL_COVE
	struct drm_i915_private *dev_priv = dev->dev_private;
	int panelid = dev_priv->mipi_panel_id;

	if(!dev_priv)
	{
		printk("[LCD]  %s: level:%d, dev_priv is NULL!!!!!!!!!!!!!\n",__func__, level);
		return;
	}
	if(!dev_priv->backlight.enabled)
	{
		printk("[LCD]  %s: level:%d, backlight already disabled!!!!!!!!!!!!!\n",__func__, level);
		return;
	}
	if (BYT_CR_CONFIG) {
		/* FixMe: if level is zero still a pulse is observed consuming
		power. To fix this issue if requested level is zero then
		disable pwm and enabled it again if brightness changes */
		lpio_bl_write_bits(0, LPIO_PWM_CTRL, (0xff - level), 0xFF);
		lpio_bl_update(0, LPIO_PWM_CTRL);
	} else{

		/*Fixed for 8 inch LCD backlight issue*/
		if(intel_dsi_dev != NULL && (panelid == MIPI_DSI_INNOLUX_NT51021_PANEL_ID
			|| panelid == MIPI_DSI_BOE_NT51021_PANEL_ID))
		{
#if 1
			if(level == 0){
				if(!backlight_level0)
				{
					lp855x_ext_write_byte(0x00, 0x00);
					backlight_level0 = true;
					if(intel_dsi_dev->dev_ops->set_backlight_level != NULL)
						intel_dsi_dev->dev_ops->set_backlight_level(intel_dsi_dev, backlight_buf[level]);
				}
			}else if(level != 0 && backlight_level0){
				backlight_level0 = false;
				/*mdelay(110);*/
				/*if(intel_dsi_dev->dev_ops->late_operations!= NULL)*/
					/*intel_dsi_dev->dev_ops->late_operations(intel_dsi_dev);*/
				/*mdelay(10);*/
                lp855x_ext_write_byte(0x00, 0x00);
                lp855x_ext_write_byte(0x10, 0x84);
				if(intel_dsi_dev->dev_ops->set_backlight_level!= NULL)
					intel_dsi_dev->dev_ops->set_backlight_level(intel_dsi_dev,backlight_buf[level]);
				mdelay(10);
				lp855x_ext_write_byte(0x00, 0x01);
			}else{
				if(intel_dsi_dev->dev_ops->set_backlight_level!= NULL && !backlight_level0)
					intel_dsi_dev->dev_ops->set_backlight_level(intel_dsi_dev,backlight_buf[level]);
				else
					printk("[LCD] NT51021 backlight level:%d, ### why here !!!!\n",level);
			}
			printk("[LCD] NT51021 backlight level:%d  actual level:%d\n",level,backlight_buf[level]);
#else
		printk("backlight level:%d\n",level);

				lp855x_ext_write_byte(0x04, level);
			intel_mid_pmic_writeb(0x4E, level);
#endif
		}else
		{
			/*printk("[LCD] backlight level:%d\n",level);*/
			printk("[LCD] backlight level =%d actual level:%d\n",level,backlight_buf[level]);
			intel_mid_pmic_writeb(0x4E, backlight_buf[level]);
		}
	}
#else
	DRM_ERROR("Non PMIC MIPI Backlight control is not supported yet\n");
#endif
}

/* set backlight brightness to level in range [0..max] */
void intel_panel_set_backlight(struct drm_device *dev, u32 level, u32 max)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 freq;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	freq = intel_panel_get_max_backlight(dev);
	if (!freq) {
		/* we are screwed, bail out */
		spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
		return;
	}

	/* scale to hardware */
	level = level * freq / max;

	/*printk("[LCD]%s: @@@@@@ level=%d\n",__func__,level);*/
	dev_priv->backlight.level = level;
	if (dev_priv->backlight.device)
		dev_priv->backlight.device->props.brightness = level;


	if (dev_priv->backlight.enabled) {
		if (dev_priv->dpst.enabled)
			level = i915_dpst_compute_brightness(dev, level);

		if (!dev_priv->is_mipi)
			intel_panel_actually_set_backlight(dev, level);
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (dev_priv->is_mipi)
		intel_panel_actually_set_mipi_backlight(dev, level);
}

void intel_panel_disable_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
    int panelid = dev_priv->mipi_panel_id;
	unsigned long flags;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		intel_panel_actually_set_mipi_backlight(dev, 0);

#ifdef CONFIG_CRYSTAL_COVE
		if (BYT_CR_CONFIG) {
			/* cancel any delayed work scheduled */
			cancel_delayed_work_sync(&dev_priv->bkl_delay_enable_work);

			/* disable the backlight enable signal */
			vlv_gpio_nc_write(dev_priv, 0x40E0, 0x2000CC00);
			vlv_gpio_nc_write(dev_priv, 0x40E8, 0x00000004);
			udelay(500);
			lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x00, 0x80000000);
		} else {
			intel_mid_pmic_writeb(0x51, 0x00);
			intel_mid_pmic_writeb(0x4B, 0x01);
            if((panelid == MIPI_DSI_INNOLUX_NT51021_PANEL_ID
                    || panelid == MIPI_DSI_BOE_NT51021_PANEL_ID
                    || panelid == MIPI_DSI_CPT_NT51011_PANEL_ID
                    || panelid == MIPI_DSI_AUO_B101UAN01E_PANEL_ID))
            {            
                lp855x_ext_write_byte(0x00, 0x00);
                lp855x_ext_write_byte(0x10, 0x85);
                lp855x_ext_write_byte(0x04, 0x00);
            }
		}
#else
		DRM_ERROR("Backlight not supported yet\n");
#endif
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	dev_priv->backlight.enabled = false;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
		printk("[LCD]%s\n",__func__);
		return;
	}

	if (INTEL_INFO(dev)->gen >= 4 &&
				!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		uint32_t reg, tmp;

		intel_panel_actually_set_backlight(dev, 0);

		reg = HAS_PCH_SPLIT(dev) ? BLC_PWM_CPU_CTL2 : BLC_PWM_CTL2;

		I915_WRITE(reg, I915_READ(reg) & ~BLM_PWM_ENABLE);

		if (HAS_PCH_SPLIT(dev)) {
			tmp = I915_READ(BLC_PWM_PCH_CTL1);
			tmp &= ~BLM_PCH_PWM_ENABLE;
			I915_WRITE(BLC_PWM_PCH_CTL1, tmp);
		}
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
}
#ifdef CONFIG_CRYSTAL_COVE
static void scheduled_led_chip_programming(struct work_struct *work)
{
	lp855x_ext_write_byte(LP8556_CFG9,
			LP8556_VBOOST_MAX_NA_21V |
			LP8556_JUMP_DIS |
			LP8556_JMP_TSHOLD_10P |
			LP8556_JMP_VOLT_0_5V);
	lp855x_ext_write_byte(LP8556_CFG5,
			LP8556_PWM_DRECT_DIS |
			LP8556_PS_MODE_5P5D |
			LP8556_PWM_FREQ_9616HZ);
	lp855x_ext_write_byte(LP8556_CFG7,
			LP8556_RSRVD_76 |
			LP8556_DRV3_EN |
			LP8556_DRV2_EN |
			LP8556_RSRVD_32 |
			LP8556_IBOOST_LIM_1_8A_NA);
	lp855x_ext_write_byte(LP8556_LEDSTREN,
			LP8556_5LEDSTR);
}
#endif

static uint32_t compute_pwm_base(uint16_t freq)
{
	uint32_t base_unit;

	if (freq < 400)
		freq = 400;
	/*The PWM block is clocked by the 25MHz oscillator clock.
	* The output frequency can be estimated with the equation:
	* Target frequency = XOSC * Base_unit_value/256
	*/
	base_unit = (freq * 256) / 25;

	/* Also Base_unit_value need to converted to QM.N notation
	* to program the value in register
	* Using the following for converting to Q8.8 notation
	* For QM.N representation, consider a floating point variable 'a' :
	* Step 1: Calculate b = a* 2^N , where N is the fractional length of the variable.
	* Note that a is represented in decimal.
	* Step 2: Round the value of 'b' to the nearest integer value. For example:
	* RoundOff (1.05) --> 1
	* RoundOff (1.5)  --> 2
	* Step 3: Convert 'b' from decimal to binary representation and name the new variable 'c'
	*/
	base_unit = base_unit * 256;
	base_unit = DIV_ROUND_CLOSEST(base_unit, 1000000);

	return base_unit;
}

void intel_panel_enable_backlight(struct drm_device *dev,
				  enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
    int panelid = dev_priv->mipi_panel_id;
	enum transcoder cpu_transcoder =
		intel_pipe_to_cpu_transcoder(dev_priv, pipe);
	unsigned long flags;
	uint32_t pwm_base;
	extern int i915_boot_mode;
	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
#ifdef CONFIG_CRYSTAL_COVE
		uint32_t val;
		if (BYT_CR_CONFIG) {
			/* GPIOC_94 config to PWM0 function */
			val = vlv_gps_core_read(dev_priv, 0x40A0);
			vlv_gps_core_write(dev_priv, 0x40A0, 0x2000CC01);
			vlv_gps_core_write(dev_priv, 0x40A8, 0x5);

			/* PWM enable
			* Assuming only 1 LFP
			*/
			pwm_base = compute_pwm_base(dev_priv->vbt.pwm_frequency);
			pwm_base = pwm_base << 8;
			lpio_bl_write(0, LPIO_PWM_CTRL, pwm_base);
			lpio_bl_update(0, LPIO_PWM_CTRL);
			lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x80000000,
							0x80000000);
			lpio_bl_update(0, LPIO_PWM_CTRL);

			/* Backlight enable */
			vlv_gpio_nc_write(dev_priv, 0x40E0, 0x2000CC00);
			vlv_gpio_nc_write(dev_priv, 0x40E8, 0x00000005);
			udelay(500);

			if (lpdata)
				schedule_delayed_work(&dev_priv->bkl_delay_enable_work,
								msecs_to_jiffies(30));
		} else {
			intel_mid_pmic_writeb(0x4B, 0x81);
			intel_mid_pmic_writeb(0x51, 0x01);

			/* Control Backlight Slope programming for LP8556 IC*/
			if (lpdata && (spid.hardware_id == BYT_TABLET_BLK_8PR1)) {
				mdelay(2);
				if (lp855x_ext_write_byte(LP8556_CFG3, LP8556_MODE_SL_50MS_FL_HV_PWM_12BIT))
					DRM_ERROR("Backlight slope programming failed\n");
				else
					DRM_INFO("Backlight slope programming success\n");
				mdelay(2);
			}
		}
        if((panelid == MIPI_DSI_AUO_B101UAN01E_PANEL_ID)
            ||panelid == MIPI_DSI_CPT_NT51011_PANEL_ID)
        {
                lp855x_ext_write_byte(0x0, 0x0);
                mdelay(2);
                lp855x_ext_write_byte(0x10, 0x84);
                mdelay(2);	
                lp855x_ext_write_byte(0x0, 0x1);
        }
        if((panelid == MIPI_DSI_INNOLUX_NT51021_PANEL_ID
                || panelid == MIPI_DSI_BOE_NT51021_PANEL_ID))
            if(lpdata) {
                lp855x_ext_write_byte(0x0, 0x0);
                mdelay(2);
                lp855x_ext_write_byte(0x10, 0x85);
                lp855x_ext_write_byte(0x03, 0x0);

                lp855x_ext_write_byte(0x04, 0x0);
                mdelay(2);	
                lp855x_ext_write_byte(0x0, 0x1);
                printk("backlight %s\n",__func__);

            }
#else
		DRM_ERROR("Backlight not supported yet\n");
#endif
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	if (dev_priv->backlight.level == 0) {
		/*dev_priv->backlight.level = intel_panel_get_max_backlight(dev);*/
		/*dev_priv->backlight.level = backlight_buf[170];*/
		dev_priv->backlight.level = 170;
		if (dev_priv->backlight.device)
			dev_priv->backlight.device->props.brightness =
				dev_priv->backlight.level;
	}

	if (INTEL_INFO(dev)->gen >= 4 &&
				!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		uint32_t reg, tmp;

		reg = HAS_PCH_SPLIT(dev) ? BLC_PWM_CPU_CTL2 : BLC_PWM_CTL2;


		tmp = I915_READ(reg);

		/* Note that this can also get called through dpms changes. And
		 * we don't track the backlight dpms state, hence check whether
		 * we have to do anything first. */
		if (tmp & BLM_PWM_ENABLE)
			goto set_level;

		if (INTEL_INFO(dev)->num_pipes == 3)
			tmp &= ~BLM_PIPE_SELECT_IVB;
		else
			tmp &= ~BLM_PIPE_SELECT;

		if (cpu_transcoder == TRANSCODER_EDP)
			tmp |= BLM_TRANSCODER_EDP;
		else
			tmp |= BLM_PIPE(cpu_transcoder);
		tmp &= ~BLM_PWM_ENABLE;

		I915_WRITE(reg, tmp);
		POSTING_READ(reg);
		I915_WRITE(reg, tmp | BLM_PWM_ENABLE);

		if (HAS_PCH_SPLIT(dev) &&
		    !(dev_priv->quirks & QUIRK_NO_PCH_PWM_ENABLE)) {
			tmp = I915_READ(BLC_PWM_PCH_CTL1);
			tmp |= BLM_PCH_PWM_ENABLE;
			tmp &= ~BLM_PCH_OVERRIDE_ENABLE;
			I915_WRITE(BLC_PWM_PCH_CTL1, tmp);
		}
	}

set_level:
	/* Call below after setting BLC_PWM_CPU_CTL2 and BLC_PWM_PCH_CTL1.
	 * BLC_PWM_CPU_CTL may be cleared to zero automatically when these
	 * registers are set.
	 */
	 if(i915_boot_mode){
    		DRM_INFO("%s: android mode\n", __func__);
     }
	dev_priv->backlight.enabled = true;
	if (!dev_priv->is_mipi)
		intel_panel_actually_set_backlight(dev,
						dev_priv->backlight.level);

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

    /*if((panelid == MIPI_DSI_INNOLUX_NT51021_PANEL_ID*/
            /*|| panelid == MIPI_DSI_BOE_NT51021_PANEL_ID))*/
        /*lp855x_ext_write_byte(0x10, 0x84);*/
//del by lenovo jinjt for resume backlight flicker 
#if 1
//	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi && power_on_init){
	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi && (!i915_boot_mode)){
		intel_panel_actually_set_mipi_backlight(dev,
					dev_priv->backlight.level);
//		power_on_init = false;
	}
#endif
}

static void intel_panel_init_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->backlight.level = intel_panel_get_backlight(dev);
	dev_priv->backlight.enabled = dev_priv->backlight.level != 0;
#ifdef CONFIG_CRYSTAL_COVE
	if (BYT_CR_CONFIG)
		INIT_DELAYED_WORK(&dev_priv->bkl_delay_enable_work,
				scheduled_led_chip_programming);
#endif
}

enum drm_connector_status
intel_panel_detect(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Assume that the BIOS does not lie through the OpRegion... */
	if (!i915_panel_ignore_lid && dev_priv->opregion.lid_state) {
		return ioread32(dev_priv->opregion.lid_state) & 0x1 ?
			connector_status_connected :
			connector_status_disconnected;
	}

	switch (i915_panel_ignore_lid) {
	case -2:
		return connector_status_connected;
	case -1:
		return connector_status_disconnected;
	default:
		return connector_status_unknown;
	}
}

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
static int intel_panel_update_status(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	intel_panel_set_backlight(dev, bd->props.brightness,
				  bd->props.max_brightness);
	return 0;
}

static int intel_panel_get_brightness(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	return intel_panel_get_backlight(dev);
}

static const struct backlight_ops intel_panel_bl_ops = {
	.update_status = intel_panel_update_status,
	.get_brightness = intel_panel_get_brightness,
};

int intel_panel_setup_backlight(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct backlight_properties props;
	unsigned long flags;

	intel_panel_init_backlight(dev);

	if (WARN_ON(dev_priv->backlight.device))
		return -ENODEV;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = dev_priv->backlight.level;

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);
	props.max_brightness = intel_panel_get_max_backlight(dev);
	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (props.max_brightness == 0) {
		DRM_DEBUG_DRIVER("Failed to get maximum backlight value\n");
		return -ENODEV;
	}
	dev_priv->backlight.device =
		backlight_device_register("intel_backlight",
					  &connector->kdev, dev,
					  &intel_panel_bl_ops, &props);

	if (IS_ERR(dev_priv->backlight.device)) {
		DRM_ERROR("Failed to register backlight: %ld\n",
			  PTR_ERR(dev_priv->backlight.device));
		dev_priv->backlight.device = NULL;
		return -ENODEV;
	}
	return 0;
}

void intel_panel_destroy_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	if (dev_priv->backlight.device) {
		backlight_device_unregister(dev_priv->backlight.device);
		dev_priv->backlight.device = NULL;
	}
}
#else
int intel_panel_setup_backlight(struct drm_connector *connector)
{
	intel_panel_init_backlight(connector->dev);
	return 0;
}

void intel_panel_destroy_backlight(struct drm_device *dev)
{
	return;
}
#endif

int intel_panel_init(struct intel_panel *panel,
			struct drm_display_mode *fixed_mode,
			struct drm_display_mode *downclock_mode)
{
	panel->fixed_mode = fixed_mode;
	panel->downclock_mode = downclock_mode;

	return 0;
}

/*
 * intel_find_panel_downclock - find the reduced downclock for LVDS in EDID
 * @dev: drm device
 * @fixed_mode : panel native mode
 * @connector: LVDS/eDP connector
 *
 * Return downclock_avail
 * Find the reduced downclock for LVDS/eDP in EDID.
 */

struct drm_display_mode *
intel_find_panel_downclock(struct drm_device *dev,
			struct drm_display_mode *fixed_mode,
			struct drm_connector *connector)
{
	struct drm_display_mode *scan, *tmp_mode;
	int temp_downclock;

	temp_downclock = fixed_mode->clock;
	tmp_mode = NULL;

	list_for_each_entry(scan, &connector->probed_modes, head) {
		/*
		 * If one mode has the same resolution with the fixed_panel
		 * mode while they have the different refresh rate, it means
		 * that the reduced downclock is found. In such
		 * case we can set the different FPx0/1 to dynamically select
		 * between low and high frequency.
		*/
		if (scan->hdisplay == fixed_mode->hdisplay &&
		scan->hsync_start == fixed_mode->hsync_start &&
		scan->hsync_end == fixed_mode->hsync_end &&
		scan->htotal == fixed_mode->htotal &&
		scan->vdisplay == fixed_mode->vdisplay &&
		scan->vsync_start == fixed_mode->vsync_start &&
		scan->vsync_end == fixed_mode->vsync_end &&
		scan->vtotal == fixed_mode->vtotal) {
			if (scan->clock < temp_downclock) {
				/*
				 * The downclock is already found. But we
				 * expect to find the lower downclock.
				 */
				temp_downclock = scan->clock;
				tmp_mode = scan;
			}
		}
	}

	if (temp_downclock < fixed_mode->clock)
		return drm_mode_duplicate(dev, tmp_mode);
	else
		return NULL;
}

void intel_panel_fini(struct intel_panel *panel)
{
	struct intel_connector *intel_connector =
		container_of(panel, struct intel_connector, panel);

	if (panel->fixed_mode)
		drm_mode_destroy(intel_connector->base.dev, panel->fixed_mode);

	if (panel->downclock_mode)
		drm_mode_destroy(intel_connector->base.dev,
				panel->downclock_mode);
}
#define MANCONV0	0x72
#define MANCONV1	0x73
#define BPTEMP1_RSLTH	0x7c
#define BPTEMP1_RSLTL	0x7d
#define THERM_ENABLE    0x90
#define ADCIRQ0		0x08
#define ADCIRQ1		0x09
int board_id3 = 0;
char *board_id_8_inch = "8inch";
char *board_id_10_inch = "10inch";
static struct kobject *blade2_board_kobj;

static ssize_t board_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if(board_id3== 1)
		return sprintf(buf, "%s\n", board_id_10_inch);
	else
		return sprintf(buf, "%s\n", board_id_8_inch);
}

static struct kobj_attribute blade2_board_id_attr = {
	.attr = {"id", 0660},
	.show = board_id_show,
	.store = NULL,
};
static struct kobj_attribute blade2_board_id1_attr = {
	.attr = {"id1", 0660},
	.show = board_id_show,
	.store = NULL,
};

static struct attribute *blade2_board_attr[]= {
	&blade2_board_id_attr.attr,
	&blade2_board_id1_attr.attr,
	NULL,
};

static struct attribute_group blade2_attr_group = {
	.attrs = blade2_board_attr,
};

/*
	8inch Innolux adc 0x9
	8inch BOE adc 0x60
	other unknow suppose 0x1ff
*/
unsigned int panel_id_adc_array[]={0,0x60,0x400}; //max adc value is 0x3ff
int intel_adc_read_panelid(unsigned int *res)
{
	int ret =0;
	int mask_bak = 0; //panel id use bit4
	unsigned int val;
	int i = 0;
	unsigned int pending0;
	unsigned int panel_id = 0;
	//int board_id3 = 0;

	mask_bak = intel_mid_pmic_readb(THERM_ENABLE);
	intel_mid_pmic_setb(THERM_ENABLE,mask_bak|0x10); //enable bptherm1
	udelay(5);
	intel_mid_pmic_setb(MANCONV0, 0x10);//panel id use bit4

	for(i=0;i<1000;i++)
	{
		udelay(1);
		pending0 = intel_mid_pmic_readb(ADCIRQ0);
		if(pending0&0x10)
		{
			intel_mid_pmic_writeb(ADCIRQ0, pending0);//clear irq status
			break;
		}
	}

	val = intel_mid_pmic_readb(BPTEMP1_RSLTH); //therm adc high bits
	val = ((val & 0x3) << 8) + intel_mid_pmic_readb(BPTEMP1_RSLTL);

	printk("[LCD]:%s,panel id value: 0x%x\n",__func__,val);
	intel_mid_pmic_setb(THERM_ENABLE,mask_bak); //set to ori value

	for(i=0;i<sizeof(panel_id_adc_array)-1;i++)
	{
		if((val>=panel_id_adc_array[i])&&(val<panel_id_adc_array[i+1]))
			break;
	}

	ret = gpio_request(GPIO_BOARD_ID, "board_id3");
	if (ret < 0)
		printk("[LCD]: Failed to request gpio 210\n");
	ret = gpio_direction_input(GPIO_BOARD_ID);
	if(ret< 0)
		printk("[LCD]: Failed to config gpio 210\n");

	//set gpio1p2 function state
	intel_mid_pmic_writeb(PMIC_GPIO1P2_ADDRESS, GPIO_INPUT_NO_DRV);

	board_id3 = gpio_get_value_cansleep(GPIO_BOARD_ID);
	printk("[LCD]:%s,gpio1P2 value: 0x%x\n",__func__,board_id3);

	blade2_board_kobj = kobject_create_and_add("mainboard", NULL);
	if(blade2_board_kobj)
		ret = sysfs_create_group( blade2_board_kobj, &blade2_attr_group);
	if (ret)
		kobject_put(blade2_board_kobj);

	if(board_id3 == 0x1)
	{
		switch(i)//10inch
		{
			case 0:
				panel_id = MIPI_DSI_AUO_B101UAN01E_PANEL_ID;
				break;
			case 1:
				panel_id = MIPI_DSI_CPT_NT51011_PANEL_ID;
				break;
			default:
				panel_id = MIPI_DSI_CPT_NT51011_PANEL_ID;
				break;
		}
	}else{
		switch(i)
		{
			case 0:
 				panel_id = MIPI_DSI_BOE_NT51021_PANEL_ID;
				break;
			case 1:
 				panel_id = MIPI_DSI_INNOLUX_NT51021_PANEL_ID;
				break;
			default:
 				panel_id = MIPI_DSI_BOE_NT51021_PANEL_ID;
				break;
		}

	}
	*res = panel_id;
	return ret;
}

