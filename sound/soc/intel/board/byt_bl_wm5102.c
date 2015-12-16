/*
 *  byt_bl_wm5102.c - ASoc Machine driver for Intel Baytrail Baylake MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/acpi_gpio.h>
#include <linux/extcon-mid.h>
#include <linux/pm_runtime.h>
#include <asm/platform_byt_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/mfd/arizona/registers.h>
#include <linux/lnw_gpio.h>
#include "../../codecs/wm5102.h"

#ifdef CONFIG_SND_SOC_COMMS_SSP
#include "byt_bl_rt5642.h"
#include "../ssp/mid_ssp.h"
#endif /* CONFIG_SND_SOC_COMMS_SSP */

#define BYT_PLAT_CLK_3_HZ	25000000
#define WM5102_MAX_SYSCLK_1 49152000 /*max sysclk for 4K family*/
#define WM5102_MAX_SYSCLK_2 45158400 /*max sysclk for 11.025K family*/

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2

#define EXT_SPEAKER_ENABLE_PIN 302    // GPIO3

struct byt_mc_private {
#ifdef CONFIG_SND_SOC_COMMS_SSP
	struct byt_comms_mc_private comms_ctl;
#endif /* CONFIG_SND_SOC_COMMS_SSP */
};

#ifdef CONFIG_SND_SOC_COMMS_SSP
static inline struct byt_comms_mc_private *kcontrol2ctl(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);
	printk("%s", __func__);
	return ctl;
}

int byt_get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	printk("%s", __func__);

	return 0;
}

int byt_set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_bt_sco_master_mode)
		ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];
	printk("%s", __func__);
	return 0;
}

int byt_get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	printk("%s", __func__);
	return 0;
}

int byt_set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct byt_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_modem_master_mode)
		ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];
	printk("%s", __func__);
	return 0;
}
#endif /* CONFIG_SND_SOC_COMMS_SSP */

static int byt_ext_speaker_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	pr_debug("%s()\n", __func__);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("%s, ON\n", __func__);
        gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN, 1);
	} else {
		pr_info("%s, OFF\n", __func__);
        gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN, 0);
    }
	return 0;
}

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", byt_ext_speaker_event),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"Headphone", NULL, "HPOUT1L"},
	{"Headphone", NULL, "HPOUT1R"},
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},
	{"Headset Mic", NULL, "MICBIAS1"},
	{"Headset Mic", NULL, "MICBIAS2"},
	{"IN1L", NULL, "Headset Mic"},
	{"Int Mic", NULL, "MICBIAS3"},
	{"IN3L", NULL, "Int Mic"},
};

static int open_aif_clk=0;
struct mutex reg_fll_lock;
static int byt_config_5102_clks(struct snd_soc_codec *wm5102_codec, int sr)
{
	int ret;
	int sr_mult = (sr % 4000 == 0) ? (WM5102_MAX_SYSCLK_1/sr) : (WM5102_MAX_SYSCLK_2/sr);

    pr_info("%s: open_aif_clk: %d\n", __func__, open_aif_clk);
    mutex_lock(&reg_fll_lock);
    open_aif_clk++;
    if (open_aif_clk > 1) {
        mutex_unlock(&reg_fll_lock);
        return 0;
    }
    mutex_unlock(&reg_fll_lock);

	/*Open MCLK before Set Codec CLK*/
    vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_ON);
	/*reset FLL1*/
	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE, 0, 0);
	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
				ARIZONA_FLL_SRC_NONE, 0, 0);

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
					ARIZONA_CLK_SRC_MCLK1, //TODO Check if clock from AP is connected to MCLK1
					BYT_PLAT_CLK_3_HZ,
					sr * sr_mult);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);		
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
			ARIZONA_CLK_SYSCLK,
			ARIZONA_CLK_SRC_FLL1,
			sr * sr_mult,
			SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set AYNCCLK: %d\n", ret);
		
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
					ARIZONA_CLK_OPCLK, 0,
					sr	* sr_mult,
					SND_SOC_CLOCK_OUT);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set OPCLK: %d\n", ret);		
		return ret;
	}
    
	return 0;
}

static int byt_free_5102_clks(struct snd_soc_codec *codec)
{
    pr_info("%s: open_aif_clk: %d\n", __func__, open_aif_clk);

    mutex_lock(&reg_fll_lock);
    open_aif_clk--;
    if (open_aif_clk > 0) {
        mutex_unlock(&reg_fll_lock);
        return 0;
    }
    mutex_unlock(&reg_fll_lock);

    snd_soc_codec_set_pll(codec, WM5102_FLL1_REFCLK,
        ARIZONA_FLL_SRC_NONE, 0, 0);

    snd_soc_codec_set_pll(codec, WM5102_FLL1,
        ARIZONA_FLL_SRC_NONE, 0, 0);

	/*Open MCLK before Set Codec CLK*/
    vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);
    return 0;
}

static int byt_aif1_free(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    pr_info("enter %s\n", __func__);

    return byt_free_5102_clks(rtd->codec);
}

static int byt_aif2_free(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    pr_info("enter %s\n", __func__);
    byt_free_5102_clks(rtd->codec);
	return pm_runtime_put(rtd->codec->dev);
}

static int byt_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
    
    pr_info("enter %s\n", __func__);
    pm_runtime_get_sync(rtd->codec->dev);
	return byt_config_5102_clks(rtd->codec, 48000);
}

static void byt_compress_shutdown(struct snd_compr_stream *cstream)
{
    struct snd_soc_pcm_runtime *rtd = cstream->private_data;

    pr_info("enter %s\n", __func__);
    byt_free_5102_clks(rtd->codec);
	pm_runtime_put(rtd->codec->dev);
}
 
static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

    pr_info("enter %s\n", __func__);
	return byt_config_5102_clks(rtd->codec, params_rate(params));
}

static int byt_aif2_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

    pr_info("enter %s\n", __func__);
    pm_runtime_get_sync(rtd->codec->dev);
	return byt_config_5102_clks(rtd->codec, params_rate(params));
}

static int byt_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
	}
	card->dapm.bias_level = level;
	pr_debug("card(%s)->bias_level %u\n", card->name,
		  	card->dapm.bias_level);
	return 0;
}

#ifdef CONFIG_SND_SOC_COMMS_SSP
static int byt_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "BYT Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

    /* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case BYT_COMMS_BT:
		str_runtime->hw = BYT_COMMS_BT_hw_param;
		break;

	case BYT_COMMS_MODEM:
		str_runtime->hw = BYT_COMMS_MODEM_hw_param;
		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device = %d\n",
		       substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
					 SNDRV_PCM_HW_PARAM_PERIODS);
}

static int byt_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;


	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

	switch (device) {
	case BYT_COMMS_BT:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SSP_DAI_SCMODE_1 |
					  SND_SOC_DAIFMT_NB_NF |
					  (ctl->ssp_bt_sco_master_mode ?
					   SND_SOC_DAIFMT_CBM_CFM :
					   SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("BYT Comms Machine: Set FMT Fails %d\n",
				ret);
			return -EINVAL;
		}

		/*
		 * BT SCO SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 16
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * ssp_psp_T2 = 1
		 * (Dummy start offset = 1 bit clock period)
		 */
		nb_slot = BYT_SSP_BT_SLOT_NB_SLOT;
		slot_width = BYT_SSP_BT_SLOT_WIDTH;
		tx_mask = BYT_SSP_BT_SLOT_TX_MASK;
		rx_mask = BYT_SSP_BT_SLOT_RX_MASK;

		if (ctl->ssp_bt_sco_master_mode)
			tristate_offset = BIT(TRISTATE_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;

	case BYT_COMMS_MODEM:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
						SND_SOC_DAIFMT_I2S |
						SSP_DAI_SCMODE_0 |
						SND_SOC_DAIFMT_NB_NF |
						(ctl->ssp_modem_master_mode ?
						SND_SOC_DAIFMT_CBM_CFM :
						SND_SOC_DAIFMT_CBS_CFS));
		if (ret < 0) {
			pr_err("BYT Comms Machine:  Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * Modem Mixing SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * Master:
		 *	tristate = 3
		 *	ssp_frmsync_timing_bit = 1, for MASTER
		 *	(NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * Slave:
		 *	tristate = 1
		 *	ssp_frmsync_timing_bit = 0, for SLAVE
		 *	(NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 *
		 */
		nb_slot = BYT_SSP_MIXING_SLOT_NB_SLOT;
		slot_width = BYT_SSP_MIXING_SLOT_WIDTH;
		tx_mask = BYT_SSP_MIXING_SLOT_TX_MASK;
		rx_mask = BYT_SSP_MIXING_SLOT_RX_MASK;

		tristate_offset = BIT(TRISTATE_BIT) |
		    BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device ID = %d\n", device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
				   rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Comms Machine:  Set TDM Slot Fails %d\n", ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Comms Machine: Set Tristate Fails %d\n", ret);
		return -EINVAL;
	}

	pr_debug("BYT Comms Machine: slot_width = %d\n",
	     slot_width);
	pr_debug("BYT Comms Machine: tx_mask = %d\n",
	     tx_mask);
	pr_debug("BYT Comms Machine: rx_mask = %d\n",
	     rx_mask);
	pr_debug("BYT Comms Machine: tristate_offset = %d\n",
	     tristate_offset);

	return 0;
}

static int byt_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct byt_comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
		__func__,
		substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if ((device == BYT_COMMS_BT && ctl->ssp_bt_sco_master_mode) ||
	    (device == BYT_COMMS_MODEM && ctl->ssp_modem_master_mode)) {
		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);
	}

	return 0;
}
#endif  /* CONFIG_SND_SOC_COMMS_SSP */

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
    int ret = 0;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
#ifdef CONFIG_SND_SOC_COMMS_SSP
	struct byt_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
#endif

	pr_info("Enter:%s\n", __func__);
	/* Set codec bias level */
	byt_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

#if 0
	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}
#endif

#ifdef CONFIG_SND_SOC_COMMS_SSP
	/* Add Comms specific controls */
	if(ctx){
		ctx->comms_ctl.ssp_bt_sco_master_mode = false;
		ctx->comms_ctl.ssp_modem_master_mode = false;
		printk("set ssp mode");

		ret = snd_soc_add_card_controls(card, byt_ssp_comms_controls,
					ARRAY_SIZE(byt_ssp_comms_controls));
	}
	else
		printk("ctx is NULL");

	if (ret) {
		pr_err("unable to add COMMS card controls\n");
		return ret;
	}
#endif /* CONFIG_SND_SOC_COMMS_SSP */

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1L");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1R");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTLP");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRP");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRN");

    snd_soc_dapm_ignore_suspend(dapm, "AIF2 Playback");
    snd_soc_dapm_ignore_suspend(dapm, "AIF2 Capture");
    snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");

    snd_soc_dapm_ignore_suspend(&card->dapm, "Headset Mic");
    snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone");
    snd_soc_dapm_ignore_suspend(&card->dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);
    pr_info("exit %s\n", __func__);

	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int byt_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops byt_aif1_ops = {
	.startup = byt_aif1_startup,
	.hw_params = byt_aif1_hw_params,
	.hw_free = byt_aif1_free,
};

static struct snd_soc_ops byt_aif2_ops = {
	.hw_params = byt_aif2_hw_params,
	.hw_free = byt_aif2_free,
};

static struct snd_soc_compr_ops byt_compr_ops = {
	.set_params = byt_compr_set_params,
	.shutdown = byt_compress_shutdown,
};

#ifdef CONFIG_SND_SOC_COMMS_SSP
static struct snd_soc_ops byt_comms_dai_link_ops = {
	.startup = byt_comms_dai_link_startup,
	.hw_params = byt_comms_dai_link_hw_params,
	.prepare = byt_comms_dai_link_prepare,
};
#endif /* CONFIG_SND_SOC_COMMS_SSP */

static struct snd_soc_dai_link byt_dailink[] = {
	[BYT_AUD_AIF1] = {
		.name = "Baytrail Audio",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.dai_fmt	= SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS,
		.init = byt_init,
		.ignore_suspend = 1,
		.ops = &byt_aif1_ops,
		.playback_count = 2,
	},
	[BYT_AUD_AIF2] = {
		.name = "Baytrail Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "wm5102-aif2",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.dai_fmt	= SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS,
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &byt_aif2_ops,
	},
	[BYT_AUD_COMPR_DEV] = {
		.name = "Baytrail Compressed Audio",
		.stream_name = "Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.dai_fmt	= SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS,
		.init = NULL,
		.ignore_suspend = 1,
		.compr_ops = &byt_compr_ops,
	},
#ifdef CONFIG_SND_SOC_COMMS_SSP
	[BYT_COMMS_BT] = {
		.name = "Baytrail Comms BT SCO",
		.stream_name = "BYT_BTSCO",
		.cpu_dai_name = SSP_BT_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
	[BYT_COMMS_MODEM] = {
		.name = "Baytrail Comms MODEM",
		.stream_name = "BYT_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &byt_comms_dai_link_ops,
	},
#endif /* CONFIG_SND_SOC_COMMS_SSP */
};

#ifdef CONFIG_PM_SLEEP
static int snd_byt_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_byt_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_byt_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_byt_prepare NULL
#define snd_byt_complete NULL
#define snd_byt_poweroff NULL
#endif

static int snd_byt_mc_late_probe(struct snd_soc_card *card)
{
	int ret;

	ret = snd_soc_dai_set_sysclk(card->rtd[0].codec_dai,  ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(card->rtd[1].codec_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	/*Configure SAMPLE_RATE_1 and ASYNC_SAMPLE_RATE_1 by default to
	48KHz these values can be changed in runtime by corresponding
	DAI hw_params callback */
	snd_soc_update_bits(card->rtd[0].codec, ARIZONA_SAMPLE_RATE_1,
		ARIZONA_SAMPLE_RATE_1_MASK, 0x03);
	snd_soc_update_bits(card->rtd[0].codec, ARIZONA_ASYNC_SAMPLE_RATE_1,
		ARIZONA_ASYNC_SAMPLE_RATE_MASK, 0x03);

	return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_byt = {
	.name = "baytrailaudio",
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.late_probe = snd_byt_mc_late_probe,
	.set_bias_level = byt_set_bias_level,
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
};

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;
	pr_info("Entry %s\n", __func__);
    mutex_init(&reg_fll_lock);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF); //force off the MCLK1

    ret_val = devm_gpio_request_one(&pdev->dev, EXT_SPEAKER_ENABLE_PIN, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "SPK POWER");
    if (ret_val) {
        pr_err("snd_byt_mc_probe() spk power gpio config failed %d\n", ret_val);
		return -EINVAL;
    }

	/* register the soc card */
	snd_soc_card_byt.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_byt, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_byt);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_byt);
	pr_info("%s successful\n", __func__);

	return ret_val;
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	
	pr_debug("In %s\n", __func__);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_byt_mc_shutdown(struct platform_device *pdev)
{
    int ret_val;

	pr_info("In %s\n", __func__);
    /* set jacd detec pin (157) and codec clock pin(99) to output low */
    ret_val = gpio_request_one(157, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "JACK DET");
    if (ret_val)
        pr_err("snd_byt_mc_probe() jack det gpio config failed %d\n", ret_val);

    lnw_gpio_set_alt(99, LNW_GPIO);
    ret_val = gpio_request_one(99, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "codec clk");
    if (ret_val)
        pr_err("snd_byt_mc_shutdown() codec clk gpio config failed\n");

    lnw_gpio_set_alt(136, LNW_GPIO);
    ret_val = gpio_request_one(136, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "codec 32k");
    if (ret_val)
        pr_err("snd_byt_mc_shutdown() codec clk gpio config failed\n");
}

static const struct dev_pm_ops snd_byt_mc_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
	.poweroff = snd_byt_poweroff,
};

static const struct acpi_device_id byt_mc_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, byt_mc_acpi_ids);

static struct platform_driver snd_byt_mc_driver = { //Done
	.driver = {
		.owner = THIS_MODULE,
		.name = "byt_wm5102",
		.pm = &snd_byt_mc_pm_ops,
	//	.acpi_match_table = ACPI_PTR(byt_mc_acpi_ids),
	},
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.shutdown = snd_byt_mc_shutdown,
};

static int __init snd_byt_driver_init(void)
{
	pr_info("Baytrail Machine Driver byt_wm5102 registerd\n");
	return platform_driver_register(&snd_byt_mc_driver);
}
late_initcall(snd_byt_driver_init);

static void __exit snd_byt_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_byt_mc_driver);
}
module_exit(snd_byt_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver");
MODULE_AUTHOR("Omair Md Abdullah <omair.m.abdullah@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytwm5102-audio");
