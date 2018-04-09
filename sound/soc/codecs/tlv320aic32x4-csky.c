/*
 * sound/soc/codecs/tlv320aic32x4-csky.c
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * Based on sound/soc/codecs/tlv320aic32x4.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tlv320aic32x4-csky.h"

#define NO_DAPM

struct aic32x4_priv {
	struct regmap *regmap;
	u32 sysclk;
	u32 power_cfg;
	bool swapdacs;
};

/* 0dB min, 0.5dB steps */
static DECLARE_TLV_DB_SCALE(tlv_step_0_5, 0, 50, 0);
/* -63.5dB min, 0.5dB steps */
static DECLARE_TLV_DB_SCALE(tlv_pcm, -6350, 50, 0);
/* -6dB min, 1dB steps */
static DECLARE_TLV_DB_SCALE(tlv_driver_gain, -600, 100, 0);
/* -12dB min, 0.5dB steps */
static DECLARE_TLV_DB_SCALE(tlv_adc_vol, -1200, 50, 0);

static const struct snd_kcontrol_new aic32x4_snd_controls[] = {
	SOC_DOUBLE_R_S_TLV("PCM Playback Volume", AIC32X4_LDACVOL,
			AIC32X4_RDACVOL, 0, -0x7f, 0x30, 7, 0, tlv_pcm),
	SOC_DOUBLE_R_S_TLV("HP Driver Gain Volume", AIC32X4_HPLGAIN,
			AIC32X4_HPRGAIN, 0, -0x6, 0x1d, 5, 0,
			tlv_driver_gain),
	SOC_DOUBLE_R_S_TLV("LO Driver Gain Volume", AIC32X4_LOLGAIN,
			AIC32X4_LORGAIN, 0, -0x6, 0x1d, 5, 0,
			tlv_driver_gain),
	SOC_DOUBLE_R("HP DAC Playback Switch", AIC32X4_HPLGAIN,
			AIC32X4_HPRGAIN, 6, 0x01, 1),
	SOC_DOUBLE_R("LO DAC Playback Switch", AIC32X4_LOLGAIN,
			AIC32X4_LORGAIN, 6, 0x01, 1),
	SOC_DOUBLE_R("Mic PGA Switch", AIC32X4_LMICPGAVOL,
			AIC32X4_RMICPGAVOL, 7, 0x01, 1),

	SOC_SINGLE("ADCFGA Left Mute Switch", AIC32X4_ADCFGA, 7, 1, 0),
	SOC_SINGLE("ADCFGA Right Mute Switch", AIC32X4_ADCFGA, 3, 1, 0),

	SOC_DOUBLE_R_S_TLV("ADC Level Volume", AIC32X4_LADCVOL,
			AIC32X4_RADCVOL, 0, -0x18, 0x28, 6, 0, tlv_adc_vol),
	SOC_DOUBLE_R_TLV("PGA Level Volume", AIC32X4_LMICPGAVOL,
			AIC32X4_RMICPGAVOL, 0, 0x5f, 0, tlv_step_0_5),

	SOC_SINGLE("Auto-mute Switch", AIC32X4_DACMUTE, 4, 7, 0),

	SOC_SINGLE("AGC Left Switch", AIC32X4_LAGC1, 7, 1, 0),
	SOC_SINGLE("AGC Right Switch", AIC32X4_RAGC1, 7, 1, 0),
	SOC_DOUBLE_R("AGC Target Level", AIC32X4_LAGC1, AIC32X4_RAGC1,
			4, 0x07, 0),
	SOC_DOUBLE_R("AGC Gain Hysteresis", AIC32X4_LAGC1, AIC32X4_RAGC1,
			0, 0x03, 0),
	SOC_DOUBLE_R("AGC Hysteresis", AIC32X4_LAGC2, AIC32X4_RAGC2,
			6, 0x03, 0),
	SOC_DOUBLE_R("AGC Noise Threshold", AIC32X4_LAGC2, AIC32X4_RAGC2,
			1, 0x1F, 0),
	SOC_DOUBLE_R("AGC Max PGA", AIC32X4_LAGC3, AIC32X4_RAGC3,
			0, 0x7F, 0),
	SOC_DOUBLE_R("AGC Attack Time", AIC32X4_LAGC4, AIC32X4_RAGC4,
			3, 0x1F, 0),
	SOC_DOUBLE_R("AGC Decay Time", AIC32X4_LAGC5, AIC32X4_RAGC5,
			3, 0x1F, 0),
	SOC_DOUBLE_R("AGC Noise Debounce", AIC32X4_LAGC6, AIC32X4_RAGC6,
			0, 0x1F, 0),
	SOC_DOUBLE_R("AGC Signal Debounce", AIC32X4_LAGC7, AIC32X4_RAGC7,
			0, 0x0F, 0),
};

static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC Switch", AIC32X4_HPLROUTE, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_L Switch", AIC32X4_HPLROUTE, 2, 1, 0),
};

static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC Switch", AIC32X4_HPRROUTE, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_R Switch", AIC32X4_HPRROUTE, 2, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC Switch", AIC32X4_LOLROUTE, 3, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC Switch", AIC32X4_LORROUTE, 3, 1, 0),
};

static const struct snd_soc_dapm_widget aic32x4_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", AIC32X4_DACSETUP, 7, 0),
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", AIC32X4_OUTPWRCTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", AIC32X4_OUTPWRCTL, 3, 0, NULL, 0),

	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", AIC32X4_DACSETUP, 6, 0),
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", AIC32X4_OUTPWRCTL, 4, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", AIC32X4_OUTPWRCTL, 2, 0, NULL, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias", AIC32X4_MICBIAS, 6, 0),

	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("LOL"),
	SND_SOC_DAPM_OUTPUT("LOR"),
	SND_SOC_DAPM_INPUT("IN1_L"),
	SND_SOC_DAPM_INPUT("IN1_R"),
	SND_SOC_DAPM_INPUT("IN2_L"),
	SND_SOC_DAPM_INPUT("IN2_R"),
	SND_SOC_DAPM_INPUT("IN3_L"),
	SND_SOC_DAPM_INPUT("IN3_R"),
};

static const struct snd_soc_dapm_route aic32x4_dapm_routes[] = {
	/* Left Output */
	{"HPL Output Mixer", "L_DAC Switch", "Left DAC"},
	{"HPL Output Mixer", "IN1_L Switch", "IN1_L"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},

	{"LOL Output Mixer", "L_DAC Switch", "Left DAC"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},

	/* Right Output */
	{"HPR Output Mixer", "R_DAC Switch", "Right DAC"},
	{"HPR Output Mixer", "IN1_R Switch", "IN1_R"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},

	{"LOR Output Mixer", "R_DAC Switch", "Right DAC"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},
};

static const struct regmap_range_cfg aic32x4_regmap_pages[] = {
	{
		.selector_reg = 0,
		.selector_mask  = 0xff,
		.window_start = 0,
		.window_len = 128,
		.range_min = 0,
		.range_max = AIC32X4_RMICPGAVOL,
	},
};

const struct regmap_config csky_aic32x4_regmap_config = {
	.max_register = AIC32X4_RMICPGAVOL,
	.ranges = aic32x4_regmap_pages,
	.num_ranges = ARRAY_SIZE(aic32x4_regmap_pages),
};
EXPORT_SYMBOL(csky_aic32x4_regmap_config);

static int aic32x4_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic32x4_priv *aic32x4 = snd_soc_codec_get_drvdata(codec);

	aic32x4->sysclk = freq;
	return 0;
}

static int aic32x4_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface_reg_1;
	u8 iface_reg_2;
	u8 iface_reg_3;

	iface_reg_1 = snd_soc_read(codec, AIC32X4_IFACE1);
	iface_reg_1 = iface_reg_1 & ~(3 << 6 | 3 << 2);
	iface_reg_2 = snd_soc_read(codec, AIC32X4_IFACE2);
	iface_reg_2 = 0;
	iface_reg_3 = snd_soc_read(codec, AIC32X4_IFACE3);
	iface_reg_3 = iface_reg_3 & ~(1 << 3);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg_1 |= AIC32X4_BCLKMASTER | AIC32X4_WCLKMASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		printk(KERN_ERR "aic32x4: invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg_1 |= (AIC32X4_DSP_MODE << AIC32X4_PLLJ_SHIFT);
		iface_reg_3 |= (1 << 3); /* invert bit clock */
		iface_reg_2 = 0x01; /* add offset 1 */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface_reg_1 |= (AIC32X4_DSP_MODE << AIC32X4_PLLJ_SHIFT);
		iface_reg_3 |= (1 << 3); /* invert bit clock */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg_1 |=
			(AIC32X4_RIGHT_JUSTIFIED_MODE << AIC32X4_PLLJ_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg_1 |=
			(AIC32X4_LEFT_JUSTIFIED_MODE << AIC32X4_PLLJ_SHIFT);
		break;
	default:
		printk(KERN_ERR "aic32x4: invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_write(codec, AIC32X4_IFACE1, iface_reg_1);
	snd_soc_write(codec, AIC32X4_IFACE2, iface_reg_2);
	snd_soc_write(codec, AIC32X4_IFACE3, iface_reg_3);
	return 0;
}

static int aic32x4_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic32x4_priv *aic32x4 = snd_soc_codec_get_drvdata(codec);
	u8 data;
	unsigned int mclk_fs_div;
	unsigned int ndac, mdac, dosr;

	if ((aic32x4->sysclk == 0) || (aic32x4->sysclk % params_rate(params))) {
		printk(KERN_ERR "aic32x4: invalid mclk(%d)\n", aic32x4->sysclk);
		return -EINVAL;
	}

	mclk_fs_div = aic32x4->sysclk / params_rate(params);
	/*
	 * mclk_fs_div = NDAC x MDAC x DOSR
	 */
	ndac = 1;
	mdac = 2;
	dosr = mclk_fs_div / (ndac * mdac);

	/* Use MCLK as CODEC_CLKIN and DAC_MOD_CLK as BDIV_CLKIN */
	snd_soc_write(codec, AIC32X4_CLKMUX, AIC32X4_MCLKIN);
	snd_soc_write(codec, AIC32X4_IFACE3, AIC32X4_DACMOD2BCLK);

	/* NDAC divider value */
	data = snd_soc_read(codec, AIC32X4_NDAC);
	data &= ~(0x7f);
	snd_soc_write(codec, AIC32X4_NDAC, data | ndac);

	/* MDAC divider value */
	data = snd_soc_read(codec, AIC32X4_MDAC);
	data &= ~(0x7f);
	snd_soc_write(codec, AIC32X4_MDAC, data | mdac);

	/* DOSR MSB & LSB values */
	snd_soc_write(codec, AIC32X4_DOSRMSB, (dosr >> 8));
	snd_soc_write(codec, AIC32X4_DOSRLSB, (dosr & 0xff));

	data = snd_soc_read(codec, AIC32X4_IFACE1);
	data = data & ~(3 << 4);
	switch (params_width(params)) {
	case 16:
		break;
	case 20:
		data |= (AIC32X4_WORD_LEN_20BITS << AIC32X4_DOSRMSB_SHIFT);
		break;
	case 24:
		data |= (AIC32X4_WORD_LEN_24BITS << AIC32X4_DOSRMSB_SHIFT);
		break;
	case 32:
		data |= (AIC32X4_WORD_LEN_32BITS << AIC32X4_DOSRMSB_SHIFT);
		break;
	}
	snd_soc_write(codec, AIC32X4_IFACE1, data);

	if (params_channels(params) == 1) {
		data = AIC32X4_RDAC2LCHN | AIC32X4_LDAC2LCHN;
	} else {
		if (aic32x4->swapdacs)
			data = AIC32X4_RDAC2LCHN | AIC32X4_LDAC2RCHN;
		else
			data = AIC32X4_LDAC2LCHN | AIC32X4_RDAC2RCHN;
	}
	snd_soc_update_bits(codec, AIC32X4_DACSETUP, AIC32X4_DAC_CHAN_MASK,
			data);

	return 0;
}

static int aic32x4_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 dac_reg;

	dac_reg = snd_soc_read(codec, AIC32X4_DACMUTE) & ~AIC32X4_MUTEON;
	if (mute)
		snd_soc_write(codec, AIC32X4_DACMUTE, dac_reg | AIC32X4_MUTEON);
	else
		snd_soc_write(codec, AIC32X4_DACMUTE, dac_reg);
	return 0;
}

static int aic32x4_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:

		/* Switch on NDAC Divider */
		snd_soc_update_bits(codec, AIC32X4_NDAC,
				    AIC32X4_NDACEN, AIC32X4_NDACEN);

		/* Switch on MDAC Divider */
		snd_soc_update_bits(codec, AIC32X4_MDAC,
				    AIC32X4_MDACEN, AIC32X4_MDACEN);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:

		/* Switch off MDAC Divider */
		snd_soc_update_bits(codec, AIC32X4_MDAC, AIC32X4_MDACEN, 0);

		/* Switch off NDAC Divider */
		snd_soc_update_bits(codec, AIC32X4_NDAC, AIC32X4_NDACEN, 0);
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}

	return 0;
}

#define AIC32X4_RATES	SNDRV_PCM_RATE_8000_96000
#define AIC32X4_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_U16_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_U24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE | \
				 SNDRV_PCM_FMTBIT_U32_LE)

static const struct snd_soc_dai_ops aic32x4_ops = {
	.hw_params = aic32x4_hw_params,
	.digital_mute = aic32x4_mute,
	.set_fmt = aic32x4_set_dai_fmt,
	.set_sysclk = aic32x4_set_dai_sysclk,
};

static struct snd_soc_dai_driver aic32x4_dai = {
	.name = "tlv320aic32x4-hifi",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AIC32X4_RATES,
		     .formats = AIC32X4_FORMATS,},
	.ops = &aic32x4_ops,
	.symmetric_rates = 1,
};

static int aic32x4_codec_probe(struct snd_soc_codec *codec)
{
	struct aic32x4_priv *aic32x4 = snd_soc_codec_get_drvdata(codec);
	u32 tmp_reg;

	snd_soc_write(codec, AIC32X4_RESET, 0x01);

	/* Power platform configuration */
	if (aic32x4->power_cfg & AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE)
		snd_soc_write(codec, AIC32X4_PWRCFG, AIC32X4_AVDDWEAKDISABLE);

	tmp_reg = (aic32x4->power_cfg & AIC32X4_PWR_AIC32X4_LDO_ENABLE) ?
			AIC32X4_LDOCTLEN : 0;
	snd_soc_write(codec, AIC32X4_LDOCTL, tmp_reg);

	tmp_reg = snd_soc_read(codec, AIC32X4_CMMODE);
	if (aic32x4->power_cfg & AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36)
		tmp_reg |= AIC32X4_LDOIN_18_36;
	if (aic32x4->power_cfg & AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED)
		tmp_reg |= AIC32X4_LDOIN2HP;
	snd_soc_write(codec, AIC32X4_CMMODE, tmp_reg);

	/* Set the default route: LDAC -> HPL, RDAC -> HPR */
	snd_soc_write(codec, AIC32X4_HPLROUTE, 0x08);
	snd_soc_write(codec, AIC32X4_HPRROUTE, 0x08);

	/* Unmute HPL/HPR Driver and set HPL/HPR Driver gain to 0dB */
	snd_soc_write(codec, AIC32X4_HPLGAIN, 0);
	snd_soc_write(codec, AIC32X4_HPRGAIN, 0);

	/*
	 * Headphone Driver Startup Control
	 *
	 * HW ext C = 220uF.
	 *
	 * Soft routing step time is 0ms
	 * Headphone ramps power up slowly in 2.0 time constants
	 * Headphone ramps power up time is determined with 2k resistance
	 */
	snd_soc_write(codec, AIC32X4_HEADSTART, 0x1A);

#ifdef NO_DAPM
	snd_soc_write(codec, AIC32X4_DACSETUP, 0xd4);
	snd_soc_write(codec, AIC32X4_OUTPWRCTL, 0x30);
	/* Switch on NDAC Divider */
	snd_soc_update_bits(codec, AIC32X4_NDAC,
			    AIC32X4_NDACEN, AIC32X4_NDACEN);

	/* Switch on MDAC Divider */
	snd_soc_update_bits(codec, AIC32X4_MDAC,
			    AIC32X4_MDACEN, AIC32X4_MDACEN);
#endif

	return 0;
}

static const struct snd_soc_dapm_widget dummy_codec_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTL"),
	SND_SOC_DAPM_OUTPUT("VOUTR"),
};

static const struct snd_soc_dapm_route dummy_codec_dapm_routes[] = {
	{ "VOUTL", NULL, "Playback" },
	{ "VOUTR", NULL, "Playback" },
};

static struct snd_soc_codec_driver soc_codec_dev_aic32x4 = {
	.probe = aic32x4_codec_probe,
	.set_bias_level = aic32x4_set_bias_level,
	.suspend_bias_off = true,

	.component_driver = {
		.controls		= aic32x4_snd_controls,
		.num_controls		= ARRAY_SIZE(aic32x4_snd_controls),
#ifndef NO_DAPM
		.dapm_widgets		= aic32x4_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(aic32x4_dapm_widgets),
		.dapm_routes		= aic32x4_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(aic32x4_dapm_routes),
#else
		.dapm_widgets		= dummy_codec_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(dummy_codec_dapm_widgets),
		.dapm_routes		= dummy_codec_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(dummy_codec_dapm_routes),
#endif
	},
};

static int aic32x4_parse_dt(struct aic32x4_priv *aic32x4,
			    struct device_node *np)
{
	aic32x4->swapdacs = false;
	aic32x4->sysclk = 0;
	aic32x4->power_cfg = AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE |
			AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED |
			AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36;
	return 0;
}

int csky_aic32x4_probe(struct device *dev, struct regmap *regmap)
{
	struct aic32x4_priv *aic32x4;
	struct device_node *np = dev->of_node;
	int ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	aic32x4 = devm_kzalloc(dev, sizeof(struct aic32x4_priv), GFP_KERNEL);
	if (aic32x4 == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, aic32x4);

	ret = aic32x4_parse_dt(aic32x4, np);
	if (ret) {
		dev_err(dev, "Failed to parse DT node\n");
		return ret;
	}

	ret = snd_soc_register_codec(dev,
			&soc_codec_dev_aic32x4, &aic32x4_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register codec\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(csky_aic32x4_probe);

int csky_aic32x4_remove(struct device *dev)
{
	snd_soc_unregister_codec(dev);
	return 0;
}
EXPORT_SYMBOL(csky_aic32x4_remove);

MODULE_DESCRIPTION("ASoC tlv320aic32x4 codec driver");
MODULE_AUTHOR("Lei Ling <lei_ling@c-sky.com>");
MODULE_LICENSE("GPL v2");
