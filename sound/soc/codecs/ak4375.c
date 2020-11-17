/* SPDX-License-Identifier: GPL-2.0 */


/* TODO: check includes */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#define AK4375_00_POWER_MANAGEMENT1		0x00
#define AK4375_01_POWER_MANAGEMENT2		0x01
#define AK4375_02_POWER_MANAGEMENT3		0x02
#define AK4375_03_POWER_MANAGEMENT4		0x03
#define AK4375_04_OUTPUT_MODE_SETTING		0x04
#define AK4375_05_CLOCK_MODE_SELECT		0x05
#define AK4375_06_DIGITAL_FILTER_SELECT		0x06
#define AK4375_07_DAC_MONO_MIXING		0x07
#define AK4375_08_JITTER_CLEANER_SETTING1	0x08
#define AK4375_09_JITTER_CLEANER_SETTING2	0x09
#define AK4375_0A_JITTER_CLEANER_SETTING3	0x0A
#define AK4375_0B_LCH_OUTPUT_VOLUME		0x0B
#define AK4375_0C_RCH_OUTPUT_VOLUME		0x0C
#define AK4375_0D_HP_VOLUME_CONTROL		0x0D
#define AK4375_0E_PLL_CLK_SOURCE_SELECT		0x0E
#define AK4375_0F_PLL_REF_CLK_DIVIDER1		0x0F
#define AK4375_10_PLL_REF_CLK_DIVIDER2		0x10
#define AK4375_11_PLL_FB_CLK_DIVIDER1		0x11
#define AK4375_12_PLL_FB_CLK_DIVIDER2		0x12
#define AK4375_13_SRC_CLK_SOURCE		0x13
#define AK4375_14_DAC_CLK_DIVIDER		0x14
#define AK4375_15_AUDIO_IF_FORMAT		0x15
#define AK4375_24_MODE_CONTROL			0x24

/* Bitfield Definitions */
#define AK4375_FS_MASK				GENMASK(4,0)
#define AK4375_FS_8KHZ				(0x00 << 0)
#define AK4375_FS_11_025KHZ			(0x01 << 0)
#define AK4375_FS_16KHZ				(0x04 << 0)
#define AK4375_FS_22_05KHZ			(0x05 << 0)
#define AK4375_FS_32KHZ				(0x08 << 0)
#define AK4375_FS_44_1KHZ			(0x09 << 0)
#define AK4375_FS_48KHZ				(0x0A << 0)
#define AK4375_FS_88_2KHZ			(0x0D << 0)
#define AK4375_FS_96KHZ				(0x0E << 0)
#define AK4375_FS_176_4KHZ			(0x11 << 0)
#define AK4375_FS_192KHZ			(0x12 << 0)

#define AK4375_CM				(0x03 << 5)
#define AK4375_CM_0				(0 << 5)
#define AK4375_CM_1				(1 << 5)
#define AK4375_CM_2				(2 << 5)
#define AK4375_CM_3				(3 << 5)

#define AK4375_06_DIGITAL_FILTER_MASK		GENMASK(7,6)
#define AK4375_0A_SELDAIN_MASK			BIT(1)
#define AK4375_0D_HPG_MASK			GENMASK(2,0)
#define AK4375_15_DEVICEID_MASK			GENMASK(7,5)
#define AK4375_15_DIF_MASK			BIT(2)
#define AK4375_15_DIF_I2S_MODE			(0 << 2)
#define AK4375_15_DIF_MSB_MODE			(1 << 2)

/* 48kHz System: 512fs */
#define XTAL_OSC_FS				24576000
/* For 44.1kHz System: 512fs			22579200 */

/* Timer delays (ms) */
#define LVDTM_HOLD_TIME				30
#define VDDTM_HOLD_TIME				500
#define HPTM_HOLD_TIME				15

/* Soft Mute cycle (ms): 0:22msec 1:43msec) */
#define SMUTE_TIME_MODE				0

/* Chip IDs */
#define DEVICEID_AK4375				0x00
#define DEVICEID_AK4375A			0x01
#define DEVICEID_AK4376A			0x02
#define DEVICEID_AK4377				0x03
#define DEVICEID_AK4331				0x07

struct ak4375_drvdata {
	struct snd_soc_dai_driver *dai_drv;
	const struct snd_soc_component_driver *comp_drv;
};

struct ak4375_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *pdn_gpiod;
	struct gpio_desc *mclk_gpiod;
	int digfil;		/* DASD, DASL bits */
	int fs1;		/* sampling rate */
	int fs2;		/* sampling rate */
	int rclk;		/* master clock */
	int nSeldain;		/* 0:Bypass, 1:SRC */
	int nBickFreq;		/* 0:32fs, 1:48fs, 2:64fs */
	int nSrcOutFsSel;	/* 0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz */
	int nPllMode;		/* 0:PLL OFF, 1: PLL ON */
	int nPllMCKI;		/* 0:PLL not use, 1: PLL use */
	int nSmt;		/* 0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO */
	int dfsrc8fs;		/* DFTHR bit and SRCO8FS bit */
	int fmt;
};

static const struct reg_default ak4375_reg_defaults[] = {
	{ 0x00, 0x00 }, { 0x01, 0x00 }, { 0x02, 0x00 },
	{ 0x03, 0x00 }, { 0x04, 0x00 }, { 0x05, 0x00 },
	{ 0x06, 0x00 }, { 0x07, 0x00 }, { 0x08, 0x00 },
	{ 0x09, 0x00 }, { 0x0A, 0x00 }, { 0x0B, 0x19 },
	{ 0x0C, 0x19 }, { 0x0D, 0x75 }, { 0x0E, 0x01 },
	{ 0x0F, 0x00 }, { 0x10, 0x00 }, { 0x11, 0x00 },
	{ 0x12, 0x00 }, { 0x13, 0x00 }, { 0x14, 0x00 },
	{ 0x15, 0x00 }, { 0x24, 0x00 },
};

/* Output Digital volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB) */
static DECLARE_TLV_DB_SCALE(ovl_tlv, -1250, 50, 0);
static DECLARE_TLV_DB_SCALE(ovr_tlv, -1250, 50, 0);

/* TODO: fixme ? AK4331 datasheet says -10 dB to 4 dB, -10 dB = mute */
/* HP-Amp Analog volume control:
 * from -4.2 to 6 dB in 2 dB steps (mute instead of -4.2 dB) */
static DECLARE_TLV_DB_SCALE(hpg_tlv, -4200, 20, 0);

static const char *ak4375_ovolcn_select_texts[] = {"Dependent", "Independent"};
static const char *ak4375_mdacl_select_texts[] = {"x1", "x1/2"};
static const char *ak4375_mdacr_select_texts[] = {"x1", "x1/2"};
static const char *ak4375_invl_select_texts[] = {"Normal", "Inverting"};
static const char *ak4375_invr_select_texts[] = {"Normal", "Inverting"};
static const char *ak4375_cpmod_select_texts[] =
	{"Automatic Switching", "+-VDD Operation", "+-1/2VDD Operation"};
static const char *ak4375_hphl_select_texts[] = {"9ohm", "200kohm"};
static const char *ak4375_hphr_select_texts[] = {"9ohm", "200kohm"};

/*
 * DASD, DASL bits Digital Filter Setting
 * 0, 0 : Sharp Roll-Off Filter
 * 0, 1 : Slow Roll-Off Filter
 * 1, 0 : Short delay Sharp Roll-Off Filter
 * 1, 1 : Short delay Slow Roll-Off Filter
 */
static const char * const ak4375_digfil_select_texts[] = {
	"Sharp Roll-Off Filter",
	"Slow Roll-Off Filter",
	"Short delay Sharp Roll-Off Filter",
	"Short delay Slow Roll-Off Filter",
};

static int get_dfsrc8fs(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ak4375->dfsrc8fs;

	return 0;
}

static int set_dfsrc8fs(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	ak4375->dfsrc8fs = ucontrol->value.enumerated.item[0];

	switch(ak4375->dfsrc8fs) {
	case 0:
		snd_soc_component_update_bits(component, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x00);
		snd_soc_component_update_bits(component, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00);
		break;
	case 1:
		snd_soc_component_update_bits(component, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x08);
		snd_soc_component_update_bits(component, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00);
		break;
	case 2:
		snd_soc_component_update_bits(component, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x08);
		snd_soc_component_update_bits(component, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x20);
		break;
	}

	return 0;
}

static int get_bickfs(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ak4375->nBickFreq;

	return 0;
}

static int set_bickfs(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	ak4375->nBickFreq = ucontrol->value.enumerated.item[0];

	switch(ak4375->nBickFreq) {
	case 0:
		snd_soc_component_update_bits(component, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x01);
		break;
	case 1:
		snd_soc_component_update_bits(component, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x00);
		break;
	default:
		snd_soc_component_update_bits(component, AK4375_15_AUDIO_IF_FORMAT, 0x02, 0x02);
		break;
	}

	return 0;
}

static int get_srcfs(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ak4375->nSrcOutFsSel;

	return 0;
}

static int set_srcfs(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	ak4375->nSrcOutFsSel = ucontrol->value.enumerated.item[0];

	return 0;
}

static int get_akpwdn(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	/* TODO: uh ? should do better than downstream... */
	return 0;
}

static int set_akpwdn(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.enumerated.item[0];

	if (ak4375->pdn_gpiod) {
		dev_info(ak4375->dev, "set_akpwdn = %d\n", value);
		gpiod_set_value_cansleep(ak4375->pdn_gpiod, value);
	}

	return 0;
}

static const struct soc_enum ak4375_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4375_0B_LCH_OUTPUT_VOLUME, 7,
			ARRAY_SIZE(ak4375_ovolcn_select_texts), ak4375_ovolcn_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 2,
			ARRAY_SIZE(ak4375_mdacl_select_texts), ak4375_mdacl_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 6,
			ARRAY_SIZE(ak4375_mdacr_select_texts), ak4375_mdacr_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 3,
			ARRAY_SIZE(ak4375_invl_select_texts), ak4375_invl_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 7,
			ARRAY_SIZE(ak4375_invr_select_texts), ak4375_invr_select_texts),
	SOC_ENUM_SINGLE(AK4375_03_POWER_MANAGEMENT4, 2,
			ARRAY_SIZE(ak4375_cpmod_select_texts), ak4375_cpmod_select_texts),
	SOC_ENUM_SINGLE(AK4375_04_OUTPUT_MODE_SETTING, 0,
			ARRAY_SIZE(ak4375_hphl_select_texts), ak4375_hphl_select_texts),
	SOC_ENUM_SINGLE(AK4375_04_OUTPUT_MODE_SETTING, 1,
			ARRAY_SIZE(ak4375_hphr_select_texts), ak4375_hphr_select_texts),
	SOC_ENUM_SINGLE(AK4375_06_DIGITAL_FILTER_SELECT, 6,
			ARRAY_SIZE(ak4375_digfil_select_texts), ak4375_digfil_select_texts),
	SOC_ENUM_SINGLE(AK4375_09_JITTER_CLEANER_SETTING2, 4,
			ARRAY_SIZE(ak4375_digfil_select_texts), ak4375_digfil_select_texts),
};

static const char *bickfreq_on_select[] = {"32fs", "48fs", "64fs"};

static const char *srcoutfs_on_select[] = {"48kHz", "96kHz", "192kHz"};
/* TODO: also manage 44.1kHz case */
/* {"44.1kHz", "88.2kHz", "176.4kHz"}; */

static const char *pllmode_on_select[] = {"OFF", "ON"};
static const char *smtcycle_on_select[] = {"1024", "2048", "4096", "8192"};
static const char *dfsrc8fs_on_select[] = {"Digital Filter", "Bypass", "8fs mode"};
static const char *tct_akm_pwdn[] = {"OFF", "ON"};

static const struct soc_enum ak4375_bitset_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bickfreq_on_select), bickfreq_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(srcoutfs_on_select), srcoutfs_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmode_on_select), pllmode_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smtcycle_on_select), smtcycle_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dfsrc8fs_on_select), dfsrc8fs_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tct_akm_pwdn), tct_akm_pwdn),
};


static const struct snd_kcontrol_new ak4375_snd_controls[] = {
        SOC_SINGLE_TLV("AK4375 Digital Output VolumeL",
			AK4375_0B_LCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovl_tlv),
        SOC_SINGLE_TLV("AK4375 Digital Output VolumeR",
                        AK4375_0C_RCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovr_tlv),
        SOC_SINGLE_TLV("AK4375 HP-Amp Analog Volume",
                        AK4375_0D_HP_VOLUME_CONTROL, 0, 0x1F, 0, hpg_tlv),

	SOC_ENUM("AK4375 Digital Volume Control", ak4375_dac_enum[0]),
	SOC_ENUM("AK4375 DACL Signal Level", ak4375_dac_enum[1]),
	SOC_ENUM("AK4375 DACR Signal Level", ak4375_dac_enum[2]),
	SOC_ENUM("AK4375 DACL Signal Invert", ak4375_dac_enum[3]),
	SOC_ENUM("AK4375 DACR Signal Invert", ak4375_dac_enum[4]),
	SOC_ENUM("AK4375 Charge Pump Mode", ak4375_dac_enum[5]),
	SOC_ENUM("AK4375 HPL Power-down Resistor", ak4375_dac_enum[6]),
	SOC_ENUM("AK4375 HPR Power-down Resistor", ak4375_dac_enum[7]),
	SOC_ENUM("AK4375 DAC Digital Filter Mode", ak4375_dac_enum[8]),
	SOC_ENUM("AK4375 SRC Digital Filter Mode", ak4375_dac_enum[9]),

	SOC_ENUM_EXT("AK4375 Data Output mode", ak4375_bitset_enum[4], get_dfsrc8fs, set_dfsrc8fs),
	SOC_ENUM_EXT("AK4375 BICK Frequency Select", ak4375_bitset_enum[0], get_bickfs, set_bickfs),
	SOC_ENUM_EXT("AK4375 SRC Output FS", ak4375_bitset_enum[1], get_srcfs, set_srcfs),
//      SOC_ENUM_EXT("AK4375 Soft Mute Cycle Select", ak4375_bitset_enum[3], get_smtcycle, set_smtcycle),

	SOC_SINGLE("AK4375 SRC Semi-Auto Mode", AK4375_09_JITTER_CLEANER_SETTING2, 1, 1, 0),
	SOC_SINGLE("AK4375 SRC Dither", AK4375_0A_JITTER_CLEANER_SETTING3, 4, 1, 0),
	SOC_SINGLE("AK4375 Soft Mute Control", AK4375_09_JITTER_CLEANER_SETTING2, 0, 1, 0),

	SOC_ENUM_EXT("AKM HP", ak4375_bitset_enum[ARRAY_SIZE(ak4375_bitset_enum)-1], get_akpwdn, set_akpwdn),

};

/* DAC MUX */
static const char *ak4375_seldain_select_texts[] = {"SDTI", "SRC"};

static const struct soc_enum ak4375_seldain_mux_enum =
	SOC_ENUM_SINGLE(AK4375_0A_JITTER_CLEANER_SETTING3, 1,
			ARRAY_SIZE(ak4375_seldain_select_texts), ak4375_seldain_select_texts);

static const struct snd_kcontrol_new ak4375_seldain_mux_control =
	SOC_DAPM_ENUM("SRC Select", ak4375_seldain_mux_enum);

/* HPL Mixer */
static const struct snd_kcontrol_new ak4375_hpl_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACL", AK4375_07_DAC_MONO_MIXING, 0, 1, 0),
	SOC_DAPM_SINGLE("RDACL", AK4375_07_DAC_MONO_MIXING, 1, 1, 0),
};

/* HPR Mixer */
static const struct snd_kcontrol_new ak4375_hpr_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACR", AK4375_07_DAC_MONO_MIXING, 4, 1, 0),
	SOC_DAPM_SINGLE("RDACR", AK4375_07_DAC_MONO_MIXING, 5, 1, 0),
};

static int ak4375_dac_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	dev_info(ak4375->dev, "ak4375_dac_event()\n");

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01); //PMCP1=1
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x01, 0x01); //PMCP1=1
		mdelay(6); udelay(500);
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x30, 0x30); //PMLDO1P/N=1
		mdelay(1);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x02, 0x02); //PMCP2=1
		mdelay(4); udelay(500);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x02, 0x00); //PMCP2=0
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x30, 0x00); //PMLDO1P/N=0
		snd_soc_component_update_bits(component, AK4375_01_POWER_MANAGEMENT2, 0x01, 0x00); //PMCP1=0
		snd_soc_component_update_bits(component, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00); //PMCP1=1
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget ak4375_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("AK4375 DAC", NULL, AK4375_02_POWER_MANAGEMENT3, 0, 0,
			   ak4375_dac_event, (SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD
				   | SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

	SND_SOC_DAPM_SUPPLY("AK4375 OSC", AK4375_00_POWER_MANAGEMENT1, 4, 0, NULL, 0),
	SND_SOC_DAPM_MUX("AK4375 DAC MUX", SND_SOC_NOPM, 0, 0, &ak4375_seldain_mux_control),

	SND_SOC_DAPM_AIF_IN("AK4375 SRC", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AK4375 SDTI", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("AK4375 HPL"),
	SND_SOC_DAPM_OUTPUT("AK4375 HPR"),

	SND_SOC_DAPM_MIXER("AK4375 HPR Mixer", AK4375_03_POWER_MANAGEMENT4, 1, 0,
			   &ak4375_hpr_mixer_controls[0], ARRAY_SIZE(ak4375_hpr_mixer_controls)),
	SND_SOC_DAPM_MIXER("AK4375 HPL Mixer", AK4375_03_POWER_MANAGEMENT4, 0, 0,
			   &ak4375_hpl_mixer_controls[0], ARRAY_SIZE(ak4375_hpl_mixer_controls)),

};

static const struct snd_soc_dapm_route ak4375_intercon[] = {
	/* {"AK4375 DAC",	NULL,		"AK4375 PLL"}, */
	{"AK4375 SRC",		NULL,		"AK4375 OSC"},

	{"AK4375 DAC MUX",	"SRC",		"AK4375 SRC"},
	{"AK4375 DAC MUX",	"SDTI",		"AK4375 SDTI"},

	{"AK4375 DAC", 		NULL,		"AK4375 DAC MUX"},

	{"AK4375 HPL Mixer",	"LDACL",	"AK4375 DAC"},
	{"AK4375 HPL Mixer",	"RDACL",	"AK4375 DAC"},
	{"AK4375 HPR Mixer",	"LDACR",	"AK4375 DAC"},
	{"AK4375 HPR Mixer",	"RDACR",	"AK4375 DAC"},

	{"AK4375 HPL",		NULL,		"AK4375 HPL Mixer"},
	{"AK4375 HPR",		NULL,		"AK4375 HPR Mixer"},
};

static int ak4375_set_mcki(struct snd_soc_component *component, int fs, int rclk)
{
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	u8 mode, mode2;
	int mcki_rate;

	dev_info(ak4375->dev, "ak4375_set_mcki()\n");

	if ((fs == 0) || (rclk == 0))
		return 0;

	if (rclk > 28800000)
		return -EINVAL;

	mcki_rate = rclk / fs;

	mode = snd_soc_component_read(component, AK4375_05_CLOCK_MODE_SELECT);
	mode &= ~AK4375_CM;

	switch (mcki_rate) {
	case 128:
		if (ak4375->nSeldain == 0)
			mode |= AK4375_CM_3;
		break;
	case 256:
		mode |= AK4375_CM_0;
		if (ak4375->nSeldain == 0) {	/* SRC Bypass mode */
			mode2 = snd_soc_component_read(component, AK4375_24_MODE_CONTROL);
			if (fs <= 12000)
				mode2 &= 0x40;	/* DSMLP = 1 */
			else
				mode2 &= ~0x40;	/* DSMLP = 0 */
			snd_soc_component_write(component, AK4375_24_MODE_CONTROL, mode2);
		}
		break;
	case 512:
		mode |= AK4375_CM_1;
		break;
	case 1024:
		mode |= AK4375_CM_2;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(component, AK4375_05_CLOCK_MODE_SELECT, mode);

	return 0;
}

static int ak4375_set_src_mcki(struct snd_soc_component *component, int fs)
{
	u8 nrate = snd_soc_component_read(component, AK4375_08_JITTER_CLEANER_SETTING1);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	/* TODO: proper way to select 48kHz/44.1kHz and set XTAL_OSC_FS accordingly */
	int src_out_fs = 48000 * (1 << ak4375->nSrcOutFsSel);
	int oclk_rate = XTAL_OSC_FS / src_out_fs;

	dev_info(ak4375->dev, "ak4375_set_src_mcki()  fs=%d  src_out_fs=%d\n", fs, src_out_fs);

	nrate &= ~0x7F; /* CM2 1-0 bits, FS2 4-0 bits */

	switch(src_out_fs) {
	case 44100:
		nrate |= AK4375_FS_44_1KHZ;
		break;
	case 48000:
		nrate |= AK4375_FS_48KHZ;
		break;
	case 88200:
		nrate |= AK4375_FS_88_2KHZ;
		break;
	case 96000:
		nrate |= AK4375_FS_96KHZ;
		break;
	case 176400:
		nrate |= AK4375_FS_176_4KHZ;
		break;
	case 192000:
		nrate |= AK4375_FS_192KHZ;
		break;
	default:
		return -EINVAL;
	}

	switch(oclk_rate) {
	case 128:
		nrate |= AK4375_CM_3;
		break;
	case 256:
		nrate |= AK4375_CM_0;
		break;
	case 512:
		nrate |= AK4375_CM_1;
		break;
	case 1024:
		nrate |= AK4375_CM_2;
		break;
	default:
		return -EINVAL;
	}

	ak4375->fs2 = src_out_fs;
	snd_soc_component_write(component, AK4375_08_JITTER_CLEANER_SETTING1, nrate);

	return 0;
}

static int ak4375_set_pllblock(struct snd_soc_component *component, int fs)
{
	u8 mode = snd_soc_component_read(component, AK4375_05_CLOCK_MODE_SELECT);
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit, DIVbit;
	int nTemp;

	dev_info(ak4375->dev, "ak4375_set_pllblock()  fs=%d\n", fs);

	mode &= ~AK4375_CM;

	if (ak4375->nSeldain == 0) {	/* SRC Bypass Mode */
		if ( fs <= 24000 ) {
			mode |= AK4375_CM_1;
			nMClk = 512 * fs;
		} else if ( fs <= 96000 ) {
			mode |= AK4375_CM_0;
			nMClk = 256 * fs;
		} else {
			mode |= AK4375_CM_3;
			nMClk = 128 * fs;
		}
	} else {
		if ( fs <= 24000 ) {
			mode |= AK4375_CM_1;
			nMClk = 512 * fs;
		} else  {
			mode |= AK4375_CM_0;
			nMClk = 256 * fs;
		}
	}

	snd_soc_component_write(component, AK4375_05_CLOCK_MODE_SELECT, mode);

	if ((fs % 8000) == 0)
		nPLLClk = 122880000;
	else if ((fs == 11025) && (ak4375->nBickFreq == 1))
		nPLLClk = 101606400;
	else
		nPLLClk = 112896000;

	/* TODO: manage PLL_MICK96M_MODE ? */
	if (ak4375->nBickFreq == 0 ) {		/* 32fs */
		if (fs <= 96000)
			PLDbit = 1;
		else
			PLDbit = 2;
		nRefClk = 32 * fs / PLDbit;
	} else if (ak4375->nBickFreq == 1) {	/* 48fs */
		if (fs <= 16000)
			PLDbit = 1;
		else
			PLDbit = 3;
		nRefClk = 48 * fs / PLDbit;
	} else {				/* 64fs */
		if (fs <= 48000)
			PLDbit = 1;
		else if (fs <= 96000)
			PLDbit = 2;
		else
			PLDbit = 4;
		nRefClk = 64 * fs / PLDbit;
	}

	PLMbit = nPLLClk / nRefClk;

	if ((ak4375->nSeldain == 0) || (fs <= 96000)) {
		MDIVbit = nPLLClk / nMClk;
		DIVbit = 0;
	} else {
		MDIVbit = 5;
		DIVbit = 1;
	}

	PLDbit--;
	PLMbit--;
	MDIVbit--;
	/* PLD 15-0 bits */
	snd_soc_component_write(component, AK4375_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_component_write(component, AK4375_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	/* PLM 15-0 bits */
	snd_soc_component_write(component, AK4375_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_component_write(component, AK4375_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
	/* DIV bit */
	nTemp = snd_soc_component_read(component, AK4375_13_SRC_CLK_SOURCE);
	nTemp &= ~0x10;
	nTemp |= (DIVbit << 4);

	/* TODO: manage PLL_MICK96M_MODE ? */
	/* TODO: check comments... */
	/* DIV=0or1,SRCCKS=1(SRC Clock Select=PLL) set */
	snd_soc_component_write(component, AK4375_13_SRC_CLK_SOURCE, (nTemp | 0x01));
	snd_soc_component_update_bits(component, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x1, 0x1); /* PLS=1(BICK) */
	snd_soc_component_write(component, AK4375_14_DAC_CLK_DIVIDER, MDIVbit); /* MDIV 7-0 bits */

	return 0;
}

static int ak4375_set_timer(struct snd_soc_component *component)
{
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	int ret, curdata;
	int count, tm, nfs;
	int lvdtm, vddtm, hptm;

	dev_info(ak4375->dev, "ak4375_set_timer()\n");

	lvdtm = 0;
	vddtm = 0;
	hptm = 0;

	if (ak4375->nSeldain == 1)
		nfs = ak4375->fs2;
	else
		nfs = ak4375->fs1;

	ret = snd_soc_component_read(component, AK4375_03_POWER_MANAGEMENT4);
	curdata = (ret & 0x70) >> 4;
	ret &= ~0x70;
	do {
		count = 1000 * (64 << lvdtm);
		tm = count / nfs;
		if (tm > LVDTM_HOLD_TIME)
			break;
		lvdtm++;
	} while (lvdtm < 7);
	if (curdata != lvdtm)
		snd_soc_component_write(component, AK4375_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));

	ret = snd_soc_component_read(component, AK4375_04_OUTPUT_MODE_SETTING);
	curdata = (ret & 0x3C) >> 2;
	ret &= ~0x3C;
	do {
		count = 1000 * (1024 << vddtm);
		tm = count / nfs;
		if (tm > VDDTM_HOLD_TIME)
			break;
		vddtm++;
	} while (vddtm < 8);
	if (curdata != vddtm)
		snd_soc_component_write(component, AK4375_04_OUTPUT_MODE_SETTING, (ret | (vddtm << 2)));

	ret = snd_soc_component_read(component, AK4375_0D_HP_VOLUME_CONTROL);
	curdata = (ret & 0xE0) >> 5;
	ret &= ~0xE0;
	do {
		count = 1000 * (128 << hptm);
		tm = count / nfs;
		if (tm > HPTM_HOLD_TIME)
			break;
		hptm++;
	} while (hptm < 4);
	if (curdata != hptm)
		snd_soc_component_write(component, AK4375_0D_HP_VOLUME_CONTROL, (ret | (hptm << 5)));

	return 0;
}

static int ak4375_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	u8 src;
	u8 fs;

	ak4375->fs1 = params_rate(params);

	dev_info(ak4375->dev, "ak4375_hw_params()     fs1=%d\n", ak4375->fs1);

	fs = snd_soc_component_read(component, AK4375_05_CLOCK_MODE_SELECT);
	fs &= ~AK4375_FS_MASK;

	switch(ak4375->fs1) {
	case 8000:
		fs |= AK4375_FS_8KHZ;
		break;
	case 11025:
		fs |= AK4375_FS_11_025KHZ;
		break;
	case 16000:
		fs |= AK4375_FS_16KHZ;
		break;
	case 22050:
		fs |= AK4375_FS_22_05KHZ;
		break;
	case 32000:
		fs |= AK4375_FS_32KHZ;
		break;
	case 44100:
		fs |= AK4375_FS_44_1KHZ;
		break;
	case 48000:
		fs |= AK4375_FS_48KHZ;
		break;
	case 88200:
		fs |= AK4375_FS_88_2KHZ;
		break;
	case 96000:
		fs |= AK4375_FS_96KHZ;
		break;
	case 176400:
		fs |= AK4375_FS_176_4KHZ;
		break;
	case 192000:
		fs |= AK4375_FS_192KHZ;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(component, AK4375_05_CLOCK_MODE_SELECT, fs);

	if (ak4375->nPllMode == 0)
		ak4375_set_mcki(component, ak4375->fs1, ak4375->rclk);
	else
		ak4375_set_pllblock(component, ak4375->fs1);

	src = snd_soc_component_read(component, AK4375_0A_JITTER_CLEANER_SETTING3);
	src = (src & 0x02) >> 1;
	if (src == 1) {
		ak4375->nSeldain = 1;
		snd_soc_component_update_bits(component, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0xC2);
		ak4375_set_src_mcki(component, ak4375->fs1);
	} else {
		ak4375->nSeldain = 0;
		snd_soc_component_update_bits(component, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0x00);
	}

	ak4375_set_timer(component);

	return 0;
}

static int ak4375_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	u8 format;

	dev_info(ak4375->dev, "ak4375_set_dai_fmt()  fmt=%d\n", fmt);

	format = snd_soc_component_read(component, AK4375_15_AUDIO_IF_FORMAT);
	format &= ~AK4375_15_DIF_MASK;

	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK4375_15_DIF_I2S_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK4375_15_DIF_MSB_MODE;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(component, AK4375_15_AUDIO_IF_FORMAT, format);

	return 0;
}

static int ak4375_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				 unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	dev_info(ak4375->dev, "ak4375_set_dai_sysclk() clk_id=%d  freq=%d  dir=%d\n", clk_id, freq, dir);

	ak4375->rclk = freq;

	if (ak4375->nPllMode == 0)
		ak4375_set_mcki(component, ak4375->fs1, freq);

	return 0;
}

static int ak4375_set_dai_mute(struct snd_soc_dai *dai, int mute, int direction)
{
        struct snd_soc_component *component = dai->component;
        struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);
	int nfs, ndt, ndt2;
	int ret = 0;

	dev_info(ak4375->dev, "ak4375_set_dai_mute()  mute=%d  direction=%d\n", mute, direction);

	if (ak4375->nSeldain == 1)
		nfs = ak4375->fs2;
	else
		nfs = ak4375->fs1;

	if (mute) {
		if (ak4375->nSeldain) {
			ret = snd_soc_component_update_bits(component, AK4375_09_JITTER_CLEANER_SETTING2, 0x01, 0x01);
			ndt = (1024000 << ak4375->nSmt) / nfs;
			mdelay(ndt);
			ret = snd_soc_component_update_bits(component, AK4375_02_POWER_MANAGEMENT3, 0x80, 0x00);
		}
	} else {
		ak4375->nSmt = ak4375->nSrcOutFsSel + SMUTE_TIME_MODE;
		ret = snd_soc_component_update_bits(component, AK4375_09_JITTER_CLEANER_SETTING2,
						    0x0c, (ak4375->nSmt << 2));
		ndt = 26 * nfs / 44100;
		if (ak4375->nSeldain) {
			ret = snd_soc_component_update_bits(component, AK4375_02_POWER_MANAGEMENT3, 0x80, 0x80);
			ndt2 = (1024000 << ak4375->nSmt) / nfs;
			ndt -= ndt2;
			if (ndt < 4)
				ndt = 4;
			mdelay(ndt);
			ret = snd_soc_component_update_bits(component, AK4375_09_JITTER_CLEANER_SETTING2, 0x01, 0x00);
			mdelay(ndt2);
		} else {
			mdelay(ndt);
		}
	}

	return ret;
};

#define AK4375_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

static const unsigned int ak4375_rates[] = {
	8000, 11025, 16000, 22050,
	32000, 44100, 48000, 88200,
	96000, 176400, 192000,
};

static const struct snd_pcm_hw_constraint_list ak4375_rate_constraints = {
	.count = ARRAY_SIZE(ak4375_rates),
	.list = ak4375_rates,
};

static int ak4375_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &ak4375_rate_constraints);

	return ret;
}

static const struct snd_soc_dai_ops ak4375_dai_ops = {
	.startup	= ak4375_startup,
	.hw_params	= ak4375_hw_params,
	.set_fmt	= ak4375_set_dai_fmt,
	.set_sysclk	= ak4375_set_dai_sysclk,
	.mute_stream	= ak4375_set_dai_mute,
};

static struct snd_soc_dai_driver ak4375_dai = {
	.name = "ak4375-hifi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_KNOT,
		.formats	= AK4375_FORMATS,
        },
	.ops = &ak4375_dai_ops,
};

static void ak4375_power_off(struct ak4375_priv *ak4375)
{
	dev_info(ak4375->dev, "power off\n");

	if (ak4375->pdn_gpiod) {
		gpiod_set_value_cansleep(ak4375->pdn_gpiod, 0);
		usleep_range(1000, 2000);
	}
}

static void ak4375_power_on(struct ak4375_priv *ak4375)
{
	dev_info(ak4375->dev, "power on\n");

	if (ak4375->pdn_gpiod) {
		gpiod_set_value_cansleep(ak4375->pdn_gpiod, 1);
		usleep_range(1000, 2000);
	}
}

static int ak4375_probe(struct snd_soc_component *component)
{
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	dev_info(ak4375->dev, "probe\n");
	ak4375->fs1 = 48000;
	ak4375->fs2 = 48000;
	ak4375->rclk = 0;
	ak4375->nSeldain = 0;
	ak4375->nBickFreq = 0;
	ak4375->nSrcOutFsSel = 0;
	ak4375->nPllMode = 1;
	ak4375->nSmt = 0;
	ak4375->dfsrc8fs = 0;

	return 0; //ak4375_init(component);
}

static void ak4375_remove(struct snd_soc_component *component)
{
	struct ak4375_priv *ak4375 = snd_soc_component_get_drvdata(component);

	dev_info(ak4375->dev, "remove\n");
	ak4375_power_off(ak4375);
}

#ifdef CONFIG_PM
static int __maybe_unused ak4375_runtime_suspend(struct device *dev)
{
	struct ak4375_priv *ak4375 = dev_get_drvdata(dev);

	dev_info(ak4375->dev, "ak4375_runtime_suspend()\n");

	//regcache_cache_only(ak4375->regmap, true);

	ak4375_power_off(ak4375);

	return 0;
}

static int __maybe_unused ak4375_runtime_resume(struct device *dev)
{
	struct ak4375_priv *ak4375 = dev_get_drvdata(dev);

	dev_info(ak4375->dev, "ak4375_runtime_resume()\n");

	ak4375_power_off(ak4375);
	ak4375_power_on(ak4375);

	//regcache_cache_only(ak4375->regmap, false);
	//regcache_mark_dirty(ak4375->regmap);

	//return regcache_sync(ak4375->regmap);
	return 0;
}
#endif /* CONFIG_PM */

static const struct snd_soc_component_driver soc_codec_dev_ak4375 = {
	.probe			= ak4375_probe,
	.remove			= ak4375_remove,
	.controls		= ak4375_snd_controls,
	.num_controls		= ARRAY_SIZE(ak4375_snd_controls),
	.dapm_widgets		= ak4375_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ak4375_dapm_widgets),
	.dapm_routes		= ak4375_intercon,
	.num_dapm_routes	= ARRAY_SIZE(ak4375_intercon),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_config ak4375_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= AK4375_24_MODE_CONTROL,
	//.reg_defaults		= ak4375_reg_defaults,
	//.num_reg_defaults	= ARRAY_SIZE(ak4375_reg_defaults),
	//.cache_type		= REGCACHE_NONE, //REGCACHE_RBTREE,
};

static const struct ak4375_drvdata ak4375_drvdata = {
	.dai_drv = &ak4375_dai,
	.comp_drv = &soc_codec_dev_ak4375,
};

static const struct dev_pm_ops ak4375_pm = {
	SET_RUNTIME_PM_OPS(ak4375_runtime_suspend, ak4375_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static int ak4375_i2c_probe(struct i2c_client *i2c)
{
	struct ak4375_priv *ak4375;
	const struct ak4375_drvdata *drvdata;
	unsigned int deviceid;
	int ret;

	dev_info(&i2c->dev, "i2c_probe\n");
	ak4375 = devm_kzalloc(&i2c->dev, sizeof(*ak4375), GFP_KERNEL);
	if (!ak4375)
		return -ENOMEM;

	ak4375->regmap = devm_regmap_init_i2c(i2c, &ak4375_regmap);
	if (IS_ERR(ak4375->regmap))
		return PTR_ERR(ak4375->regmap);

	i2c_set_clientdata(i2c, ak4375);
	ak4375->dev = &i2c->dev;

	drvdata = of_device_get_match_data(&i2c->dev);

	ak4375->pdn_gpiod = devm_gpiod_get_optional(ak4375->dev, "pdn", GPIOD_OUT_LOW);
	if (IS_ERR(ak4375->pdn_gpiod))
		return PTR_ERR(ak4375->pdn_gpiod);

	ak4375_power_on(ak4375);

	ret = regmap_read(ak4375->regmap, AK4375_15_AUDIO_IF_FORMAT, &deviceid);
	if (ret < 0) {
		dev_err(ak4375->dev, "unable to read DEVICEID!\n");
		return ret;
	}

	deviceid = (deviceid & AK4375_15_DEVICEID_MASK) >> 5;

	switch (deviceid) {
	case DEVICEID_AK4331:
		dev_info(ak4375->dev, "found AK4331\n");
		break;
	case DEVICEID_AK4375:
		dev_info(ak4375->dev, "found AK4375\n");
		break;
	case DEVICEID_AK4375A:
		dev_info(ak4375->dev, "found AK4375A\n");
		break;
	case DEVICEID_AK4376A:
		dev_err(ak4375->dev, "found unsupported AK4376/A!\n");
		return -EINVAL;
		break;
	case DEVICEID_AK4377:
		dev_err(ak4375->dev, "found unsupported AK4377!\n");
		return -EINVAL;
		break;
	default:
		dev_err(ak4375->dev, "unrecognized DEVICEID!\n");
		return -EINVAL;
	}

	ak4375->mclk_gpiod = devm_gpiod_get_optional(ak4375->dev, "mclk", GPIOD_OUT_LOW);
	if (IS_ERR(ak4375->mclk_gpiod))
		return PTR_ERR(ak4375->mclk_gpiod);

	ret = devm_snd_soc_register_component(ak4375->dev, drvdata->comp_drv,
					      drvdata->dai_drv, 1);
	if (ret < 0) {
		dev_err(ak4375->dev, "Failed to register CODEC: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(&i2c->dev);

	return 0;
}

static int ak4375_i2c_remove(struct i2c_client *i2c)
{
	dev_info(&i2c->dev, "i2c_remove\n");
	pm_runtime_disable(&i2c->dev);

	return 0;
}

static const struct of_device_id ak4375_of_match[] = {
	/* { .compatible = "asahi-kasei,ak4331", .data = &ak4375_drvdata }, */
	{ .compatible = "asahi-kasei,ak4375", .data = &ak4375_drvdata },
	{ },
};
MODULE_DEVICE_TABLE(of, ak4375_of_match);

static struct i2c_driver ak4375_i2c_driver = {
	.driver = {
		.name = "ak4375",
		.pm = &ak4375_pm,
		.of_match_table = ak4375_of_match,
	},
	.probe_new = ak4375_i2c_probe,
	.remove = ak4375_i2c_remove,
};
module_i2c_driver(ak4375_i2c_driver);

MODULE_AUTHOR("Vincent Knecht <vincent.knecht@mailoo.org>");
MODULE_DESCRIPTION("ASoC AK4375 DAC driver");
MODULE_LICENSE("GPL v2");
