#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <dt-bindings/sound/qcom,q6voice.h>
#include "q6cvp.h"
#include "q6cvs.h"
#include "q6mvm.h"

#define DRV_NAME	"q6voice-dai"

static struct snd_pcm_hardware q6voice_dai_hardware = {
	.info =			(SNDRV_PCM_INFO_INTERLEAVED /*|
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME*/),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
	.rate_min =		8000,
	.rate_max =		16000,
	.channels_min =		1,
	.channels_max =		1,

	.buffer_bytes_max =	4096 * 2,
	.period_bytes_min =	2048,
	.period_bytes_max =	4096,
	.periods_min =		2,
	.periods_max =		4,

	.fifo_size =		0,
};

static struct snd_soc_dai_driver q6voice_dais[] = {
	{
		.id = CS_VOICE,
		.name = "CS-VOICE",
		.playback = {
			.stream_name = "CS-VOICE Playback",
			//.aif_name = "CS-VOICE_DL1",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min =	1,
			.channels_max =	2,
			.rate_min =	8000,
			.rate_max =	48000,
		},
		.capture = {
			.stream_name = "CS-VOICE Capture",
			//.aif_name = "CS-VOICE_UL1",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min =	1,
			.channels_max =	2,
			.rate_min =	8000,
			.rate_max =	48000,
		},
	},
};

#define AFE_PORT_ID_PRIMARY_MI2S_RX         0x1000
#define AFE_PORT_ID_PRIMARY_MI2S_TX         0x1001
#define AFE_PORT_ID_SECONDARY_MI2S_RX       0x1002
#define AFE_PORT_ID_SECONDARY_MI2S_TX       0x1003
#define AFE_PORT_ID_TERTIARY_MI2S_RX        0x1004
#define AFE_PORT_ID_TERTIARY_MI2S_TX        0x1005
#define AFE_PORT_ID_QUATERNARY_MI2S_RX      0x1006
#define AFE_PORT_ID_QUATERNARY_MI2S_TX      0x1007

static void voc_start(struct device *dev)
{
	struct q6mvm *mvm;
	struct q6cvs *cvs;
	struct q6cvp *cvp;
	int ret;

	dev_err(dev, "Hello from voc_start()\n");

	mvm = q6mvm_create_session();
	if (IS_ERR(mvm)) {
		dev_err(dev, "Failed to create mvm session: %ld\n", PTR_ERR(mvm));
		return;
	}

	cvs = q6cvs_create_session();
	if (IS_ERR(cvs)) {
		dev_err(dev, "Failed to create cvs session: %ld\n", PTR_ERR(cvs));
		return;
	}

	ret = q6mvm_set_dual_control(mvm);
	if (ret) {
		dev_err(dev, "Failed to set dual control: %d\n", ret);
		return;
	}

	cvp = q6cvp_create_session(AFE_PORT_ID_TERTIARY_MI2S_TX, AFE_PORT_ID_PRIMARY_MI2S_RX);
	if (IS_ERR(cvp)) {
		dev_err(dev, "Failed to create cvp session: %ld\n", PTR_ERR(cvp));
		return;
	}

	ret = q6cvp_enable(cvp);
	if (ret) {
		dev_err(dev, "Failed to enable cvp: %d\n", ret);
		return;
	}

	ret = q6mvm_attach(mvm, cvp);
	if (ret) {
		dev_err(dev, "Failed to attach cvp to mvm: %d\n", ret);
		return;
	}

	ret = q6mvm_start(mvm);
	if (ret) {
		dev_err(dev, "Failed to start voice: %d\n", ret);
		return;
	}
}

#define PLAYBACK_AND_CAPTURE	(BIT(SNDRV_PCM_STREAM_PLAYBACK) | BIT(SNDRV_PCM_STREAM_CAPTURE))

static unsigned int stream_active = 0;

static int q6voice_dai_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = q6voice_dai_hardware;

	dev_err(component->dev, "open: %s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");
	return 0;
}

static int q6voice_dai_prepare(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	dev_err(component->dev, "prepare: %s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");

	stream_active |= BIT(substream->stream);

	if (stream_active == PLAYBACK_AND_CAPTURE) {
		dev_err(component->dev, "both active, starting\n");
		voc_start(component->dev);
	}

	return 0;
}

static int q6voice_dai_close(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	dev_err(component->dev, "close: %s\n",
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");

	if (stream_active == PLAYBACK_AND_CAPTURE) {
		dev_err(component->dev, "both active, stopping\n");
	}

	stream_active &= ~BIT(substream->stream);

	return 0;
}

/*
// FIXME: Stop hard-coding this
static const struct snd_soc_dapm_widget q6voice_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("CS-VOICE_DL1", "CS-VOICE Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("CS-VOICE_UL1", "CS-VOICE Capture", 0, 0, 0, 0),
};

static const struct snd_soc_dapm_route q6voice_dapm_routes[] = {
	{"CS-VOICE_UL1", NULL, "TERT_MI2S_TX"},
	{"PRI_MI2S_RX", NULL, "CS-VOICE_DL1"},

	{"CS-VOICE_DL1", NULL, "CS-VOICE Playback" },
	{"CS-VOICE Capture", NULL, "CS-VOICE_UL1"},
};
*/

static const struct snd_soc_component_driver q6voice_dai_component = {
	.name		= DRV_NAME,
	.open		= q6voice_dai_open,
	//.hw_params	= q6voice_dai_hw_params,
	.close		= q6voice_dai_close,
	.prepare	= q6voice_dai_prepare,
	//.trigger	= q6voice_dai_trigger,
	//.pcm_construct	= q6voice_dai_pcm_new,

/*
	.dapm_widgets	= q6voice_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(q6voice_dapm_widgets),
	.dapm_routes	= q6voice_dapm_routes,
	.num_dapm_routes  = ARRAY_SIZE(q6voice_dapm_routes),
*/
};

static int q6voice_dai_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_err(dev, "Hello World!\n");

	return devm_snd_soc_register_component(dev, &q6voice_dai_component,
					       q6voice_dais,
					       ARRAY_SIZE(q6voice_dais));
}

static const struct of_device_id q6voice_dai_device_id[] = {
	{ .compatible = "qcom,q6voice-dais" },
	{},
};
MODULE_DEVICE_TABLE(of, q6voice_dai_device_id);

static struct platform_driver q6voice_dai_platform_driver = {
	.driver = {
		.name = "q6voice-dai",
		.of_match_table = of_match_ptr(q6voice_dai_device_id),
	},
	.probe = q6voice_dai_probe,
};
module_platform_driver(q6voice_dai_platform_driver);

MODULE_DESCRIPTION("Q6VOICE dai driver");
MODULE_LICENSE("GPL v2");
