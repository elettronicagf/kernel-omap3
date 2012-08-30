/*
 * omap3egf.c  --  SoC audio for Pandora Handheld Console
 *
 * Author: Gražvydas Ignotas <notasas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"


#define PREFIX "ASoC omap3egf: "

static struct regulator *omap3egf_dac_reg;

static int omap3egf_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		  SND_SOC_DAIFMT_CBS_CFS;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err(PREFIX "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err(PREFIX "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(PREFIX "can't set codec system clock\n");
		return ret;
	}

	/* Set McBSP clock to external */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT,
				     256 * params_rate(params),
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(PREFIX "can't set cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 8);
	if (ret < 0) {
		pr_err(PREFIX "can't set SRG clock divider\n");
		return ret;
	}

	return 0;
}


static struct snd_soc_ops omap3egf_ops = {
	.hw_params = omap3egf_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3egf_dai[] = {
	{
		.name = "TWL4030",
		.stream_name = "Line/Mic In",
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.ops = &omap3egf_ops,
	},
	{
		.name = "CS8406",
		.stream_name = "HiFi Out",
		.cpu_dai_name = "omap-mcbsp-dai.0",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.ops = &omap3egf_ops,
	}
};

/* SoC card */
static struct snd_soc_card snd_soc_card_omap3egf = {
	.name = "omap3egf",
	.dai_link = omap3egf_dai,
	.num_links = ARRAY_SIZE(omap3egf_dai),
};

static struct platform_device *omap3egf_snd_device;

static int __init omap3egf_soc_init(void)
{
	int ret;

	if (!machine_is_omap3_egf())
		return -ENODEV;

	pr_info("OMAP3 egf SoC init\n");


	omap3egf_snd_device = platform_device_alloc("soc-audio", -1);
	if (omap3egf_snd_device == NULL) {
		pr_err(PREFIX "Platform device allocation failed\n");
		ret = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(omap3egf_snd_device, &snd_soc_card_omap3egf);

	ret = platform_device_add(omap3egf_snd_device);
	if (ret) {
		pr_err(PREFIX "Unable to add platform device\n");
		goto fail2;
	}


	return 0;

fail2:
	platform_device_put(omap3egf_snd_device);
fail1:
	return ret;
}
module_init(omap3egf_soc_init);

static void __exit omap3egf_soc_exit(void)
{
	regulator_put(omap3egf_dac_reg);
	platform_device_unregister(omap3egf_snd_device);
}
module_exit(omap3egf_soc_exit);

MODULE_AUTHOR("Grazvydas Ignotas <notasas@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 Pandora");
MODULE_LICENSE("GPL");
