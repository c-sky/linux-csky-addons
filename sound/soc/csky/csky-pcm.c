/*
 * C-SKY SoCs I2S PCM driver
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * based on soc-generic-dmaengine-pcm.c
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
#include <linux/init.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <sound/dmaengine_pcm.h>
#include <linux/dma/dw.h>

struct dmaengine_pcm_runtime_data {
	struct dma_chan *dma_chan;
	dma_cookie_t cookie;
	unsigned int pos;
};

static inline struct dmaengine_pcm_runtime_data *
substream_to_prtd(const struct snd_pcm_substream *substream)
{
	return substream->runtime->private_data;
}

static void dmaengine_pcm_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);

	prtd->pos += snd_pcm_lib_period_bytes(substream);
	if (prtd->pos >= snd_pcm_lib_buffer_bytes(substream))
		prtd->pos = 0;

	snd_pcm_period_elapsed(substream);
}

static int
csky_dmaengine_pcm_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct dma_chan *chan = prtd->dma_chan;
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK;
	struct dw_cyclic_desc *cdesc;

	direction = snd_pcm_substream_to_dma_direction(substream);

	if (!substream->runtime->no_period_wakeup)
		flags |= DMA_PREP_INTERRUPT;

	prtd->pos = 0;

	cdesc = dw_dma_cyclic_prep(chan,
				   substream->runtime->dma_addr,
				   snd_pcm_lib_buffer_bytes(substream),
				   snd_pcm_lib_period_bytes(substream),
				   direction);
	if (IS_ERR(cdesc)) {
		return PTR_ERR(cdesc);
	}

	cdesc->period_callback = dmaengine_pcm_dma_complete;
	cdesc->period_callback_param = substream;
	return 0;
}

static int
csky_snd_dmaengine_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = csky_dmaengine_pcm_prepare_and_submit(substream);
		if (ret)
			return ret;

		dw_dma_cyclic_start(prtd->dma_chan);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dw_dma_cyclic_stop(prtd->dma_chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int csky_snd_pcm_lib_free_pages(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;

	if (PCM_RUNTIME_CHECK(substream))
		return -EINVAL;

	runtime = substream->runtime;
	if (runtime->dma_area == NULL)
		return 0;

	dw_dma_cyclic_free(snd_dmaengine_pcm_get_chan(substream));

	if (runtime->dma_buffer_p != &substream->dma_buffer) {
		/* it's a newly allocated buffer.  release it now. */
		snd_dma_free_pages(runtime->dma_buffer_p);
		kfree(runtime->dma_buffer_p);
	}
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

/*
 * The platforms dmaengine driver does not support reporting the amount of
 * bytes that are still left to transfer.
 */
#define SND_DMAENGINE_PCM_FLAG_NO_RESIDUE BIT(31)

struct dmaengine_pcm {
	struct dma_chan *chan[SNDRV_PCM_STREAM_LAST + 1];
	const struct snd_dmaengine_pcm_config *config;
	struct snd_soc_platform platform;
	unsigned int flags;
};

static struct dmaengine_pcm *soc_platform_to_pcm(struct snd_soc_platform *p)
{
	return container_of(p, struct dmaengine_pcm, platform);
}

static struct device *dmaengine_dma_dev(struct dmaengine_pcm *pcm,
	struct snd_pcm_substream *substream)
{
	if (!pcm->chan[substream->stream])
		return NULL;

	return pcm->chan[substream->stream]->device->dev;
}

int csky_snd_dmaengine_pcm_prepare_slave_config(
		struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dma_data;
	int ret;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params, slave_config);
	if (ret)
		return ret;

	snd_dmaengine_pcm_set_config_from_dai_data(substream,
						   dma_data, slave_config);
	return 0;
}
EXPORT_SYMBOL_GPL(csky_snd_dmaengine_pcm_prepare_slave_config);

static int dmaengine_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dmaengine_pcm *pcm = soc_platform_to_pcm(rtd->platform);
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	int (*prepare_slave_config)(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct dma_slave_config *slave_config);
	struct dma_slave_config slave_config;
	int ret;

	memset(&slave_config, 0, sizeof(slave_config));

	if (!pcm->config)
		prepare_slave_config =
			csky_snd_dmaengine_pcm_prepare_slave_config;
	else
		prepare_slave_config = pcm->config->prepare_slave_config;

	if (prepare_slave_config) {
		ret = prepare_slave_config(substream, params, &slave_config);
		if (ret)
			return ret;

		ret = dmaengine_slave_config(chan, &slave_config);
		if (ret)
			return ret;
	}

	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

static int
dmaengine_pcm_set_runtime_hwparams(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dmaengine_pcm *pcm = soc_platform_to_pcm(rtd->platform);
	struct device *dma_dev = dmaengine_dma_dev(pcm, substream);
	struct dma_chan *chan = pcm->chan[substream->stream];
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct dma_slave_caps dma_caps;
	struct snd_pcm_hardware hw;
	u32 addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
			  BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
			  BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	int i, ret;

	if (pcm->config && pcm->config->pcm_hardware)
		return snd_soc_set_runtime_hwparams(substream,
				pcm->config->pcm_hardware);

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	memset(&hw, 0, sizeof(hw));
	hw.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		  SNDRV_PCM_INFO_INTERLEAVED;
	hw.periods_min = 2;
	hw.periods_max = UINT_MAX;
	hw.period_bytes_min = 256;
	hw.period_bytes_max = dma_get_max_seg_size(dma_dev);
	hw.buffer_bytes_max = SIZE_MAX;
	hw.fifo_size = dma_data->fifo_size;

	if (pcm->flags & SND_DMAENGINE_PCM_FLAG_NO_RESIDUE)
		hw.info |= SNDRV_PCM_INFO_BATCH;

	ret = dma_get_slave_caps(chan, &dma_caps);
	if (ret == 0) {
		if (dma_caps.cmd_pause)
			hw.info |= SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME;
		if (dma_caps.residue_granularity <=
					DMA_RESIDUE_GRANULARITY_SEGMENT)
			hw.info |= SNDRV_PCM_INFO_BATCH;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			addr_widths = dma_caps.dst_addr_widths;
		else
			addr_widths = dma_caps.src_addr_widths;
	}

	/*
	 * If SND_DMAENGINE_PCM_DAI_FLAG_PACK is set keep
	 * hw.formats set to 0, meaning no restrictions are in place.
	 * In this case it's the responsibility of the DAI driver to
	 * provide the supported format information.
	 */
	if (!(dma_data->flags & SND_DMAENGINE_PCM_DAI_FLAG_PACK))
		/*
		 * Prepare formats mask for valid/allowed sample types. If the
		 * dma does not have support for the given physical word size,
		 * it needs to be masked out so user space can not use the
		 * format which produces corrupted audio.
		 * In case the dma driver does not implement the slave_caps the
		 * default assumption is that it supports 1, 2 and 4 bytes
		 * widths.
		 */
		for (i = 0; i <= SNDRV_PCM_FORMAT_LAST; i++) {
			int bits = snd_pcm_format_physical_width(i);

			/*
			 * Enable only samples with DMA supported physical
			 * widths
			 */
			switch (bits) {
			case 8:
			case 16:
			case 24:
			case 32:
			case 64:
				if (addr_widths & (1 << (bits / 8)))
					hw.formats |= (1LL << i);
				break;
			default:
				/* Unsupported types */
				break;
			}
		}

	return snd_soc_set_runtime_hwparams(substream, &hw);
}

static int dmaengine_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dmaengine_pcm *pcm = soc_platform_to_pcm(rtd->platform);
	struct dma_chan *chan = pcm->chan[substream->stream];
	int ret;

	ret = dmaengine_pcm_set_runtime_hwparams(substream);
	if (ret)
		return ret;

	return snd_dmaengine_pcm_open(substream, chan);
}

static struct dma_chan *dmaengine_pcm_compat_request_channel(
	struct snd_soc_pcm_runtime *rtd,
	struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm *pcm = soc_platform_to_pcm(rtd->platform);
	struct snd_dmaengine_dai_dma_data *dma_data;
	dma_filter_fn fn = NULL;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	if ((pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX) && pcm->chan[0])
		return pcm->chan[0];

	if (pcm->config && pcm->config->compat_request_channel)
		return pcm->config->compat_request_channel(rtd, substream);

	if (pcm->config)
		fn = pcm->config->compat_filter_fn;

	return snd_dmaengine_pcm_request_channel(fn, dma_data->filter_data);
}

static bool dmaengine_pcm_can_report_residue(struct device *dev,
	struct dma_chan *chan)
{
	struct dma_slave_caps dma_caps;
	int ret;

	ret = dma_get_slave_caps(chan, &dma_caps);
	if (ret != 0) {
		dev_warn(dev, "Failed to get DMA channel capabilities\n");
		return false;
	}

	if (dma_caps.residue_granularity == DMA_RESIDUE_GRANULARITY_DESCRIPTOR)
		return false;

	return true;
}

static int dmaengine_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct dmaengine_pcm *pcm = soc_platform_to_pcm(rtd->platform);
	const struct snd_dmaengine_pcm_config *config = pcm->config;
	struct device *dev = rtd->platform->dev;
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct snd_pcm_substream *substream;
	size_t prealloc_buffer_size;
	size_t max_buffer_size;
	unsigned int i;
	int ret;

	if (config && config->prealloc_buffer_size) {
		prealloc_buffer_size = config->prealloc_buffer_size;
		max_buffer_size = config->pcm_hardware->buffer_bytes_max;
	} else {
		prealloc_buffer_size = 512 * 1024;
		max_buffer_size = SIZE_MAX;
	}


	for (i = SNDRV_PCM_STREAM_PLAYBACK;
	     i <= SNDRV_PCM_STREAM_CAPTURE; i++) {
		substream = rtd->pcm->streams[i].substream;
		if (!substream)
			continue;

		dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

		if (!pcm->chan[i] &&
		    (pcm->flags & SND_DMAENGINE_PCM_FLAG_CUSTOM_CHANNEL_NAME))
			pcm->chan[i] = dma_request_slave_channel(dev,
				dma_data->chan_name);

		if (!pcm->chan[i] &&
		    (pcm->flags & SND_DMAENGINE_PCM_FLAG_COMPAT)) {
			pcm->chan[i] = dmaengine_pcm_compat_request_channel(rtd,
				substream);
		}

		if (!pcm->chan[i]) {
			dev_err(rtd->platform->dev,
				"Missing dma channel for stream: %d\n", i);
			return -EINVAL;
		}

		ret = snd_pcm_lib_preallocate_pages(substream,
				SNDRV_DMA_TYPE_DEV_IRAM,
				dmaengine_dma_dev(pcm, substream),
				prealloc_buffer_size,
				max_buffer_size);
		if (ret)
			return ret;

		if (!dmaengine_pcm_can_report_residue(dev, pcm->chan[i]))
			pcm->flags |= SND_DMAENGINE_PCM_FLAG_NO_RESIDUE;
	}

	return 0;
}

static snd_pcm_uframes_t dmaengine_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime	*runtime = substream->runtime;
	snd_pcm_uframes_t	frames;
	unsigned long		bytes;

	bytes = dw_dma_get_src_addr(snd_dmaengine_pcm_get_chan(substream));
	bytes -= runtime->dma_addr;

	frames = bytes_to_frames(runtime, bytes);
	if (frames >= runtime->buffer_size)
		frames -= runtime->buffer_size;

	return frames;
}

static const struct snd_pcm_ops dmaengine_pcm_ops = {
	.open		= dmaengine_pcm_open,
	.close		= snd_dmaengine_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= dmaengine_pcm_hw_params,
	.hw_free	= csky_snd_pcm_lib_free_pages,
	.trigger	= csky_snd_dmaengine_pcm_trigger,
	.pointer	= dmaengine_pcm_pointer,
};

static const struct snd_soc_platform_driver dmaengine_pcm_platform = {
	.component_driver = {
		.probe_order = SND_SOC_COMP_ORDER_LATE,
	},
	.ops		= &dmaengine_pcm_ops,
	.pcm_new	= dmaengine_pcm_new,
};

static const char * const dmaengine_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

static int dmaengine_pcm_request_chan_of(struct dmaengine_pcm *pcm,
	struct device *dev, const struct snd_dmaengine_pcm_config *config)
{
	unsigned int i;
	const char *name;
	struct dma_chan *chan;

	if ((pcm->flags & (SND_DMAENGINE_PCM_FLAG_NO_DT |
			   SND_DMAENGINE_PCM_FLAG_CUSTOM_CHANNEL_NAME)) ||
	    !dev->of_node)
		return 0;

	if (config && config->dma_dev) {
		/*
		 * If this warning is seen, it probably means that your Linux
		 * device structure does not match your HW device structure.
		 * It would be best to refactor the Linux device structure to
		 * correctly match the HW structure.
		 */
		dev_warn(dev, "DMA channels sourced from device %s",
			 dev_name(config->dma_dev));
		dev = config->dma_dev;
	}

	for (i = SNDRV_PCM_STREAM_PLAYBACK; i <= SNDRV_PCM_STREAM_CAPTURE;
	     i++) {
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			name = "rx-tx";
		else
			name = dmaengine_pcm_dma_channel_names[i];
		if (config && config->chan_names[i])
			name = config->chan_names[i];
		chan = dma_request_slave_channel_reason(dev, name);
		if (IS_ERR(chan)) {
			if (PTR_ERR(chan) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			pcm->chan[i] = NULL;
		} else {
			pcm->chan[i] = chan;
		}
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			break;
	}

	if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
		pcm->chan[1] = pcm->chan[0];

	return 0;
}

static void dmaengine_pcm_release_chan(struct dmaengine_pcm *pcm)
{
	unsigned int i;

	for (i = SNDRV_PCM_STREAM_PLAYBACK; i <= SNDRV_PCM_STREAM_CAPTURE;
	     i++) {
		if (!pcm->chan[i])
			continue;
		dma_release_channel(pcm->chan[i]);
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			break;
	}
}

/**
 * csky_snd_dmaengine_pcm_register - Register a dmaengine based PCM device
 * @dev: The parent device for the PCM device
 * @config: Platform specific PCM configuration
 * @flags: Platform specific quirks
 */
int csky_snd_dmaengine_pcm_register(struct device *dev,
		const struct snd_dmaengine_pcm_config *config,
		unsigned int flags)
{
	struct dmaengine_pcm *pcm;
	int ret;

	pcm = kzalloc(sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->config = config;
	pcm->flags = flags;

	ret = dmaengine_pcm_request_chan_of(pcm, dev, config);
	if (ret)
		goto err_free_dma;

	ret = snd_soc_add_platform(dev, &pcm->platform,
		&dmaengine_pcm_platform);
	if (ret)
		goto err_free_dma;

	return 0;

err_free_dma:
	dmaengine_pcm_release_chan(pcm);
	kfree(pcm);
	return ret;
}
EXPORT_SYMBOL_GPL(csky_snd_dmaengine_pcm_register);

/**
 * csky_snd_dmaengine_pcm_unregister - Removes a dmaengine based PCM device
 * @dev: Parent device the PCM was register with
 *
 * Removes a dmaengine based PCM device previously registered with
 * csky_snd_dmaengine_pcm_register.
 */
void csky_snd_dmaengine_pcm_unregister(struct device *dev)
{
	struct snd_soc_platform *platform;
	struct dmaengine_pcm *pcm;

	platform = snd_soc_lookup_platform(dev);
	if (!platform)
		return;

	pcm = soc_platform_to_pcm(platform);

	snd_soc_remove_platform(platform);
	dmaengine_pcm_release_chan(pcm);
	kfree(pcm);
}
EXPORT_SYMBOL_GPL(csky_snd_dmaengine_pcm_unregister);

MODULE_DESCRIPTION("C-SKY SoCs I2S PCM Driver");
MODULE_AUTHOR("Lei Ling <lei_ling@c-sky.com>");
MODULE_LICENSE("GPL v2");
