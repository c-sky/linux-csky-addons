/*
 * C-SKY SoCs PIO PCM for I2S driver
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * The PIO PCM is specially suited for I2S devices that don't have DMA support.
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

#include <linux/io.h>
#include <linux/rcupdate.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "csky-i2s.h"

#define BUFFER_BYTES_MAX	(256 * 1024)
#define PERIOD_BYTES_MIN	4096
#define PERIODS_MIN		4

static unsigned int
csky_pcm_pio_tx_16_mono(struct csky_i2s *i2s,
			struct snd_pcm_runtime *runtime,
			unsigned int tx_ptr,
			bool *period_elapsed)
{
	const u16 *p = (void *)runtime->dma_area;
	unsigned int period_pos = tx_ptr % runtime->period_size;
	int cnt = i2s->fifo_depth - i2s->intr_tx_threshold;
	int i;

	for (i = 0; i < cnt; i++) {
		iowrite16(*(p + tx_ptr), i2s->regs + IIS_DR);
		period_pos++;
		if (++tx_ptr >= runtime->buffer_size)
			tx_ptr = 0;
	}

	*period_elapsed = period_pos >= runtime->period_size;
	return tx_ptr;
}

static unsigned int
csky_pcm_pio_tx_16_stereo(struct csky_i2s *i2s,
			  struct snd_pcm_runtime *runtime,
			  unsigned int tx_ptr,
			  bool *period_elapsed)
{
	const u32 *p = (void *)runtime->dma_area;
	unsigned int period_pos = tx_ptr % runtime->period_size;
	int cnt = i2s->fifo_depth - i2s->intr_tx_threshold;
	int i;

	for (i = 0; i < cnt; i++) {
		iowrite32(*(p + tx_ptr), i2s->regs + IIS_DR);
		period_pos++;
		if (++tx_ptr >= runtime->buffer_size)
			tx_ptr = 0;
	}

	*period_elapsed = period_pos >= runtime->period_size;
	return tx_ptr;
}

static unsigned int
csky_pcm_pio_tx_32(struct csky_i2s *i2s,
		   struct snd_pcm_runtime *runtime,
		   unsigned int tx_ptr,
		   bool *period_elapsed)
{
	const u32 *p = (void *)runtime->dma_area;
	u32 offset;
	unsigned int period_pos = tx_ptr % runtime->period_size;
	int cnt = i2s->fifo_depth - i2s->intr_tx_threshold;
	int i = 0;

	while (i < cnt) {
		offset = tx_ptr * runtime->channels;

		iowrite32(*(p + offset), i2s->regs + IIS_DR);
		i++;
		if (runtime->channels == 2) {
			iowrite32(*(p + offset + 1), i2s->regs + IIS_DR);
			i++;
		}

		period_pos++;
		if (++tx_ptr >= runtime->buffer_size)
			tx_ptr = 0;
	}

	*period_elapsed = period_pos >= runtime->period_size;
	return tx_ptr;
}

static const struct snd_pcm_hardware csky_pcm_pio_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = BUFFER_BYTES_MAX,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = BUFFER_BYTES_MAX / PERIODS_MIN,
	.periods_min = PERIODS_MIN,
	.periods_max = BUFFER_BYTES_MAX / PERIOD_BYTES_MIN,
};

void csky_pcm_pio_push_tx(struct csky_i2s *i2s)
{
	struct snd_pcm_substream *tx_substream;
	bool tx_active;
	bool period_elapsed;
	unsigned int tx_ptr;
	unsigned int new_tx_ptr;

#ifdef RCU_USED
	rcu_read_lock();
	tx_substream = rcu_dereference(i2s->tx_substream);
#else
	tx_substream = i2s->tx_substream;
#endif

	tx_active = tx_substream && snd_pcm_running(tx_substream);
	if (tx_active) {
		tx_ptr = READ_ONCE(i2s->tx_ptr);
		new_tx_ptr = i2s->tx_fn(i2s, tx_substream->runtime,
					tx_ptr, &period_elapsed);
	#ifdef CONFIG_HAVE_CMPXCHG_LOCAL
		cmpxchg(&i2s->tx_ptr, tx_ptr, new_tx_ptr);
	#else
		if (i2s->tx_ptr == tx_ptr)
			i2s->tx_ptr = new_tx_ptr;
	#endif
		if (period_elapsed)
			snd_pcm_period_elapsed(tx_substream);
	}

#ifdef RCU_USED
	rcu_read_unlock();
#endif
}
EXPORT_SYMBOL_GPL(csky_pcm_pio_push_tx);

static int csky_pcm_pio_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct csky_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	snd_soc_set_runtime_hwparams(substream, &csky_pcm_pio_hardware);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	runtime->private_data = i2s;
	return 0;
}

static int csky_pcm_pio_close(struct snd_pcm_substream *substream)
{
#ifdef RCU_USED
	synchronize_rcu();
#endif
	return 0;
}

static int csky_pcm_pio_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct csky_i2s *i2s = runtime->private_data;
	int ret;

	if (params_channels(hw_params) > 2)
		return -EINVAL;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		dev_err(i2s->dev, "only playback is available\n");
		return -EINVAL;
	}

	switch (params_physical_width(hw_params)) {
	case 16:
		if (params_channels(hw_params) == 1)
			i2s->tx_fn = csky_pcm_pio_tx_16_mono;
		else
			i2s->tx_fn = csky_pcm_pio_tx_16_stereo;
		break;
	case 24:
		dev_err(i2s->dev, "storage size 24 not supported\n");
		return -EINVAL;
	case 32:
		i2s->tx_fn = csky_pcm_pio_tx_32;
		break;
	default:
		dev_err(i2s->dev, "invalid format\n");
		return -EINVAL;
	}

	ret = snd_pcm_lib_malloc_pages(substream,
				       params_buffer_bytes(hw_params));
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int csky_pcm_pio_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int csky_pcm_pio_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct csky_i2s *i2s = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		WRITE_ONCE(i2s->tx_ptr, 0);
#ifdef RCU_USED
		rcu_assign_pointer(i2s->tx_substream, substream);
#else
		i2s->tx_substream = substream;
#endif
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#ifdef RCU_USED
		rcu_assign_pointer(i2s->tx_substream, NULL);
#else
		i2s->tx_substream = NULL;
#endif
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t
csky_pcm_pio_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct csky_i2s *i2s = runtime->private_data;
	snd_pcm_uframes_t pos = READ_ONCE(i2s->tx_ptr);

	return pos < runtime->buffer_size ? pos : 0;
}

static int csky_pcm_pio_new(struct snd_soc_pcm_runtime *rtd)
{
	size_t size = csky_pcm_pio_hardware.buffer_bytes_max;

	return snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL), size, size);
}

static void csky_pcm_pio_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static const struct snd_pcm_ops csky_pcm_pio_ops = {
	.open = csky_pcm_pio_open,
	.close = csky_pcm_pio_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = csky_pcm_pio_hw_params,
	.hw_free = csky_pcm_pio_hw_free,
	.trigger = csky_pcm_pio_trigger,
	.pointer = csky_pcm_pio_pointer,
};

static const struct snd_soc_platform_driver csky_pcm_pio_platform = {
	.pcm_new = csky_pcm_pio_new,
	.pcm_free = csky_pcm_pio_free,
	.ops = &csky_pcm_pio_ops,
};

int csky_pcm_pio_register(struct platform_device *pdev)
{
	return devm_snd_soc_register_platform(&pdev->dev,
					      &csky_pcm_pio_platform);
}
EXPORT_SYMBOL_GPL(csky_pcm_pio_register);
