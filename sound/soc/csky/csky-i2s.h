/*
 * C-SKY SoCs I2S Controller driver
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
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

#ifndef __CSKY_I2S_H__
#define __CSKY_I2S_H__

#include <sound/dmaengine_pcm.h>

/* Hardware register definitions */
#define IIS_AUDIOEN	0x00	/* Enable/Disable IIS/SPDIF */
#define IIS_FUNCMODE	0x04	/* Function Mode(receiver/transmitter) */
#define IIS_IISCNF_IN	0x08	/* Input: IIS Configuration */
#define IIS_FSSTA	0x0C	/* Input: Config Sample Frequency / Work Mode */
#define IIS_IISCNF_OUT	0x10	/* Output: IIS Configuration */
#define IIS_FADTLR	0x14	/* FS Auto Detected Threshold Level Register */
#define IIS_SCCR	0x18	/* Sample Compress Control Register */
#define IIS_TXFTLR	0x1C	/* Transmit FIFO Threshold Level */
#define IIS_RXFTLR	0x20	/* Receive FIFO Threshold Level */
#define IIS_TXFLR	0x24	/* Transmit FIFO Level */
#define IIS_RXFLR	0x28	/* Receive FIFO Level */
#define IIS_SR		0x2C	/* Status Register */
#define IIS_IMR		0x30	/* FIFO Interrupt Mask */
#define IIS_ISR		0x34	/* FIFO Interrupt Status */
#define IIS_RISR	0x38	/* Raw FIFO Interrupt Status */
#define IIS_FICR	0x3C	/* FIFO Interrupt Clear */
#define IIS_DMACR	0x4C	/* DMA Control(enable/disable rx/tx dma) */
#define IIS_DMATDLR	0x50	/* DMA Transmit Data Level */
#define IIS_DMARDLR	0x54	/* DMA Receive Data Level */
#define IIS_DR		0x60	/* Data Register */
#define IIS_SRCR	0x70	/* SPDIF Receiver Configuration */
#define IIS_SRSSR	0x74	/* SPDIF Receiver Signal Status */
#define IIS_STSSR	0x78	/* SPDIF Transmitter Signal Status */
#define IIS_SCSR	0x7C	/* SPDIF Channel status */
#define IIS_MIMR	0x80	/* Mode Interrupt Mask */
#define IIS_MISR	0x84	/* Mode Interrupt Status */
#define IIS_RMISR	0x88	/* Raw Mode Interrupt Status */
#define IIS_CMIR	0x8C	/* Clear Mode Interrupt */
#define IIS_DIV0_LEVEL	0x90	/* clock divider for mclk */
#define IIS_DIV1_LEVEL	0x94	/* clock divider for spdif_clk */
#define IIS_DIV2_LEVEL	0x98	/* clock divider for wsclk */
#define IIS_DIV3_LEVEL	0x9C	/* clock divider for ref_clk */

/* Bitfields in IIS_AUDIOEN */
#define AUDIOEN_IIS_EN		(1 << 0)	/* IIS enable */
#define AUDIOEN_SPDIF_EN	(1 << 1)	/* SPDIF enable */

/* Bitfields in IIS_FUNCMODE */
#define FUNCMODE_MODE_RX	(0 << 0)
#define FUNCMODE_MODE_TX	(1 << 0)
#define FUNCMODE_MODE_WEN	(1 << 1)	/* MODE write enable */

/* Bitfields in IIS_IISCNF_IN */
#define IISCNF_IN_AUDFMT_I2S		(0 << 0)
#define IISCNF_IN_AUDFMT_RIGHT_J	(1 << 0)
#define IISCNF_IN_AUDFMT_LEFT_J		(2 << 0)
#define IISCNF_IN_WS_POLARITY_NORMAL	(0 << 2)
#define IISCNF_IN_WS_POLARITY_INVERTED	(1 << 2)
#define IISCNF_IN_SAMPLE_SOURCE_VOICE	(1 << 4)
#define IISCNF_IN_SLAVE			(0 << 8)
#define IISCNF_IN_MASTER		(1 << 8)

/* Bitfields in IIS_FSSTA */
#define FSSTA_RATE_SET_BY_USER	(0 << 0)	/* input rate is set by user */
#define FSSTA_RATE_AUTO_DETECT	(1 << 0)	/* input rate auto detected */
#define FSSTA_RES16_FIFO16	(0 << 1)
#define FSSTA_RES16_FIFO24	(1 << 1)
#define FSSTA_RES24_FIFO16	(2 << 1)
#define FSSTA_RES24_FIFO24	(3 << 1)
#define FSSTA_RES_MASK		0x3
#define FSSTA_RES_SHIFT		1
#define FSSTA_AFR(x)		((x) << 4)	/* Audio Fundamental Rate */
#define FSSTA_AFR_MASK		0x3
#define FSSTA_ARS(x)		((x) << 6)	/* Audio Rate Scale(I2S only) */
#define FSSTA_ARS_MASK		0x3

/* Bitfields in IIS_IISCNF_OUT */
#define IISCNF_OUT_AUDFMT_I2S		(0 << 0)
#define IISCNF_OUT_AUDFMT_RIGHT_J	(1 << 0)
#define IISCNF_OUT_AUDFMT_LEFT_J	(2 << 0)
#define OUT_AUDFMT_MASK			0x3
#define OUT_AUDFMT_SHIFT		0
#define IISCNF_OUT_WS_POLARITY_NORMAL	(0 << 2)
#define IISCNF_OUT_WS_POLARITY_INVERTED	(1 << 2)
#define OUT_WS_POLARITY_MASK		0x1
#define OUT_WS_POLARITY_SHIFT		2
#define IISCNF_OUT_SAMPLE_SOURCE_VOICE	(1 << 3)
#define IISCNF_OUT_MASTER		(0 << 4)
#define IISCNF_OUT_SLAVE		(1 << 4)
#define OUT_M_S_MASK			0x1
#define OUT_M_S_SHIFT			4

/* Bitfields in IIS_FADTLR */
#define FADTLR_48FTR(x)		((x) << 0)	/* the center count of 48K */
#define FADTLR_48FTR_MASK	0x7F
#define FADTLR_44FTR(x)		((x) << 8)	/* the center count of 44.1K */
#define FADTLR_44FTR_MASK	0x7F
#define FADTLR_32FTR(x)		((x) << 16)	/* the center count of 32K */
#define FADTLR_32FTR_MASK	0x7F
#define FADTLR_96FTR(x)		((x) << 24)	/* the center count of 96K */
#define FADTLR_96FTR_MASK	0x3F

/* Bitfields in IIS_SCCR */
/* TODO */

/* Bitfields in IIS_SR */
#define SR_RX_BUSY		(1 << 0)
#define SR_TX_BUSY		(1 << 1)
#define SR_TX_FIFO_NOT_FULL	(1 << 2)
#define SR_TX_FIFO_EMPTY	(1 << 3)
#define SR_RX_FIFO_NOT_EMPTY	(1 << 4)
#define SR_RX_FIFO_FULL		(1 << 5)

/* Bitfields in IIS_IMR/IIS_ISR/IIS_RISR/IIS_FICR */
#define IIS_FIFOINT_TX_FIFO_EMPTY	(1 << 0)
#define IIS_FIFOINT_TX_FIFO_OVERFLOW	(1 << 1)
#define IIS_FIFOINT_RX_FIFO_UNDERFLOW	(1 << 2)
#define IIS_FIFOINT_RX_FIFO_OVERFLOW	(1 << 3)
#define IIS_FIFOINT_RX_FIFO_FULL	(1 << 4)
#define IIS_FIFOINT_ALL	(IIS_FIFOINT_TX_FIFO_EMPTY | \
			 IIS_FIFOINT_TX_FIFO_OVERFLOW | \
			 IIS_FIFOINT_RX_FIFO_UNDERFLOW | \
			 IIS_FIFOINT_RX_FIFO_OVERFLOW | \
			 IIS_FIFOINT_RX_FIFO_FULL)

/* Bitfields in IIS_DMACR */
#define DMACR_EN_RX_DMA		(1 << 0)	/* receiver dma enable */
#define DMACR_EN_TX_DMA		(1 << 1)	/* transmitter dma enable*/

/* Bitfields in IIS_SRCR */
/* TODO */
/* Bitfields in IIS_SRSSR */
/* TODO */
/* Bitfields in IIS_STSSR */
/* TODO */
/* Bitfields in IIS_SCSR */
/* TODO */

/* Bitfields in IIS_MIMR/IIS_MISR/IIS_RMISR/IIS_CMIR */
#define IIS_MODEINT_I2S_RX_BUSY_CHANGE		(1 << 0)
#define IIS_MODEINT_I2S_TX_BUSY_CHANGE		(1 << 1)
#define IIS_MODEINT_SPDIF_RX_BUSY_CHANGE	(1 << 2)
#define IIS_MODEINT_SPDIF_TX_BUSY_CHANGE	(1 << 3)
#define IIS_MODEINT_INPUT_FS_CHANGE		(1 << 4)
#define IIS_MODEINT_SPDIF_SCSR_DATA_CHANGE	(1 << 5)
#define IIS_MODEINT_ALL	(IIS_MODEINT_I2S_RX_BUSY_CHANGE | \
			 IIS_MODEINT_I2S_TX_BUSY_CHANGE | \
			 IIS_MODEINT_SPDIF_RX_BUSY_CHANGE | \
			 IIS_MODEINT_SPDIF_TX_BUSY_CHANGE | \
			 IIS_MODEINT_INPUT_FS_CHANGE | \
			 IIS_MODEINT_SPDIF_SCSR_DATA_CHANGE)

struct csky_i2s {
	struct device *dev;
	void __iomem *regs;
	int irq;
	unsigned int src_clk;
	struct clk *i2s_clk;
	struct clk *i2s_clk_gate;
	unsigned int clk_fs_44k; /* clock for 11.025k/22.05k/44.1k/88.2k fs */
	unsigned int clk_fs_48k; /* clock for 8k/16k/32k/48k/96k fs */
	unsigned int audio_fmt;
	unsigned int sclk_ws_divider; /* sclk = sclk_ws_divider * wsclk */
	struct snd_dmaengine_dai_dma_data playback_dma_data;

	unsigned int fifo_depth; /* in words */
	unsigned int intr_tx_threshold;
	unsigned int intr_rx_threshold;
	unsigned int dma_tx_threshold;
	unsigned int dma_rx_threshold;

	/* data related to PIO transfers (TX) */
	bool use_pio;
	struct snd_pcm_substream __rcu *tx_substream;
	unsigned int (*tx_fn)(struct csky_i2s *dev,
			      struct snd_pcm_runtime *runtime,
			      unsigned int tx_ptr,
			      bool *period_elapsed);
	unsigned int tx_ptr;
};

#define csky_i2s_readl(i2s, offset) \
		readl((i2s)->regs + (offset))
#define csky_i2s_writel(i2s, offset, val) \
		writel((val), (i2s)->regs + (offset))

extern int csky_snd_dmaengine_pcm_register(struct device *dev,
		const struct snd_dmaengine_pcm_config *config,
		unsigned int flags);
extern void csky_snd_dmaengine_pcm_unregister(struct device *dev);
extern int csky_snd_dmaengine_pcm_prepare_slave_config(
		struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct dma_slave_config *slave_config);

extern int csky_pcm_pio_register(struct platform_device *pdev);
extern void csky_pcm_pio_push_tx(struct csky_i2s *i2s);

#endif /* __CSKY_I2S_H__ */
