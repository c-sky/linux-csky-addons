/*
 * C-SKY SoCs LCDC driver
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

#ifndef __CSKY_FB_H__
#define __CSKY_FB_H__

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/reset.h>

/* LCDC registers */
#define CSKY_LCD_CONTROL	0x00
#define CSKY_LCD_TIMING0	0x04
#define CSKY_LCD_TIMING1	0x08
#define CSKY_LCD_TIMING2	0x0c
#define CSKY_LCD_PBASE		0x10
#define CSKY_LCD_PCURR		0x18
#define CSKY_LCD_INT_STAT	0x20
#define CSKY_LCD_INT_MASK	0x24
#define CSKY_LCD_DP1_2		0x28
#define CSKY_LCD_DP4_7		0x2c
#define CSKY_LCD_DP3_5		0x30
#define CSKY_LCD_DP2_3		0x34
#define CSKY_LCD_DP5_7		0x38
#define CSKY_LCD_DP3_4		0x3c
#define CSKY_LCD_DP4_5		0x40
#define CSKY_LCD_DP6_7		0x44
#define CSKY_LCD_PBASE_Y	0x48
#define CSKY_LCD_PBASE_U	0x50
#define CSKY_LCD_PBASE_V	0x58
#define CSKY_LCD_VIDEOSIZE	0x60
#define CSKY_LCD_PALETTE_BASE	0x800

#define CSKY_LCD_PALETTE_ENTRIES_NUM	256
#define CSKY_FB_1080P_SIZE		(1920 * 1080)

/* bits definition */

/*
 * LCD_CONTROL register
 */

/* LEN, LCD controller Enable */
#define CSKY_LCDCON_LEN			1
/*
 * CMS, *STN LCD* color/monochrome select
 * 0: Monochrome
 * 1: Color
 * Note: CMS is ignored in active/TFT mode (PAS=1)
 */
#define CSKY_LCDCON_CMS_COLOR		(1 << 1)
/*
 * PAS, Passive/active display select
 * 0: Passive or STN display
 * 1: Active or TFT display
 */
#define CSKY_LCDCON_PAS_TFT		(1 << 3)
/* PBS, Pixel bit size */
#define CSKY_LCDCON_PBS_8BITS		(1 << 5)
#define CSKY_LCDCON_PBS_16BITS		(2 << 5)
/* BES, Little/ Big-endian Select(read only) */
#define CSKY_LCDCON_BES_BIG		(1 << 8)
/* VBL: Video memory Burst Length */
#define CSKY_LCDCON_VBL_1CYCLES		(0 << 9)
#define CSKY_LCDCON_VBL_4CYCLES		(1 << 9)
#define CSKY_LCDCON_VBL_8CYCLES		(2 << 9)
#define CSKY_LCDCON_VBL_16CYCLES	(3 << 9)
/* WML, LCD DMA FIFO Watermark level. 0=4, 1=8 */
#define CSKY_LCDCON_WML_8WORD		(1 << 11)
/*
 * Real color/pseudo color select
 * 0: pseudo colors 8bit or 16bit (which depend on PBS)
 * 1: RGB data is 24bit
 */
#define CSKY_LCDCON_OUT_24BIT		(1 << 12)
/*
 * DFS, Data format select(Storage format of YUV is planar mode)
 */
#define CSKY_LCDCON_DFS_RGB		(0 << 13)
#define CSKY_LCDCON_DFS_YUV444		(1 << 13)
#define CSKY_LCDCON_DFS_YUV422		(2 << 13)
#define CSKY_LCDCON_DFS_YUV420		(3 << 13)
#define CSKY_LCDCON_DFS_MASK_SHIFTED	(3 << 13)

/*
 * LCD_TIMING2 register
 */

/* CLKS, Pixel clock source select */
#define CSKY_LCDTIM2_CLKS_HCLK		(0 << 8) /* HCLK */
#define CSKY_LCDTIM2_CLKS_EXT		(1 << 8) /* External clock */
/* VSP, Vertical sync polarity (VSYNC pulse polarity) */
#define CSKY_LCDTIM2_VSP_ACT_LOW	(1 << 9)
/* HSP, Horizontal sync polarity (HSYNC pulse polarity) */
#define CSKY_LCDTIM2_HSP_ACT_LOW	(1 << 10)
/* PCP, Pixel clock polarity */
#define CSKY_LCDTIM2_PCP_RISING		(0 << 11)
#define CSKY_LCDTIM2_PCP_FALLING	(1 << 11)
/* OEP, Output Enable polarity */
#define CSKY_LCDTIM2_OEP_ACT_LOW	(1 << 12)

#define CSKY_LCDTIM2_PPL_MSB		(1 << 24)
#define CSKY_LCDTIM2_LPP_MSB		(1 << 25)

/*
 * LCD_INT_STAT register
 */
#define CSKY_LCDINT_STAT_LDD	(1 << 0) /* LDD, LCD disable done status */
#define CSKY_LCDINT_STAT_BAU	(1 << 1) /* BAU, Base address update flag */
#define CSKY_LCDINT_STAT_BER	(1 << 2) /* BER, Bus error status */
#define CSKY_LCDINT_STAT_LFU	(1 << 3) /* LFU, Line FIFO under run status */
/*
 * LCD_INT_MASK register
 * 0: disable interrupt
 * 1: enable interrupt
 */
#define CSKY_LCDINT_MASK_LDD	(1 << 0)
#define CSKY_LCDINT_MASK_BAU	(1 << 1)
#define CSKY_LCDINT_MASK_BER	(1 << 2)
#define CSKY_LCDINT_MASK_LFU	(1 << 3)

/**
 * struct csky_fb_vsync - vsync information
 * @wait:  a queue for processes waiting for vsync
 * @count: vsync interrupt count
 */
struct csky_fb_vsync {
	wait_queue_head_t wait;
	unsigned int count;
};

#define CSKY_FBIO_BASE	0x30
#define CSKY_FBIO_SET_PIXEL_FMT	_IOW('F', CSKY_FBIO_BASE+0, \
					enum csky_fb_pixel_format)
#define CSKY_FBIO_GET_PIXEL_FMT	_IOW('F', CSKY_FBIO_BASE+1, \
					enum csky_fb_pixel_format)
#define CSKY_FBIO_SET_PBASE_YUV	_IOW('F', CSKY_FBIO_BASE+2, \
					struct csky_fb_lcd_pbase_yuv)
#define CSKY_FBIO_SET_OUT_MODE	_IOW('F', CSKY_FBIO_BASE+3, \
					enum csky_fb_out_mode)

enum csky_fb_pixel_format {
	CSKY_FB_PIXEL_FMT_RGB = CSKY_LCDCON_DFS_RGB,
	CSKY_FB_PIXEL_FMT_YUV444 = CSKY_LCDCON_DFS_YUV444,
	CSKY_FB_PIXEL_FMT_YUV422 = CSKY_LCDCON_DFS_YUV422,
	CSKY_FB_PIXEL_FMT_YUV420 = CSKY_LCDCON_DFS_YUV420,
};

enum csky_fb_out_mode {
	CSKY_FB_OUT_LCD_MODE = 0,
	CSKY_FB_OUT_HDMI_MODE,
};

struct csky_fb_lcd_pbase_yuv {
	unsigned int y; /* LCD_PBASE_Y */
	unsigned int u; /* LCD_PBASE_U */
	unsigned int v; /* LCD_PBASE_V */
};

struct csky_fb_info {
	spinlock_t slock;
	struct device *dev;
	void __iomem *iobase;
	struct reset_control *rst;
	int irq;
	struct clk *clk;
	struct videomode vm;
	u32 pixel_clk_src;	/* pixel clock source */
	u32 hclk_freq;		/* hclk frequence */
	u32 pcd;		/* pixel clock divider. f=HCLK/2(pcd+1) */
	u32 hsync_pulse_pol;	/* HSYNC pulse polarity */
	u32 vsync_pulse_pol;	/* VSYNC pulse polarity */
	u32 pixel_clock_pol;	/* pixel clock polarity */
	struct csky_fb_vsync vsync_info;
	enum csky_fb_pixel_format pixel_fmt;
	struct csky_fb_lcd_pbase_yuv pbase_yuv;
	bool lcdc_enabled;	/* indicate whether the lcdc is enabled */
};

#endif /* __CSKY_FB_H__ */
