/*
 * HDMI driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Huoqing Cai <huoqing_cai@c-sky.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/hdmi.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/i2c.h>

#include "csky_hdmi.h"

struct hdmi_data_info {
	unsigned int vid;
	unsigned int deep_color;
	unsigned int out_format;
	unsigned int dvi_mode;
	unsigned int hdcp_mode;
};

struct cksy_edid_info {
	unsigned int edid_current;
	unsigned int edid_retry;
	unsigned int edid_size;
	unsigned int ex_flag;
	unsigned int edid_done;
	unsigned char edid_data[HDMI_EDID_BLK_SIZE * 2];
};

struct csky_hdmi {
	struct device *dev;

	int irq;
	spinlock_t edid_lock;
	void __iomem *regs;

	struct videomode vm;
	struct i2c_adapter *i2c_adap;
	struct hdmi_data_info	hdmi_data;
	struct cksy_edid_info	edid_info;
	struct workqueue_struct *edid_workq;
	struct work_struct edid_work;
};

static const char phy_dat[][PHY_DATA_SIZE] = {
	/* PHY setting for TMDS clock 27 (480i, 480p) */
	{
		0x11, 0x00, 0x00, 0x44, 0x32,
		0x4b, 0x0e, 0x70, 0x00, 0x22,
	},
	/* PHY setting for TMDS clock 33.75 (480i, 480p 10 bit) */
	{
		0x15, 0x00, 0x00, 0x48, 0x32,
		0x46, 0x0e, 0x70, 0x00, 0x62,
	},
	/* PHY setting for TMDS clock 40 (480i, 480p 12 bit) */
	{
		0x15, 0x00, 0x00, 0x48, 0x32,
		0x48, 0x0e, 0x70, 0x00, 0xa2,
	},
	/* PHY setting for TMDS clock 54 & 74.25 (720p, 1080i) */
	{
		0x19, 0x00, 0x00, 0x44, 0x32,
		0x48, 0x0e, 0x70, 0x00, 0x22,
	},
	/* PHY setting for TMDS clock 98.8125 (720p, 1080i 10 bit) */
	{
		0x19, 0x00, 0x00, 0x44, 0x32,
		0x48, 0x0e, 0x70, 0x00, 0x62,
	},
	/* PHY setting for TMDS clock 111.375 (720p, 1080i 12 bit) */
	{
		0x19, 0x00, 0x00, 0x44, 0x32,
		0x4b, 0x0e, 0x70, 0x00, 0xa2,
	},
	/* PHY setting for TMDS clock 148.5 (1080p) */
	{
		0x1d, 0x00, 0x00, 0x4c, 0x1e,
		0x47, 0x0e, 0x70, 0x00, 0x22,
	},
	/* PHY setting for TMDS clock 185.6 (1080p deep color 10 bit) */
	{
		0x1d, 0x00, 0x00, 0x4c, 0x1e,
		0x48, 0x0e, 0x70, 0x00, 0x62,
	},
	/* PHY setting for TMDS clock 222.75 (1080p deep color 12 bit) */
	{
		0x1d, 0x00, 0x00, 0x4c, 0x1e,
		0x48, 0x0e, 0x70, 0x00, 0xa2,
	},
};

static u8 hdmi_readb(struct csky_hdmi *hdmi, u8 offset)
{
	return ioread32(hdmi->regs + (offset));
}

static void hdmi_writeb(struct csky_hdmi *hdmi, u8 offset, u8 val)
{
	iowrite32(val, hdmi->regs + (offset));
}

static int csky_hdmi_modeb_reset(struct csky_hdmi *hdmi)
{
	u8 stat;

	stat = hdmi_readb(hdmi, X00_SYSTEM_CONTROL);
	if (!(stat & PWR_MOD_B_X00)) {
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RST_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RSTB_X00);
	}

	return 0;
}

static int csky_hdmi_edid_checksum(struct csky_hdmi *hdmi)
{
	int index;
	int base = 0;
	int sum = 0;
	unsigned char *edid_dat;

	edid_dat = hdmi->edid_info.edid_data;
	if (hdmi->edid_info.ex_flag == 1)
		base += HDMI_EDID_BLK_SIZE;
	for (index = base; index < HDMI_EDID_BLK_SIZE; ++index) {
		sum += *(edid_dat + index);
	}

	return sum & 0xff;
}

static int csky_hdmi_tx_start(struct csky_hdmi *hdmi)
{
	u8 vidset;
	u8 int_mask;

	/* enable hotplug interupt */
	int_mask = HPG_MSK_X92 | MSENS_MSK_X92;
	hdmi_writeb(hdmi, X92_INT_MASK1, int_mask);
	/* enable video output */
	hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_E_X00);
	vidset = hdmi_readb(hdmi, X45_VIDEO2) & ENVIDEO_X45;
	hdmi_writeb(hdmi, X45_VIDEO2, vidset);

	return 0;
}

static int csky_hdmi_edid_start(struct csky_hdmi *hdmi)
{
	u8 edid_word;
	u8 int_mask;
	unsigned int flag;

	spin_lock_irq(&hdmi->edid_lock);
	flag = hdmi->edid_info.ex_flag;
	/* enable EDID interrupt */
	int_mask = hdmi_readb(hdmi, X92_INT_MASK1);
	hdmi_writeb(hdmi, X92_INT_MASK1, int_mask | EDID_MSK_X92);
	/* set EDID word address */
	edid_word = (flag ? EDID_ADDR_80_XC5 : EDID_ADDR_00_XC5);
	hdmi_writeb(hdmi, XC5_EDID_WD_ADDR, edid_word);
	/* set EDID segment pointer */
	hdmi_writeb(hdmi, XC4_SEG_PTR, 0x00);
	spin_unlock_irq(&hdmi->edid_lock);

	return 0;
}

static int csky_hdmi_phy_setting(struct csky_hdmi *hdmi, int type)
{
	int index;
	int size;

	size = PHY_DATA_SIZE - 1;
	hdmi_writeb(hdmi, X17_DC_REG, phy_dat[type][size]);
	for (index = 0; index < size; ++index)
		hdmi_writeb(hdmi, X56_PHY_CTRL + index, phy_dat[type][index]);

	return 0;
}

static int csky_hdmi_phy_setup(struct csky_hdmi *hdmi)
{
	unsigned int vid;
	unsigned int color;

	color = hdmi->hdmi_data.deep_color;
	vid = hdmi->hdmi_data.vid;
	switch (vid & 0x7f) {
	case VID_04_1280X720P:
	case VID_19_1280X720P:
	case VID_05_1920X1080I:
	case VID_20_1920X1080I:
	case VID_32_1920X1080P:
	case VID_33_1920X1080P:
	case VID_34_1920X1080P:
	case VID_39_1920X1080I:
		if (color == DEEP_COLOR_8BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_54);
		else if (color == DEEP_COLOR_12BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_98);
		else
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_111);
		break;
	case VID_16_1920X1080P:
	case VID_31_1920X1080P:
	case VID_40_1920X1080I:
		if (color == DEEP_COLOR_8BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_148);
		else if (color == DEEP_COLOR_12BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_185);
		else
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_222);
		break;
	default:
		if (color == DEEP_COLOR_8BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_27);
		else if (color == DEEP_COLOR_12BIT)
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_33);
		else
			csky_hdmi_phy_setting(hdmi, PHY_TMDS_CLOCK_40);
		break;
	}


	return 0;
}

static int csky_hdmi_video_set_output(struct csky_hdmi *hdmi)
{
	u8 vid_tmp;
	u8 dc_tmp;
	u8 pb_tmp;
	unsigned int out_fmt;
	unsigned int color_depth;

	/* set color mode */
	out_fmt = hdmi->hdmi_data.out_format;
	vid_tmp = hdmi_readb(hdmi, X16_VIDEO1);
	pb_tmp = hdmi_readb(hdmi, X64_PKT_PB1);
	switch (out_fmt) {
	case HDMI_COLORSPACE_RGB:
		vid_tmp = vid_tmp & VID_MASK_X16;
		hdmi_writeb(hdmi, X16_VIDEO1, vid_tmp | VID_RGB_X16);
		hdmi_writeb(hdmi, X64_PKT_PB1, pb_tmp & PB1_MASK_X64);
		break;
	case HDMI_COLORSPACE_YUV444:
		vid_tmp = vid_tmp & VID_MASK_X16;
		hdmi_writeb(hdmi, X16_VIDEO1, vid_tmp | VID_YCC444_X16);
		pb_tmp = pb_tmp & PB1_MASK_X64;
		hdmi_writeb(hdmi, X64_PKT_PB1, pb_tmp | PB1_YCC444_X64);
		break;
	default:
		/* default to YCC 422 */
		vid_tmp = vid_tmp & VID_MASK_X16;
		hdmi_writeb(hdmi, X16_VIDEO1, vid_tmp | VID_YCC422_X16);
		pb_tmp = pb_tmp & PB1_MASK_X64;
		hdmi_writeb(hdmi, X64_PKT_PB1, pb_tmp | PB1_YCC422_X64);
		break;
	}

	/* set color depth */
	color_depth = hdmi->hdmi_data.deep_color;
	vid_tmp = hdmi_readb(hdmi, X16_VIDEO1);
	dc_tmp = hdmi_readb(hdmi, X17_DC_REG);
	switch (color_depth) {
	case DEEP_COLOR_10BIT:
		dc_tmp = (dc_tmp & SPEED_MASK_X17) | SPEED_12BIT_X17;
		vid_tmp = (vid_tmp & DAT_WIDTH_MASK_X16) | WIDTH_10BITS_X16;
		break;
	case DEEP_COLOR_12BIT:
		dc_tmp = (dc_tmp & SPEED_MASK_X17) | SPEED_10BIT_X17;
		vid_tmp = (vid_tmp & DAT_WIDTH_MASK_X16) | WIDTH_12BITS_X16;
		break;
	default:
		/* default: set to 8 bit */
		dc_tmp = (dc_tmp & SPEED_MASK_X17) | SPEED_8BIT_X17;
		vid_tmp = (vid_tmp & DAT_WIDTH_MASK_X16) | WIDTH_8BITS_X16;
		break;
	}

	hdmi_writeb(hdmi, X17_DC_REG, dc_tmp);
	hdmi_writeb(hdmi, X16_VIDEO1, vid_tmp);

	return 0;
}

static int csky_hdmi_video_set_format(struct csky_hdmi *hdmi)
{
	u8 val;

	/* set HDCP control mode */
	if (hdmi->hdmi_data.dvi_mode == 1) {
		/* set RGB and DVI */
		hdmi->hdmi_data.out_format = HDMI_COLORSPACE_RGB;
		val = hdmi_readb(hdmi, XAF_HDCP_CTRL) & (~HDMI_MODE_CTRL_XAF);
	}
	else
		val = hdmi_readb(hdmi, XAF_HDCP_CTRL) | HDMI_MODE_CTRL_XAF;

	if (hdmi->hdmi_data.hdcp_mode == 1)
		val |= FRAME_ENC_XAF;
	else
		val &= (~FRAME_ENC_XAF);

	hdmi_writeb(hdmi,XAF_HDCP_CTRL, val);

	/* set AVI InfoFrame */
	hdmi_writeb(hdmi, X5F_PACKET_INDEX, AVI_INFO_PKT_X5F);
	hdmi_writeb(hdmi, X60_PKT_HB0, HB0_AVI_TYPE_X60);
	hdmi_writeb(hdmi, X61_PKT_HB1, HB1_VERSION_X61);
	hdmi_writeb(hdmi, X62_PKT_HB2, HB2_LENTH_X62);
	hdmi_writeb(hdmi, X67_PKT_PB4, hdmi->hdmi_data.vid & 0x7f);

	return 0;
}

/* i.e. non-preprogrammed VID used. */
static int csky_hdmi_set_external_timing(struct csky_hdmi *hdmi)
{
	int val;
	struct videomode *vm;

	vm = &hdmi->vm;
	/* Set detail external video timing polarity and interlace mode */
	val = BIT(0);
	val |= vm->flags & DISPLAY_FLAGS_HSYNC_HIGH ?
		 HSYNC_POLARITY_X30 : 0x0;
	val |= vm->flags & DISPLAY_FLAGS_VSYNC_HIGH ?
		 VSYNC_POLARITY_X30 : 0x0;
	val |= vm->flags & DISPLAY_FLAGS_INTERLACED ?
		 INETLACE_X30 : 0x0;
	hdmi_writeb(hdmi, X30_EXT_VPARAMS, val);

	/* Set detail external video timing */
	val = vm->hactive + vm->hfront_porch + vm->hsync_len + vm->hback_porch;
	hdmi_writeb(hdmi, X31_EXT_HTOTAL, val & 0xff);
	hdmi_writeb(hdmi, X32_EXT_HTOTAL, (val >> 8) & 0xff);

	val = vm->hfront_porch + vm->hsync_len + vm->hback_porch;
	hdmi_writeb(hdmi, X33_EXT_HBLANK, val & 0xff);
	hdmi_writeb(hdmi, X34_EXT_HBLANK, (val >> 8) & 0xff);

	val = vm->hback_porch + vm->hsync_len;
	hdmi_writeb(hdmi, X35_EXT_HDLY, val & 0xff);
	hdmi_writeb(hdmi, X36_EXT_HDLY, (val >> 8) & 0xff);

	val = vm->hsync_len;
	hdmi_writeb(hdmi, X37_EXT_HS_DUR, val & 0xff);
	hdmi_writeb(hdmi, X38_EXT_HS_DUR, (val >> 8) & 0xff);

	val = vm->vactive + vm->vfront_porch + vm->vsync_len + vm->vback_porch;
	hdmi_writeb(hdmi, X39_EXT_VTOTAL, val & 0xff);
	hdmi_writeb(hdmi, X3A_EXT_VTOTAL, (val >> 8) & 0xff);

	val = vm->vfront_porch + vm->vsync_len + vm->vback_porch;
	hdmi_writeb(hdmi, X3D_EXT_VBLANK, val & 0xff);

	val = vm->vback_porch + vm->vsync_len;
	hdmi_writeb(hdmi, X3E_EXT_VDLY, val & 0xff);

	val = vm->vsync_len;
	hdmi_writeb(hdmi, X3F_EXT_VS_DUR, val & 0xff);

	return 0;
}

static int csky_hdmi_apply_setting(struct csky_hdmi *hdmi)
{
	csky_hdmi_video_set_format(hdmi);
	/*
	 * The pre-programed timing 720p vid4 is not
	 * standard for CEA-861-D. so set external
	 * timing again.
	*/
	csky_hdmi_set_external_timing(hdmi);

	csky_hdmi_video_set_output(hdmi);
	csky_hdmi_phy_setup(hdmi);

	return 0;
}

/* read one block EDID fifo for 128 byte */
static int csky_hdmi_edid_read_block(struct csky_hdmi *hdmi)
{
	int chk_sum;
	unsigned char *edid_dat;
	unsigned int edid_cur;

	edid_dat = hdmi->edid_info.edid_data;
	edid_cur = hdmi->edid_info.edid_current;
	if (hdmi->edid_info.ex_flag == 0) {
		/* read first block */
		*(edid_dat) = 0x00;
		ioread8_rep(hdmi->regs + X80_EDID_FIFO,
			    edid_dat + 1, HDMI_EDID_BLK_SIZE - 1);
		hdmi->edid_info.ex_flag = *(edid_dat + HDMI_EDID_BLK_SIZE -2);
	} else {
		/* read extension block */
		*(edid_dat + HDMI_EDID_BLK_SIZE) = 0x02;
		ioread8_rep(hdmi->regs + X80_EDID_FIFO,
			    edid_dat + HDMI_EDID_BLK_SIZE + 1,
			    HDMI_EDID_BLK_SIZE - 1);
		hdmi->edid_info.ex_flag = 0;
	}

	chk_sum = csky_hdmi_edid_checksum(hdmi);

	return chk_sum;
}

static irqreturn_t csky_hdmi_edid_irq(struct csky_hdmi *hdmi, u8 int_stat)
{
	irqreturn_t ret = IRQ_HANDLED;

	if (int_stat & EDID_ERR_INT_X94) {
		hdmi->edid_info.edid_retry --;
		if (hdmi->edid_info.edid_retry == 0)
			dev_err(hdmi->dev, "Cannot read EDID data\n");
		else
			csky_hdmi_edid_start(hdmi);
	}
	else if (int_stat & EDID_RDY_INT_X94)
		ret = IRQ_WAKE_THREAD;

	return ret;
}

static int csky_hdmi_unplug_irq(struct csky_hdmi *hdmi)
{
	u8 vidset;
	u8 int_stat;

	int_stat = hdmi_readb(hdmi, XDF_HPG_STATUS);
	if (!(int_stat & HPG_MSENS_PRT_XDF)) {
		/* disable video output */
		vidset = hdmi_readb(hdmi, X45_VIDEO2) | NOVIDEO_X45;
		hdmi_writeb(hdmi, X45_VIDEO2, vidset);
		/* enable hotplug int */
		hdmi_writeb(hdmi, X92_INT_MASK1, HPG_MSK_X92 | MSENS_MSK_X92);
		/* PS mode e -> d ->b ->a */
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_D_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_B_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_A_X00);
		if (hdmi->edid_workq)
			flush_workqueue(hdmi->edid_workq);
	}
	else
		dev_err(hdmi->dev, "Unplug error\n");

	return 0;
}

static int csky_hdmi_hotplug_irq(struct csky_hdmi *hdmi)
{
	u8 int_stat;

	int_stat = hdmi_readb(hdmi, XDF_HPG_STATUS);
	/* hotplug and unplug */
	if (int_stat & HPG_MSENS_PRT_XDF) {
		hdmi->edid_info.edid_retry = HDMI_EDID_RETRY_TIMES;
		hdmi->edid_info.ex_flag = 0;
		csky_hdmi_edid_start(hdmi);
	}
	else
		dev_err(hdmi->dev, "Hotplug error\n");

	return 0;
}

static irqreturn_t csky_hdmi_irq(int irq, void *dev_id)
{
	u8 int_stat;
	irqreturn_t ret = IRQ_HANDLED;
	struct csky_hdmi *hdmi = dev_id;

	csky_hdmi_modeb_reset(hdmi);
	/* clear all interrupt */
	int_stat = hdmi_readb(hdmi, X94_INT_STATUS1);
	hdmi_writeb(hdmi, X94_INT_STATUS1, INT_CLR_X94);
	hdmi_writeb(hdmi, X95_INT_STATUS2, INT_CLR_X95);

	if (int_stat & EDID_INT_X94)
		ret = csky_hdmi_edid_irq(hdmi, int_stat);
	else if (int_stat & HPG_MSENS_INT_X94)
		csky_hdmi_hotplug_irq(hdmi);
	else
		csky_hdmi_unplug_irq(hdmi);

	return ret;
}

static void csky_hdmi_main_work(struct work_struct *work)
{
	int chk_sum;
	unsigned int int_mask;
	unsigned int flag;
	struct csky_hdmi *hdmi;

	hdmi = container_of(work, struct csky_hdmi, edid_work);
	chk_sum = csky_hdmi_edid_read_block(hdmi);
	if (chk_sum != 0)
		dev_err(hdmi->dev, "EDID info is error\n");
	flag = hdmi->edid_info.ex_flag;

	if (flag == 0) {
		/* disable EDID interrupt */
		int_mask = hdmi_readb(hdmi, X92_INT_MASK1);
		int_mask = int_mask & (~EDID_MSK_X92);
		hdmi_writeb(hdmi, X92_INT_MASK1, int_mask);

		csky_hdmi_apply_setting(hdmi);
		csky_hdmi_tx_start(hdmi);
	}
	else
		csky_hdmi_edid_start(hdmi);
}

static int csky_timing_data_init(struct csky_hdmi *hdmi)
{
	/* get timing from dts */
	int ret;
	int vid;
	struct device *dev = hdmi->dev;
	struct device_node *screen_node;
	struct videomode vm;

	screen_node = of_parse_phandle(dev->of_node, "screen-timings", 0);
	ret = of_get_videomode(screen_node, &vm, HDMI_TIMING_INDEX);
	if (ret)
		dev_err(dev, "Failed to get videomode from DT\n");
	else
		memcpy(&hdmi->vm, &vm, sizeof(vm));

	ret = of_property_read_u32(dev->of_node, "vid-code", &vid);
	if (ret < 0)
		dev_err(dev, "Failed to get property vid-code\n");
	else
		hdmi->hdmi_data.vid = vid;

	return 0;
}

/* to config init format, hope the data comes from fb */
static int csky_hdmi_data_init(struct csky_hdmi *hdmi)
{
	/* hdmi edid data set */
	hdmi->edid_info.edid_current = 0;
	hdmi->edid_info.ex_flag = 0;
	hdmi->edid_info.edid_retry = HDMI_EDID_RETRY_TIMES;
	/* video data set */
	hdmi->hdmi_data.dvi_mode = 0;
	hdmi->hdmi_data.hdcp_mode = 0;
	hdmi->hdmi_data.deep_color = DEEP_COLOR_8BIT;
	hdmi->hdmi_data.out_format = HDMI_COLORSPACE_RGB;

	return 0;
}

static int csky_hdmi_init(struct csky_hdmi *hdmi)
{
	csky_hdmi_data_init(hdmi);
	csky_timing_data_init(hdmi);
	csky_hdmi_modeb_reset(hdmi);
	INIT_WORK(&hdmi->edid_work, csky_hdmi_main_work);
	hdmi->edid_workq= create_singlethread_workqueue("hdmi-csky");

	return 0;
}

static irqreturn_t csky_hdmi_work_irq(int irq, void *dev_id)
{
	struct csky_hdmi *hdmi = dev_id;

	/* Process EDID work: */
	queue_work(hdmi->edid_workq, &hdmi->edid_work);

	return IRQ_HANDLED;
}

static int csky_hdmi_probe(struct platform_device *pdev)
{
	int ret;
	struct csky_hdmi *hdmi;
	struct resource *iores;
	struct device *dev = &pdev->dev;

	hdmi = devm_kzalloc(dev, sizeof(struct csky_hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dev = dev;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENXIO;

	hdmi->regs = devm_ioremap_resource(dev, iores);
	if (IS_ERR(hdmi->regs))
		return PTR_ERR(hdmi->regs);

	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq < 0)
		return hdmi->irq;

	dev_set_drvdata(dev, hdmi);
	csky_hdmi_init(hdmi);

	ret = devm_request_threaded_irq(dev, hdmi->irq, csky_hdmi_irq,
					csky_hdmi_work_irq,
					IRQF_SHARED, dev_name(dev),
					hdmi);

	return ret;
}

static int csky_hdmi_remove(struct platform_device *pdev)
{
	struct csky_hdmi *hdmi = dev_get_drvdata(&pdev->dev);

	if (hdmi->edid_workq) {
		flush_workqueue(hdmi->edid_workq);
		destroy_workqueue(hdmi->edid_workq);
	}

	return 0;
}

static const struct of_device_id csky_hdmi_dt_ids[] = {
	{ .compatible = "csky,eragon-hdmi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, csky_hdmi_dt_ids);

static struct platform_driver csky_hdmi_driver = {
	.probe  = csky_hdmi_probe,
	.remove = csky_hdmi_remove,
	.driver = {
		.name = "hdmi-csky",
		.of_match_table = csky_hdmi_dt_ids,
	},
};

module_platform_driver(csky_hdmi_driver);

MODULE_AUTHOR("Huoqing Cai <huoqing_cai@c-sky.com>");
MODULE_DESCRIPTION("CSKY HDMI");
MODULE_LICENSE("GPL v2");
