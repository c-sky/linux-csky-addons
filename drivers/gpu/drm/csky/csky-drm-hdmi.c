/*
 * DRM HDMI driver for C-SKY's SoCs.
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

#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/of_device.h>

#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>

#include "csky-drm-hdmi.h"
#include "csky-drm-drv.h"
#define to_csky_hdmi(x)	container_of(x, struct csky_hdmi, x)

struct hdmi_data_info {
	int vic;
	bool sink_is_hdmi;
	bool sink_has_audio;
	unsigned int enc_in_format;
	unsigned int enc_out_format;
	unsigned int colorimetry;
	unsigned int deep_color;
	unsigned int dvi_mode;
	unsigned int hdcp_mode;
};

struct csky_hdmi_ddc {
	struct i2c_adapter adap;

	u8 ddc_addr;
	u8 segment_addr;

	struct mutex lock;
	struct completion cmp;
};

struct csky_hdmi {
	struct device *dev;
	struct drm_device *drm_dev;

	int irq;
	void __iomem *regs;

	struct reset_control *rst;
	struct csky_hdmi_ddc *ddc;
	struct drm_bridge bridge;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct i2c_adapter *edid_adap;
	unsigned int state_chk;
	unsigned int ctrl_pkt_en;
	unsigned int tmds_rate;

	struct hdmi_data_info	hdmi_data;
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
		0x4a, 0x0e, 0x70, 0x00, 0x22,
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
    u8 val;
    val = ioread32(hdmi->regs + (offset));
    //printk("++++hdmi_readb offset 0x%x:0x%x\n", offset, val);
    return val;
	//return ioread32(hdmi->regs + (offset));
}

static void hdmi_writeb(struct csky_hdmi *hdmi, u8 offset, u8 val)
{
    //printk("++++hdmi_writeb offset 0x%x:0x%x\n", offset, val);
	iowrite32(val, hdmi->regs + (offset));
}

static void hdmi_modb(struct csky_hdmi *hdmi, u8 data, u8 mask, u8 offset)
{
	u8 val = hdmi_readb(hdmi, offset) & ~mask;

	val |= data & mask;
	hdmi_writeb(hdmi, val, offset);
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


static int csky_hdmi_phy_setting(struct csky_hdmi *hdmi, int type)
{
	int index;
	int size;

	size = PHY_DATA_SIZE - 1;
	/* set registers from X56_PHY_CTRL to X5E_PHY_CTRL */
	for (index = 0; index < size; ++index)
		hdmi_writeb(hdmi, X56_PHY_CTRL + index, phy_dat[type][index]);

	hdmi_writeb(hdmi, X17_DC_REG, phy_dat[type][size]);

	return 0;
}

static int csky_hdmi_upload_frame(struct csky_hdmi *hdmi, int setup_rc,
				  union hdmi_infoframe *frame, u32 frame_index,
				  u32 mask, u32 disable, u32 enable)
{
	if (mask)
		hdmi_modb(hdmi, X41_SEND_CPKT_AUTO, mask, disable);

	hdmi_writeb(hdmi, X5F_PACKET_INDEX, frame_index);

	if (setup_rc >= 0) {
		u8 packed_frame[HDMI_MAXIMUM_INFO_FRAME_SIZE];
		ssize_t rc, i;

		rc = hdmi_infoframe_pack(frame, packed_frame,
					 sizeof(packed_frame));
		if (rc < 0)
			return rc;

		for (i = 0; i < rc; i++)
			hdmi_writeb(hdmi, X60_PKT_HB0 + i,
				    packed_frame[i]);

		if (mask)
			hdmi_modb(hdmi, X41_SEND_CPKT_AUTO, mask, enable);
	}

	return setup_rc;
}

static int csky_hdmi_config_video_avi(struct csky_hdmi *hdmi,
				      struct drm_display_mode *mode)
{
	union hdmi_infoframe frame;
	int rc;

	rc = drm_hdmi_avi_infoframe_from_display_mode(&frame.avi, mode);

	if (hdmi->hdmi_data.enc_out_format == HDMI_COLORSPACE_YUV444)
		frame.avi.colorspace = HDMI_COLORSPACE_YUV444;
	else if (hdmi->hdmi_data.enc_out_format == HDMI_COLORSPACE_YUV422)
		frame.avi.colorspace = HDMI_COLORSPACE_YUV422;
	else
		frame.avi.colorspace = HDMI_COLORSPACE_RGB;

	return csky_hdmi_upload_frame(hdmi, rc, &frame, INFOFRAME_AVI, 0, 0, 0);
}

static int csky_hdmi_config_video_vsi(struct csky_hdmi *hdmi,
				      struct drm_display_mode *mode)
{
	union hdmi_infoframe frame;
	int rc;

	rc = drm_hdmi_vendor_infoframe_from_display_mode(&frame.vendor.hdmi,
							 mode);

	return csky_hdmi_upload_frame(hdmi, rc, &frame, INFOFRAME_VSI, 0, 0, 0);
}


static irqreturn_t csky_hdmi_hotplug_irq(int irq, void *dev_id)
{
	struct csky_hdmi *hdmi = dev_id;

	printk("++++csky_hdmi_hotplug_irq\n");
	//drm_helper_hpd_irq_event(hdmi->connector.dev);
	drm_kms_helper_hotplug_event(hdmi->connector.dev);

	return IRQ_HANDLED;
}

static irqreturn_t csky_hdmi_ddc_irq(struct csky_hdmi *hdmi, u8 stat)
{
	struct csky_hdmi_ddc *ddc = hdmi->ddc;

	if (stat & EDID_INT_X94) {
		complete(&ddc->cmp);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int csky_hdmi_modeb_reset(struct csky_hdmi *hdmi)
{
	u8 stat;

	stat = hdmi_readb(hdmi, X00_SYSTEM_CONTROL);
	if (stat & PWR_MOD_A_X00) {
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RST_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RSTB_X00);
	} else if (stat & PWR_MOD_E_X00) {
		/* PS mode e -> d ->b ->a */
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_D_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_B_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_A_X00);
		/* mode a -> mode b */
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RST_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RSTB_X00);
	}

	return 0;
}

static irqreturn_t csky_hdmi_hard_irq(int irq, void *dev_id)
{
	struct csky_hdmi *hdmi = dev_id;
	irqreturn_t ret = IRQ_NONE;
	u8 int_stat;
    csky_hdmi_modeb_reset(hdmi);
	int_stat = hdmi_readb(hdmi, X94_INT_STATUS1);
	hdmi_writeb(hdmi, X94_INT_STATUS1, INT_CLR_X94);
	hdmi_writeb(hdmi, X95_INT_STATUS2, INT_CLR_X95);

	if (hdmi->ddc)
		ret = csky_hdmi_ddc_irq(hdmi, int_stat);
 
	if (int_stat & HPG_MSENS_INT_X94)
		ret = IRQ_WAKE_THREAD;

	return ret;
}

static int csky_hdmi_edid_read(struct csky_hdmi *hdmi, struct i2c_msg *msgs)
{
	int length = msgs->len;
	u8 *buf = msgs->buf;
	int ret;

	ret = wait_for_completion_timeout(&hdmi->ddc->cmp, HZ / 10);
	if (!ret)
		return -EAGAIN;

	while (length--)
		*buf++ = hdmi_readb(hdmi, X80_EDID_FIFO);

	return 0;
}

static int csky_hdmi_edid_write(struct csky_hdmi *hdmi, struct i2c_msg *msgs)
{
	/*
	 * The DDC module only support read EDID message, so
	 * we assume that each word write to this i2c adapter
	 * should be the offset of EDID word address.
	 */
	if ((msgs->len != 1) ||
	    ((msgs->addr != XC5_EDID_WD_ADDR) && (msgs->addr != XC4_SEG_PTR)))
		return -EINVAL;

	reinit_completion(&hdmi->ddc->cmp);

	if (msgs->addr == XC4_SEG_PTR)
		hdmi->ddc->segment_addr = msgs->buf[0];
	if (msgs->addr == XC5_EDID_WD_ADDR)
		hdmi->ddc->ddc_addr = msgs->buf[0];

	/* Set edid fifo first addr */
	hdmi_writeb(hdmi, X80_EDID_FIFO, 0x00);

	/* Set edid word address 0x00/0x80 */
	hdmi_writeb(hdmi, XC5_EDID_WD_ADDR, hdmi->ddc->ddc_addr);

	/* Set edid segment pointer */
	hdmi_writeb(hdmi, XC4_SEG_PTR, hdmi->ddc->segment_addr);

	return 0;
}

static enum drm_connector_status
csky_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct csky_hdmi *hdmi = to_csky_hdmi(connector);

	return (hdmi_readb(hdmi, XDF_HPG_STATUS) & HPG_MSENS_PRT_XDF) ?
		connector_status_connected : connector_status_disconnected;
}

static enum drm_mode_status
csky_hdmi_connector_mode_valid(struct drm_connector *connector,
			       struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int csky_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct csky_hdmi *hdmi = to_csky_hdmi(connector);
	struct edid *edid;
	int ret = 0;

	if (!hdmi->ddc)
		return 0;

	edid = drm_get_edid(connector, hdmi->edid_adap);
	if (edid) {
		hdmi->hdmi_data.sink_is_hdmi = drm_detect_hdmi_monitor(edid);
		hdmi->hdmi_data.sink_has_audio = drm_detect_monitor_audio(edid);
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return ret;
}

static struct drm_connector_funcs csky_hdmi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = csky_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs csky_hdmi_connector_helper_funcs = {
	.get_modes = csky_hdmi_connector_get_modes,
	.mode_valid = csky_hdmi_connector_mode_valid,
};

static int csky_hdmi_edid_xfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	struct csky_hdmi *hdmi = i2c_get_adapdata(adap);
	struct csky_hdmi_ddc *ddc = hdmi->ddc;
	int i, ret = 0;

	mutex_lock(&ddc->lock);

	/* Clear the EDID interrupt flag and unmute the interrupt */
	hdmi_writeb(hdmi, X92_INT_MASK1, EDID_MSK_X92);
	hdmi_writeb(hdmi, X94_INT_STATUS1, EDID_INT_X94);

	for (i = 0; i < num; i++) {
		dev_dbg(hdmi->dev, "xfer: num: %d/%d, len: %d, flags: %#x\n",
			i + 1, num, msgs[i].len, msgs[i].flags);

		if (msgs[i].flags & I2C_M_RD)
			ret = csky_hdmi_edid_read(hdmi, &msgs[i]);
		else
			ret = csky_hdmi_edid_write(hdmi, &msgs[i]);

		if (ret < 0)
			break;
	}

	if (!ret)
		ret = num;

	/* Mute HDMI EDID interrupt */
	hdmi_writeb(hdmi, X92_INT_MASK1, 0);

	mutex_unlock(&ddc->lock);

	return ret;
}

static u32 csky_hdmi_edid_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm csky_hdmi_algorithm = {
	.master_xfer	= csky_hdmi_edid_xfer,
	.functionality	= csky_hdmi_edid_func,
};

static struct i2c_adapter *csky_hdmi_i2c_adapter(struct csky_hdmi *hdmi)
{
	struct i2c_adapter *adap;
	struct csky_hdmi_ddc *ddc;
	int ret;

	ddc = devm_kzalloc(hdmi->dev, sizeof(struct csky_hdmi_ddc), GFP_KERNEL);
	if (!ddc)
		return ERR_PTR(-ENOMEM);

	mutex_init(&ddc->lock);
	init_completion(&ddc->cmp);

	adap = &ddc->adap;
	adap->class = I2C_CLASS_DDC;
	adap->owner = THIS_MODULE;
	adap->dev.parent = hdmi->dev;
	adap->dev.of_node = hdmi->dev->of_node;
	adap->algo = &csky_hdmi_algorithm;
	strlcpy(adap->name, "Csky HDMI", sizeof(adap->name));
	i2c_set_adapdata(adap, hdmi);

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_warn(hdmi->dev, "cannot add %s I2C adapter\n", adap->name);
		devm_kfree(hdmi->dev, ddc);
		return ERR_PTR(ret);
	}

	hdmi->ddc = ddc;
	dev_info(hdmi->dev, "registered %s I2C bus driver\n", adap->name);

	return adap;
}

static int csky_hdmi_video_set_output(struct csky_hdmi *hdmi)
{
	u8 vid_tmp;
	u8 dc_tmp;
	u8 pb_tmp;
	unsigned int out_fmt;
	unsigned int color_depth;

	/* set color mode */
	out_fmt = hdmi->hdmi_data.enc_out_format;
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


static int csky_hdmi_config_video_timing(struct csky_hdmi *hdmi,
					 struct drm_display_mode *mode)
{
	int val;

	/* Set detail external video timing polarity and interlace mode */
	val = BIT(0);
	val |= mode->flags & DRM_MODE_FLAG_PHSYNC ?
		 HSYNC_POLARITY_X30 : 0x0;
	val |= mode->flags & DRM_MODE_FLAG_PVSYNC ?
		 VSYNC_POLARITY_X30 : 0x0;
	val |= mode->flags& DRM_MODE_FLAG_INTERLACE ?
		 INETLACE_X30 : 0x0;
    val = 0x01;
	hdmi_writeb(hdmi, X30_EXT_VPARAMS, val);

	/* Set detail external video timing */
	val = mode->htotal;
	hdmi_writeb(hdmi, X31_EXT_HTOTAL, val & 0xff);
	hdmi_writeb(hdmi, X32_EXT_HTOTAL, (val >> 8) & 0xff);

	val = mode->htotal - mode->hdisplay;
	hdmi_writeb(hdmi, X33_EXT_HBLANK, val & 0xff);
	hdmi_writeb(hdmi, X34_EXT_HBLANK, (val >> 8) & 0xff);

	val = mode->hsync_start - mode->hdisplay;
	hdmi_writeb(hdmi, X35_EXT_HDLY, val & 0xff);
	hdmi_writeb(hdmi, X36_EXT_HDLY, (val >> 8) & 0xff);

	val = mode->hsync_end - mode->hsync_start;
	hdmi_writeb(hdmi, X37_EXT_HS_DUR, val & 0xff);
	hdmi_writeb(hdmi, X38_EXT_HS_DUR, (val >> 8) & 0xff);

	val = mode->vtotal;
	hdmi_writeb(hdmi, X39_EXT_VTOTAL, val & 0xff);
	hdmi_writeb(hdmi, X3A_EXT_VTOTAL, (val >> 8) & 0xff);

	val = mode->vtotal - mode->vdisplay;
	hdmi_writeb(hdmi, X3D_EXT_VBLANK, val & 0xff);

	val = mode->vsync_start - mode->vdisplay;
	hdmi_writeb(hdmi, X3E_EXT_VDLY, val & 0xff);

	val = mode->vsync_end - mode->vsync_start;
	hdmi_writeb(hdmi, X3F_EXT_VS_DUR, val & 0xff);

	return 0;
}

static int csky_hdmi_phy_setup(struct csky_hdmi *hdmi)
{
	unsigned int vid;
	unsigned int color;

	color = DEEP_COLOR_8BIT;//hdmi->hdmi_data.deep_color;
	vid = VID_04_1280X720P;//hdmi->hdmi_data.vic;
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

static void csky_hdmi_i2c_init(struct csky_hdmi *hdmi)
{
	/* Clear the EDID interrupt flag and mute the interrupt */
	//hdmi_writeb(hdmi, HDMI_INTERRUPT_MASK1, 0);
	//hdmi_writeb(hdmi, HDMI_INTERRUPT_STATUS1, m_INT_EDID_READY);
}


static int csky_hdmi_setup(struct csky_hdmi *hdmi,
			   struct drm_display_mode *mode)
{
	unsigned int val;
	unsigned int stat;

	/* set to mode b */
	stat = hdmi_readb(hdmi, X00_SYSTEM_CONTROL);
	if (stat & PWR_MOD_A_X00) {
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RST_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RSTB_X00);
	}

	hdmi->hdmi_data.vic = drm_match_cea_mode(mode);
	hdmi->hdmi_data.enc_in_format = HDMI_COLORSPACE_RGB;
	hdmi->hdmi_data.enc_out_format = HDMI_COLORSPACE_RGB;

	if ((hdmi->hdmi_data.vic == 6) || (hdmi->hdmi_data.vic == 7) ||
	    (hdmi->hdmi_data.vic == 21) || (hdmi->hdmi_data.vic == 22) ||
	    (hdmi->hdmi_data.vic == 2) || (hdmi->hdmi_data.vic == 3) ||
	    (hdmi->hdmi_data.vic == 17) || (hdmi->hdmi_data.vic == 18))
		hdmi->hdmi_data.colorimetry = HDMI_COLORIMETRY_ITU_601;
	else
		hdmi->hdmi_data.colorimetry = HDMI_COLORIMETRY_ITU_709;

	/* set HDCP control mode */
	if (hdmi->hdmi_data.dvi_mode == 1)
		val = hdmi_readb(hdmi, XAF_HDCP_CTRL) & (~HDMI_MODE_CTRL_XAF);
	else
		val = hdmi_readb(hdmi, XAF_HDCP_CTRL) | HDMI_MODE_CTRL_XAF;

	if (hdmi->hdmi_data.hdcp_mode == 1)
		val |= FRAME_ENC_XAF;
	else
		val &= (~FRAME_ENC_XAF);

	csky_hdmi_config_video_timing(hdmi, mode);
	csky_hdmi_video_set_output(hdmi);

	if (hdmi->hdmi_data.sink_is_hdmi) {
		csky_hdmi_config_video_avi(hdmi, mode);
		csky_hdmi_config_video_vsi(hdmi, mode);
	}

	/*
	 * When IP controller have configured to an accurate video
	 * timing, then the TMDS clock source would be switched to
	 * DCLK_LCDC, so we need to init the TMDS rate to mode pixel
	 * clock rate, and reconfigure the DDC clock.
	 */
	hdmi->tmds_rate = mode->clock * 1000;
	csky_hdmi_i2c_init(hdmi);
	csky_hdmi_phy_setup(hdmi);

	return 0;
}

static void csky_hdmi_encoder_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adj_mode)
{
	struct csky_hdmi *hdmi = to_csky_hdmi(encoder);

	csky_hdmi_setup(hdmi, adj_mode);
}

static void csky_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	u8 stat;
	struct csky_hdmi *hdmi = to_csky_hdmi(encoder);
	csky_hdmi_tx_start(hdmi);
#if 0
	/* set to mode b */
	stat = hdmi_readb(hdmi, X00_SYSTEM_CONTROL);
	if (stat & PWR_MOD_A_X00) {
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RST_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MODB_RSTB_X00);
	}
#endif
}

static void csky_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	u8 stat;
	struct csky_hdmi *hdmi = to_csky_hdmi(encoder);
#if 0
	/* set to mode a */
	stat = hdmi_readb(hdmi, X00_SYSTEM_CONTROL);
	if (!(stat & PWR_MOD_A_X00)) {
		/* PS mode e -> d ->b ->a */
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_D_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_B_X00);
		hdmi_writeb(hdmi, X00_SYSTEM_CONTROL, PWR_MOD_A_X00);
	}
#endif
}

static bool csky_hdmi_encoder_mode_fixup(struct drm_encoder *encoder,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adj_mode)
{
	return true;
}

static int
csky_hdmi_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{

	return 0;
}


static struct drm_encoder_funcs csky_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct drm_encoder_helper_funcs csky_hdmi_encoder_helper_funcs = {
	.enable     = csky_hdmi_encoder_enable,
	.disable    = csky_hdmi_encoder_disable,
	.mode_fixup = csky_hdmi_encoder_mode_fixup,
	.mode_set   = csky_hdmi_encoder_mode_set,
	.atomic_check = csky_hdmi_encoder_atomic_check,
};


static int csky_hdmi_register(struct drm_device *drm, struct csky_hdmi *hdmi)
{
	struct drm_encoder *encoder = &hdmi->encoder;
	struct device *dev = hdmi->dev;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &csky_hdmi_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &csky_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	hdmi->connector.polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(&hdmi->connector,
				 &csky_hdmi_connector_helper_funcs);
	drm_connector_init(drm, &hdmi->connector, &csky_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);

	drm_mode_connector_attach_encoder(&hdmi->connector, encoder);

	return 0;
}

static int csky_hdmi_bind(struct device *dev, struct device *master,
				 void *data)
{
	int ret;
	struct csky_hdmi *hdmi;
	struct resource *iores;
	struct drm_device *drm = data;
	struct platform_device *pdev = to_platform_device(dev);

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
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

	//csky_hdmi_init(hdmi);
	hdmi->edid_adap = csky_hdmi_i2c_adapter(hdmi);
	if (IS_ERR(hdmi->edid_adap)) {
		ret = PTR_ERR(hdmi->edid_adap);
		hdmi->edid_adap = NULL;
		return ret;
	}

	dev_set_drvdata(dev, hdmi);
	//hdmi->bridge.funcs = &csky_bridge_funcs;
	//hdmi->bridge.of_node = dev->of_node;
	//ret = drm_bridge_add(&hdmi->bridge);
	/* init kms poll for handling hpd */

	ret = csky_hdmi_register(drm, hdmi);
	if (ret)
		return ret;
	drm_kms_helper_poll_init(hdmi->connector.dev);
	dev_set_drvdata(dev, hdmi);

	ret = devm_request_threaded_irq(dev, hdmi->irq, csky_hdmi_hard_irq,
					csky_hdmi_hotplug_irq,
					IRQF_SHARED, dev_name(dev),
					hdmi);
	if (ret < 0)
		dev_err(dev, "request hdmi irq error: %d\n", ret);

	return ret;
}

static void csky_hdmi_unbind(struct device *dev, struct device *master,
			     void *data)

{
	struct csky_hdmi *hdmi = dev_get_drvdata(dev);

	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);

	i2c_put_adapter(hdmi->edid_adap);
}

static const struct component_ops csky_hdmi_ops = {
	.bind	= csky_hdmi_bind,
	.unbind	= csky_hdmi_unbind,
};

static int csky_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &csky_hdmi_ops);
}

static int csky_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &csky_hdmi_ops);

	return 0;
}

static const struct of_device_id csky_hdmi_dt_ids[] = {
	{ .compatible = "csky,hdmi-v1",},
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
MODULE_DESCRIPTION("CSKY HDMI DRIVER");
MODULE_LICENSE("GPL v2");

