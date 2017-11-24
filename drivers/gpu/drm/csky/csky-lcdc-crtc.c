/*
 * DRM CRTC driver for C-SKY's SoCs.
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
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <video/videomode.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>

#include "csky-lcdc-crtc.h"
#include "csky-drm-drv.h"
#include "csky-drm-plane.h"

static u8 crtc_readb(struct csky_drm_crtc *csky_crtc, u8 offset)
{
	return ioread32(csky_crtc->regs + (offset));
}

static void crtc_writeb(struct csky_drm_crtc *csky_crtc, u8 offset, u8 val)
{
	iowrite32(val, csky_crtc->regs + (offset));
}

static void csky_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct csky_drm_crtc *csky_crtc = to_csky_crtc(crtc);

	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct videomode vm;
	unsigned int polarities, err;
	u32 control;
	u32 pixel_format;
	u32 timing0, timing1, timing2;

	pixel_format = crtc->primary->state->fb->pixel_format;
	vm.vfront_porch = mode->crtc_vsync_start - mode->crtc_vdisplay;
	vm.vback_porch = mode->crtc_vtotal - mode->crtc_vsync_end;
	vm.vsync_len = mode->crtc_vsync_end - mode->crtc_vsync_start;
	vm.hfront_porch = mode->crtc_hsync_start - mode->crtc_hdisplay;
	vm.hback_porch = mode->crtc_htotal - mode->crtc_hsync_end;
	vm.hsync_len = mode->crtc_hsync_end - mode->crtc_hsync_start;
	vm.vactive = mode->crtc_vdisplay;
	vm.hactive = mode->crtc_hdisplay;

	timing0 = (((vm.hback_porch - 1) & 0xff) << 24) |
		      (((vm.hfront_porch - 1) & 0xff) << 16) |
		      (((vm.hsync_len - 1) & 0x3f) << 10) |
		      ((vm.hactive - 1) & 0x3ff);
	timing1 = (((vm.vback_porch - 1) & 0xff) << 24) |
		      (((vm.vfront_porch - 1) & 0xff) << 16) |
		      (((vm.vsync_len - 1) & 0x3f) << 10) |
		      ((vm.vactive - 1) & 0x3ff);
	timing2 = 0;
	timing2 |= mode->flags & DRM_MODE_FLAG_PHSYNC ?
		 CSKY_LCDTIM2_HSP_ACT_LOW : 0x0;
	timing2 |= mode->flags & DRM_MODE_FLAG_PVSYNC ?
		 CSKY_LCDTIM2_VSP_ACT_LOW : 0x0;
	timing2 |= mode->flags& DRM_MODE_FLAG_INTERLACE ?
		 0x0 : CSKY_LCDTIM2_PCP_FALLING;

	timing2 |= csky_crtc->pcd & 0xff;

	if (vm.hactive > 1024)
		timing2 |= CSKY_LCDTIM2_PPL_MSB;
	if (vm.vactive > 1024)
		timing2 |= CSKY_LCDTIM2_LPP_MSB;

	crtc_writeb(csky_crtc, CSKY_LCD_TIMING0, timing0);
	crtc_writeb(csky_crtc, CSKY_LCD_TIMING1, timing1);
	crtc_writeb(csky_crtc, CSKY_LCD_TIMING2, timing2);

	/* set pixel fmt */
	control = crtc_readb(csky_crtc, CSKY_LCD_CONTROL);
	control &= ~CSKY_LCDCON_DFS_MASK_SHIFTED;
	control |= pixel_format;
	crtc_writeb(csky_crtc, CSKY_LCD_CONTROL, control);

}

static void csky_drm_crtc_enable(struct drm_crtc *crtc)
{
	csky_drm_crtc_mode_set_nofb(crtc);
	drm_crtc_vblank_on(crtc);
}

static void csky_drm_crtc_disable(struct drm_crtc *crtc)
{
	drm_crtc_vblank_off(crtc);
}

static int csky_crtc_atomic_check(struct drm_crtc *crtc,
				     struct drm_crtc_state *state)
{
	/* struct csky_drm_crtc *csky_crtc = to_csky_crtc(crtc); */
	if (!state->enable)
		return 0;

	return 0;
}

static void csky_crtc_atomic_begin(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs csky_crtc_helper_funcs = {
	.mode_set	= drm_helper_crtc_mode_set,
	.mode_set_base	= drm_helper_crtc_mode_set_base,
	.mode_set_nofb	= csky_drm_crtc_mode_set_nofb,
	.enable 	= csky_drm_crtc_enable,
	.disable	= csky_drm_crtc_disable,
	.prepare	= csky_drm_crtc_disable,
	.commit 	= csky_drm_crtc_enable,
	.atomic_check	= csky_crtc_atomic_check,
	.atomic_begin	= csky_crtc_atomic_begin,
};

static void csky_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct csky_drm_crtc *csky_crtc = to_csky_crtc(crtc);

	/*stop crtc controller register */
	drm_crtc_cleanup(crtc);
	kfree(csky_crtc);
}

static const struct drm_crtc_funcs csky_crtc_funcs = {
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.destroy	= csky_drm_crtc_destroy,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

struct csky_drm_crtc *csky_drm_crtc_create(struct drm_device *drm_dev,
					   struct drm_plane *plane, int pipe)
{
	struct csky_drm_crtc *csky_crtc;
	struct drm_crtc *crtc;
	int ret;
	unsigned int crtc_id;

	csky_crtc = kzalloc(sizeof(struct csky_drm_crtc), GFP_KERNEL);
	if (!csky_crtc)
		return ERR_PTR(-ENOMEM);

	csky_crtc->pipe = pipe;
	crtc = &csky_crtc->base;
	//private->crtc[pipe] = crtc;

	ret = drm_crtc_init_with_planes(drm_dev, crtc, plane, NULL,
					&csky_crtc_funcs, NULL);
	if (ret < 0)
		goto err_crtc;
	crtc_id = drm_crtc_index(crtc);
	drm_crtc_helper_add(crtc, &csky_crtc_helper_funcs);

	return csky_crtc;

err_crtc:
	plane->funcs->destroy(plane);
	kfree(csky_crtc);
	return ERR_PTR(ret);
}

static int csky_crtc_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_plane *csky_plane;
	struct csky_drm_crtc * csky_crtc;
	struct device_node *port;
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm_dev = data;

	struct resource *mem;
	void __iomem *iobase;
	u32 hclk_freq;
	struct clk *hclk;
	unsigned int pcd;
	unsigned int lcd_pixelclock;
	int ret;

	csky_plane = csky_plane_init(drm_dev, DRM_PLANE_TYPE_PRIMARY);
	if(!csky_plane) {
		DRM_DEV_ERROR(dev, "csky plane init failed\n");
		return -EINVAL;
	}
	csky_crtc = csky_drm_crtc_create(drm_dev, csky_plane, 0);
	if(!csky_crtc) {
		DRM_DEV_ERROR(dev, "csky crtc create failed\n");
		return -EINVAL;
	}

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port) {
		DRM_DEV_ERROR(dev, "no port node found in %s\n",
			      dev->of_node->full_name);
		return -ENOENT;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iobase = devm_ioremap_resource(dev, mem);
	if (IS_ERR(iobase))
		return PTR_ERR(iobase);

	hclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(hclk)) {
		dev_err(&pdev->dev, "Failed to get hclk");
		return PTR_ERR(hclk);
	}

	hclk_freq = clk_get_rate(hclk);
	if (!hclk_freq) {
		dev_err(&pdev->dev, "Failed, hclk is 0!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node,
				   "lcd-pixel-clock", &lcd_pixelclock);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to get property pixelclock\n");
		return ret;
	}

	if (hclk_freq % (lcd_pixelclock * 2) != 0) {
		dev_err(&pdev->dev, "Failed to calculate the value of pcd\n");
		return -EINVAL;
	}

	csky_crtc->clk = hclk;
	csky_crtc->hclk_freq = hclk_freq;
	csky_crtc->regs = mem;
	csky_crtc->pcd = hclk_freq / (lcd_pixelclock * 2) - 1;
	csky_crtc->base.port = port;

	return 0;
}

static void csky_crtc_unbind(struct device *dev,
			     struct device *master, void *data)
{

}

const struct component_ops csky_crtc_component_ops = {
	.bind = csky_crtc_bind,
	.unbind = csky_crtc_unbind,
};

static const struct of_device_id csky_crtc_driver_dt_ids[] = {
	{ .compatible = "csky,lcdc-v1",},
	{},
};
MODULE_DEVICE_TABLE(of, vop_driver_dt_match);

static int csky_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->of_node) {
		dev_err(dev, "can't find crtc devices\n");
		return -ENODEV;
	}

	return component_add(dev, &csky_crtc_component_ops);
}

static int csky_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &csky_crtc_component_ops);

	return 0;
}

static struct platform_driver csky_crtc_platform_driver = {
	.probe = csky_crtc_probe,
	.remove = csky_crtc_remove,
	.driver = {
		.name = "csky-hdmi",
		.of_match_table = csky_crtc_driver_dt_ids,
	},
};

module_platform_driver(csky_crtc_platform_driver);

MODULE_AUTHOR("Huoqing Cai <huoqing_cai@c-sky.com>");
MODULE_DESCRIPTION("CSKY CRTC Driver");
MODULE_LICENSE("GPL v2");
