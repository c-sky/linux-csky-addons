/*
 * DRM driver for C-SKY's SoCs.
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

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include "csky-drm-drv.h"
#include "csky-drm-fb.h"
#include "csky-drm-fbdev.h"

#define PREFERRED_BPP		32
#define to_drm_private(x) \
		container_of(x, struct csky_drm_private, fbdev_helper)

static const struct drm_framebuffer_funcs csky_drm_fb_funcs = {
	.destroy	= drm_framebuffer_cleanup,
};

static struct fb_ops csky_drm_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= drm_fb_helper_cfb_fillrect,
	.fb_copyarea	= drm_fb_helper_cfb_copyarea,
	.fb_imageblit	= drm_fb_helper_cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int csky_drm_fbdev_create(struct drm_fb_helper *helper,
				     struct drm_fb_helper_surface_size *sizes)
{
	struct csky_drm_private *private = to_drm_private(helper);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	phys_addr_t map_phys;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;

	fb = kzalloc(sizeof(struct drm_framebuffer), GFP_KERNEL);
	if (!fb)
		return -ENOMEM;

	helper->fb = fb;
	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);
	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;

	fbi = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(fbi)) {
		dev_err(dev->dev, "Failed to create framebuffer info.\n");
		return PTR_ERR(fbi);
	}

	drm_helper_mode_fill_fb_struct(helper->fb, &mode_cmd);
	ret = drm_framebuffer_init(dev, helper->fb, &csky_drm_fb_funcs);
	if (ret) {
		dev_err(dev->dev, "Failed to initialize framebuffer: %d\n",
			ret);
		drm_framebuffer_cleanup(helper->fb);
		return ret;
	}

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	helper->fbdev->fbops = &csky_drm_fbdev_ops;

	unsigned map_size = PAGE_ALIGN(CSKY_FB_1080P_SIZE *
				       (fbi->var.bits_per_pixel / 8));

	fbi->screen_base = kmalloc(map_size, GFP_KERNEL);

	if (fbi->screen_base == NULL) {
		return -ENOMEM;
	}

	map_phys = virt_to_phys(fbi->screen_base);
	memset(fbi->screen_base, 0, map_size);

	dev->mode_config.fb_base = 0;
	fbi->screen_size = map_size;
	fbi->fix.smem_start = map_phys;
	fbi->fix.smem_len = map_size;

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, sizes->fb_width, sizes->fb_height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	DRM_DEBUG_KMS("FB [%dx%d]-%d kvaddr=%p offset=%ld size=%zu\n",
		      fb->width, fb->height, fb->depth,fbi->fix.smem_start,
		      offset, size);

	fbi->skip_vt_switch = true;

	return 0;

err_release_fbi:
	drm_fb_helper_release_fbi(helper);
}

static const struct drm_fb_helper_funcs csky_drm_fb_helper_funcs = {
	.fb_probe = csky_drm_fbdev_create,
};

int csky_drm_fbdev_init(struct drm_device *dev)
{
	struct csky_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return -EINVAL;

	num_crtc = dev->mode_config.num_crtc;

	helper = &private->fbdev_helper;

	drm_fb_helper_prepare(dev, helper, &csky_drm_fb_helper_funcs);

	ret = drm_fb_helper_init(dev, helper, num_crtc, CSKY_MAX_CONNECTOR);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper - %d.\n",
			ret);
		return ret;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors - %d.\n", ret);
		goto err_drm_fb_helper_fini;
	}

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set initial hw config - %d.\n",
			ret);
		goto err_drm_fb_helper_fini;
	}

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
	return ret;
}

void csky_drm_fbdev_fini(struct drm_device *dev)
{
	struct csky_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;

	helper = &private->fbdev_helper;

	drm_fb_helper_unregister_fbi(helper);
	drm_fb_helper_release_fbi(helper);

	if (helper->fb)
		drm_framebuffer_unreference(helper->fb);

	drm_fb_helper_fini(helper);
}
