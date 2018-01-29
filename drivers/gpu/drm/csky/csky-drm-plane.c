/*
 * DRM plane driver for C-SKY's SoCs.
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

#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "csky-drm-drv.h"
#include "csky-drm-plane.h"
#include "csky-drm-fb.h"
#include "csky-drm-gem.h"

/* If a modeset involves changing the setup of a plane, the atomic
 * infrastructure will call this to validate a proposed plane setup.
 * However, if a plane isn't getting updated, this (and the
 * corresponding csky_plane_atomic_update) won't get called.  Thus, we
 * compute the dlist here and have all active plane dlists get updated
 * in the CRTC's flush.
 */
static int csky_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	u32 src_w, src_h;

	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	/* we can't do any scaling of the plane source */
	if ((src_w != state->crtc_w) || (src_h != state->crtc_h))
		return -EINVAL;

	return 0;
}

static void csky_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct csky_drm_private *private = plane->dev->dev_private;
	struct csky_drm_crtc *csky_crtc = private->csky_crtc;
	struct drm_fb_helper *helper;
	struct fb_info *fbi;
	struct drm_gem_object *obj;
	struct csky_gem_object *ck_obj;
	dma_addr_t scanout_start;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj;
	u32 width, height;
	
	width = csky_crtc->base.mode.hdisplay;
	height = csky_crtc->base.mode.vdisplay;
	printk("csky_plane_atomic_update\n");
	/*
	 * can't update plane when vop is disabled.
	 */
	if (WARN_ON(!plane->crtc))
		return;

	if (WARN_ON(!csky_crtc->is_enabled))
		return;

	obj = csky_fb_get_gem_obj(fb, 0);
    	if (obj) {
    		ck_obj = to_csky_obj(obj);
        	scanout_start = ck_obj->dma_addr;
    		/* set pbase */
    		iowrite32(scanout_start, csky_crtc->regs + CSKY_LCD_PBASE_Y);
    		iowrite32(scanout_start + width * height, csky_crtc->regs + CSKY_LCD_PBASE_U);
    		iowrite32(scanout_start + width * height + width * height / 4, csky_crtc->regs + CSKY_LCD_PBASE_V);
    	}

}

static const struct drm_plane_helper_funcs csky_plane_helper_funcs = {
	.atomic_check = csky_plane_atomic_check,
	.atomic_update = csky_plane_atomic_update,
};

static void csky_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs csky_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = csky_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const u32 csky_primary_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

struct drm_plane *csky_plane_init(struct drm_device *dev,
				 enum drm_plane_type type)
{
	struct drm_plane *plane;
	struct csky_drm_crtc *csky_crtc;
 	const u32 *formats;
	u32 num_formats;
	int ret = 0;

	num_formats = ARRAY_SIZE(csky_primary_plane_formats);
	formats = csky_primary_plane_formats;
	plane = devm_kzalloc(dev->dev, sizeof(struct drm_plane),
				 GFP_KERNEL);
	if (!plane) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = drm_universal_plane_init(dev, plane, 0xff,
				       &csky_plane_funcs,
				       formats, num_formats,
				       type, NULL);
	drm_plane_helper_add(plane, &csky_plane_helper_funcs);

	return plane;
fail:
	if (plane)
		csky_plane_destroy(plane);
	return NULL;
}
