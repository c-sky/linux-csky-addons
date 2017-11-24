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

#include <linux/kernel.h>
#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_crtc_helper.h>

#include "csky-drm-drv.h"
#include "csky-drm-fb.h"

#define to_csky_fb(x) container_of(x, struct csky_drm_fb, fb)

static void csky_drm_output_poll_changed(struct drm_device *dev)
{
	struct csky_drm_private *private = dev->dev_private;
	struct drm_fb_helper *fb_helper = &private->fbdev_helper;

	if (fb_helper)
		drm_fb_helper_hotplug_event(fb_helper);
}

static void csky_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

static struct drm_mode_config_helper_funcs csky_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail,
};

static const struct drm_mode_config_funcs csky_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = csky_drm_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

void csky_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &csky_drm_mode_config_funcs;
	dev->mode_config.helper_private = &csky_mode_config_helpers;
}
