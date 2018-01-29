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
#include <drm/drm_crtc_helper.h>

#include "csky-drm-drv.h"
#include "csky-drm-fb.h"
#include "csky-drm-gem.h"

#define to_csky_fb(x) container_of(x, struct csky_drm_fb, fb)

struct csky_drm_fb {
	struct drm_framebuffer fb;
	struct drm_gem_object *obj[CSKY_MAX_FB_BUFFER];
};

struct drm_gem_object *csky_fb_get_gem_obj(struct drm_framebuffer *fb,
						 unsigned int plane)
{
	struct csky_drm_fb *ck_fb = to_csky_fb(fb);

	if (plane >= CSKY_MAX_FB_BUFFER)
		return NULL;

	return ck_fb->obj[plane];
}

static void csky_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct csky_drm_fb *csky_fb = to_csky_fb(fb);
	int i;

	for (i = 0; i < CSKY_MAX_FB_BUFFER; i++)
		drm_gem_object_unreference_unlocked(csky_fb->obj[i]);

	drm_framebuffer_cleanup(fb);
	kfree(csky_fb);
}

static int csky_drm_fb_create_handle(struct drm_framebuffer *fb,
					 struct drm_file *file_priv,
					 unsigned int *handle)
{
	struct csky_drm_fb *csky_fb = to_csky_fb(fb);

	return drm_gem_handle_create(file_priv,
				     csky_fb->obj[0], handle);
}

static int csky_drm_fb_dirty(struct drm_framebuffer *fb,
				 struct drm_file *file,
				 unsigned int flags, unsigned int color,
				 struct drm_clip_rect *clips,
				 unsigned int num_clips)
{
	//csky_drm_psr_flush_all(fb->dev);
	return 0;
}

static const struct drm_framebuffer_funcs csky_drm_fb_funcs = {
	.destroy	= csky_drm_fb_destroy,
	.create_handle	= csky_drm_fb_create_handle,
	.dirty		= csky_drm_fb_dirty,
};

static struct csky_drm_fb *
csky_fb_alloc(struct drm_device *dev, const struct drm_mode_fb_cmd2 *mode_cmd,
		  struct drm_gem_object **obj, unsigned int num_planes)
{
	struct csky_drm_fb *csky_fb;
	int ret;
	int i;

	csky_fb = kzalloc(sizeof(*csky_fb), GFP_KERNEL);
	if (!csky_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&csky_fb->fb, mode_cmd);

	for (i = 0; i < num_planes; i++)
		csky_fb->obj[i] = obj[i];

	ret = drm_framebuffer_init(dev, &csky_fb->fb,
				   &csky_drm_fb_funcs);
	if (ret) {
		dev_err(dev->dev, "Failed to initialize framebuffer: %d\n",
			ret);
		kfree(csky_fb);
		return ERR_PTR(ret);
	}

	return csky_fb;
}

static struct drm_framebuffer *
csky_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
			const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct csky_drm_fb *csky_fb;
	struct drm_gem_object *objs[CSKY_MAX_FB_BUFFER];
	struct drm_gem_object *obj;
	unsigned int hsub;
	unsigned int vsub;
	int num_planes;
	int ret;
	int i;

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);
	num_planes = min(drm_format_num_planes(mode_cmd->pixel_format),
			 CSKY_MAX_FB_BUFFER);

	for (i = 0; i < num_planes; i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[i]);
		if (!obj) {
			dev_err(dev->dev, "Failed to lookup GEM object\n");
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		min_size = (height - 1) * mode_cmd->pitches[i] +
			mode_cmd->offsets[i] +
			width * drm_format_plane_cpp(mode_cmd->pixel_format, i);

		if (obj->size < min_size) {
			drm_gem_object_unreference_unlocked(obj);
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}
		objs[i] = obj;
	}

	csky_fb = csky_fb_alloc(dev, mode_cmd, objs, i);
	if (IS_ERR(csky_fb)) {
		ret = PTR_ERR(csky_fb);
		goto err_gem_object_unreference;
	}

	return &csky_fb->fb;

err_gem_object_unreference:
	for (i--; i >= 0; i--)
		drm_gem_object_unreference_unlocked(objs[i]);
	return ERR_PTR(ret);
}

static void csky_drm_output_poll_changed(struct drm_device *dev)
{
	struct csky_drm_private *private = dev->dev_private;
	struct drm_fb_helper *fb_helper = &private->fbdev_helper;

	if (fb_helper)
		drm_fb_helper_hotplug_event(fb_helper);
}

static void csky_atomic_commit_hw_done(struct drm_atomic_state *state)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *drm = state->dev;
	struct csky_drm_private *csky_drm = drm->dev_private;
	struct drm_crtc *crtc = &csky_drm->csky_crtc->base;

	event = crtc->state->event;
	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&drm->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&drm->event_lock);
	}
	drm_atomic_helper_commit_hw_done(state);
}

static void csky_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;
	struct csky_drm_private *csky_drm = dev->dev_private;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

    csky_atomic_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

static int csky_atomic_helper_commit(struct drm_device *dev,
			     struct drm_atomic_state *state,
			     bool nonblock)
{
    int ret;
   // csky_atomic_commit_hw_done(state);
    ret = drm_atomic_helper_commit(dev, state, nonblock);
    return ret;
}


static struct drm_mode_config_helper_funcs csky_mode_config_helpers = {
	.atomic_commit_tail = csky_atomic_commit_tail,
};

static const struct drm_mode_config_funcs csky_drm_mode_config_funcs = {
	.fb_create = csky_user_fb_create,
	.output_poll_changed = csky_drm_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = csky_atomic_helper_commit,//drm_atomic_helper_commit,
};

struct drm_framebuffer *
csky_drm_framebuffer_init(struct drm_device *dev,
				const struct drm_mode_fb_cmd2 *mode_cmd,
				struct drm_gem_object *obj)
{
	struct csky_drm_fb *csky_fb;

	csky_fb = csky_fb_alloc(dev, mode_cmd, &obj, 1);
	if (IS_ERR(csky_fb))
		return NULL;

	return &csky_fb->fb;
}

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
