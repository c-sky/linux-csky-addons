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

#ifndef _CSKY_DRM_DRV_H
#define _CSKY_DRM_DRV_H

#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem.h>
#include <linux/module.h>
#include <linux/component.h>

#define CSKY_MAX_FB_BUFFER	1
#define CSKY_MAX_CONNECTOR	1
#define CSKY_MAX_CRTC		1

struct drm_device;
struct drm_connector;

#define to_csky_crtc(x)		container_of(x, struct csky_drm_crtc, base)

struct csky_drm_crtc {
	struct drm_crtc base;
	void __iomem *regs;
	struct clk *clk;
	u32 hclk_freq;		/* hclk frequence */
	int irq;
	unsigned int	pipe;
	unsigned int pcd;
	bool is_enabled;
	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;
	/* lock vop irq reg */
	spinlock_t irq_lock;
};

struct csky_drm_plane {
	struct drm_plane base;
};

/*
 * Csky drm private crtc funcs.
 * @enable_vblank: enable crtc vblank irq.
 * @disable_vblank: disable crtc vblank irq.
 */
struct csky_crtc_funcs {
	int (*enable_vblank)(struct drm_crtc *crtc);
	void (*disable_vblank)(struct drm_crtc *crtc);
};

struct csky_crtc_state {
	struct drm_crtc_state base;
	int output_type;
	int output_mode;
};
#define to_csky_crtc_state(s) \
		container_of(s, struct csky_crtc_state, base)

/*
 * CSKY drm private structure.
 *
 * @crtc: array of enabled CRTCs, used to map from "pipe" to drm_crtc.
 * @num_pipe: number of pipes for this device.
 */
struct csky_drm_private {
	struct drm_fb_helper fbdev_helper;
	struct drm_gem_object *fbdev_bo;
	const struct csky_crtc_funcs *crtc_funcs[CSKY_MAX_CRTC];
	struct drm_atomic_state *state;
	struct csky_drm_crtc *csky_crtc;

	struct list_head psr_list;
	spinlock_t psr_list_lock;
};

int csky_register_crtc_funcs(struct drm_crtc *crtc,
				 const struct csky_crtc_funcs *crtc_funcs);
void csky_unregister_crtc_funcs(struct drm_crtc *crtc);

#endif /* _CSKY_DRM_DRV_H_ */
