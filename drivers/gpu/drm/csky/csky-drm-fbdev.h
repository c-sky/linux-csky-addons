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

#ifndef _CSKY_DRM_FBDEV_H
#define _CSKY_DRM_FBDEV_H

#ifdef CONFIG_DRM_FBDEV_EMULATION
#define CSKY_FB_1080P_SIZE		(1920 * 1080)
int csky_drm_fbdev_init(struct drm_device *dev);
void csky_drm_fbdev_fini(struct drm_device *dev);
#else
static inline int csky_drm_fbdev_init(struct drm_device *dev)
{
	return 0;
}

static inline void csky_drm_fbdev_fini(struct drm_device *dev)
{
}
#endif

#endif /* _CSKY_DRM_FBDEV_H */
