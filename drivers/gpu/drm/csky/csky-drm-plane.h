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

#ifndef __CSKY_PANEL_H__
#define __CSKY_PANEL_H__

#define CSKY_LCD_PBASE 0x10

/* sub-module for generic lcd panel output */
struct drm_plane *csky_plane_init(struct drm_device *dev,
				 enum drm_plane_type type);

#endif /* __CSKY_PANEL_H__ */
