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

#ifndef _CSKY_DRM_GEM_H
#define _CSKY_DRM_GEM_H

#define to_csky_obj(x) container_of(x, struct csky_gem_object, base)

struct csky_gem_object {
	struct drm_gem_object base;
	unsigned int flags;
	void *cookie;
	void __iomem *kvaddr;
	dma_addr_t dma_addr;
	unsigned long dma_attrs;
};

struct sg_table *csky_gem_prime_get_sg_table(struct drm_gem_object *obj);
struct drm_gem_object *
csky_gem_prime_import_sg_table(struct drm_device *dev, size_t size,
				   struct sg_table *sgt);
void *csky_gem_prime_vmap(struct drm_gem_object *obj);
void csky_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);

/* drm driver mmap file operations */
int csky_gem_mmap(struct file *filp, struct vm_area_struct *vma);

/* mmap a gem object to userspace. */
int csky_gem_mmap_buf(struct drm_gem_object *obj,
			  struct vm_area_struct *vma);

struct csky_gem_object *
	csky_gem_create_object(struct drm_device *drm, unsigned int size,
				   bool alloc_kmap);

void csky_gem_free_object(struct drm_gem_object *obj);

int csky_gem_dumb_create(struct drm_file *file_priv,
			     struct drm_device *dev,
			     struct drm_mode_create_dumb *args);
int csky_gem_dumb_map_offset(struct drm_file *file_priv,
				 struct drm_device *dev, uint32_t handle,
				 uint64_t *offset);
#endif /* _CSKY_DRM_GEM_H */
