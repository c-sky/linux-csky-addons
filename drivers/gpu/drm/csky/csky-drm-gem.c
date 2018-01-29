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
#include <drm/drm_gem.h>
#include <drm/drm_vma_manager.h>

#include "csky-drm-drv.h"
#include "csky-drm-gem.h"

static int csky_gem_alloc_buf(struct csky_gem_object *ck_obj,
				  bool alloc_kmap)
{
	struct drm_gem_object *obj = &ck_obj->base;
	struct drm_device *drm = obj->dev;

	ck_obj->dma_attrs = DMA_ATTR_WRITE_COMBINE;

	if (!alloc_kmap)
		ck_obj->dma_attrs |= DMA_ATTR_NO_KERNEL_MAPPING;

	ck_obj->cookie = dma_alloc_attrs(drm->dev, obj->size,
					 &ck_obj->dma_addr, GFP_KERNEL,
					 ck_obj->dma_attrs);
	if (!ck_obj->cookie) {
		DRM_ERROR("failed to allocate %zu byte dma buffer", obj->size);
		return -ENOMEM;
	}

	return 0;
}

static void csky_gem_free_buf(struct csky_gem_object *ck_obj)
{
	struct drm_gem_object *obj = &ck_obj->base;
	struct drm_device *drm = obj->dev;

	dma_free_attrs(drm->dev, obj->size, ck_obj->cookie, ck_obj->dma_addr,
		       ck_obj->dma_attrs);
}

static int csky_drm_gem_object_mmap(struct drm_gem_object *obj,
					struct vm_area_struct *vma)

{
	int ret;
	struct csky_gem_object *ck_obj = to_csky_obj(obj);
	struct drm_device *drm = obj->dev;

	/*
	 * dma_alloc_attrs() allocated a struct page table for ck_obj, so clear
	 * VM_PFNMAP flag that was set by drm_gem_mmap_obj()/drm_gem_mmap().
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;
	ck_obj->dma_attrs = 0;

	ret = dma_mmap_attrs(drm->dev, vma, ck_obj->cookie, ck_obj->dma_addr,
			     obj->size, ck_obj->dma_attrs);
	if (ret)
		drm_gem_vm_close(vma);

	return ret;
}

int csky_gem_mmap_buf(struct drm_gem_object *obj,
			  struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret)
		return ret;

	return csky_drm_gem_object_mmap(obj, vma);
}

/* drm driver mmap file operations */
int csky_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_gem_object *obj;

	int ret;
	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;

	obj = vma->vm_private_data;

	return csky_drm_gem_object_mmap(obj, vma);
}

struct csky_gem_object *
	csky_gem_create_object(struct drm_device *drm, unsigned int size,
				   bool alloc_kmap)
{
	struct csky_gem_object *ck_obj;
	struct drm_gem_object *obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	ck_obj = kzalloc(sizeof(*ck_obj), GFP_KERNEL);
	if (!ck_obj)
		return ERR_PTR(-ENOMEM);

	obj = &ck_obj->base;

	ret = drm_gem_object_init(drm, obj, size);
	if (ret < 0) {
		DRM_ERROR("failed to initialize gem object\n");
		goto err_free_ck_obj;
	}

	ret = csky_gem_alloc_buf(ck_obj, alloc_kmap);
	if (ret)
		goto err_free_ck_obj;

	return ck_obj;

err_free_ck_obj:
	kfree(ck_obj);
	return ERR_PTR(ret);
}

/*
 * csky_gem_free_object - (struct drm_driver)->gem_free_object callback
 * function
 */
void csky_gem_free_object(struct drm_gem_object *obj)
{
	struct csky_gem_object *ck_obj;

	drm_gem_free_mmap_offset(obj);

	ck_obj = to_csky_obj(obj);

	csky_gem_free_buf(ck_obj);

	kfree(ck_obj);
}

/*
 * csky_gem_create_with_handle - allocate an object with the given
 * size and create a gem handle on it
 *
 * returns a struct csky_gem_object* on success or ERR_PTR values
 * on failure.
 */
static struct csky_gem_object *
csky_gem_create_with_handle(struct drm_file *file_priv,
				struct drm_device *drm, unsigned int size,
				unsigned int *handle)
{
	struct csky_gem_object *ck_obj;
	struct drm_gem_object *obj;
	int ret;

	ck_obj = csky_gem_create_object(drm, size, false);
	if (IS_ERR(ck_obj))
		return ERR_CAST(ck_obj);

	obj = &ck_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, obj, handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(obj);

	return ck_obj;

err_handle_create:
	csky_gem_free_object(obj);

	return ERR_PTR(ret);
}

int csky_gem_dumb_map_offset(struct drm_file *file_priv,
				 struct drm_device *dev, uint32_t handle,
				 uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		return -EINVAL;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto out;

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
	DRM_DEBUG_KMS("offset = 0x%llx\n", *offset);

out:
	drm_gem_object_unreference_unlocked(obj);

	return 0;
}

/*
 * csky_gem_dumb_create - (struct drm_driver)->dumb_create callback
 * function
 *
 * This aligns the pitch and size arguments to the minimum required. wrap
 * this into your own function if you need bigger alignment.
 */
int csky_gem_dumb_create(struct drm_file *file_priv,
			     struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	struct csky_gem_object *ck_obj;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	/*
	 * align to 64 bytes since Mali requires it.
	 */
	args->pitch = ALIGN(min_pitch, 64);
	args->size = args->pitch * args->height;

	ck_obj = csky_gem_create_with_handle(file_priv, dev, args->size,
						 &args->handle);

	return PTR_ERR_OR_ZERO(ck_obj);
}

/*
 * Allocate a sg_table for this GEM object.
 * Note: Both the table's contents, and the sg_table itself must be freed by
 *       the caller.
 * Returns a pointer to the newly allocated sg_table, or an ERR_PTR() error.
 */
struct sg_table *csky_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct csky_gem_object *ck_obj = to_csky_obj(obj);
	struct drm_device *drm = obj->dev;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = dma_get_sgtable_attrs(drm->dev, sgt, ck_obj->cookie,
				    ck_obj->dma_addr, obj->size,
				    ck_obj->dma_attrs);
	if (ret) {
		DRM_ERROR("failed to allocate sgt, %d\n", ret);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	return sgt;
}

void *csky_gem_prime_vmap(struct drm_gem_object *obj)
{
	return NULL;
}

void csky_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	/* Nothing to do */
}
