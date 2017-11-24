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

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/component.h>
#include <linux/console.h>

#include "csky-drm-drv.h"
#include "csky-drm-fbdev.h"
#include "csky-drm-fb.h"

#define DRIVER_NAME	"csky-drm"
#define DRIVER_DESC	"Csky Soc DRM"
#define DRIVER_DATE	"20171218"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static struct drm_driver csky_drm_driver;
int csky_register_crtc_funcs(struct drm_crtc *crtc,
				 const struct csky_crtc_funcs *crtc_funcs)
{
	int pipe = drm_crtc_index(crtc);
	struct csky_drm_private *priv = crtc->dev->dev_private;

	if (pipe >= CSKY_MAX_CRTC)
		return -EINVAL;

	priv->crtc_funcs[pipe] = crtc_funcs;

	return 0;
}

void csky_unregister_crtc_funcs(struct drm_crtc *crtc)
{
	int pipe = drm_crtc_index(crtc);
	struct csky_drm_private *priv = crtc->dev->dev_private;

	if (pipe >= CSKY_MAX_CRTC)
		return;

	priv->crtc_funcs[pipe] = NULL;
}

static struct drm_crtc *csky_crtc_from_pipe(struct drm_device *drm,
						int pipe)
{
	struct drm_crtc *crtc;
	int i = 0;

	list_for_each_entry(crtc, &drm->mode_config.crtc_list, head)
		if (i++ == pipe)
			return crtc;

	return NULL;
}

static int csky_drm_crtc_enable_vblank(struct drm_device *dev,
					   unsigned int pipe)
{
	struct csky_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc = csky_crtc_from_pipe(dev, pipe);

	if (crtc && priv->crtc_funcs[pipe] &&
	    priv->crtc_funcs[pipe]->enable_vblank)
		return priv->crtc_funcs[pipe]->enable_vblank(crtc);

	return 0;
}

static void csky_drm_crtc_disable_vblank(struct drm_device *dev,
					     unsigned int pipe)
{
	struct csky_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc = csky_crtc_from_pipe(dev, pipe);

	if (crtc && priv->crtc_funcs[pipe] &&
	    priv->crtc_funcs[pipe]->enable_vblank)
		priv->crtc_funcs[pipe]->disable_vblank(crtc);
}

static void csky_drm_lastclose(struct drm_device *dev)
{
	struct csky_drm_private *priv = dev->dev_private;

	drm_fb_helper_restore_fbdev_mode_unlocked(&priv->fbdev_helper);
}

static const struct file_operations csky_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.poll = drm_poll,
	.read = drm_read,
	.mmap = drm_gem_cma_mmap,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release = drm_release,
};

static struct drm_driver csky_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM |
				  DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose		= csky_drm_lastclose,
	.get_vblank_counter	= drm_vblank_no_hw_counter,
	.enable_vblank		= csky_drm_crtc_enable_vblank,
	.disable_vblank 	= csky_drm_crtc_disable_vblank,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	.fops			= &csky_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

static int compare_of(struct device *dev, void *data)
{
	struct device_node *np = data;

	return dev->of_node == np;
}

static void csky_add_endpoints(struct device *dev,
				   struct component_match **match,
				   struct device_node *port)
{
	struct device_node *ep, *remote;

	for_each_child_of_node(port, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote || !of_device_is_available(remote)) {
			of_node_put(remote);
			continue;
		} else if (!of_device_is_available(remote->parent)) {
			dev_warn(dev, "parent device of %s is not available\n",
				 remote->full_name);
			of_node_put(remote);
			continue;
		}

		component_match_add(dev, match, compare_of, remote);
		of_node_put(remote);
	}
}

static int csky_drm_bind(struct device *dev)
{
	struct drm_device *drm_dev;
	struct csky_drm_private *private;
	int ret;

	drm_dev = drm_dev_alloc(&csky_drm_driver, dev);
	if (IS_ERR(drm_dev))
		return PTR_ERR(drm_dev);

	dev_set_drvdata(dev, drm_dev);

	private = devm_kzalloc(drm_dev->dev, sizeof(*private), GFP_KERNEL);
	if (!private) {
		ret = -ENOMEM;
		goto err_free;
	}

	drm_dev->dev_private = private;

	drm_mode_config_init(drm_dev);

	csky_drm_mode_config_init(drm_dev);

	dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms),
				      GFP_KERNEL);
	if (!dev->dma_parms) {
		ret = -ENOMEM;
		goto err_config_cleanup;
	}

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev, drm_dev);
	if (ret)
		return ret;

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(drm_dev);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = true, we can use the vblank feature.
	 */
	drm_dev->irq_enabled = true;

	ret = drm_vblank_init(drm_dev, CSKY_MAX_CRTC);
	if (ret)
		goto err_kms_helper_poll_fini;

	drm_mode_config_reset(drm_dev);

	ret = csky_drm_fbdev_init(drm_dev);
	if (ret)
		goto err_vblank_cleanup;

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto err_fbdev_fini;

	return 0;
err_fbdev_fini:
	csky_drm_fbdev_fini(drm_dev);
err_vblank_cleanup:
	drm_vblank_cleanup(drm_dev);
err_kms_helper_poll_fini:
	drm_kms_helper_poll_fini(drm_dev);
	component_unbind_all(dev, drm_dev);
err_config_cleanup:
	drm_mode_config_cleanup(drm_dev);
	drm_dev->dev_private = NULL;
err_free:
	drm_dev_unref(drm_dev);
	return ret;
}

static void csky_drm_unbind(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	csky_drm_fbdev_fini(drm_dev);
	drm_vblank_cleanup(drm_dev);
	drm_kms_helper_poll_fini(drm_dev);
	component_unbind_all(dev, drm_dev);
	drm_mode_config_cleanup(drm_dev);
	drm_dev->dev_private = NULL;
	drm_dev_unregister(drm_dev);
	drm_dev_unref(drm_dev);
	dev_set_drvdata(dev, NULL);
}

static const struct component_master_ops csky_drm_ops = {
	.bind = csky_drm_bind,
	.unbind = csky_drm_unbind,
};

static int csky_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *np = dev->of_node;
	struct device_node *port;
	int i;

	//drm_debug = 1;
	if (!np)
		return -ENODEV;
	/*
	 * Bind the crtc ports first, so that
	 * drm_of_find_possible_crtcs called from encoder .bind callbacks
	 * works as expected.
	 */
	for (i = 0;; i++) {
		port = of_parse_phandle(np, "ports", i);
		if (!port)
			break;

		if (!of_device_is_available(port->parent)) {
			of_node_put(port);
			continue;
		}

		component_match_add(dev, &match, compare_of, port->parent);
		of_node_put(port);
	}

	if (i == 0) {
		dev_err(dev, "missing 'ports' property\n");
		return -ENODEV;
	}

	if (!match) {
		dev_err(dev, "No available vop found for display-subsystem.\n");
		return -ENODEV;
	}
	/*
	 * For each bound crtc, bind the encoders attached to its
	 * remote endpoint.
	 */
	for (i = 0;; i++) {
		port = of_parse_phandle(np, "ports", i);
		if (!port)
			break;

		if (!of_device_is_available(port->parent)) {
			of_node_put(port);
			continue;
		}

		csky_add_endpoints(dev, &match, port);
		of_node_put(port);
	}

	return component_master_add_with_match(dev, &csky_drm_ops, match);
}

static int csky_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &csky_drm_ops);

	return 0;
}

static const struct of_device_id csky_drm_dt_ids[] = {
	{ .compatible = "csky,display-subsystem", },
	{},
};
MODULE_DEVICE_TABLE(of, csky_drm_dt_ids);

static struct platform_driver csky_drm_platform_driver = {
	.probe = csky_drm_platform_probe,
	.remove = csky_drm_platform_remove,
	.driver = {
		.name = "csky-drm",
		.of_match_table = csky_drm_dt_ids,
	},
};

module_platform_driver(csky_drm_platform_driver);

MODULE_AUTHOR("Huoqing Cai <huoqing_cai@c-sky.com>");
MODULE_DESCRIPTION("CSKY DRM DRIVER");
MODULE_LICENSE("GPL v2");

