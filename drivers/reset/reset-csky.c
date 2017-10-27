/*
 * C-SKY SoCs Reset Controller driver
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct csky_reset_data {
	spinlock_t			lock;
	void __iomem			*membase;
	struct reset_controller_dev	rcdev;
};

static int csky_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct csky_reset_data *data = container_of(rcdev,
						    struct csky_reset_data,
						    rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&data->lock, flags);

	reg = readl(data->membase + (bank * 4));
	writel(reg & ~BIT(offset), data->membase + (bank * 4));

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int csky_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct csky_reset_data *data = container_of(rcdev,
						    struct csky_reset_data,
						    rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&data->lock, flags);

	reg = readl(data->membase + (bank * 4));
	writel(reg | BIT(offset), data->membase + (bank * 4));

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static const struct reset_control_ops csky_reset_ops = {
	.assert		= csky_reset_assert,
	.deassert	= csky_reset_deassert,
};

static const struct of_device_id csky_reset_dt_ids[] = {
	{ .compatible = "csky,reset-v1", },
	{ }
};
MODULE_DEVICE_TABLE(of, csky_reset_dt_ids);

static int csky_reset_probe(struct platform_device *pdev)
{
	struct csky_reset_data *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->membase))
		return PTR_ERR(data->membase);

	spin_lock_init(&data->lock);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = resource_size(res) * 8;
	data->rcdev.ops = &csky_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	return devm_reset_controller_register(&pdev->dev, &data->rcdev);
}

static struct platform_driver csky_reset_driver = {
	.probe	= csky_reset_probe,
	.driver = {
		.name		= "csky-reset",
		.of_match_table	= csky_reset_dt_ids,
	},
};

static int __init csky_reset_driver_register(void)
{
	return platform_driver_register(&csky_reset_driver);
}
postcore_initcall(csky_reset_driver_register);

MODULE_DESCRIPTION("C-SKY SoCs Reset Controller Driver");
MODULE_AUTHOR("Lei Ling <lei_ling@c-sky.com>");
MODULE_LICENSE("GPL v2");