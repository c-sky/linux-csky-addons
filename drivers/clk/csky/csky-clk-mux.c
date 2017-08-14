/*
 * C-SKY SoCs Clock driver
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

static DEFINE_SPINLOCK(clk_mux_lock);

/* Maximum number of parents our clocks have */
#define CSKY_CLK_MAX_PARENTS	16

static void __init csky_clk_mux_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parents[CSKY_CLK_MAX_PARENTS];
	int num_parents;
	void __iomem *reg;
	u32 bit_shift;
	u32 bit_width;

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("Failed to map registers for clk: %s\n",
		       of_node_full_name(node));
		return;
	}

	num_parents = of_clk_parent_fill(node, parents, CSKY_CLK_MAX_PARENTS);

	if (of_property_read_string(node, "clock-output-names", &clk_name)) {
		pr_err("%s: failed to read clock-output-names from \"%s\"\n",
		       __func__, of_node_full_name(node));
		goto out_unmap;
	}

	if (of_property_read_u32(node, "bit-shift", &bit_shift) < 0) {
		pr_err("Failed to get property: bit-shift\n");
		goto out_unmap;
	}

	if (of_property_read_u32(node, "bit-width", &bit_width) < 0) {
		pr_err("Failed to get property: bit-width\n");
		goto out_unmap;
	}

	clk = clk_register_mux(NULL, clk_name,
			       parents, num_parents,
			       CLK_SET_RATE_PARENT,
			       reg, bit_shift, bit_width,
			       0, &clk_mux_lock);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clk %s: %ld\n", __func__,
		       clk_name, PTR_ERR(clk));
		goto out_unmap;
	}

	if (of_clk_add_provider(node, of_clk_src_simple_get, clk)) {
		pr_err("%s: failed to add clock provider for %s\n",
		       __func__, clk_name);
		clk_unregister_mux(clk);
		goto out_unmap;
	}

	return;
out_unmap:
	iounmap(reg);
	return;
}
CLK_OF_DECLARE(csky_clk_mux, "csky,clk-mux", csky_clk_mux_init);
