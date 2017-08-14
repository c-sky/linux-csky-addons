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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

static DEFINE_SPINLOCK(clk_gate_lock);

static void unregister_clk_gate(struct clk_onecell_data *clk_data)
{
	unsigned int i;

	if ((clk_data == NULL) || (clk_data->clks == NULL))
		return;

	for (i = 0; i <= clk_data->clk_num; i++) {
		if (clk_data->clks[i] != NULL) {
			clk_unregister_gate(clk_data->clks[i]);
			clk_data->clks[i] = NULL;
		}
	}
}

static void __init csky_clk_gate_init(struct device_node *node)
{
	struct clk_onecell_data *clk_data;
	int clk_num;
	const char *clk_parent;
	const char *clk_name;
	void __iomem *reg;
	void __iomem *clk_reg;
	u8 clk_bit;
	struct property *prop;
	const __be32 *p;
	u32 index;
	int i = 0;
	struct resource res;

	reg = of_io_request_and_map(node, 0, of_node_full_name(node));
	if (IS_ERR(reg)) {
		pr_err("Failed to map registers for clk: %s\n",
		       of_node_full_name(node));
		return;
	}

	clk_data = kzalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
	if (!clk_data) {
		pr_err("Failed to allocate memory\n");
		goto err_unmap;
	}

	clk_num = of_property_count_u32_elems(node, "clock-indices");
	of_property_read_u32_index(node, "clock-indices",
				   clk_num - 1, &clk_num);
	clk_num++;

	clk_data->clk_num = clk_num;
	clk_data->clks = kzalloc(clk_num * sizeof(struct clk *), GFP_KERNEL);
	if (!clk_data->clks) {
		pr_err("Failed to allocate clk\n");
		goto err_free_data;
	}

	clk_parent = of_clk_get_parent_name(node, 0);

	of_property_for_each_u32(node, "clock-indices", prop, p, index) {
		of_property_read_string_index(node, "clock-output-names",
					      i, &clk_name);
		i++;

		clk_reg = reg + 4 * (index / 32);
		clk_bit = index % 32;

		clk_data->clks[index] = clk_register_gate(NULL, clk_name,
							  clk_parent, 0,
							  clk_reg, clk_bit,
							  0, &clk_gate_lock);
		if (IS_ERR(clk_data->clks[index])) {
			pr_err("Failed to register clk: %s\n", clk_name);
			goto err_unregister_clk;
		}
	}

	if (of_clk_add_provider(node, of_clk_src_onecell_get, clk_data)) {
		pr_err("Failed to add clock provider for gate\n");
		goto err_unregister_clk;
	}

	return;

err_unregister_clk:
	unregister_clk_gate(clk_data);
err_free_data:
	if (clk_data->clks != NULL)
		kfree(clk_data->clks);
	kfree(clk_data);
err_unmap:
	iounmap(reg);
	of_address_to_resource(node, 0, &res);
	release_mem_region(res.start, resource_size(&res));
}

CLK_OF_DECLARE(csky_clk_gate, "csky,clk-gate", csky_clk_gate_init);
