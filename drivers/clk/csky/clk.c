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
#include "clk.h"

/**
 * csky_of_clk_get_by_phandle() - Find a clk pointed by phandle in a list
 * @np:		pointer to a device tree node containing a phandle list
 * @list_name:	property name that contains a phandle list
 * @index:	index of a phandle
 *
 * Returns clk pointer on success, or appropriate errno value on error.
 *
 * Note: clk_put() should be called when the clk pointer is no longer used.
 */
struct clk *csky_of_clk_get_by_phandle(struct device_node *np,
				       const char *list_name, int index)
{
	struct of_phandle_args clkspec;
	struct clk *clk;
	int rc;

	if (index < 0)
		return ERR_PTR(-EINVAL);

	rc = of_parse_phandle_with_args(np, list_name, "#clock-cells", index,
					&clkspec);
	if (rc)
		return ERR_PTR(rc);

	clk = of_clk_get_from_provider(&clkspec);
	of_node_put(clkspec.np);

	return clk;
}
EXPORT_SYMBOL(csky_of_clk_get_by_phandle);
