/*
 * pin-controller/pin-mux/pin-config/gpio-driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 Lu Chongzhi <chongzhi_lu@c-sky.com>.
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/clk.h>
#include <linux/regmap.h>

extern void eragon_pinctrl_init(void);

static int csky_pinctrl_probe(struct platform_device *pdev)
{
	// eragon_pinctrl_init();
	pr_info("csky_pinctrl_probe\n");
	return 0;
}

static const struct of_device_id csky_pinctrl_dt_match[] = {
	{ .compatible = "csky,eragon-pinctrl",
		.data = (void *)&eragon_pinctrl_init },
	{},
};

static struct platform_driver csky_pinctrl_driver = {
	.probe  = csky_pinctrl_probe,
	.driver = {
		.name	= "csky-pinctrl",
		.of_match_table = csky_pinctrl_dt_match,
	},
};

static int __init csky_pinctrl_init(void)
{
	return platform_driver_register(&csky_pinctrl_driver);
}
device_initcall(csky_pinctrl_init);

