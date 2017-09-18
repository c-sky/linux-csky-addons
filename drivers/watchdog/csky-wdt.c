/*
 * watchdog driver for C-SKY's SoCs.
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

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "csky-wdt.h"

static int csky_wdt_calc_period(struct csky_wdt_priv *priv)
{
	int i;
	unsigned long counters;
	u32 period = 0;
	struct watchdog_device *wdd;

	wdd = &priv->wdd;
	counters = priv->wdt_cnts >> 16;

	/* transfer counters to period */
	for (i = 0; i < 16; ++i) {
		if (counters < 2) {
			period = i;
			break;
		}
		counters = counters >> 1;
	}

	/* max period is 15 and the period round up */
	priv->wdt_period = period;

	return 0;
}

static int csky_wdt_feed(struct watchdog_device *wdd)
{
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);

	/* WDT counter restart */
	iowrite32(WDTCNF_CCR_EN, priv->iobase + WDT_CRR);
	iowrite32(WDTCNF_CR_RMOD_INT |
		  ioread32(priv->iobase + WDT_CR),
		  priv->iobase + WDT_CR);

	return 0;
}

static unsigned int csky_wdt_gettimeleft(struct watchdog_device *wdd)
{
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);
	unsigned long counters;

	counters = ioread32(priv->iobase + WDT_CCVR);

	return DIV_ROUND_CLOSEST(counters, priv->wdt_freq);
}


static int csky_wdt_updatetimeout(struct csky_wdt_priv *priv)
{
	csky_wdt_calc_period(priv);
	iowrite32(priv->wdt_period, priv->iobase + WDT_TORR);

	return 0;
}

static int csky_wdt_enable(struct watchdog_device *wdd)
{
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);

	iowrite32(WDTCNF_TORR_DEFAULT, priv->iobase + WDT_TORR);
	iowrite32(WDTCNF_CR_EN |
		  ioread32(priv->iobase + WDT_CR),
		  priv->iobase + WDT_CR);
	iowrite32(WDTCNF_CCR_EN, priv->iobase + WDT_CRR);

	return 0;
}

static int csky_wdt_restart(struct watchdog_device *wdd,
			    unsigned long action, void *data)
{
	u32 dly;
	unsigned long counters;
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);

	/* set the shortest time to restart system */
	iowrite32(0x0, priv->iobase + WDT_TORR);
	iowrite32(WDTCNF_CCR_EN, priv->iobase + WDT_CRR);
	/* delay the left time of reset */
	counters = ioread32(priv->iobase + WDT_CCVR);
	dly = (counters / priv->wdt_freq + 1) * 100;
	mdelay(dly);

	return 0;
}

static int csky_wdt_disable(struct watchdog_device *wdd)
{
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);

	/*
	 * disable wdt controler
	 * Once this bit has been enabled,
	 * it can only be cleared by a system reset.
	 */
	iowrite32(WDTCNF_CR_DIS &
		  ioread32(priv->iobase + WDT_CR),
		  priv->iobase + WDT_CR);

	return 0;
}

static int csky_wdt_settimeout(struct watchdog_device *wdd, unsigned int to)
{
	struct csky_wdt_priv *priv = watchdog_get_drvdata(wdd);

	wdd->timeout = to;
	priv->wdt_cnts = wdd->timeout * priv->wdt_freq;
	if (priv->wdt_cnts > WDT_MAX_COUNTS) {
		dev_err(priv->dev, "timeout %d too big\n", wdd->timeout);
		return -EINVAL;
	}
	csky_wdt_updatetimeout(priv);
	csky_wdt_feed(wdd);

	return 0;
}

static irqreturn_t csky_wdt_irq(int irq, void *devid)
{
	u32 clr_intr;
	struct csky_wdt_priv *priv = devid;

	/* read-clear type interupt */
	clr_intr = ioread32(priv->iobase + WDT_EOI);
	iowrite32(WDTCNF_CR_RMOD_RST &
		  ioread32(priv->iobase + WDT_CR),
		  priv->iobase + WDT_CR);

	return IRQ_HANDLED;
}

static const struct watchdog_info csky_wdt_ident = {
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
	.identity	= "csky watchdog",
};

static struct watchdog_ops csky_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= csky_wdt_enable,
	.stop		= csky_wdt_disable,
	.get_timeleft	= csky_wdt_gettimeleft,
	.ping		= csky_wdt_feed,
	.set_timeout	= csky_wdt_settimeout,
	.restart	= csky_wdt_restart,
};

static int csky_wdt_probe(struct platform_device *pdev)
{
	struct csky_wdt_priv *priv;
	struct watchdog_device *wdd;
	struct resource *res;
	unsigned long clk;
	int ret;

	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct csky_wdt_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->iobase))
		return PTR_ERR(priv->iobase);

	priv->clk_apb = devm_clk_get(priv->dev, NULL);
	if (IS_ERR(priv->clk_apb)) {
		dev_err(priv->dev, "Failed to get \"apb\" clk\n");
		return PTR_ERR(priv->clk_apb);
	}

	clk = clk_get_rate(priv->clk_apb);
	if (!clk) {
		dev_err(priv->dev, "Failed, apb clk is 0!\n");
		return -EINVAL;
	}

	priv->wdt_freq = clk;
	wdd = &priv->wdd;
	wdd->info = &csky_wdt_ident;
	wdd->ops = &csky_wdt_ops;
	wdd->parent = &pdev->dev;

	watchdog_set_drvdata(wdd, priv);

	priv->irq = platform_get_irq(pdev, 0);

	if (priv->irq > 0) {
		/*
		 * Not all supported platforms specify an interrupt for the
		 * watchdog, so let's make it optional.
		 */
		ret = devm_request_irq(&pdev->dev, priv->irq,
				       csky_wdt_irq, 0, pdev->name, priv);
		if (ret < 0)
			dev_warn(&pdev->dev, "failed to request IRQ\n");
	}

	ret = watchdog_register_device(wdd);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int csky_wdt_remove(struct platform_device *pdev)
{
	struct csky_wdt_priv *priv = platform_get_drvdata(pdev);

	csky_wdt_disable(&priv->wdd);

	watchdog_unregister_device(&priv->wdd);

	return 0;
}

static const struct of_device_id csky_wdt_of_match[] = {
	{ .compatible = "csky,eragon-wdt"},
	{},
};

MODULE_DEVICE_TABLE(of, csky_wdt_of_match);

static struct platform_driver csky_wdt_driver = {
	.probe	= csky_wdt_probe,
	.remove	= csky_wdt_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = csky_wdt_of_match,
	},
};

module_platform_driver(csky_wdt_driver);

MODULE_AUTHOR("Huoqing Cai <huoqing_cai@c-sky.com>");
MODULE_DESCRIPTION("CSKY Watchdog");
MODULE_LICENSE("GPL v2");
