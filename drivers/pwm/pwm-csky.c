/*
 * pwm driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Minfeng Zhang <minfeng_zhang@c-sky.com>
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#define NUM_PWM			3
#define PWM_MIN_PRESCALE	0
#define PWM_MAX_PRESCALE	0x3FFF
#define PWM_MIN_PERIOD		0x0001
#define PWM_MAX_PERIOD		0xFFFF
#define PWM_CLK_RATE		60000000
#define PWM_PWM_ENABLE		1
#define PWM_MIN_DUTY		0x0001
#define PWM_MAX_DUTY		0xFFFF

/* PWM registers and bits definitions */
#define PWMCFG		0x0
#define PWMCTL		0x34
#define PWM01LOAD	0x38
#define PWM23LOAD	0x3c
#define PWM45LOAD	0x40
#define PWM0CMP		0x50
#define PWM1CMP		0x54
#define PWM2CMP		0x58
#define PWM3CMP		0x5c
#define PWM4CMP		0x60
#define PWM5CMP		0x64
#define PWM01DB		0x68
#define PWM23DB		0x6c
#define PWM45DB		0x70


/**
 * struct spear_pwm_chip - struct representing pwm chip
 *
 * @base: base address of pwm chip
 * @clk: pointer to clk structure of pwm chip
 * @chip: linux pwm chip representation
 */
struct csky_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
	int	period;
	int	duty;
};

static inline struct csky_pwm_chip *to_csky_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct csky_pwm_chip, chip);
}

/*
 * period_ns = 10^9 * period_cycles / PWM_CLK_RATE
 * duty_ns   = 10^9 * duty_cycles / PWM_CLK_RATE
 */
static int csky_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	struct csky_pwm_chip *pc = to_csky_pwm_chip(chip);
	u64 val, div, clk_rate;
	unsigned long prescale = PWM_MIN_PRESCALE, pv, dc;
	int ret;

	if (pc->period == 0 || pc->duty == 0) {
		if (duty_ns != 0)
			pc->duty = duty_ns;
		if (period_ns != 0)
			pc->period = period_ns;
		if (pc->duty == 0 || pc->period == 0) {
			return 0;
		}
	}
	/*
	 * Find pv, dc and prescale to suit duty_ns and period_ns. This is done
	 * according to formulas described below:
	 *
	 * period_ns = 10^9 * (PRESCALE + 1) * PV / PWM_CLK_RATE
	 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
	 *
	 * PV = (PWM_CLK_RATE * period_ns) / (10^9 * (PRESCALE + 1))
	 * DC = (PWM_CLK_RATE * duty_ns) / (10^9 * (PRESCALE + 1))
	 */
	clk_rate = clk_get_rate(pc->clk);
	while (1) {
		div = 1000000000;
		div *= 1 + prescale;
		val = clk_rate * period_ns;
		pv = div64_u64(val, div);
		val = clk_rate * duty_ns;
		dc = div64_u64(val, div);

		/* if duty_ns and period_ns are not achievable then return */
		if (pv < PWM_MIN_PERIOD || dc < PWM_MIN_DUTY)
			return -EINVAL;

		/*
		 * if pv and dc have crossed their upper limit, then increase
		 * prescale and recalculate pv and dc.
		 */
		if (pv > PWM_MAX_PERIOD || dc > PWM_MAX_DUTY) {
			if (++prescale > PWM_MAX_PRESCALE)
				return -EINVAL;
			continue;
		}
		break;
	}

	ret = clk_enable(pc->clk);
	if (ret)
		return ret;

	writel_relaxed(0x0, pc->base + PWMCTL);
	/* pwm->hwpwm is hardware pwm number */
	switch(pwm->hwpwm) {
	case 0:
		writel_relaxed(pv, pc->base + PWM01LOAD);
		writel_relaxed(dc, pc->base + PWM0CMP);
		writel_relaxed(0x0, pc->base + PWM01DB);
		break;
	case 1:
		writel_relaxed(pv << 16, pc->base + PWM01LOAD);
		writel_relaxed(dc, pc->base + PWM1CMP);
		writel_relaxed(0x0, pc->base + PWM01DB);
		break;
	case 2:
		writel_relaxed(pv, pc->base + PWM23LOAD);
		writel_relaxed(dc, pc->base + PWM2CMP);
		writel_relaxed(0x0, pc->base + PWM23DB);
		break;
	default:
		return -EINVAL;
	}

	pc->period = 0;
	pc->duty = 0;
	return 0;
}

static int csky_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct csky_pwm_chip *pc = to_csky_pwm_chip(chip);
	u32 val;

	clk_enable(pc->clk);
	val = readl_relaxed(pc->base + PWMCFG);
	val |= PWM_PWM_ENABLE << (pwm->hwpwm) * 2;
	writel_relaxed(val, pc->base + PWMCFG);

	return 0;
}


static void csky_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct csky_pwm_chip *pc = to_csky_pwm_chip(chip);
	u32 val;

	val = readl_relaxed(pc->base + PWMCFG);
	val &= ~PWM_PWM_ENABLE << (pwm->hwpwm) * 2;
	writel_relaxed(val, pc->base + PWMCFG);

	clk_disable(pc->clk);
}

static struct pwm_ops csky_pwm_ops = {
	.enable = csky_pwm_enable,
	.disable = csky_pwm_disable,
	.config = csky_pwm_config,
	.owner = THIS_MODULE,
};

static int csky_pwm_probe(struct platform_device *pdev)
{
	struct csky_pwm_chip *pc;
	struct device *dev = &pdev->dev;
	struct resource *r;
	int ret;
	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pc->base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	platform_set_drvdata(pdev, pc);

	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &csky_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = NUM_PWM;
	pc->duty = 0;
	pc->period = 0;

	pc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pc->clk)) {
		dev_err(dev, "failed to get PWM clock\n");
		return PTR_ERR(pc->clk);
	}

	ret = clk_prepare(pc->clk);
	if (ret) {
		dev_err(dev, "failed to prepare clock\n");
		return ret;
	}

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		clk_unprepare(pc->clk);
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
	}

	return ret;
}

static int csky_pwm_remove(struct platform_device *pdev)
{
	struct csky_pwm_chip *pc = platform_get_drvdata(pdev);
	int i;

	for(i = 0;i < NUM_PWM; i++)
		pwm_disable(&pc->chip.pwms[i]);

	/* clk was prepared in probe, hence unprepare it here */
	clk_unprepare(pc->clk);
	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id csky_pwm_dt_ids[] = {
	{ .compatible = "csky,pwm-v1", },
	{}
};
MODULE_DEVICE_TABLE(of, csky_pwm_dt_ids);

static struct platform_driver csky_pwm_driver = {
	.probe		= csky_pwm_probe,
	.remove		= csky_pwm_remove,
	.driver		= {
		.name	= "csky-pwm",
		.of_match_table = csky_pwm_dt_ids,
	},
};
module_platform_driver(csky_pwm_driver);

MODULE_DESCRIPTION("CSKY PWM Driver");
MODULE_AUTHOR("Minfeng Zhang <minfeng_zhang@c-sky.com>");
MODULE_LICENSE("GPL");
