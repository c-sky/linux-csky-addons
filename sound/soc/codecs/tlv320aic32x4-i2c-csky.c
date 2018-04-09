/*
 * sound/soc/codecs/tlv320aic32x4-i2c-csky.c
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * Based on sound/soc/codecs/tlv320aic32x4-i2c.c
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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "tlv320aic32x4-csky.h"

static int csky_aic32x4_i2c_probe(struct i2c_client *i2c,
				  const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct regmap_config config;

	config = csky_aic32x4_regmap_config;
	config.reg_bits = 8;
	config.val_bits = 8;

	regmap = devm_regmap_init_i2c(i2c, &config);
	return csky_aic32x4_probe(&i2c->dev, regmap);
}

static int csky_aic32x4_i2c_remove(struct i2c_client *i2c)
{
	return csky_aic32x4_remove(&i2c->dev);
}

static const struct i2c_device_id csky_aic32x4_i2c_id[] = {
	{ "tlv320aic32x4-csky", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, csky_aic32x4_i2c_id);

static const struct of_device_id csky_aic32x4_of_id[] = {
	{ .compatible = "ti,tlv320aic32x4-csky", },
	{ /* senitel */ }
};
MODULE_DEVICE_TABLE(of, csky_aic32x4_of_id);

static struct i2c_driver aic32x4_i2c_driver = {
	.driver = {
		.name = "tlv320aic32x4-csky",
		.of_match_table = csky_aic32x4_of_id,
	},
	.probe =    csky_aic32x4_i2c_probe,
	.remove =   csky_aic32x4_i2c_remove,
	.id_table = csky_aic32x4_i2c_id,
};

module_i2c_driver(aic32x4_i2c_driver);

MODULE_DESCRIPTION("ASoC TLV320AIC32x4 codec driver I2C");
MODULE_AUTHOR("Lei Ling <lei_ling@c-sky.com>");
MODULE_LICENSE("GPL v2");
