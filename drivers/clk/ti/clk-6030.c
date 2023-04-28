/*
 * drivers/clk/ti/clk-6030.c
 *
 *  Copyright (C) 2014 Stefan Assmann <sassmann@kpanic.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Clock driver for ti twl6030.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h> 
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/mfd/twl.h>
#include <linux/platform_device.h>

struct twl6030_desc {
	struct clk *clk;
	struct clk_hw hw;
	bool enabled;
};

#define to_twl6030_desc(_hw) container_of(_hw, struct twl6030_desc, hw)

#define TWL6030_GRP_APP			(1 << 0)
#define TWL6030_GRP_CON			(1 << 1)
#define TWL6030_GRP_MOD			(1 << 2)
#define TWL6030_CFG_STATE_OFF		0x00
#define TWL6030_CFG_STATE_ON		0x01
#define TWL6030_CFG_STATE_OFF2		0x02
#define TWL6030_CFG_STATE_SLEEP		0x03
#define TWL6030_CFG_STATE_GRP_SHIFT	5
#define TWL6030_CFG_STATE_APP_SHIFT	2
#define TWL6030_CFG_STATE_APP_MASK	(0x03 << TWL6030_CFG_STATE_APP_SHIFT)
#define TWL6030_CFG_STATE_APP(v)	(((v) & TWL6030_CFG_STATE_APP_MASK) >>\
						TWL6030_CFG_STATE_APP_SHIFT)

#define TWL6030_PM_RECEIVER_CLK32KAUDIO_CFG_STATE	0xc1

static int twl6030_clk32kaudio_enable(struct clk_hw *hw)
{
	struct twl6030_desc *desc = to_twl6030_desc(hw);
	int ret;

	ret = twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER,
			       TWL6030_GRP_CON << TWL6030_CFG_STATE_GRP_SHIFT |
			       TWL6030_CFG_STATE_ON,
			       TWL6030_PM_RECEIVER_CLK32KAUDIO_CFG_STATE);
	if (ret == 0)
		desc->enabled = true;

	return ret;
}
void twl6030_clk32kaudio_disable(struct clk_hw *hw)
{
	struct twl6030_desc *desc = to_twl6030_desc(hw);

	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER,
			 TWL6030_GRP_CON << TWL6030_CFG_STATE_GRP_SHIFT |
			 TWL6030_CFG_STATE_OFF,
			 TWL6030_PM_RECEIVER_CLK32KAUDIO_CFG_STATE);
	desc->enabled = false;
}

static int twl6030_clk32kaudio_is_enabled(struct clk_hw *hw)
{
	struct twl6030_desc *desc = to_twl6030_desc(hw);

	return desc->enabled;
}

static const struct clk_ops twl6030_clk32kaudio_ops = {
	.enable		= twl6030_clk32kaudio_enable,
	.disable	= twl6030_clk32kaudio_disable,
	.is_enabled	= twl6030_clk32kaudio_is_enabled,
};

static void __init of_ti_twl6030_clk32kaudio_setup(struct device_node *node)
{
	struct twl6030_desc *clk_hw = NULL;
	struct clk_init_data init = { 0 };
	struct clk_lookup *clookup;
	struct clk *clk;

	clookup = kzalloc(sizeof(*clookup), GFP_KERNEL);
	if (!clookup) {
		pr_err("%s: could not allocate clookup\n", __func__);
		return;
	}
	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if (!clk_hw) {
		pr_err("%s: could not allocate clk_hw\n", __func__);
		goto err_clk_hw;
	}

	clk_hw->hw.init = &init;

	init.name = node->name;
	init.ops = &twl6030_clk32kaudio_ops;
	//init.flags = CLK_IS_ROOT;

	clk = clk_register(NULL, &clk_hw->hw);
	if (!IS_ERR(clk)) {
		clookup->con_id = kstrdup("clk32kaudio", GFP_KERNEL);
		clookup->clk = clk;
		clkdev_add(clookup);

		return;
	}

	kfree(clookup);
err_clk_hw:
	kfree(clk_hw);
}
CLK_OF_DECLARE(of_ti_twl6030_clk32kaudio, "ti,twl6030-clk32kaudio", of_ti_twl6030_clk32kaudio_setup);

static int of_twl6030_clk32kaudio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk *clk;
	int ret = 0;

	if (!node)
		return -ENODEV;

	clk = clk_get(&pdev->dev, "clk32kaudio");
	if (IS_ERR(clk))
		ret = -EPROBE_DEFER;
	else
		clk_prepare_enable(clk);

	return ret;
}

static int of_twl6030_clk32kaudio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id of_twl6030_clk32kaudio_match_tbl[] = {
	{ .compatible = "ti,twl6030-clk32kaudio", },
	{},
};
MODULE_DEVICE_TABLE(of, of_twl6030_clk32kaudio_match_tbl);

static struct platform_driver twl6030_clk_driver = {
	.driver = {
		.name = "twl6030-clk32kaudio",
		.owner = THIS_MODULE,
		.of_match_table = of_twl6030_clk32kaudio_match_tbl,
	},
	.probe = of_twl6030_clk32kaudio_probe,
	.remove = of_twl6030_clk32kaudio_remove,
};
module_platform_driver(twl6030_clk_driver);

MODULE_AUTHOR("Stefan Assmann <sassmann@kpanic.de>");
MODULE_DESCRIPTION("clock driver for TI SoC based boards with twl6030");
MODULE_LICENSE("GPL");
