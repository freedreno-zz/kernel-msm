/*
 * Copyright (C) 2015 AngeloGioacchino Del Regno <kholk11@gmail.com>
 *
 * May contain portions of code (C) The Linux Foundation
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

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/qcom,rpmcc-msm8974.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "reset.h"
#include "gdsc.h"


static const struct qcom_cc_desc rpmcc_msm8974_desc = {
	.config =
	.clks
	.num_clks
	.resets
	.num_resets
	.gdscs
	.num_gdscs
};

static const struct of_device_id rpmcc_msm8974_match_table[] = {
	{ .compatible = "qcom,rpmcc-msm8974" },
	{ }
};
MODULE_DEVICE_TABLE(of, rpmcc_msm8974_match_table);

static int rpmcc_msm8974_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device *dev = &pdev->dev;

	id = of_match_device(rpmcc_msm8974_match_table, dev);
	if (!id)
		return -ENODEV;

	return qcom_cc_probe(pdev, &rpmcc_msm8974_desc);
}

static int rpmcc_msm8974_remove(struct platform_device *pdev)
{
	qcom_cc_remove(pdev);
	return 0;
}

static struct platform_driver rpmcc_msm8974_driver = {
	.probe		= rpmcc_msm8974_probe,
	.remove		= rpmcc_msm8974_remove,
	.driver		= {
		.name	= "rpmcc-msm8974",
		.of_match_table = rpmcc_msm8974_match_table,
	},
};

static int __init rpmcc_msm8974_init(void)
{
	return platform_driver_register(&rpmcc_msm8974_driver);
}

static void __exit rpmcc_msm8974_exit(void)
{
	platform_driver_unregister(&rpmcc_msm8974_driver);
}

MODULE_DESCRIPTION("QCOM RPMCC MSM8974 Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rpmcc-msm8974");
