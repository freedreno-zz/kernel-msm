/*
 * Copyright (C) 2015, AngeloGioacchino Del Regno <kholk11@gmail.com>
 * Copyright (C) 2014, The Linux Foundation. All rights reserved.
 *
 * Based on drivers/clk/qcom/rpmcc-msm8916.c (C) 2015 Linaro Limited
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
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <dt-bindings/clock/qcom,rpmcc-msm8974.h>

#include "clk-rpm.h"

#define CXO_ID		0x0
#define QDSS_ID		0x1

#define PNOC_ID		0x0
#define SNOC_ID		0x1
#define CNOC_ID		0x2
#define MMSSNOC_AHB_ID  0x3

#define BIMC_ID		0x0
#define OXILI_ID	0x1
#define OCMEM_ID	0x2

#define D0_ID		 1
#define D1_ID		 2
#define A0_ID		 4
#define A1_ID		 5
#define A2_ID		 6
#define DIFF_CLK_ID	 7
#define DIV_CLK1_ID	11
#define DIV_CLK2_ID	12

struct rpm_cc {
	struct clk_onecell_data data;
	struct clk *clks[];
};

/* SMD Clocks */
DEFINE_CLK_RPM_SMD(pnoc_clk, pnoc_a_clk,
			QCOM_SMD_RPM_BUS_CLK, PNOC_ID, NULL);
DEFINE_CLK_RPM_SMD(snoc_clk, snoc_a_clk,
			QCOM_SMD_RPM_BUS_CLK, SNOC_ID, NULL);
DEFINE_CLK_RPM_SMD(cnoc_clk, cnoc_a_clk,
			QCOM_SMD_RPM_BUS_CLK, CNOC_ID, NULL);
DEFINE_CLK_RPM_SMD(mmssnoc_ahb_clk, mmssnoc_ahb_a_clk,
			QCOM_SMD_RPM_BUS_CLK, MMSSNOC_AHB_ID, NULL);
DEFINE_CLK_RPM_SMD(bimc_clk, bimc_a_clk,
			QCOM_SMD_RPM_MEM_CLK, BIMC_ID, NULL);
DEFINE_CLK_RPM_SMD(ocmemgx_clk, ocmemgx_a_clk,
			QCOM_SMD_RPM_MEM_CLK, OCMEM_ID, NULL);
DEFINE_CLK_RPM_SMD(gfx3d_clk_src, gfx3d_a_clk_src,
			QCOM_SMD_RPM_MEM_CLK, OXILI_ID, NULL);

DEFINE_CLK_RPM_SMD_BRANCH(cxo_clk_src, cxo_a_clk_src,
			QCOM_SMD_RPM_MISC_CLK, CXO_ID, 19200000);
DEFINE_CLK_RPM_SMD_QDSS(qdss_clk, qdss_a_clk,
			QCOM_SMD_RPM_MISC_CLK, QDSS_ID);

/* SMD XO BUFFER */
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_d0, cxo_d0_a, D0_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_d1, cxo_d1_a, D1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_a0, cxo_a0_a, A0_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_a1, cxo_a1_a, A1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_a2, cxo_a2_a, A2_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(div_clk1, div_a_clk1, DIV_CLK1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(div_clk2, div_a_clk2, DIV_CLK2_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(diff_clk, diff_a_clk, DIFF_CLK_ID);

DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_d0_pin, cxo_d0_a_pin, D0_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_d1_pin, cxo_d1_a_pin, D1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_a0_pin, cxo_a0_a_pin, A0_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_a1_pin, cxo_a1_a_pin, A1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_a2_pin, cxo_a2_a_pin, A2_ID);

static struct clk_rpm *rpmcc_msm8974_clks[] = {
	[RPM_CXO_CLK_SRC]	= &cxo_clk_src,
	[RPM_CXO_CLK_A_SRC]	= &cxo_a_clk_src,
	[RPM_CXO_D0]		= &cxo_d0,
	[RPM_CXO_D0_A]		= &cxo_d0_a,
	[RPM_CXO_D1]		= &cxo_d1,
	[RPM_CXO_D1_A]		= &cxo_d1_a,
	[RPM_CXO_A0]		= &cxo_a0,
	[RPM_CXO_A0_A]		= &cxo_a0_a,
	[RPM_CXO_A1]		= &cxo_a1,
	[RPM_CXO_A1_A]		= &cxo_a1_a,
	[RPM_CXO_A2]		= &cxo_a2,
	[RPM_CXO_A2_A]		= &cxo_a2_a,
	[RPM_CXO_D0_PIN]	= &cxo_d0_pin,
	[RPM_CXO_D0_A_PIN]	= &cxo_d0_a_pin,
	[RPM_CXO_D1_PIN]	= &cxo_d1_pin,
	[RPM_CXO_D1_A_PIN]	= &cxo_d1_a_pin,
	[RPM_CXO_A0_PIN]	= &cxo_a0_pin,
	[RPM_CXO_A0_A_PIN]	= &cxo_a0_a_pin,
	[RPM_CXO_A1_PIN]	= &cxo_a1_pin,
	[RPM_CXO_A1_A_PIN]	= &cxo_a1_a_pin,
	[RPM_CXO_A2_PIN]	= &cxo_a2_pin,
	[RPM_CXO_A2_A_PIN]	= &cxo_a2_a_pin,
	[RPM_PNOC_CLK]		= &pnoc_clk,
	[RPM_PNOC_A_CLK]	= &pnoc_a_clk,
	[RPM_SNOC_CLK]		= &snoc_clk,
	[RPM_SNOC_A_CLK]	= &snoc_a_clk,
	[RPM_CNOC_CLK]		= &cnoc_clk,
	[RPM_CNOC_A_CLK]	= &cnoc_a_clk,
	[RPM_MMSSNOC_AHB_CLK]	= &mmssnoc_ahb_clk,
	[RPM_MMSSNOC_AHB_A_CLK]	= &mmssnoc_ahb_a_clk,
	[RPM_BIMC_CLK]		= &bimc_clk,
	[RPM_BIMC_A_CLK]	= &bimc_a_clk,
	[RPM_OCMEMGX_CLK]	= &ocmemgx_clk,
	[RPM_OCMEMGX_A_CLK]	= &ocmemgx_a_clk,
/* TODO: Can't register this clock -- why?? */
/*	[RPM_GFX3D_CLK_SRC]	= &gfx3d_clk_src, */
	[RPM_GFX3D_A_CLK_SRC]	= &gfx3d_a_clk_src,
	[RPM_QDSS_CLK]		= &qdss_clk,
	[RPM_QDSS_A_CLK]	= &qdss_a_clk,
};

static const struct of_device_id rpmcc_msm8974_match_table[] = {
	{ .compatible = "qcom,rpmcc-msm8974" },
	{ }
};
MODULE_DEVICE_TABLE(of, rpmcc_msm8974_match_table);

static int rpmcc_msm8974_vote(struct device *dev)
{
	int ret;

	clk_set_rate(pnoc_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(pnoc_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(snoc_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(snoc_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(cnoc_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(cnoc_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(bimc_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(bimc_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(bimc_a_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(bimc_a_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(gfx3d_clk_src.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(gfx3d_clk_src.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(ocmemgx_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(ocmemgx_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(ocmemgx_a_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(ocmemgx_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(pnoc_a_clk.hw.clk, LONG_MAX);
	ret = clk_prepare_enable(pnoc_a_clk.hw.clk);
	if (ret)
		goto err;

	clk_set_rate(cxo_clk_src.hw.clk, 19200000);
	ret = clk_prepare_enable(cxo_clk_src.hw.clk);
	if (ret)
		goto err;

	ret = clk_prepare_enable(cxo_a_clk_src.hw.clk);
	if (ret)
		goto err;

err:
	return ret;
}

static int rpmcc_msm8974_probe(struct platform_device *pdev)
{
	struct clk **clks;
	struct clk *clk;
	struct rpm_cc *rcc;
	struct qcom_smd_rpm *rpm;
	struct clk_onecell_data *data;
	int num_clks = ARRAY_SIZE(rpmcc_msm8974_clks);
	int ret, i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "Unable to retrieve handle to RPM\n");
		return -EPROBE_DEFER;
	}

	ret = clk_rpm_enable_scaling(rpm);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable RPM scaling\n");
		return ret;
	}

	rcc = devm_kzalloc(&pdev->dev, sizeof(*rcc) + sizeof(*clks) * num_clks,
			   GFP_KERNEL);
	if (!rcc)
		return -ENOMEM;

	clks = rcc->clks;
	data = &rcc->data;
	data->clks = clks;
	data->clk_num = num_clks;

	clk = clk_register_fixed_rate(&pdev->dev, "sleep_clk_src", NULL,
				      CLK_IS_ROOT, 32768);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	for (i = 0; i < num_clks; i++) {
		if (!rpmcc_msm8974_clks[i]) {
			clks[i] = ERR_PTR(-ENOENT);
			continue;
		}

		rpmcc_msm8974_clks[i]->rpm = rpm;
		clk = devm_clk_register(&pdev->dev, &rpmcc_msm8974_clks[i]->hw);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		clks[i] = clk;
	}

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				  data);
	if (ret)
		return ret;

	ret = rpmcc_msm8974_vote(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot vote clocks.\n");
		return ret;
	}

	dev_info(&pdev->dev, "RPM Clock Controller registered.\n");

	return 0;
}

static int rpmcc_msm8974_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
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

module_platform_driver(rpmcc_msm8974_driver);
core_initcall(rpmcc_msm8974_driver_init);

MODULE_DESCRIPTION("RPM Clock Controller Driver for QCOM MSM8974");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rpmcc-msm8974");
