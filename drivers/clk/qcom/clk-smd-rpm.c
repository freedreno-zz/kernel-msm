/*
 * Copyright (c) 2015, Linaro Limited
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "clk-smd-rpm.h"

static int clk_smd_rpm_set_rate_active(struct clk_smd_rpm *r,
				       unsigned long value)
{
	struct clk_smd_rpm_req req = {
		.key = QCOM_RPM_SMD_KEY_RATE,
		.nbytes = sizeof(u32),
		.value = DIV_ROUND_UP(value, 1000), /* RPM expects KHz */
	};

	return qcom_rpm_smd_write(r->rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				  r->rpm_res_type, r->rpm_clk_id, &req,
				  sizeof(req));
}

static int clk_smd_rpm_prepare(struct clk_hw *hw)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);
	struct clk_smd_rpm *peer = r->peer;
	u32 value;
	int ret = 0;

	/* Don't send requests to the RPM if the rate has not been set. */
	if (!r->rate)
		goto out;

	/* Take peer clock's rate into account only if it's enabled. */
	if (peer->enabled)
		value = max(r->rate, peer->rate);
	else
		value = r->rate;

	if (r->branch)
		value = !!value;

	ret = clk_smd_rpm_set_rate_active(r, value);
	if (ret)
		goto out;

out:
	if (!ret)
		r->enabled = true;

	return ret;
}

static void clk_smd_rpm_unprepare(struct clk_hw *hw)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);

	if (r->rate) {
		struct clk_smd_rpm *peer = r->peer;
		unsigned long peer_rate;
		u32 value;
		int ret;

		/* Take peer clock's rate into account only if it's enabled. */
		peer_rate = peer->enabled ? peer->rate : 0;
		value = r->branch ? !!peer_rate : peer_rate;
		ret = clk_smd_rpm_set_rate_active(r, value);
		if (ret)
			return;
	}
	r->enabled = false;
}

static int clk_smd_rpm_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);
	int ret = 0;

	if (r->enabled) {
		u32 value;
		struct clk_smd_rpm *peer = r->peer;

		/* Take peer clock's rate into account only if it's enabled. */
		if (peer->enabled)
			value = max(rate, peer->rate);
		else
			value = rate;

		ret = clk_smd_rpm_set_rate_active(r, value);
		if (ret)
			goto out;
	}
	r->rate = rate;
out:
	return ret;
}

static long clk_smd_rpm_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *parent_rate)
{
	return rate;
}

static unsigned long clk_smd_rpm_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct clk_smd_rpm *r = to_clk_smd_rpm(hw);

	return r->rate;
}

int clk_smd_rpm_enable_scaling(struct qcom_smd_rpm *rpm)
{
	int ret;
	struct clk_smd_rpm_req req = {
		.key = QCOM_RPM_SMD_KEY_ENABLE,
		.nbytes = sizeof(u32),
		.value = 1,
	};

	ret = qcom_rpm_smd_write(rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				 QCOM_SMD_RPM_MISC_CLK,
				 QCOM_RPM_SCALING_ENABLE_ID, &req, sizeof(req));
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			pr_err("RPM clock scaling (active set) not enabled!\n");
		return ret;
	}

	pr_debug("%s: RPM clock scaling is enabled\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(clk_smd_rpm_enable_scaling);

const struct clk_ops clk_smd_rpm_ops = {
	.prepare = clk_smd_rpm_prepare,
	.unprepare = clk_smd_rpm_unprepare,
	.set_rate = clk_smd_rpm_set_rate,
	.round_rate = clk_smd_rpm_round_rate,
	.recalc_rate = clk_smd_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_smd_rpm_ops);

const struct clk_ops clk_smd_rpm_branch_ops = {
	.prepare = clk_smd_rpm_prepare,
	.unprepare = clk_smd_rpm_unprepare,
	.round_rate = clk_smd_rpm_round_rate,
	.recalc_rate = clk_smd_rpm_recalc_rate,
};
EXPORT_SYMBOL_GPL(clk_smd_rpm_branch_ops);
