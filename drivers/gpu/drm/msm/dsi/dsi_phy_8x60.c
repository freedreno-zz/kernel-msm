/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "dsi.h"

struct dsi_phy_8x60 {
	struct dsi_phy base;
	struct dsi *dsi;
};
#define to_dsi_phy_8x60(x) container_of(x, struct dsi_phy_8x60, base)

static void dsi_phy_8x60_destroy(struct dsi_phy *phy)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	kfree(phy_8x60);
}

static void do_calibration(struct dsi *dsi)
{
	uint32_t cnt = 5000;

	dsi_write(dsi, REG_DSI_8x60_PHY_CAL_CTRL, 0x0000ff11);
	dsi_write(dsi, REG_DSI_8x60_PHY_CAL_HW_TRIGGER, 1);

	/* wait for ready: */
	do {
		uint32_t status = dsi_read(dsi, REG_DSI_8x60_PHY_CAL_STATUS);
		if (!(status & DSI_8x60_PHY_CAL_STATUS_CAL_BUSY))
			break;
	} while (--cnt > 0);
}

// XXX see mipi_dsi_phy_init()
static void dsi_phy_8x60_config(struct dsi_phy *phy,
		const struct mipi_panel_config *pcfg)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	struct dsi *dsi = phy_8x60->dsi;

	dsi_write(dsi, REG_DSI_PHY_RESET, 1);
	wmb();
	usleep(1);
	dsi_write(dsi, REG_DSI_PHY_RESET, 0);
	wmb();
	usleep(1);

	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_0, 0x0003);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_1, 0x0001);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_2, 0x0001);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_3, 0x0000);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_4, 0x0100);

	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_0, pcfg->phy.regulator[0]);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_1, pcfg->phy.regulator[1]);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_2, pcfg->phy.regulator[2]);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_3, pcfg->phy.regulator[3]);

	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_0,  pcfg->phy.timing[0]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_1,  pcfg->phy.timing[1]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_2,  pcfg->phy.timing[2]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_3,  pcfg->phy.timing[3]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_4,  pcfg->phy.timing[4]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_5,  pcfg->phy.timing[5]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_6,  pcfg->phy.timing[6]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_7,  pcfg->phy.timing[7]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_8,  pcfg->phy.timing[8]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_9,  pcfg->phy.timing[9]);
	dsi_write(dsi, REG_DSI_8x60_PHY_TIMING_CTRL_10, pcfg->phy.timing[10]);

	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_0, pcfg->phy.ctrl[0]);
	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_1, pcfg->phy.ctrl[1]);
	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_2, pcfg->phy.ctrl[2]);
	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_3, pcfg->phy.ctrl[3]);

	dsi_write(dsi, REG_DSI_8x60_PHY_STRENGTH_0, pcfg->phy.strength[0]);
	dsi_write(dsi, REG_DSI_8x60_PHY_STRENGTH_1, pcfg->phy.strength[1]);
	dsi_write(dsi, REG_DSI_8x60_PHY_STRENGTH_2, pcfg->phy.strength[2]);
	dsi_write(dsi, REG_DSI_8x60_PHY_STRENGTH_3, pcfg->phy.strength[3]);

	do_calibration(dsi);

	/* NOTE: msm_dss_io_8960.c doesn't set this one.. not sure if that
	 * is an oversight:
	 */
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_20, pcfg->phy.pll[20]);
	dsi_pll_config(dsi, pcfg);
}

static void dsi_phy_8x60_powerup(struct dsi_phy *phy,
		unsigned long int pixclock)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	struct dsi *dsi = phy_8x60->dsi;

	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_5, 0x50);
	dsi_write(dsi, REG_DSI_8x60_PHY_TPA_CTRL_1, 0x00f);
	dsi_write(dsi, REG_DSI_8x60_PHY_TPA_CTRL_2, 0x000);
}

static void dsi_phy_8x60_powerdown(struct dsi_phy *phy)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	struct dsi *dsi = phy_8x60->dsi;

	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_5, 0x5f);
	dsi_write(dsi, REG_DSI_8x60_PHY_TPA_CTRL_1, 0x08f);
	dsi_write(dsi, REG_DSI_8x60_PHY_TPA_CTRL_2, 0x001);
	dsi_write(dsi, REG_DSI_8x60_PHY_REGULATOR_CTRL_0, 0x02);
	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_0, 0x00);
	dsi_write(dsi, REG_DSI_8x60_PHY_CTRL_1, 0x7f);

	/* disable dsi clk */
	dsi_write(dsi, REG_DSI_CLK_CTRL, 0);
}

static void dsi_phy_8x60_clk_enable(struct dsi_phy *phy)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	struct dsi *dsi = phy_8x60->dsi;
	uint32_t pll_ctrl;

	pll_ctrl = dsi_read(dsi, REG_DSI_PHY_PLL_CTRL_0);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_0,
			pll_ctrl | DSI_PHY_PLL_CTRL_0_ENABLE);
	mb();

	if (clk_set_rate(dsi->byte_div_clk, 1) < 0)	/* divided by 1 */
		dev_err(dsi->dev->dev, "failed to set byte_div_clk\n");

	dsi_clk_enable_pclk(dsi);
	dsi_clk_enable_clk(dsi);

	clk_enable(dsi->byte_div_clk);
	clk_enable(dsi->esc_clk);
}

static void dsi_phy_8x60_clk_disable(struct dsi_phy *phy)
{
	struct dsi_phy_8x60 *phy_8x60 = to_dsi_phy_8x60(phy);
	struct dsi *dsi = phy_8x60->dsi;
	uint32_t pll_ctrl;

	clk_disable(dsi->esc_clk);
	clk_disable(dsi->byte_div_clk);

	dsi_clk_disable_clk(dsi);
	dsi_clk_disable_pclk(dsi);

	pll_ctrl = dsi_read(dsi, REG_DSI_PHY_PLL_CTRL_0);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_0,
			pll_ctrl & ~DSI_PHY_PLL_CTRL_0_ENABLE);
}

static const struct dsi_phy_funcs dsi_phy_8x60_funcs = {
		.destroy = dsi_phy_8x60_destroy,
		.config = dsi_phy_8x60_config,
		.powerup = dsi_phy_8x60_powerup,
		.powerdown = dsi_phy_8x60_powerdown,
		.clk_enable = dsi_phy_8x60_clk_enable,
		.clk_disable = dsi_phy_8x60_clk_disable,
};

struct dsi_phy *dsi_phy_8x60_init(struct dsi *dsi)
{
	struct dsi_phy_8x60 *phy_8x60;
	struct dsi_phy *phy = NULL;
	int ret;

	phy_8x60 = kzalloc(sizeof(*phy_8x60), GFP_KERNEL);
	if (!phy_8x60) {
		ret = -ENOMEM;
		goto fail;
	}

	phy = &phy_8x60->base;

	phy->funcs = &dsi_phy_8x60_funcs;

	phy_8x60->dsi = dsi;

	return phy;

fail:
	if (phy)
		dsi_phy_8x60_destroy(phy);
	return ERR_PTR(ret);
}
