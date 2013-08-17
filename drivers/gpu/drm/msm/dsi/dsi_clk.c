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

static const struct dsi_clk_mnd mnd_table[] = {
	{ 1, 2, 8, 1, 1, 0, 1,  2, 1},
	{ 1, 3, 8, 1, 1, 0, 1,  3, 2},
	{ 2, 2, 4, 1, 1, 0, 1,  2, 1},
	{ 2, 3, 4, 1, 1, 0, 1,  3, 2},
	{ 3, 2, 1, 3, 8, 4, 3, 16, 8},
	{ 3, 3, 1, 3, 8, 4, 1,  8, 4},
	{ 4, 2, 2, 1, 1, 0, 1,  2, 1},
	{ 4, 3, 2, 1, 1, 0, 1,  3, 2},
};

const struct dsi_clk_mnd *dsi_find_clk_mnd(unsigned bpp, unsigned nlanes)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mnd_table); i++)
		if ((mnd_table[i].bpp == bpp) && (mnd_table[i].nlanes == nlanes))
			return &mnd_table[i];
	return NULL;
}

void dsi_clk_prepare(struct dsi *dsi)
{
	clk_prepare(dsi->amp_pclk);
	clk_prepare(dsi->m_pclk);
	clk_prepare(dsi->s_pclk);
	clk_prepare(dsi->byte_div_clk);
	clk_prepare(dsi->esc_clk);
}

void dsi_clk_unprepare(struct dsi *dsi)
{
	clk_unprepare(dsi->esc_clk);
	clk_unprepare(dsi->byte_div_clk);
	clk_unprepare(dsi->m_pclk);
	clk_unprepare(dsi->s_pclk);
	clk_unprepare(dsi->amp_pclk);
}

void dsi_clk_enable_ahb(struct dsi *dsi)
{
	uint32_t reg;

	clk_enable(dsi->amp_pclk); /* clock for AHB-master to AXI */
	clk_enable(dsi->m_pclk);
	clk_enable(dsi->s_pclk);

	reg = cc_read(dsi, REG_MMSS_CC_AHB);
	DBG("ahb: %08x", reg);

	reg = sfpb_read(dsi, REG_SFPB_CFG);
	sfpb_write(dsi, REG_SFPB_CFG, reg | 0x1800);
}

void dsi_clk_disable_ahb(struct dsi *dsi)
{
	clk_disable(dsi->m_pclk);
	clk_disable(dsi->s_pclk);
	clk_disable(dsi->amp_pclk); /* clock for AHB-master to AXI */
}

static void enable_clk(struct dsi *dsi, enum mmss_cc_clk clk,
		struct dsi_clk_desc *desc)
{
	/* TODO: this should eventually be hidden behind clk_set_rate().. */
	if (desc->mnd_mode == 0) {
		cc_write(dsi, REG_MMSS_CC_CLK_NS(clk),
				MMSS_CC_CLK_NS_SRC(desc->src) |
				MMSS_CC_CLK_NS_PRE_DIV_FUNC(desc->pre_div_func));
		cc_write(dsi, REG_MMSS_CC_CLK_CC(clk),
				MMSS_CC_CLK_CC_CLK_EN |
				MMSS_CC_CLK_CC_ROOT_EN |
				MMSS_CC_CLK_CC_MND_MODE(desc->mnd_mode) |
				MMSS_CC_CLK_CC_PMXO_SEL(0));
	} else {
		cc_write(dsi, REG_MMSS_CC_CLK_MD(clk),
				MMSS_CC_CLK_MD_D(~(2 * desc->d)) |
				MMSS_CC_CLK_MD_M(desc->m));
		cc_write(dsi, REG_MMSS_CC_CLK_NS(clk),
				MMSS_CC_CLK_NS_SRC(desc->src) |
				MMSS_CC_CLK_NS_VAL(~(desc->n - desc->m)));
		cc_write(dsi, REG_MMSS_CC_CLK_CC(clk),
				MMSS_CC_CLK_CC_CLK_EN |
				MMSS_CC_CLK_CC_ROOT_EN |
				MMSS_CC_CLK_CC_MND_EN |
				MMSS_CC_CLK_CC_MND_MODE(desc->mnd_mode) |
				MMSS_CC_CLK_CC_PMXO_SEL(0));
	}
	wmb();
}

static void disable_clk(struct dsi *dsi, enum mmss_cc_clk clk)
{
	cc_write(dsi, REG_MMSS_CC_CLK_CC(clk), 0);
	wmb();
}

void dsi_clk_enable_clk(struct dsi *dsi)
{
	enable_clk(dsi, CLK, &dsi->clk);
}

void dsi_clk_disable_clk(struct dsi *dsi)
{
	disable_clk(dsi, CLK);
}

void dsi_clk_enable_pclk(struct dsi *dsi)
{
	enable_clk(dsi, PCLK, &dsi->pclk);
}

void dsi_clk_disable_pclk(struct dsi *dsi)
{
	disable_clk(dsi, PCLK);
}

void dsi_pll_config(struct dsi *dsi, const struct mipi_panel_config *pcfg)
{
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_1,  pcfg->phy.pll[1]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_2,  pcfg->phy.pll[2]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_3,  pcfg->phy.pll[3]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_4,  pcfg->phy.pll[4]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_5,  pcfg->phy.pll[5]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_6,  pcfg->phy.pll[6]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_7,  pcfg->phy.pll[7]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_8,  pcfg->phy.pll[8]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_9,  pcfg->phy.pll[9]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_10, pcfg->phy.pll[10]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_11, pcfg->phy.pll[11]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_12, pcfg->phy.pll[12]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_13, pcfg->phy.pll[13]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_14, pcfg->phy.pll[14]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_15, pcfg->phy.pll[15]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_16, pcfg->phy.pll[16]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_17, pcfg->phy.pll[17]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_18, pcfg->phy.pll[18]);
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_19, pcfg->phy.pll[19]);
	/* NOTE: msmfb patches up some of the phy_pll_ctrl regs at
	 * this point based on clk values it calculates.. but at
	 * least for lgit panel, it doesn't seem any of the values
	 * change.. so leave it like this for now.  Once the bit-
	 * fields of these registers are understood, we'll just
	 * calculate all the values from more generic information
	 * provided by the panel, so this is just a temporary
	 * solution..
	 */
	dsi_write(dsi, REG_DSI_PHY_PLL_CTRL_0, pcfg->phy.pll[0]);
}
