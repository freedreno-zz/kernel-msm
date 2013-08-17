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

#ifndef __DSI_CLK_H__
#define __DSI_CLK_H__

/* utilities for clk calculations */

struct dsi_clk_desc {
	uint32_t src;
	uint32_t m;
	uint32_t n;
	uint32_t d;
	uint32_t mnd_mode;
	uint32_t pre_div_func;
};

struct dsi_clk_mnd {
	uint8_t nlanes;
	uint8_t bpp;
	uint8_t dsiclk_div;
	uint8_t dsiclk_m;
	uint8_t dsiclk_n;
	uint8_t dsiclk_d;
	uint8_t pclk_m;
	uint8_t pclk_n;
	uint8_t pclk_d;
};

const struct dsi_clk_mnd *dsi_find_clk_mnd(unsigned bpp, unsigned nlanes);

void dsi_clk_prepare(struct dsi *dsi);
void dsi_clk_unprepare(struct dsi *dsi);

void dsi_clk_enable_ahb(struct dsi *dsi);
void dsi_clk_disable_ahb(struct dsi *dsi);

void dsi_clk_enable_clk(struct dsi *dsi);
void dsi_clk_disable_clk(struct dsi *dsi);

void dsi_clk_enable_pclk(struct dsi *dsi);
void dsi_clk_disable_pclk(struct dsi *dsi);

void dsi_pll_config(struct dsi *dsi, const struct mipi_panel_config *pcfg);

#endif /* __DSI_CLK_H__ */
