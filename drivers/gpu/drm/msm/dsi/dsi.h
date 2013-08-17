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

#ifndef __DSI_H__
#define __DSI_H__

#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "msm_drv.h"
#include "dsi.xml.h"
#include "sfpb.xml.h"
#include "mmss_cc.xml.h"
#include "panel/mipi.h"
#include "panel/panel.h"

struct dsi_phy;
struct dsi;

#include "dsi_clk.h"

struct dsi {
	struct kref refcount;

	struct drm_device *dev;
	struct platform_device *pdev;

	void __iomem *mmio, *sfpb, *cc;

	struct dsi_clk_desc clk, pclk;

	struct clk *byte_div_clk;
	struct clk *esc_clk;
	struct clk *m_pclk;
	struct clk *s_pclk;
	struct clk *amp_pclk;

	struct panel *panel;
	struct dsi_phy *phy;
	struct mipi_adapter *mipi;
	struct drm_connector *connector;
	struct drm_bridge *bridge;

	/* the encoder we are hooked to (outside of dsi block) */
	struct drm_encoder *encoder;

	int irq;
};

/* platform config data (ie. from DT, or pdata) */
struct dsi_platform_config {
	struct dsi_phy *(*phy_init)(struct dsi *dsi);
};

struct panel *dsi_panel(struct dsi *dsi);
void dsi_destroy(struct kref *kref);

static inline void dsi_write(struct dsi *dsi, u32 reg, u32 data)
{
	msm_writel(data, dsi->mmio + reg);
}

static inline u32 dsi_read(struct dsi *dsi, u32 reg)
{
	return msm_readl(dsi->mmio + reg);
}

static inline void sfpb_write(struct dsi *dsi, u32 reg, u32 data)
{
	msm_writel(data, dsi->sfpb + reg);
}

static inline u32 sfpb_read(struct dsi *dsi, u32 reg)
{
	return msm_readl(dsi->sfpb + reg);
}

static inline void cc_write(struct dsi *dsi, u32 reg, u32 data)
{
	msm_writel(data, dsi->cc + reg);
}

static inline u32 cc_read(struct dsi *dsi, u32 reg)
{
	return msm_readl(dsi->cc + reg);
}

static inline struct dsi * dsi_reference(struct dsi *dsi)
{
	kref_get(&dsi->refcount);
	return dsi;
}

static inline void dsi_unreference(struct dsi *dsi)
{
	kref_put(&dsi->refcount, dsi_destroy);
}

/*
 * The phy appears to be different, for example between 8960 and 8x60,
 * so split the phy related functions out and load the correct one at
 * runtime:
 */

struct dsi_phy_funcs {
	void (*destroy)(struct dsi_phy *phy);
	void (*config)(struct dsi_phy *phy, const struct mipi_panel_config *pcfg);
	void (*powerup)(struct dsi_phy *phy, unsigned long int pixclock);
	void (*powerdown)(struct dsi_phy *phy);
	void (*clk_enable)(struct dsi_phy *phy);
	void (*clk_disable)(struct dsi_phy *phy);
};

struct dsi_phy {
	const struct dsi_phy_funcs *funcs;
};

struct dsi_phy *dsi_phy_8960_init(struct dsi *dsi);
struct dsi_phy *dsi_phy_8x60_init(struct dsi *dsi);

/*
 * dsi bridge:
 */

struct drm_bridge *dsi_bridge_init(struct dsi *dsi);

/*
 * dsi connector:
 */

struct drm_connector *dsi_connector_init(struct dsi *dsi);

/*
 * dsi/mipi adapter:
 */

void dsi_mipi_irq(struct mipi_adapter *mipi);
struct mipi_adapter *dsi_mipi_init(struct dsi *dsi);

#endif /* __DSI_H__ */
