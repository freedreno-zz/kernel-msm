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

struct dsi_bridge {
	struct drm_bridge base;

	struct dsi *dsi;

	unsigned long int pixclock;
	bool enabled;                 /* DPMS state */
};
#define to_dsi_bridge(x) container_of(x, struct dsi_bridge, base)

static void dsi_bridge_destroy(struct drm_bridge *bridge)
{
	struct dsi_bridge *dsi_bridge = to_dsi_bridge(bridge);
	dsi_unreference(dsi_bridge->dsi);
	drm_bridge_cleanup(bridge);
	kfree(dsi_bridge);
}

static void dsi_bridge_dpms(struct drm_bridge *bridge, int mode)
{
	struct dsi_bridge *dsi_bridge = to_dsi_bridge(bridge);
	bool enabled = (mode == DRM_MODE_DPMS_ON);
	struct panel *panel = dsi_bridge->dsi->panel;

	DBG("mode=%d", mode);

	if (enabled == dsi_bridge->enabled)
		return;

	if (enabled)
		panel_on(panel);
	else
		panel_off(panel);

	dsi_bridge->enabled = enabled;
}

static void dsi_bridge_post_disable(struct drm_bridge *bridge)
{
	dsi_bridge_dpms(bridge, DRM_MODE_DPMS_OFF);
}

static void dsi_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct dsi_bridge *dsi_bridge = to_dsi_bridge(bridge);
	struct dsi *dsi = dsi_bridge->dsi;
	dsi_write(dsi, REG_DSI_RESET, 1);
	dsi_write(dsi, REG_DSI_RESET, 0);
	dsi_bridge_dpms(bridge, DRM_MODE_DPMS_ON);
}

static void dsi_bridge_mode_set(struct drm_bridge *bridge,
		 struct drm_display_mode *mode,
		 struct drm_display_mode *adjusted_mode)
{
	struct dsi_bridge *dsi_bridge = to_dsi_bridge(bridge);
	struct dsi *dsi = dsi_bridge->dsi;
	int hstart, hend, vstart, vend;

	mode = adjusted_mode;

	dsi_bridge->pixclock = mode->clock * 1000;

	hstart = mode->htotal - mode->hsync_start;
	hend   = mode->htotal - mode->hsync_start + mode->hdisplay;

	vstart = mode->vtotal - mode->vsync_start;
	vend   = mode->vtotal - mode->vsync_start + mode->vdisplay;

	DBG("htotal=%d, vtotal=%d, hstart=%d, hend=%d, vstart=%d, vend=%d",
			mode->htotal, mode->vtotal, hstart, hend, vstart, vend);

	dsi_write(dsi, REG_DSI_ACTIVE_H,
			DSI_ACTIVE_H_START(hstart) |
			DSI_ACTIVE_H_END(hend));
	dsi_write(dsi, REG_DSI_ACTIVE_V,
			DSI_ACTIVE_V_START(vstart) |
			DSI_ACTIVE_V_END(vend));

	dsi_write(dsi, REG_DSI_TOTAL,
			DSI_TOTAL_H_TOTAL(mode->htotal - 1) |
			DSI_TOTAL_V_TOTAL(mode->vtotal - 1));

	dsi_write(dsi, REG_DSI_ACTIVE_HSYNC,
			DSI_ACTIVE_HSYNC_START(0) |
			DSI_ACTIVE_HSYNC_END(mode->hsync_end - mode->hsync_start));
	dsi_write(dsi, 0x30, 0); /* ??? */
	dsi_write(dsi, REG_DSI_ACTIVE_VSYNC,
			DSI_ACTIVE_VSYNC_START(0) |
			DSI_ACTIVE_VSYNC_END(mode->vsync_end - mode->vsync_start));
}

static const struct drm_bridge_funcs dsi_bridge_funcs = {
	.destroy = dsi_bridge_destroy,
};

static const struct drm_bridge_helper_funcs dsi_bridge_helper_funcs = {
		.dpms = dsi_bridge_dpms,
		.post_disable = dsi_bridge_post_disable,
		.pre_enable = dsi_bridge_pre_enable,
		.mode_set = dsi_bridge_mode_set,
};


/* initialize bridge */
struct drm_bridge *dsi_bridge_init(struct dsi *dsi)
{
	struct drm_bridge *bridge = NULL;
	struct dsi_bridge *dsi_bridge;
	int ret;

	dsi_bridge = kzalloc(sizeof(*dsi_bridge), GFP_KERNEL);
	if (!dsi_bridge) {
		ret = -ENOMEM;
		goto fail;
	}

	dsi_bridge->dsi = dsi_reference(dsi);

	bridge = &dsi_bridge->base;

	drm_bridge_init(dsi->dev, bridge, &dsi_bridge_funcs);
	drm_bridge_helper_add(bridge, &dsi_bridge_helper_funcs);

	return bridge;

fail:
	if (bridge)
		dsi_bridge_destroy(bridge);

	return ERR_PTR(ret);
}
