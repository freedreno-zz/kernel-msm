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

#include <linux/gpio.h>

#include "dsi.h"

struct dsi_connector {
	struct drm_connector base;
	struct drm_display_mode *mode;
	struct dsi *dsi;
};
#define to_dsi_connector(x) container_of(x, struct dsi_connector, base)

static enum drm_connector_status dsi_connector_detect(
		struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int dsi_connector_set_property(struct drm_connector *connector,
		struct drm_property *property, uint64_t val)
{
	struct dsi_connector *dsi_connector = to_dsi_connector(connector);
	return panel_set_property(dsi_connector->dsi->panel, property, val);
}

static void dsi_connector_destroy(struct drm_connector *connector)
{
	struct dsi_connector *dsi_connector = to_dsi_connector(connector);

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);

	dsi_unreference(dsi_connector->dsi);

	kfree(dsi_connector);
}

static int dsi_connector_get_modes(struct drm_connector *connector)
{
	struct dsi_connector *dsi_connector = to_dsi_connector(connector);

	if (!dsi_connector->mode) {
		dsi_connector->mode = panel_mode(dsi_connector->dsi->panel);
		DRM_DEBUG_KMS("Got panel mode:\n");
		drm_mode_debug_printmodeline(dsi_connector->mode);
	}

	if (dsi_connector->mode) {
		struct drm_device *dev = connector->dev;
		drm_mode_probed_add(connector,
				drm_mode_duplicate(dev, dsi_connector->mode));
		return 1;
	}

	return 0;
}

static int dsi_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	// TODO check that it matches our mode..
	return 0;
}

static struct drm_encoder *
dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct dsi_connector *dsi_connector = to_dsi_connector(connector);
	return dsi_connector->dsi->encoder;
}

static const struct drm_connector_funcs dsi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = dsi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = dsi_connector_set_property,
	.destroy = dsi_connector_destroy,
};

static const struct drm_connector_helper_funcs dsi_connector_helper_funcs = {
	.get_modes = dsi_connector_get_modes,
	.mode_valid = dsi_connector_mode_valid,
	.best_encoder = dsi_connector_best_encoder,
};

/* initialize connector */
struct drm_connector *dsi_connector_init(struct dsi *dsi)
{
	struct drm_connector *connector = NULL;
	struct dsi_connector *dsi_connector;
	int ret;

	dsi_connector = kzalloc(sizeof(*dsi_connector), GFP_KERNEL);
	if (!dsi_connector) {
		ret = -ENOMEM;
		goto fail;
	}

	dsi_connector->dsi = dsi_reference(dsi);

	connector = &dsi_connector->base;

	drm_connector_init(dsi->dev, connector, &dsi_connector_funcs,
			DRM_MODE_CONNECTOR_LVDS); // TODO DRM_MODE_CONNECTOR_DSI
	drm_connector_helper_add(connector, &dsi_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_sysfs_connector_add(connector);
	drm_mode_connector_attach_encoder(connector, dsi->encoder);

	panel_install_properties(dsi->panel, &connector->base);

	return connector;

fail:
	if (connector)
		dsi_connector_destroy(connector);

	return ERR_PTR(ret);
}
