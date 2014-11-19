/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "edp.h"

struct edp_bridge {
	struct drm_bridge base;
	struct msm_edp *edp;
};
#define to_edp_bridge(x) container_of(x, struct edp_bridge, base)

static inline bool edp_bridge_helper_mode_match(
	const struct list_head *modes,
	const struct drm_display_mode *req_mode,
	struct drm_display_mode **suggested_mode)
{
	struct drm_display_mode *m;

	if (!modes || !req_mode || !suggested_mode)
		return false;

	/*
	 * Return suggested_mode as the matching mode if found,
	 * or the first good mode in the mode list if existed,
	 * or NULL
	 */
	*suggested_mode = NULL;

	list_for_each_entry(m, modes, head) {
		if (m->status == MODE_OK) {
			if (*suggested_mode == NULL)
				*suggested_mode = m;

			if ((m->hdisplay == req_mode->hdisplay) &&
			(m->vdisplay == req_mode->vdisplay) &&
			(m->htotal == req_mode->htotal) &&
			(m->vtotal == req_mode->vtotal) &&
			(m->clock == req_mode->clock)) {
				*suggested_mode = m;
				return true;
			}
		}
	}

	return false;
}

static void edp_bridge_destroy(struct drm_bridge *bridge)
{
	struct edp_bridge *edp_bridge = to_edp_bridge(bridge);

	DBG("");
	drm_bridge_cleanup(bridge);
	kfree(edp_bridge);
}

static void edp_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct edp_bridge *edp_bridge = to_edp_bridge(bridge);
	struct msm_edp *edp = edp_bridge->edp;

	DBG("");
	msm_edp_ctrl_power(edp->ctrl_priv, true);
}

static void edp_bridge_enable(struct drm_bridge *bridge)
{
	DBG("");
}

static void edp_bridge_disable(struct drm_bridge *bridge)
{
	DBG("");
}

static void edp_bridge_post_disable(struct drm_bridge *bridge)
{
	struct edp_bridge *edp_bridge = to_edp_bridge(bridge);
	struct msm_edp *edp = edp_bridge->edp;

	DBG("");
	msm_edp_ctrl_power(edp->ctrl_priv, false);
}

static void edp_bridge_mode_set(struct drm_bridge *bridge,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = bridge->dev;
	struct drm_connector *connector;
	struct edp_bridge *edp_bridge = to_edp_bridge(bridge);
	struct msm_edp *edp = edp_bridge->edp;

	DBG("set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			mode->base.id, mode->name,
			mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal,
			mode->type, mode->flags);

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if ((connector->encoder != NULL) &&
			(connector->encoder->bridge == bridge)) {
			msm_edp_ctrl_timing_cfg(edp->ctrl_priv,
				adjusted_mode, &connector->display_info);
			break;
		}
	}
}

static bool edp_bridge_mode_fixup(struct drm_bridge *bridge,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = bridge->dev;
	struct drm_connector *connector;
	struct list_head head;
	struct drm_mode_object base;
	struct drm_display_mode *suggested_mode = NULL;

	DBG("");

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if ((connector->encoder != NULL) &&
			(connector->encoder->bridge == bridge)) {
			if (edp_bridge_helper_mode_match(&connector->modes,
							adjusted_mode,
							&suggested_mode))
				return true;

			/*
			 * Can not find a matching mode.
			 * Copy the suggested mode if existed.
			 * Preserve display mode header while copying.
			 */
			if (suggested_mode) {
				head = adjusted_mode->head;
				base = adjusted_mode->base;
				memcpy(adjusted_mode,
					suggested_mode,
					sizeof(*suggested_mode));
				adjusted_mode->head = head;
				adjusted_mode->base = base;

				return true;
			}

			break;
		}
	}

	/* No matching connector, or suggested mode found */
	return false;
}

static const struct drm_bridge_funcs edp_bridge_funcs = {
		.pre_enable = edp_bridge_pre_enable,
		.enable = edp_bridge_enable,
		.disable = edp_bridge_disable,
		.post_disable = edp_bridge_post_disable,
		.mode_set = edp_bridge_mode_set,
		.destroy = edp_bridge_destroy,
		.mode_fixup = edp_bridge_mode_fixup,
};

/* initialize bridge */
struct drm_bridge *msm_edp_bridge_init(struct msm_edp *edp)
{
	struct drm_bridge *bridge = NULL;
	struct edp_bridge *edp_bridge;
	int ret;

	edp_bridge = kzalloc(sizeof(*edp_bridge), GFP_KERNEL);
	if (!edp_bridge) {
		ret = -ENOMEM;
		goto fail;
	}

	edp_bridge->edp = edp;

	bridge = &edp_bridge->base;

	ret = drm_bridge_init(edp->dev, bridge, &edp_bridge_funcs);
	if (ret)
		goto fail;

	return bridge;

fail:
	if (bridge)
		edp_bridge_destroy(bridge);

	return ERR_PTR(ret);
}
