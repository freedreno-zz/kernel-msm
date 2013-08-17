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

#ifndef __PANEL_H__
#define __PANEL_H__

#include <drm/drmP.h>

#include "mipi.h"

/* This is something relatively generic that could be outside msm driver */

struct panel;

struct panel_funcs {
	void (*destroy)(struct panel *panel);
	int (*on)(struct panel *panel);
	int (*off)(struct panel *panel);
	struct drm_display_mode *(*mode)(struct panel *panel);
	/* optional: */
	void (*install_properties)(struct panel *panel,
			struct drm_mode_object *obj);
	int (*set_property)(struct panel *panel,
			struct drm_property *property, uint64_t val);
};

struct panel {
	struct drm_device *dev;
	const struct panel_funcs *funcs;
};

static inline void panel_destroy(struct panel *panel)
{
	panel->funcs->destroy(panel);
}

static inline int panel_on(struct panel *panel)
{
	return panel->funcs->on(panel);
}

static inline int panel_off(struct panel *panel)
{
	return panel->funcs->off(panel);
}

static inline struct drm_display_mode *panel_mode(struct panel *panel)
{
	return panel->funcs->mode(panel);
}

static inline void panel_install_properties(struct panel *panel,
		struct drm_mode_object *obj)
{
	if (!panel->funcs->install_properties)
		return;
	panel->funcs->install_properties(panel, obj);
}

static inline int panel_set_property(struct panel *panel,
		struct drm_property *property, uint64_t val)
{
	if (!panel->funcs->set_property)
		return -EINVAL;
	return panel->funcs->set_property(panel, property, val);
}

static inline int panel_init(struct drm_device *dev, struct panel *panel,
		const struct panel_funcs *funcs)
{
	panel->dev = dev;
	panel->funcs = funcs;
	return 0;
}

/* TODO we could have other sorts of panels, ie. dpi/lcdc.. */
struct panel *mipi_panel_init(struct drm_device *dev,
		struct platform_device *pdev, struct mipi_adapter *mipi);

#endif /* __PANEL_H__ */
