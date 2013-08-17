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

#include "panel.h"

struct panel *panel_lgit_init(struct drm_device *dev,
		struct platform_device *pdev, struct mipi_adapter *mipi);

struct panel *mipi_panel_init(struct drm_device *dev,
		struct platform_device *pdev, struct mipi_adapter *mipi)
{
	/* TODO actually choose right panel.. */
	return panel_lgit_init(dev, pdev, mipi);
}
