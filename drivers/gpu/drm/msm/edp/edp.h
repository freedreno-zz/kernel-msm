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

#ifndef __EDP_CONNECTOR_H__
#define __EDP_CONNECTOR_H__

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include <drm/drm_dp_helper.h>

#include "drm_crtc.h"
#include "msm_drv.h"

#define edp_read(offset) msm_readl((offset))
#define edp_write(offset, data) msm_writel((data), (offset))

struct msm_edp {
	struct drm_device *dev;
	struct platform_device *pdev;

	struct drm_connector *connector;
	struct drm_bridge *bridge;

	/* the encoder we are hooked to (outside of edp block) */
	struct drm_encoder *encoder;

	void *ctrl_priv;

	int irq;
};

/* eDP bridge */
struct drm_bridge *msm_edp_bridge_init(struct msm_edp *edp);

/* eDP connector */
struct drm_connector *msm_edp_connector_init(struct msm_edp *edp);

/* AUX */
void *msm_edp_aux_init(struct device *dev, void __iomem *regbase,
			struct drm_dp_aux **drm_aux);
void msm_edp_aux_destroy(void *edp_aux);
irqreturn_t msm_edp_aux_irq(void *edp_aux, u32 isr);
void msm_edp_aux_ctrl(void *edp_aux, int enable);

/* Phy */
bool msm_edp_phy_ready(void *edp_phy);
void msm_edp_phy_ctrl(void *edp_phy, int enable);
void msm_edp_phy_vm_pe_init(void *edp_phy);
void msm_edp_phy_vm_pe_cfg(void *edp_phy, u32 v0, u32 v1);
void msm_edp_phy_lane_power_ctrl(void *edp_phy, int up, int max_lane);
void *msm_edp_phy_init(void __iomem *regbase);
void msm_edp_phy_destroy(void *edp_phy);

/* Ctrl */
irqreturn_t msm_edp_ctrl_irq(void *edp_ctrl);
int msm_edp_ctrl_set_backlight(void *edp_ctrl, u32 bl_level);
void msm_edp_ctrl_power(void *edp_ctrl, bool on);
int msm_edp_ctrl_init(struct msm_edp *edp);
void msm_edp_ctrl_destroy(void *edp_ctrl);
bool msm_edp_ctrl_panel_connected(void *edp_ctrl);
int msm_edp_ctrl_get_edid(void *edp_ctrl,
	struct drm_connector *connector, struct edid **edid);
int msm_edp_ctrl_timing_cfg(void *edp_ctrl,
	struct drm_display_mode *mode, struct drm_display_info *info);
/* @pixel_rate is in kHz */
bool msm_edp_ctrl_pixel_clock_valid(void *edp_ctrl,
	u32 pixel_rate, u32 *pm, u32 *pn);

#endif /* __EDP_CONNECTOR_H__ */
