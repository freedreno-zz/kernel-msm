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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include "edp.h"
#include "edp_reg.h"
#include "drm_dp_helper.h"
#include "drm_edid.h"
#include "drm_crtc.h"

#define VDDA_MIN_UV		1800000	/* uV units */
#define VDDA_MAX_UV		1800000	/* uV units */
#define VDDA_UA_ON_LOAD		100000	/* uA units */
#define VDDA_UA_OFF_LOAD	100	/* uA units */

#define RGB_COMPONENTS		3

#define DPCD_LINK_VOLTAGE_MAX		4
#define DPCD_LINK_PRE_EMPHASIS_MAX	4

#define EDP_LINK_BW_MAX		DP_LINK_BW_2_7

/* Link training return value */
#define EDP_TRAIN_FAIL		-1
#define EDP_TRAIN_SUCCESS	0
#define EDP_TRAIN_RECONFIG	1

#define EDP_CLK_MASK_AHB		BIT(0)
#define EDP_CLK_MASK_AUX		BIT(1)
#define EDP_CLK_MASK_LINK		BIT(2)
#define EDP_CLK_MASK_PIXEL		BIT(3)
#define EDP_CLK_MASK_MDP_CORE		BIT(4)
#define EDP_CLK_MASK_LINK_CHAN	(EDP_CLK_MASK_LINK | EDP_CLK_MASK_PIXEL)
#define EDP_CLK_MASK_AUX_CHAN	\
	(EDP_CLK_MASK_AHB | EDP_CLK_MASK_AUX | EDP_CLK_MASK_MDP_CORE)
#define EDP_CLK_MASK_ALL	(EDP_CLK_MASK_AUX_CHAN | EDP_CLK_MASK_LINK_CHAN)

#define EDP_BACKLIGHT_MAX	255

static int edp_cont_splash;	/* 1 to enable continuous splash screen */

module_param(edp_cont_splash, int, 0);
MODULE_PARM_DESC(edp_cont_splash, "Enable continuous splash screen on eDP");

struct edp_ctrl {
	struct platform_device *pdev;

	void __iomem *base;

	/* regulators */
	struct regulator *vdda_vreg;
	struct regulator *lvl_reg;

	/* clocks */
	struct clk *aux_clk;
	struct clk *pixel_clk;
	struct clk *ahb_clk;
	struct clk *link_clk;
	struct clk *mdp_core_clk;

	/* gpios */
	int gpio_panel_en;
	int gpio_panel_hpd;
	int gpio_lvl_en;
	int gpio_bkl_en;

	/* backlight */
	struct pwm_device *bl_pwm;
	u32 pwm_period;
	u32 bl_level;
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	struct backlight_device *backlight_dev;
#endif

	/* completion and mutex */
	struct completion idle_comp;
	struct mutex dev_mutex; /* To protect device power status */
	struct mutex bl_mutex; /* To protect bl_level and pwm status */

	/* work queue */
	struct work_struct on_work;
	struct work_struct off_work;
	struct workqueue_struct *workqueue;

	/* Interrupt register lock */
	spinlock_t irq_lock;

	int cont_splash;
	bool edp_connected;
	bool power_on;

	/* edid raw data */
	struct edid *edid;

	struct drm_dp_link dp_link;
	struct drm_dp_aux *drm_aux;

	/* dpcd raw data */
	u8 dpcd[DP_RECEIVER_CAP_SIZE];

	/* Link status */
	u8 link_rate;
	u8 lane_cnt;
	u8 v_level;
	u8 p_level;

	/* Timing status */
	u8 interlaced;
	u32 pixel_rate; /* in kHz */
	u32 color_depth;

	void *aux_priv;
	void *phy_priv;
};

struct edp_pixel_clk_div {
	u32 rate; /* in kHz */
	u32 m;
	u32 n;
};

#define EDP_PIXEL_CLK_NUM 8
static const struct edp_pixel_clk_div clk_divs[2][EDP_PIXEL_CLK_NUM] = {
	{ /* Link clock = 162MHz, source clock = 810MHz */
		{119000, 31,  211}, /* WSXGA+ 1680x1050@60Hz CVT */
		{130250, 32,  199}, /* UXGA 1600x1200@60Hz CVT */
		{148500, 11,  60},  /* FHD 1920x1080@60Hz */
		{154000, 50,  263}, /* WUXGA 1920x1200@60Hz CVT */
		{209250, 31,  120}, /* QXGA 2048x1536@60Hz CVT */
		{268500, 119, 359}, /* WQXGA 2560x1600@60Hz CVT */
		{138530, 33,  193}, /* AUO B116HAN03.0 Panel */
		{141400, 48,  275}, /* AUO B133HTN01.2 Panel */
	},
	{ /* Link clock = 270MHz, source clock = 675MHz */
		{119000, 52,  295}, /* WSXGA+ 1680x1050@60Hz CVT */
		{130250, 11,  57},  /* UXGA 1600x1200@60Hz CVT */
		{148500, 11,  50},  /* FHD 1920x1080@60Hz */
		{154000, 47,  206}, /* WUXGA 1920x1200@60Hz CVT */
		{209250, 31,  100}, /* QXGA 2048x1536@60Hz CVT */
		{268500, 107, 269}, /* WQXGA 2560x1600@60Hz CVT */
		{138530, 63,  307}, /* AUO B116HAN03.0 Panel */
		{141400, 53,  253}, /* AUO B133HTN01.2 Panel */
	},
};

static void edp_clk_deinit(struct edp_ctrl *ctrl)
{
	if (ctrl->aux_clk)
		clk_put(ctrl->aux_clk);
	if (ctrl->pixel_clk)
		clk_put(ctrl->pixel_clk);
	if (ctrl->ahb_clk)
		clk_put(ctrl->ahb_clk);
	if (ctrl->link_clk)
		clk_put(ctrl->link_clk);
	if (ctrl->mdp_core_clk)
		clk_put(ctrl->mdp_core_clk);
}

static int edp_clk_init(struct edp_ctrl *ctrl)
{
	struct device *dev = &ctrl->pdev->dev;

	ctrl->aux_clk = clk_get(dev, "core_clk");
	if (IS_ERR(ctrl->aux_clk)) {
		pr_err("%s: Can't find aux_clk", __func__);
		ctrl->aux_clk = NULL;
		goto edp_clk_err;
	}

	ctrl->pixel_clk = clk_get(dev, "pixel_clk");
	if (IS_ERR(ctrl->pixel_clk)) {
		pr_err("%s: Can't find pixel_clk", __func__);
		ctrl->pixel_clk = NULL;
		goto edp_clk_err;
	}

	ctrl->ahb_clk = clk_get(dev, "iface_clk");
	if (IS_ERR(ctrl->ahb_clk)) {
		pr_err("%s: Can't find ahb_clk", __func__);
		ctrl->ahb_clk = NULL;
		goto edp_clk_err;
	}

	ctrl->link_clk = clk_get(dev, "link_clk");
	if (IS_ERR(ctrl->link_clk)) {
		pr_err("%s: Can't find link_clk", __func__);
		ctrl->link_clk = NULL;
		goto edp_clk_err;
	}

	/* need mdss clock to receive irq */
	ctrl->mdp_core_clk = clk_get(dev, "mdp_core_clk");
	if (IS_ERR(ctrl->mdp_core_clk)) {
		pr_err("%s: Can't find mdp_core_clk", __func__);
		ctrl->mdp_core_clk = NULL;
		goto edp_clk_err;
	}

	return 0;

edp_clk_err:
	edp_clk_deinit(ctrl);
	return -EPERM;
}

static int edp_clk_enable(struct edp_ctrl *ctrl, u32 clk_mask)
{
	int ret = 0;

	DBG("mask=%x", clk_mask);
	/* ahb_clk should be enabled first */
	if (clk_mask & EDP_CLK_MASK_AHB) {
		ret = clk_prepare_enable(ctrl->ahb_clk);
		if (ret) {
			pr_err("%s: Failed to enable ahb clk\n", __func__);
			goto f0;
		}
	}
	if (clk_mask & EDP_CLK_MASK_AUX) {
		ret = clk_set_rate(ctrl->aux_clk, 19200000);
		if (ret) {
			pr_err("%s: Failed to set rate aux clk\n", __func__);
			goto f1;
		}
		ret = clk_prepare_enable(ctrl->aux_clk);
		if (ret) {
			pr_err("%s: Failed to enable aux clk\n", __func__);
			goto f1;
		}
	}
	/* Need to set rate and enable link_clk prior to pixel_clk */
	if (clk_mask & EDP_CLK_MASK_LINK) {
		DBG("edp->link_clk, set_rate %ld",
				(unsigned long)ctrl->link_rate * 27000000);
		ret = clk_set_rate(ctrl->link_clk,
				(unsigned long)ctrl->link_rate * 27000000);
		if (ret) {
			pr_err("%s: Failed to set rate to link clk\n",
				__func__);
			goto f2;
		}

		ret = clk_prepare_enable(ctrl->link_clk);
		if (ret) {
			pr_err("%s: Failed to enable link clk\n", __func__);
			goto f2;
		}
	}
	if (clk_mask & EDP_CLK_MASK_PIXEL) {
		DBG("edp->pixel_clk, set_rate %ld",
				(unsigned long)ctrl->pixel_rate * 1000);
		ret = clk_set_rate(ctrl->pixel_clk,
				(unsigned long)ctrl->pixel_rate * 1000);
		if (ret) {
			pr_err("%s: Failed to set rate to pixel clk\n",
				__func__);
			goto f3;
		}

		ret = clk_prepare_enable(ctrl->pixel_clk);
		if (ret) {
			pr_err("%s: Failed to enable pixel clk\n", __func__);
			goto f3;
		}
	}
	if (clk_mask & EDP_CLK_MASK_MDP_CORE) {
		ret = clk_prepare_enable(ctrl->mdp_core_clk);
		if (ret) {
			pr_err("%s: Failed to enable mdp core clk\n", __func__);
			goto f4;
		}
	}

	return 0;

f4:
	if (clk_mask & EDP_CLK_MASK_PIXEL)
		clk_disable_unprepare(ctrl->pixel_clk);
f3:
	if (clk_mask & EDP_CLK_MASK_LINK)
		clk_disable_unprepare(ctrl->link_clk);
f2:
	if (clk_mask & EDP_CLK_MASK_AUX)
		clk_disable_unprepare(ctrl->aux_clk);
f1:
	if (clk_mask & EDP_CLK_MASK_AHB)
		clk_disable_unprepare(ctrl->ahb_clk);
f0:
	return ret;
}

static void edp_clk_disable(struct edp_ctrl *ctrl, u32 clk_mask)
{
	if (clk_mask & EDP_CLK_MASK_MDP_CORE)
		clk_disable_unprepare(ctrl->mdp_core_clk);
	if (clk_mask & EDP_CLK_MASK_PIXEL)
		clk_disable_unprepare(ctrl->pixel_clk);
	if (clk_mask & EDP_CLK_MASK_LINK)
		clk_disable_unprepare(ctrl->link_clk);
	if (clk_mask & EDP_CLK_MASK_AUX)
		clk_disable_unprepare(ctrl->aux_clk);
	if (clk_mask & EDP_CLK_MASK_AHB)
		clk_disable_unprepare(ctrl->ahb_clk);
}

static void edp_regulator_deinit(struct edp_ctrl *ctrl)
{
	if (ctrl->vdda_vreg) {
		regulator_disable(ctrl->vdda_vreg);
		regulator_set_optimum_mode(ctrl->vdda_vreg, VDDA_UA_OFF_LOAD);
		devm_regulator_put(ctrl->vdda_vreg);
		ctrl->vdda_vreg = NULL;
	}
}

static int edp_regulator_init(struct edp_ctrl *ctrl)
{
	struct device *dev = &ctrl->pdev->dev;
	int ret;

	DBG("");
	ctrl->vdda_vreg = devm_regulator_get(dev, "vdda");
	if (IS_ERR(ctrl->vdda_vreg)) {
		pr_err("%s: Could not get vdda reg, ret = %ld\n", __func__,
				PTR_ERR(ctrl->vdda_vreg));
		ctrl->vdda_vreg = NULL;
		ret = -ENODEV;
		goto f0;
	}

	ret = regulator_set_voltage(ctrl->vdda_vreg, VDDA_MIN_UV, VDDA_MAX_UV);
	if (ret) {
		pr_err("%s:vdda_vreg set_voltage failed, %d\n", __func__, ret);
		goto f1;
	}

	ret = regulator_set_optimum_mode(ctrl->vdda_vreg, VDDA_UA_ON_LOAD);
	if (ret < 0) {
		pr_err("%s: vdda_vreg set regulator mode failed.\n", __func__);
		goto f1;
	}

	ret = regulator_enable(ctrl->vdda_vreg);
	if (ret) {
		pr_err("%s: Failed to enable vdda_vreg regulator.\n", __func__);
		goto f2;
	}

	DBG("exit");
	return 0;

f2:
	regulator_set_optimum_mode(ctrl->vdda_vreg, VDDA_UA_OFF_LOAD);
f1:
	devm_regulator_put(ctrl->vdda_vreg);
	ctrl->vdda_vreg = NULL;
f0:
	return ret;
}

static int edp_gpio_config(struct edp_ctrl *ctrl, bool on)
{
	struct device *dev = &ctrl->pdev->dev;
	int ret, enable;

	if (on) {
		if (ctrl->cont_splash)
			enable = 1;
		else
			enable = 0;

		ctrl->gpio_panel_hpd = of_get_named_gpio(
				dev->of_node, "panel-hpd-gpio", 0);

		if (!gpio_is_valid(ctrl->gpio_panel_hpd)) {
			pr_err("%s gpio_panel_hpd %d is not valid ", __func__,
					ctrl->gpio_panel_hpd);
			ret = -ENODEV;
			goto f0;
		}

		ret = gpio_request(ctrl->gpio_panel_hpd, "edp_hpd_irq_gpio");
		if (ret) {
			pr_err("%s unable to request gpio_panel_hpd %d",
				__func__, ctrl->gpio_panel_hpd);
			goto f0;
		}

		ret = gpio_direction_input(ctrl->gpio_panel_hpd);
		if (ret) {
			pr_err("%s: Set direction for hpd failed, ret = %d\n",
				__func__, ctrl->gpio_panel_hpd);
			goto f1;
		}

		ctrl->gpio_panel_en = of_get_named_gpio(dev->of_node,
				"panel-en-gpio", 0);
		if (!gpio_is_valid(ctrl->gpio_panel_en)) {
			pr_err("%s: gpio_panel_en=%d not specified\n", __func__,
					ctrl->gpio_panel_en);
			ret = -ENODEV;
			goto f2;
		}

		ret = gpio_request(ctrl->gpio_panel_en, "disp_enable");
		if (ret) {
			pr_err("%s: Request gpio_panel_en failed, ret=%d\n",
					__func__, ret);
			goto f2;
		}

		ret = gpio_direction_output(ctrl->gpio_panel_en, enable);
		if (ret) {
			pr_err("%s: Set direction for panel_en failed, %d\n",
					__func__, ret);
			goto f3;
		}

		ctrl->gpio_bkl_en = of_get_named_gpio(dev->of_node,
				"backlight-en-gpio", 0);
		if (!gpio_is_valid(ctrl->gpio_bkl_en)) {
			pr_err("%s: gpio_bkl_en=%d not specified\n", __func__,
					ctrl->gpio_bkl_en);
			ret = -ENODEV;
			goto f4;
		}

		ret = gpio_request(ctrl->gpio_bkl_en, "edp_bkl_enable");
		if (ret) {
			pr_err("%s: Request reset gpio_bkl_en failed, ret=%d\n",
					__func__, ret);
			goto f4;
		}

		ret = gpio_direction_output(ctrl->gpio_bkl_en, enable);
		if (ret) {
			pr_err("%s: Set direction for bkl_en failed, %d\n",
					__func__, ret);
			goto f5;
		}

		DBG("gpio on");
	} else {
		if (gpio_is_valid(ctrl->gpio_bkl_en)) {
			gpio_free(ctrl->gpio_bkl_en);
			ctrl->gpio_bkl_en = -1;
		}
		if (gpio_is_valid(ctrl->gpio_panel_hpd)) {
			gpio_free(ctrl->gpio_panel_hpd);
			ctrl->gpio_panel_hpd = -1;
		}
		if (gpio_is_valid(ctrl->gpio_panel_en)) {
			gpio_free(ctrl->gpio_panel_en);
			ctrl->gpio_panel_en = -1;
		}
		DBG("gpio off");
	}

	return 0;

f5:
	gpio_free(ctrl->gpio_bkl_en);
f4:
	ctrl->gpio_bkl_en = -1;
	gpio_set_value(ctrl->gpio_panel_en, 0);
f3:
	gpio_free(ctrl->gpio_panel_en);
f2:
	ctrl->gpio_panel_en = -1;
f1:
	gpio_free(ctrl->gpio_panel_hpd);
f0:
	ctrl->gpio_panel_hpd = -1;

	return ret;
}

static int edp_pwm_config(struct edp_ctrl *ctrl, bool on)
{
	struct device *dev = &ctrl->pdev->dev;
	int ret = 0;
	u32 lpg_channel;

	if (!on) {
		if (ctrl->bl_pwm) {
			pwm_free(ctrl->bl_pwm);
			ctrl->bl_pwm = NULL;
		}
		return 0;
	}

	ret = of_property_read_u32(dev->of_node,
			"qcom,panel-lpg-channel", &lpg_channel);
	if (ret) {
		pr_err("%s: panel lpg channel is not specified, %d", __func__,
				lpg_channel);
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node,
			"qcom,panel-pwm-period", &ctrl->pwm_period);
	if (ret) {
		pr_err("%s: panel pwm period is not specified, %d", __func__,
				ctrl->pwm_period);
		ctrl->pwm_period = -EINVAL;
		return -EINVAL;
	}

	ctrl->bl_pwm = pwm_request(lpg_channel, "lcd-backlight");
	if (ctrl->bl_pwm == NULL || IS_ERR(ctrl->bl_pwm)) {
		pr_err("%s: pwm request failed", __func__);
		ctrl->bl_pwm = NULL;
		return -EIO;
	}

	return 0;
}

/* The power source of the level translation chip is different on different
 * target boards, i.e. a gpio or a regulator.
 */
static int edp_level_trans_enable(struct edp_ctrl *ctrl)
{
	struct device *dev = &ctrl->pdev->dev;
	int ret = 0;

	ctrl->gpio_lvl_en = of_get_named_gpio(dev->of_node,
			"lvl-en-gpio", 0);
	if (!gpio_is_valid(ctrl->gpio_lvl_en)) {
		DBG("gpio_lvl_en=%d not specified, try regulator",
			ctrl->gpio_lvl_en);
		ctrl->gpio_lvl_en = -1;
		goto lvl_reg_enable;
	}
	ret = gpio_request(ctrl->gpio_lvl_en, "edp_lvl_enable");
	if (ret) {
		DBG("Request gpio_lvl_en failed, %d, try regulator", ret);
		ctrl->gpio_lvl_en = -1;
		goto lvl_reg_enable;
	}
	ret = gpio_direction_output(ctrl->gpio_lvl_en, 1);
	if (ret) {
		DBG("Set dir for lvl_en failed, %d, try regulator", ret);
		gpio_free(ctrl->gpio_lvl_en);
		ctrl->gpio_lvl_en = -1;
	}
	DBG("gpio_lvl is enabled");

lvl_reg_enable:
	ctrl->lvl_reg = devm_regulator_get(dev, "lvl-vdd");
	if (IS_ERR(ctrl->lvl_reg)) {
		DBG("Could not get lvl-vdd reg, %ld",
				PTR_ERR(ctrl->vdda_vreg));
		ctrl->lvl_reg = NULL;
		goto exit;
	}
	ret = regulator_enable(ctrl->lvl_reg);
	if (ret) {
		DBG("Failed to enable lvl-vdd reg regulator, %d", ret);
		devm_regulator_put(ctrl->lvl_reg);
		ctrl->lvl_reg = NULL;
	}
	DBG("lvl regulator is enabled");

exit:
	if (ctrl->lvl_reg && gpio_is_valid(ctrl->gpio_lvl_en))
		pr_warn("%s: both gpio and regulator are enabled\n", __func__);
	else if (!ctrl->lvl_reg && !gpio_is_valid(ctrl->gpio_lvl_en))
		pr_warn("%s: nothing is enabled\n", __func__);

	return 0;
}

static void edp_level_trans_disable(struct edp_ctrl *ctrl)
{
	if (ctrl->lvl_reg) {
		regulator_disable(ctrl->lvl_reg);
		devm_regulator_put(ctrl->lvl_reg);
		ctrl->lvl_reg = NULL;
	}
	if (gpio_is_valid(ctrl->gpio_lvl_en)) {
		gpio_set_value(ctrl->gpio_lvl_en, 0);
		gpio_free(ctrl->gpio_lvl_en);
		ctrl->gpio_lvl_en = -1;
	}
}

static void edp_ctrl_irq_enable(struct edp_ctrl *ctrl, int enable)
{
	unsigned long flags;

	DBG("%d", enable);
	spin_lock_irqsave(&ctrl->irq_lock, flags);
	if (enable) {
		edp_write(ctrl->base + EDP_INTERRUPT_REG_1, EDP_INTR_MASK1);
		edp_write(ctrl->base + EDP_INTERRUPT_REG_2, EDP_INTR_MASK2);
	} else {
		edp_write(ctrl->base + EDP_INTERRUPT_REG_1, 0x0);
		edp_write(ctrl->base + EDP_INTERRUPT_REG_2, 0x0);
	}
	spin_unlock_irqrestore(&ctrl->irq_lock, flags);
	DBG("exit");
}

static void edp_fill_link_cfg(struct edp_ctrl *ctrl)
{
	u32 prate;
	u32 lrate;
	u32 bpp;
	u8 max_lane = ctrl->dp_link.num_lanes;
	u8 lane;

	prate = ctrl->pixel_rate;
	bpp = ctrl->color_depth * RGB_COMPONENTS;

	/*
	 * By default, use the maximum link rate and minimum lane count,
	 * so that we can do rate down shift during link training.
	 */
	ctrl->link_rate = drm_dp_link_rate_to_bw_code(ctrl->dp_link.rate);

	prate *= bpp;
	prate /= 8; /* in kByte */

	lrate = 270000; /* in kHz */
	lrate *= ctrl->link_rate;
	lrate /= 10; /* in kByte, 10 bits --> 8 bits */

	for (lane = 1; lane <= max_lane; lane <<= 1) {
		if (lrate >= prate)
			break;
		lrate <<= 1;
	}

	ctrl->lane_cnt = lane;
	DBG("rate=%d lane=%d", ctrl->link_rate, ctrl->lane_cnt);
}

static void edp_config_ctrl(struct edp_ctrl *ctrl)
{
	u32 data = 0;

	data = ctrl->lane_cnt - 1;
	data <<= 4;

	if (ctrl->dp_link.capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		data |= 0x40;

	if (ctrl->color_depth == 8)
		data |= 0x100;	/* 0 == 6 bits, 1 == 8 bits */

	if (!ctrl->interlaced)	/* progressive */
		data |= 0x04;

	data |= 0x03;	/* sycn clock & static Mvid */

	edp_write(ctrl->base + EDP_CONFIGURATION_CTRL, data);
}

static void edp_state_ctrl(struct edp_ctrl *ctrl, u32 state)
{
	edp_write(ctrl->base + EDP_STATE_CTRL, state);
	/* Make sure H/W status is set */
	wmb();
}

static int edp_lane_set_write(struct edp_ctrl *ctrl,
	u8 voltage_level, u8 pre_emphasis_level)
{
	int i;
	u8 buf[4];

	if (voltage_level >= DPCD_LINK_VOLTAGE_MAX)
		voltage_level |= 0x04;

	if (pre_emphasis_level >= DPCD_LINK_PRE_EMPHASIS_MAX)
		pre_emphasis_level |= 0x04;

	pre_emphasis_level <<= 3;

	for (i = 0; i < 4; i++)
		buf[i] = voltage_level | pre_emphasis_level;

	DBG("%s: p|v=0x%x", __func__, voltage_level | pre_emphasis_level);
	if (drm_dp_dpcd_write(ctrl->drm_aux, 0x103, buf, 4) < 4) {
		pr_err("%s: Set sw/pe to panel failed\n", __func__);
		return -ENOLINK;
	}

	return 0;
}

static int edp_train_pattern_set_write(struct edp_ctrl *ctrl, u8 pattern)
{
	u8 p = pattern;

	DBG("pattern=%x", p);
	if (drm_dp_dpcd_write(ctrl->drm_aux, 0x102, &p, 1) < 1) {
		pr_err("%s: Set training pattern to panel failed\n", __func__);
		return -ENOLINK;
	}

	return 0;
}

static void edp_sink_train_set_adjust(struct edp_ctrl *ctrl,
	const u8 *link_status)
{
	int i;
	u8 max = 0;
	u8 data;

	/* use the max level across lanes */
	for (i = 0; i < ctrl->lane_cnt; i++) {
		data = drm_dp_get_adjust_request_voltage(link_status, i);
		DBG("lane=%d req_voltage_swing=0x%x", i, data);
		if (max < data)
			max = data;
	}

	ctrl->v_level = max >> DP_TRAIN_VOLTAGE_SWING_SHIFT;

	/* use the max level across lanes */
	max = 0;
	for (i = 0; i < ctrl->lane_cnt; i++) {
		data = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		DBG("lane=%d req_pre_emphasis=0x%x", i, data);
		if (max < data)
			max = data;
	}

	ctrl->p_level = max >> DP_TRAIN_PRE_EMPHASIS_SHIFT;
	DBG("v_level=%d, p_level=%d", ctrl->v_level, ctrl->p_level);
}

static void edp_host_train_set(struct edp_ctrl *ctrl, u32 train)
{
	int cnt;
	u32 data;
	u32 bit;

	bit = 1;
	bit <<= (train - 1);
	DBG("%s: bit=%d train=%d", __func__, bit, train);

	edp_state_ctrl(ctrl, bit);
	bit = 8;
	bit <<= (train - 1);
	cnt = 10;
	while (cnt--) {
		data = edp_read(ctrl->base + EDP_MAINLINK_READY);
		if (data & bit)
			break;
	}

	if (cnt == 0)
		pr_err("%s: set link_train=%d failed\n", __func__, train);
}

static const u8 vm_pre_emphasis[4][4] = {
	{0x03, 0x06, 0x09, 0x0C},	/* pe0, 0 db */
	{0x03, 0x06, 0x09, 0xFF},	/* pe1, 3.5 db */
	{0x03, 0x06, 0xFF, 0xFF},	/* pe2, 6.0 db */
	{0x03, 0xFF, 0xFF, 0xFF}	/* pe3, 9.5 db */
};

/* voltage swing, 0.2v and 1.0v are not support */
static const u8 vm_voltage_swing[4][4] = {
	{0x14, 0x18, 0x1A, 0x1E}, /* sw0, 0.4v  */
	{0x18, 0x1A, 0x1E, 0xFF}, /* sw1, 0.6 v */
	{0x1A, 0x1E, 0xFF, 0xFF}, /* sw1, 0.8 v */
	{0x1E, 0xFF, 0xFF, 0xFF}  /* sw1, 1.2 v, optional */
};

static int edp_voltage_pre_emphasise_set(struct edp_ctrl *ctrl)
{
	u32 value0;
	u32 value1;

	DBG("v=%d p=%d", ctrl->v_level, ctrl->p_level);

	value0 = vm_pre_emphasis[(int)(ctrl->v_level)][(int)(ctrl->p_level)];
	value1 = vm_voltage_swing[(int)(ctrl->v_level)][(int)(ctrl->p_level)];

	/* Configure host and panel only if both values are allowed */
	if (value0 != 0xFF && value1 != 0xFF) {
		msm_edp_phy_vm_pe_cfg(ctrl->phy_priv, value0, value1);
		return edp_lane_set_write(ctrl, ctrl->v_level, ctrl->p_level);
	}

	return -EINVAL;
}

static int edp_start_link_train_1(struct edp_ctrl *ctrl)
{
	u8 link_status[DP_LINK_STATUS_SIZE];
	u8 old_v_level;
	int tries;
	int ret = 0;
	int rlen;

	DBG("");

	edp_host_train_set(ctrl, 0x01); /* train_1 */
	ret = edp_voltage_pre_emphasise_set(ctrl);
	if (ret)
		return ret;
	ret = edp_train_pattern_set_write(ctrl, 0x21); /* train_1 */
	if (ret)
		return ret;

	tries = 0;
	old_v_level = ctrl->v_level;
	while (1) {
		drm_dp_link_train_clock_recovery_delay(ctrl->dpcd);

		rlen = drm_dp_dpcd_read_link_status(ctrl->drm_aux, link_status);
		if (rlen < DP_LINK_STATUS_SIZE) {
			pr_err("%s: read link status failed\n", __func__);
			return -ENOLINK;
		}
		if (drm_dp_clock_recovery_ok(link_status, ctrl->lane_cnt)) {
			ret = 0;
			break;
		}

		if (ctrl->v_level == DPCD_LINK_VOLTAGE_MAX) {
			ret = -1;
			break;	/* quit */
		}

		if (old_v_level == ctrl->v_level) {
			tries++;
			if (tries >= 5) {
				ret = -1;
				break;	/* quit */
			}
		} else {
			tries = 0;
			old_v_level = ctrl->v_level;
		}

		edp_sink_train_set_adjust(ctrl, link_status);
		ret = edp_voltage_pre_emphasise_set(ctrl);
		if (ret)
			return ret;
	}

	return ret;
}

static int edp_start_link_train_2(struct edp_ctrl *ctrl)
{
	u8 link_status[DP_LINK_STATUS_SIZE];
	int tries;
	int ret = 0;
	int rlen;

	DBG("");

	edp_host_train_set(ctrl, 0x02); /* train_2 */
	ret = edp_voltage_pre_emphasise_set(ctrl);
	if (ret)
		return ret;

	ret = edp_train_pattern_set_write(ctrl, 0x22); /* train_2 */
	if (ret)
		return ret;

	tries = 0;
	while (1) {
		drm_dp_link_train_channel_eq_delay(ctrl->dpcd);

		rlen = drm_dp_dpcd_read_link_status(ctrl->drm_aux, link_status);
		if (rlen < DP_LINK_STATUS_SIZE) {
			pr_err("%s: read link status failed\n", __func__);
			return -ENOLINK;
		}
		if (drm_dp_channel_eq_ok(link_status, ctrl->lane_cnt)) {
			ret = 0;
			break;
		}

		tries++;
		if (tries > 10) {
			ret = -1;
			break;
		}

		edp_sink_train_set_adjust(ctrl, link_status);
		ret = edp_voltage_pre_emphasise_set(ctrl);
		if (ret)
			return ret;
	}

	return ret;
}

static int edp_link_rate_down_shift(struct edp_ctrl *ctrl)
{
	u32 prate, lrate, bpp;
	u8 rate, lane, max_lane;
	int changed = 0;

	rate = ctrl->link_rate;
	lane = ctrl->lane_cnt;
	max_lane = ctrl->dp_link.num_lanes;

	bpp = ctrl->color_depth * RGB_COMPONENTS;
	prate = ctrl->pixel_rate;
	prate *= bpp;
	prate /= 8; /* in kByte */

	if (rate > DP_LINK_BW_1_62 && rate <= EDP_LINK_BW_MAX) {
		rate -= 4;	/* reduce rate */
		changed++;
	}

	if (changed) {
		if (lane >= 1 && lane < max_lane)
			lane <<= 1;	/* increase lane */

		lrate = 270000; /* in kHz */
		lrate *= rate;
		lrate /= 10; /* kByte, 10 bits --> 8 bits */
		lrate *= lane;

		DBG("new lrate=%u prate=%u(kHz) rate=%d lane=%d p=%u b=%d",
			lrate, prate, rate, lane,
			ctrl->pixel_rate,
			bpp);

		if (lrate > prate) {
			ctrl->link_rate = rate;
			ctrl->lane_cnt = lane;
			DBG("new rate=%d %d", rate, lane);
			return 0;
		}
	}

	return -EINVAL;
}

static int edp_clear_training_pattern(struct edp_ctrl *ctrl)
{
	int ret;

	ret = edp_train_pattern_set_write(ctrl, 0);

	drm_dp_link_train_channel_eq_delay(ctrl->dpcd);

	return ret;
}

static int edp_do_link_train(struct edp_ctrl *ctrl)
{
	int ret = EDP_TRAIN_SUCCESS;
	struct drm_dp_link dp_link;

	DBG("");
	/*
	 * Set the current link rate and lane cnt to panel. They may have been
	 * adjusted and the values are different from them in DPCD CAP
	 */
	dp_link.num_lanes = ctrl->lane_cnt;
	dp_link.rate = drm_dp_bw_code_to_link_rate(ctrl->link_rate);
	dp_link.capabilities = ctrl->dp_link.capabilities;
	if (drm_dp_link_configure(ctrl->drm_aux, &dp_link) < 0)
		return EDP_TRAIN_FAIL;

	ctrl->v_level = 0; /* start from default level */
	ctrl->p_level = 0;

	edp_state_ctrl(ctrl, 0);
	if (edp_clear_training_pattern(ctrl))
		return EDP_TRAIN_FAIL;

	ret = edp_start_link_train_1(ctrl);
	if (ret < 0) {
		if (edp_link_rate_down_shift(ctrl) == 0) {
			DBG("link reconfig");
			ret = EDP_TRAIN_RECONFIG;
			goto clear;
		} else {
			pr_err("%s: Training 1 failed", __func__);
			ret = EDP_TRAIN_FAIL;
			goto clear;
		}
	}
	DBG("Training 1 completed successfully");

	edp_state_ctrl(ctrl, 0);
	if (edp_clear_training_pattern(ctrl))
		return EDP_TRAIN_FAIL;

	ret = edp_start_link_train_2(ctrl);
	if (ret < 0) {
		if (edp_link_rate_down_shift(ctrl) == 0) {
			DBG("link reconfig");
			ret = EDP_TRAIN_RECONFIG;
			goto clear;
		} else {
			pr_err("%s: Training 2 failed", __func__);
			ret = EDP_TRAIN_FAIL;
			goto clear;
		}
	}
	DBG("Training 2 completed successfully");

	edp_state_ctrl(ctrl, ST_SEND_VIDEO);
clear:
	edp_clear_training_pattern(ctrl);

	return ret;
}

static void edp_clock_synchrous(struct edp_ctrl *ctrl, int sync)
{
	u32 data;
	u32 color;

	data = edp_read(ctrl->base + EDP_MISC1_MISC0);

	if (sync)
		data |= 0x01;
	else
		data &= ~0x01;

	/* only legacy rgb mode supported */
	color = 0; /* 6 bits */
	if (ctrl->color_depth == 8)
		color = 0x01;
	else if (ctrl->color_depth == 10)
		color = 0x02;
	else if (ctrl->color_depth == 12)
		color = 0x03;
	else if (ctrl->color_depth == 16)
		color = 0x04;

	color <<= 5;    /* bit 5 to bit 7 */

	data &= 0xffffff1f;
	data |= color;

	edp_write(ctrl->base + EDP_MISC1_MISC0, data);
}

static int edp_sw_mvid_nvid(struct edp_ctrl *ctrl, u32 m, u32 n)
{
	u32 n_multi, m_multi = 5;

	if (ctrl->link_rate == DP_LINK_BW_1_62) {
		n_multi = 1;
	} else if (ctrl->link_rate == DP_LINK_BW_2_7) {
		n_multi = 2;
	} else {
		pr_err("%s: Invalid link rate, %d\n", __func__,
			ctrl->link_rate);
		return -EINVAL;
	}

	edp_write(ctrl->base + EDP_SOFTWARE_MVID, m * m_multi);
	edp_write(ctrl->base + EDP_SOFTWARE_NVID, n * n_multi);

	return 0;
}

static void edp_mainlink_ctrl(struct edp_ctrl *ctrl, int enable)
{
	u32 data = 0;

	edp_write(ctrl->base + EDP_MAINLINK_CTRL, 0x02); /* reset */
	wmb();
	usleep_range(500, 1000);

	if (enable)
		data |= 0x1;

	edp_write(ctrl->base + EDP_MAINLINK_CTRL, data);
}

static void edp_ctrl_phy_aux_enable(struct edp_ctrl *ctrl, int enable)
{
	if (enable) {
		edp_clk_enable(ctrl, EDP_CLK_MASK_AUX_CHAN);
		msm_edp_phy_ctrl(ctrl->phy_priv, 1);
		msm_edp_aux_ctrl(ctrl->aux_priv, 1);
		gpio_set_value(ctrl->gpio_panel_en, 1);
	} else {
		gpio_set_value(ctrl->gpio_panel_en, 0);
		msm_edp_aux_ctrl(ctrl->aux_priv, 0);
		msm_edp_phy_ctrl(ctrl->phy_priv, 0);
		edp_clk_disable(ctrl, EDP_CLK_MASK_AUX_CHAN);
	}
}

static void edp_ctrl_link_enable(struct edp_ctrl *ctrl, int enable)
{
	u32 m, n;

	if (enable) {
		/* Enable link channel clocks */
		edp_clk_enable(ctrl, EDP_CLK_MASK_LINK_CHAN);

		msm_edp_phy_lane_power_ctrl(ctrl->phy_priv, 1, ctrl->lane_cnt);

		msm_edp_phy_vm_pe_init(ctrl->phy_priv);

		/* Make sure phy is programed */
		wmb();
		msm_edp_phy_ready(ctrl->phy_priv);

		edp_config_ctrl(ctrl);
		msm_edp_ctrl_pixel_clock_valid(ctrl, ctrl->pixel_rate, &m, &n);
		edp_sw_mvid_nvid(ctrl, m, n);
		edp_mainlink_ctrl(ctrl, 1);
	} else {
		edp_mainlink_ctrl(ctrl, 0);

		msm_edp_phy_lane_power_ctrl(ctrl->phy_priv, 0, 0);
		edp_clk_disable(ctrl, EDP_CLK_MASK_LINK_CHAN);
	}
}

static int edp_ctrl_training(struct edp_ctrl *ctrl)
{
	int ret;

	/* Do link training only when power is on */
	if (ctrl->cont_splash || (!ctrl->power_on))
		return -EINVAL;

train_start:
	ret = edp_do_link_train(ctrl);
	if (ret == EDP_TRAIN_RECONFIG) {
		/* Re-configure main link */
		edp_ctrl_irq_enable(ctrl, 0);
		edp_ctrl_link_enable(ctrl, 0);
		msm_edp_phy_ctrl(ctrl->phy_priv, 0);

		/* Make sure link is fully disabled */
		wmb();
		usleep_range(500, 1000);

		msm_edp_phy_ctrl(ctrl->phy_priv, 1);
		edp_ctrl_link_enable(ctrl, 1);
		edp_ctrl_irq_enable(ctrl, 1);
		goto train_start;
	}

	return ret;
}

static int edp_ctrl_set_backlight(struct edp_ctrl *ctrl, u32 bl_level)
{
	int ret = 0;
	int period_ns;

	if (!ctrl->bl_pwm) {
		pr_err("%s: pwm is not initialized\n", __func__);
		return -ENODEV;
	}

	if (bl_level > EDP_BACKLIGHT_MAX)
		bl_level = EDP_BACKLIGHT_MAX;

	mutex_lock(&ctrl->bl_mutex);
	if (bl_level == ctrl->bl_level)
		goto unlock_ret;

	period_ns = ctrl->pwm_period * NSEC_PER_USEC;
	ret = pwm_config(ctrl->bl_pwm,
		div_u64((u64)bl_level * period_ns, EDP_BACKLIGHT_MAX),
		period_ns);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		goto unlock_ret;
	}

	if (ctrl->bl_level > 0) {
		pwm_disable(ctrl->bl_pwm);
		ctrl->bl_level = 0;
	}

	/* Keep the power off when level is 0 */
	if (!bl_level) {
		gpio_set_value(ctrl->gpio_bkl_en, 0);
		ctrl->bl_level = 0;
		goto unlock_ret;
	}

	ret = pwm_enable(ctrl->bl_pwm);
	if (ret) {
		pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
		goto unlock_ret;
	}
	ctrl->bl_level = bl_level;
	gpio_set_value(ctrl->gpio_bkl_en, 1);

unlock_ret:
	mutex_unlock(&ctrl->bl_mutex);
	return ret;
}

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
/*
 * Backlight Interfaces
 */
static int edp_backlight_get_brightness(struct backlight_device *bd)
{
	struct edp_ctrl *ctrl = bl_get_data(bd);

	return ctrl->bl_level;
}

static int edp_backlight_set_brightness(struct backlight_device *bd)
{
	struct edp_ctrl *ctrl = bl_get_data(bd);

	if (bd->props.brightness > bd->props.max_brightness)
		bd->props.brightness = bd->props.max_brightness;

	return edp_ctrl_set_backlight(ctrl, bd->props.brightness);
}

static const struct backlight_ops edp_backlight_ops = {
	.get_brightness = edp_backlight_get_brightness,
	.update_status  = edp_backlight_set_brightness,
};

static int edp_backlight_dev_register(struct edp_ctrl *ctrl)
{
	struct backlight_properties props;
	int ret;

	memset(&props, 0, sizeof(struct backlight_properties));
	/* Use the same max value as in driver */
	props.max_brightness = EDP_BACKLIGHT_MAX;
	props.type = BACKLIGHT_RAW;

	ctrl->backlight_dev = backlight_device_register("edp-backlight",
				NULL, (void *)ctrl, &edp_backlight_ops, &props);
	if (IS_ERR(ctrl->backlight_dev)) {
		pr_err("%s: Register backlight device failed\n", __func__);
		ret = PTR_ERR(ctrl->backlight_dev);
		ctrl->backlight_dev = NULL;
		return ret;
	}

	ctrl->backlight_dev->props.brightness =
			edp_backlight_get_brightness(ctrl->backlight_dev);
	backlight_update_status(ctrl->backlight_dev);

	return 0;
}

static void edp_backlight_dev_unregister(struct edp_ctrl *ctrl)
{
	if (ctrl->backlight_dev) {
		backlight_device_unregister(ctrl->backlight_dev);
		ctrl->backlight_dev = NULL;
	}
}
#else
static int edp_backlight_dev_register(struct edp_ctrl *ctrl)
{
	return 0;
}

static void edp_backlight_dev_unregister(struct edp_ctrl *ctrl)
{
}
#endif

static void edp_ctrl_on_worker(struct work_struct *work)
{
	struct edp_ctrl *ctrl = container_of(
				work, struct edp_ctrl, on_work);
	int ret;

	mutex_lock(&ctrl->dev_mutex);

	if (ctrl->power_on) {
		DBG("already on");
		goto unlock_ret;
	}

	DBG("+, cont_splash=%d", ctrl->cont_splash);

	if (!ctrl->cont_splash) {
		edp_ctrl_phy_aux_enable(ctrl, 1);
		edp_ctrl_link_enable(ctrl, 1);
	}

	edp_ctrl_irq_enable(ctrl, 1);
	ret = drm_dp_link_power_up(ctrl->drm_aux, &ctrl->dp_link);
	if (ret)
		goto fail;

	ctrl->power_on = true;
	ctrl->cont_splash = 0;

	/* Start link training */
	ret = edp_ctrl_training(ctrl);
	if (ret != EDP_TRAIN_SUCCESS)
		goto fail;

	/* Turn on backlight to max brightness */
	edp_ctrl_set_backlight(ctrl, EDP_BACKLIGHT_MAX);

	DBG("DONE");
	goto unlock_ret;

fail:
	edp_ctrl_irq_enable(ctrl, 0);
	if (!ctrl->cont_splash) {
		edp_ctrl_link_enable(ctrl, 0);
		edp_ctrl_phy_aux_enable(ctrl, 0);
	}
	ctrl->power_on = false;
unlock_ret:
	mutex_unlock(&ctrl->dev_mutex);
}

static void edp_ctrl_off_worker(struct work_struct *work)
{
	struct edp_ctrl *ctrl = container_of(
				work, struct edp_ctrl, off_work);
	int ret = 0;

	mutex_lock(&ctrl->dev_mutex);

	if (!ctrl->power_on) {
		DBG("already off");
		goto unlock_ret;
	}

	/* Turn off backlight */
	edp_ctrl_set_backlight(ctrl, 0);

	reinit_completion(&ctrl->idle_comp);
	edp_state_ctrl(ctrl, ST_PUSH_IDLE);

	ret = wait_for_completion_timeout(&ctrl->idle_comp,
						msecs_to_jiffies(500));
	if (ret <= 0)
		pr_err("%s: idle pattern timedout, %d\n",
				__func__, ret);

	edp_state_ctrl(ctrl, 0);

	drm_dp_link_power_down(ctrl->drm_aux, &ctrl->dp_link);

	edp_ctrl_irq_enable(ctrl, 0);

	edp_ctrl_link_enable(ctrl, 0);

	edp_ctrl_phy_aux_enable(ctrl, 0);

	ctrl->cont_splash = false;
	ctrl->power_on = false;

unlock_ret:
	mutex_unlock(&ctrl->dev_mutex);
}

irqreturn_t msm_edp_ctrl_irq(void *edp_ctrl)
{
	struct edp_ctrl *ctrl = edp_ctrl;
	u32 isr1, isr2, mask1, mask2;
	u32 ack;

	DBG("");
	spin_lock(&ctrl->irq_lock);
	isr1 = edp_read(ctrl->base + EDP_INTERRUPT_REG_1);
	isr2 = edp_read(ctrl->base + EDP_INTERRUPT_REG_2);

	mask1 = isr1 & EDP_INTR_MASK1;
	mask2 = isr2 & EDP_INTR_MASK2;

	isr1 &= ~mask1;	/* remove masks bit */
	isr2 &= ~mask2;

	DBG("isr=%x mask=%x isr2=%x mask2=%x",
			isr1, mask1, isr2, mask2);

	ack = isr1 & EDP_INTR_STATUS1;
	ack <<= 1;	/* ack bits */
	ack |= mask1;
	edp_write(ctrl->base + EDP_INTERRUPT_REG_1, ack);

	ack = isr2 & EDP_INTR_STATUS2;
	ack <<= 1;	/* ack bits */
	ack |= mask2;
	edp_write(ctrl->base + EDP_INTERRUPT_REG_2, ack);
	spin_unlock(&ctrl->irq_lock);

	if (isr1 & EDP_INTR_HPD)
		DBG("edp_hpd");

	if (isr2 & EDP_INTR_READY_FOR_VIDEO)
		DBG("edp_video_ready");

	if (isr2 & EDP_INTR_IDLE_PATTERNs_SENT) {
		DBG("idle_patterns_sent");
		complete(&ctrl->idle_comp);
	}

	msm_edp_aux_irq(ctrl->aux_priv, isr1);

	return IRQ_HANDLED;
}

void msm_edp_ctrl_power(void *edp_ctrl, bool on)
{
	struct edp_ctrl *ctrl = edp_ctrl;

	if (on)
		queue_work(ctrl->workqueue, &ctrl->on_work);
	else
		queue_work(ctrl->workqueue, &ctrl->off_work);
}

int msm_edp_ctrl_init(struct msm_edp *edp)
{
	struct edp_ctrl *ctrl = NULL;
	int ret = 0;

	if (!edp) {
		pr_err("%s: edp is NULL!\n", __func__);
		return -EINVAL;
	}

	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	edp->ctrl_priv = ctrl;
	ctrl->pdev = edp->pdev;

	ctrl->gpio_bkl_en = -1;
	ctrl->gpio_lvl_en = -1;
	ctrl->gpio_panel_en = -1;
	ctrl->gpio_panel_hpd = -1;

	ctrl->cont_splash = edp_cont_splash;

	DBG("cont_splash=%d", ctrl->cont_splash);

	ctrl->base = msm_ioremap(ctrl->pdev, "edp", "eDP");
	if (IS_ERR(ctrl->base))
		goto f0;

	spin_lock_init(&ctrl->irq_lock);

	mutex_init(&ctrl->dev_mutex);
	mutex_init(&ctrl->bl_mutex);
	init_completion(&ctrl->idle_comp);

	/* Get regulator, clock, gpio, pwm */
	ret = edp_regulator_init(ctrl);
	if (ret) {
		pr_err("%s:regulator init fail\n", __func__);
		goto f1;
	}
	ret = edp_clk_init(ctrl);
	if (ret) {
		pr_err("%s:clk init fail\n", __func__);
		goto f2;
	}
	ret = edp_gpio_config(ctrl, true);
	if (ret) {
		pr_err("%s:failed to configure GPIOs: %d", __func__, ret);
		goto f3;
	}
	ret = edp_pwm_config(ctrl, true);
	if (ret) {
		pr_err("%s:failed to get pwm: %d", __func__, ret);
		goto f4;
	}

	edp_level_trans_enable(ctrl);

	if (ctrl->cont_splash) {
		/* vote for clocks */
		edp_clk_enable(ctrl, EDP_CLK_MASK_ALL);
		ctrl->power_on = true;
		pwm_enable(ctrl->bl_pwm);
		ctrl->bl_level = EDP_BACKLIGHT_MAX;
	} else {
		/* Make sure the back light is off before on */
		pwm_disable(ctrl->bl_pwm);
		ctrl->bl_level = 0;
	}

	/* Init aux and phy */
	ctrl->aux_priv = msm_edp_aux_init(&ctrl->pdev->dev, ctrl->base,
					&ctrl->drm_aux);
	if (!ctrl->aux_priv || !ctrl->drm_aux) {
		pr_err("%s:failed to init aux\n", __func__);
		goto f5;
	}

	ctrl->phy_priv = msm_edp_phy_init(ctrl->base);
	if (!ctrl->phy_priv) {
		pr_err("%s:failed to init phy\n", __func__);
		goto f6;
	}

	/* setup workqueue */
	ctrl->workqueue = alloc_ordered_workqueue("edp_drm_work", 0);
	INIT_WORK(&ctrl->on_work, edp_ctrl_on_worker);
	INIT_WORK(&ctrl->off_work, edp_ctrl_off_worker);

	/* Register backlight device */
	edp_backlight_dev_register(ctrl);

	return 0;

f6:
	msm_edp_aux_destroy(ctrl->aux_priv);
	ctrl->aux_priv = NULL;
f5:
	edp_pwm_config(ctrl, false);
f4:
	edp_gpio_config(ctrl, false);
f3:
	edp_clk_deinit(ctrl);
f2:
	edp_regulator_deinit(ctrl);
f1:
	mutex_destroy(&ctrl->bl_mutex);
	mutex_destroy(&ctrl->dev_mutex);
	iounmap(ctrl->base);
	ctrl->base = NULL;
	kfree(ctrl);
	edp->ctrl_priv = NULL;
f0:
	return ret;
}

void msm_edp_ctrl_destroy(void *edp_ctrl)
{
	struct edp_ctrl *ctrl = edp_ctrl;

	if (!ctrl)
		return;

	edp_backlight_dev_unregister(ctrl);
	if (ctrl->workqueue) {
		flush_workqueue(ctrl->workqueue);
		destroy_workqueue(ctrl->workqueue);
		ctrl->workqueue = NULL;
	}

	if (ctrl->phy_priv) {
		msm_edp_phy_destroy(ctrl->phy_priv);
		ctrl->phy_priv = NULL;
	}

	if (ctrl->aux_priv) {
		msm_edp_aux_destroy(ctrl->aux_priv);
		ctrl->aux_priv = NULL;
	}

	kfree(ctrl->edid);
	ctrl->edid = NULL;

	edp_level_trans_disable(ctrl);
	edp_pwm_config(ctrl, false);
	edp_gpio_config(ctrl, false);
	edp_clk_deinit(ctrl);
	edp_regulator_deinit(ctrl);

	mutex_destroy(&ctrl->bl_mutex);
	mutex_destroy(&ctrl->dev_mutex);
	if (ctrl->base) {
		devm_iounmap(&ctrl->pdev->dev, ctrl->base);
		ctrl->base = NULL;
	}

	kfree(ctrl);
}

bool msm_edp_ctrl_panel_connected(void *edp_ctrl)
{
	struct edp_ctrl *ctrl = edp_ctrl;
	bool ret;

	mutex_lock(&ctrl->dev_mutex);
	DBG("connect status = %d", ctrl->edp_connected);
	if (ctrl->edp_connected) {
		mutex_unlock(&ctrl->dev_mutex);
		return true;
	}

	if (!ctrl->power_on) {
		if (!ctrl->cont_splash)
			edp_ctrl_phy_aux_enable(ctrl, 1);
		edp_ctrl_irq_enable(ctrl, 1);
	}

	if (drm_dp_dpcd_read(ctrl->drm_aux, DP_DPCD_REV, ctrl->dpcd,
				DP_RECEIVER_CAP_SIZE) < DP_RECEIVER_CAP_SIZE) {
		pr_err("%s: AUX channel is NOT ready\n", __func__);
		memset(ctrl->dpcd, 0, DP_RECEIVER_CAP_SIZE);
	} else {
		ctrl->edp_connected = true;
	}

	if (!ctrl->power_on) {
		edp_ctrl_irq_enable(ctrl, 0);
		if (!ctrl->cont_splash)
			edp_ctrl_phy_aux_enable(ctrl, 0);
	}

	DBG("exit: connect status=%d", ctrl->edp_connected);

	ret = ctrl->edp_connected;
	mutex_unlock(&ctrl->dev_mutex);

	return ret;
}

int msm_edp_ctrl_get_edid(void *edp_ctrl,
		struct drm_connector *connector, struct edid **edid)
{
	struct edp_ctrl *ctrl = edp_ctrl;
	int ret = 0;

	mutex_lock(&ctrl->dev_mutex);

	if (ctrl->edid) {
		if (edid) {
			DBG("Just return edid buffer");
			*edid = ctrl->edid;
		}
		goto unlock_ret;
	}

	if (!ctrl->power_on) {
		if (!ctrl->cont_splash)
			edp_ctrl_phy_aux_enable(ctrl, 1);
		edp_ctrl_irq_enable(ctrl, 1);
	}

	ret = drm_dp_link_probe(ctrl->drm_aux, &ctrl->dp_link);
	if (ret) {
		pr_err("%s: read dpcd cap failed, %d\n", __func__, ret);
		goto disable_ret;
	}

	/* Initialize link rate as panel max link rate */
	ctrl->link_rate = drm_dp_link_rate_to_bw_code(ctrl->dp_link.rate);

	ctrl->edid = drm_get_edid(connector, &ctrl->drm_aux->ddc);
	if (!ctrl->edid) {
		pr_err("%s: edid read fail\n", __func__);
		goto disable_ret;
	}

	if (edid)
		*edid = ctrl->edid;

disable_ret:
	if (!ctrl->power_on) {
		edp_ctrl_irq_enable(ctrl, 0);
		if (!ctrl->cont_splash)
			edp_ctrl_phy_aux_enable(ctrl, 0);
	}
unlock_ret:
	mutex_unlock(&ctrl->dev_mutex);
	return ret;
}

int msm_edp_ctrl_timing_cfg(void *edp_ctrl,
	struct drm_display_mode *mode, struct drm_display_info *info)
{
	struct edp_ctrl *ctrl = edp_ctrl;
	u32 data = 0;
	int ret = 0;

	mutex_lock(&ctrl->dev_mutex);
	/*
	 * Need to keep color depth, pixel rate and
	 * interlaced information in ctrl context
	 */
	ctrl->color_depth = info->bpc;
	ctrl->pixel_rate = mode->clock;
	ctrl->interlaced = !!(mode->flags & DRM_MODE_FLAG_INTERLACE);

	/* Fill initial link config based on passed in timing */
	edp_fill_link_cfg(ctrl);

	if (edp_clk_enable(ctrl, EDP_CLK_MASK_AHB)) {
		pr_err("%s, fail to prepare enable ahb clk\n", __func__);
		ret = -EINVAL;
		goto unlock_ret;
	}
	edp_clock_synchrous(ctrl, 1);

	/* Configure eDP timing to HW */
	data = mode->vtotal;
	data <<= 16;
	data |= mode->htotal;
	edp_write(ctrl->base + EDP_TOTAL_HOR_VER, data);

	data = mode->vtotal - mode->vsync_start;
	data <<= 16;
	data |= (mode->htotal - mode->hsync_start);
	edp_write(ctrl->base + EDP_START_HOR_VER_FROM_SYNC, data);

	data = mode->vsync_end - mode->vsync_start;
	data <<= 16;
	data |= (mode->hsync_end - mode->hsync_start);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		data |= BIT(31);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		data |= BIT(15);
	edp_write(ctrl->base + EDP_HSYNC_VSYNC_WIDTH_POLARITY, data);

	data = mode->vdisplay;
	data <<= 16;
	data |= mode->hdisplay;
	edp_write(ctrl->base + EDP_ACTIVE_HOR_VER, data);

	edp_clk_disable(ctrl, EDP_CLK_MASK_AHB);

unlock_ret:
	mutex_unlock(&ctrl->dev_mutex);
	return ret;
}

bool msm_edp_ctrl_pixel_clock_valid(void *edp_ctrl,
	u32 pixel_rate, u32 *pm, u32 *pn)
{
	struct edp_ctrl *ctrl = edp_ctrl;
	const struct edp_pixel_clk_div *divs;
	u32 err = 1; /* 1% error tolerance */
	u32 clk_err;
	int i;

	if (ctrl->link_rate == DP_LINK_BW_1_62) {
		divs = clk_divs[0];
	} else if (ctrl->link_rate == DP_LINK_BW_2_7) {
		divs = clk_divs[1];
	} else {
		pr_err("%s: Invalid link rate,%d\n", __func__, ctrl->link_rate);
		return false;
	}

	for (i = 0; i < EDP_PIXEL_CLK_NUM; i++) {
		clk_err = abs(divs[i].rate - pixel_rate);
		if ((divs[i].rate * err / 100) >= clk_err) {
			if (pm)
				*pm = divs[i].m;
			if (pn)
				*pn = divs[i].n;
			return true;
		}
	}

	DBG("pixel clock %d(kHz) not supported", pixel_rate);

	return false;
}

