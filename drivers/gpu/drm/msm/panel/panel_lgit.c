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
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/regulator/consumer.h>

#include "panel.h"

struct panel_lgit {
	struct panel base;
	struct mipi_adapter *mipi;
	struct regulator *reg_l8;
	struct regulator *reg_l2;
	struct regulator *reg_lvs6;
	struct regulator *ext_dsv_load;
	int gpio42;
};
#define to_panel_lgit(x) container_of(x, struct panel_lgit, base)

#define DSV_ONBST 57
#define DISP_RST  (41 + NR_GPIO_IRQS)

// XXX hack.. use proper bl framework..
void lm3530_lcd_backlight_set_level(int level);
void lm3530_lcd_backlight_pwm_disable(void);

/* --------- start copy/paste: -------------------------------------- */
/* command sequences taken from board-mako-display.c.. avoiding
 * reformatting to make copy/paste updates easier.
 */
static char lcd_mirror [2] = {0x36, 0x02};
static char panel_setting_1 [6] = {0xB0, 0x43, 0x00, 0x00, 0x00, 0x00};
static char panel_setting_2 [3] = {0xB3, 0x0A, 0x9F};

static char display_mode1 [6] = {0xB5, 0x50, 0x20, 0x40, 0x00, 0x20};
static char display_mode2 [8] = {0xB6, 0x00, 0x14, 0x0F, 0x16, 0x13, 0x05, 0x05};

static char p_gamma_r_setting[10] = {0xD0, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char n_gamma_r_setting[10] = {0xD1, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char p_gamma_g_setting[10] = {0xD2, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char n_gamma_g_setting[10] = {0xD3, 0x40, 0x44, 0x76, 0x01, 0x00, 0x00, 0x30, 0x20, 0x01};
static char p_gamma_b_setting[10] = {0xD4, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03};
static char n_gamma_b_setting[10] = {0xD5, 0x20, 0x23, 0x74, 0x00, 0x1F, 0x10, 0x50, 0x33, 0x03};

static char ief_on_set0[2] = {0xE0, 0x00};
static char ief_on_set4[4] = {0xE4, 0x00, 0x00, 0x00};
static char ief_on_set5[4] = {0xE5, 0x00, 0x00, 0x00};
static char ief_on_set6[4] = {0xE6, 0x00, 0x00, 0x00};

static char ief_set1[5] = {0xE1, 0x00, 0x00, 0x01, 0x01};
static char ief_set2[3] = {0xE2, 0x01, 0x00};
static char ief_set3[6] = {0xE3, 0x00, 0x00, 0x42, 0x35, 0x00};
static char ief_set7[9] = {0xE7, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
static char ief_set8[9] = {0xE8, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D};
static char ief_set9[9] = {0xE9, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B};
static char ief_setA[9] = {0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char ief_setB[9] = {0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char ief_setC[9] = {0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static char osc_setting[4] =     {0xC0, 0x00, 0x0A, 0x10};
static char power_setting3[13] = {0xC3, 0x00, 0x88, 0x03, 0x20, 0x01, 0x57, 0x4F, 0x33,0x02,0x38,0x38,0x00};
static char power_setting4[6] =  {0xC4, 0x31, 0x24, 0x11, 0x11, 0x3D};
static char power_setting5[4] =  {0xC5, 0x3B, 0x3B, 0x03};

static char cabc_set0[2] = {0x51, 0xFF};
static char cabc_set1[2] = {0x5E, 0x00}; // CABC MIN
static char cabc_set2[2] = {0x53, 0x2C};
static char cabc_set3[2] = {0x55, 0x02};
static char cabc_set4[6] = {0xC8, 0x22, 0x22, 0x22, 0x33, 0x80};//A-CABC applied

static char exit_sleep_power_control_2[2] =  {0xC2,0x06};
static char exit_sleep_power_control_3[2] =  {0xC2,0x0E};
static char otp_protection[3] =  {0xF1,0x10,0x00};
static char sleep_out_for_cabc[2] = {0x11,0x00};
static char gate_output_enabled_by_manual[2] = {0xC1,0x08};

static char display_on[2] =  {0x29,0x00};

static char display_off[2] = {0x28,0x00};

static char enter_sleep[2] = {0x10,0x00};

static char analog_boosting_power_control[2] = {0xC2,0x00};
static char enter_sleep_power_control_3[2] = {0xC2,0x01};
static char enter_sleep_power_control_2[2] = {0xC2,0x00};

static char deep_standby[2] = {0xC1,0x02};
/* --------- end copy/paste: ---------------------------------------- */


static void panel_lgit_destroy(struct panel *panel)
{
	struct panel_lgit *panel_lgit = to_panel_lgit(panel);
	kfree(panel_lgit);
}

static int toggle_lcd_reset(struct panel *panel, int val)
{
	struct pm_gpio gpio_param = {
		.direction = PM_GPIO_DIR_OUT,
		.output_buffer = PM_GPIO_OUT_BUF_CMOS,
		.output_value = val,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.out_strength = PM_GPIO_STRENGTH_HIGH,
		.function = PM_GPIO_FUNC_PAIRED,
		.inv_int_pol = 0,
		.disable_pin = 0,
	};
	int ret;

	DRM_DEBUG_KMS("set LCD RESET %s\n", val ? "HIGH" : "LOW");

	ret = pm8xxx_gpio_config(DISP_RST, &gpio_param);
	if (ret) {
		dev_err(panel->dev->dev, "failed to set LCD reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_lgit_power_on(struct panel *panel)
{
	struct drm_device *dev = panel->dev;
	struct panel_lgit *panel_lgit = to_panel_lgit(panel);
	int ret = 0;

	if (panel_lgit->ext_dsv_load) {
		ret = regulator_enable(panel_lgit->ext_dsv_load);
		if (ret) {
			dev_err(dev->dev, "failed to enable ext_dsv_load: %d\n", ret);
			goto fail1;
		}
	}

	ret = regulator_set_optimum_mode(panel_lgit->reg_l8, 100000);
	if (ret < 0) {
		dev_err(dev->dev, "failed to set l8 mode: %d\n", ret);
		goto fail2;
	}

	ret = regulator_set_optimum_mode(panel_lgit->reg_l2, 100000);
	if (ret < 0) {
		dev_err(dev->dev, "failed to set l2 mode: %d\n", ret);
		goto fail2;
	}

	ret = regulator_enable(panel_lgit->reg_l8);
	if (ret) {
		dev_err(dev->dev, "failed to enable l8: %d\n", ret);
		goto fail2;
	}

	udelay(100);

	ret = regulator_enable(panel_lgit->reg_lvs6);
	if (ret) {
		dev_err(dev->dev, "failed to enable lvs6: %d\n", ret);
		goto fail3;
	}

	udelay(100);

	ret = regulator_enable(panel_lgit->reg_l2);
	if (ret) {
		dev_err(dev->dev, "failed to enable l2: %d\n", ret);
		goto fail4;
	}

	mdelay(2);

	ret = toggle_lcd_reset(panel, 1);
	if (ret)
		goto fail5;

	mdelay(5);

	return 0;

fail5:
	regulator_disable(panel_lgit->reg_l2);
fail4:
	regulator_disable(panel_lgit->reg_lvs6);
fail3:
	regulator_disable(panel_lgit->reg_l8);
fail2:
	if (panel_lgit->ext_dsv_load)
		regulator_disable(panel_lgit->ext_dsv_load);
fail1:
	return ret;
}

static int panel_lgit_power_off(struct panel *panel)
{
	struct drm_device *dev = panel->dev;
	struct panel_lgit *panel_lgit = to_panel_lgit(panel);
	int ret;

	toggle_lcd_reset(panel, 0);
	udelay(100);

	ret = regulator_disable(panel_lgit->reg_lvs6);
	if (ret)
		dev_err(dev->dev, "failed to disable lvs6: %d\n", ret);

	udelay(100);

	ret = regulator_disable(panel_lgit->reg_l8);
	if (ret)
		dev_err(dev->dev, "failed to disable l8: %d\n", ret);

	udelay(100);

	ret = regulator_disable(panel_lgit->reg_l2);
	if (ret)
		dev_err(dev->dev, "failed to disable l2: %d\n", ret);

	ret = regulator_set_optimum_mode(panel_lgit->reg_l8, 100);
	if (ret < 0)
		dev_err(dev->dev, "failed to set l8 mode: %d\n", ret);

	ret = regulator_set_optimum_mode(panel_lgit->reg_l2, 100);
	if (ret < 0)
		dev_err(dev->dev, "failed to set l2 mode: %d\n", ret);

	if (panel_lgit->ext_dsv_load) {
		ret = regulator_disable(panel_lgit->ext_dsv_load);
		if (ret)
			dev_err(dev->dev, "failed to disable ext_dsv_load: %d\n", ret);
	}

	return 0;
}

static int panel_lgit_on(struct panel *panel)
{
	struct panel_lgit *panel_lgit = to_panel_lgit(panel);
	struct mipi_adapter *mipi = panel_lgit->mipi;
	int ret = 0;

	DRM_DEBUG_KMS("panel on\n");

	ret = panel_lgit_power_on(panel);
	if (ret)
		return ret;

	mipi_set_panel_config(mipi, &(struct mipi_panel_config){
		.cmd_mode = false,
		.format = DST_FORMAT_RGB888,
		.traffic_mode = NON_BURST_SYNCH_EVENT,
		.bllp_power_stop = true,
		.eof_bllp_power_stop = false,
		.hsa_power_stop = false,
		.hbp_power_stop = false,
		.hfp_power_stop = false,
		.pulse_mode_hsa_he = false,
		.rgb_swap = SWAP_RGB,
		.interleave_max = 0,
		.dma_trigger = TRIGGER_SW,
		.mdp_trigger = TRIGGER_NONE,
		.te = false,
		.dlane_swap = 0,
		.t_clk_pre = 0x36,
		.t_clk_post = 0x22,
		.rx_eot_ignore = false,
		.tx_eot_append = false,
		.ecc_check = false,
		.crc_check = false,
		.phy = {
			/* regulator */
			{0x03, 0x0a, 0x04, 0x00, 0x20},
			/* timing */
			{0x66, 0x26, 0x1D, 0x00, 0x20, 0x95, 0x1E, 0x8F,
			0x20, 0x03, 0x04, 0xA0},
			/* phy ctrl */
			{0x5f, 0x00, 0x00, 0x10},
			/* strength */
			{0xff, 0x00, 0x06, 0x00},
			/* pll control */
			{0x00, 0xC4, 0x01, 0x1A, 0x00, 0x50, 0x48, 0x63,
			0x41, 0x0F, 0x03, 0x00, 0x14, 0x03, 0x00, 0x02,
			0x00, 0x20, 0x00, 0x01 },
		},
	});

	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = false,
		.lanes = 0xf,
	});

	mipi_on(mipi);

	/* Display Initial Set: */
	mipi_lwrite(mipi, true, 0, lcd_mirror);
	mipi_lwrite(mipi, true, 0, panel_setting_1);
	mipi_lwrite(mipi, true, 0, panel_setting_2);
	mipi_lwrite(mipi, true, 0, display_mode1);
	mipi_lwrite(mipi, true, 0, display_mode2);

	/* Gamma Set: */
	mipi_lwrite(mipi, true, 0, p_gamma_r_setting);
	mipi_lwrite(mipi, true, 0, n_gamma_r_setting);
	mipi_lwrite(mipi, true, 0, p_gamma_g_setting);
	mipi_lwrite(mipi, true, 0, n_gamma_g_setting);
	mipi_lwrite(mipi, true, 0, p_gamma_b_setting);
	mipi_lwrite(mipi, true, 0, n_gamma_b_setting);

	/* IEF Set: */
	mipi_lwrite(mipi, true, 0, ief_on_set0);
	mipi_lwrite(mipi, true, 0, ief_set1);
	mipi_lwrite(mipi, true, 0, ief_set2);
	mipi_lwrite(mipi, true, 0, ief_set3);
	mipi_lwrite(mipi, true, 0, ief_on_set4);
	mipi_lwrite(mipi, true, 0, ief_on_set5);
	mipi_lwrite(mipi, true, 0, ief_on_set6);
	mipi_lwrite(mipi, true, 0, ief_set7);
	mipi_lwrite(mipi, true, 0, ief_set8);
	mipi_lwrite(mipi, true, 0, ief_set9);
	mipi_lwrite(mipi, true, 0, ief_setA);
	mipi_lwrite(mipi, true, 0, ief_setB);
	mipi_lwrite(mipi, true, 0, ief_setC);

	/* Power Supply Set: */
	mipi_lwrite(mipi, true, 0, osc_setting);
	mipi_lwrite(mipi, true, 0, power_setting3);
	mipi_lwrite(mipi, true, 0, power_setting4);
	mipi_lwrite(mipi, true, 0, power_setting5);

	mipi_lwrite(mipi, true, 0, cabc_set0);
	mipi_lwrite(mipi, true, 0, cabc_set1);
	mipi_lwrite(mipi, true, 0, cabc_set2);
	mipi_lwrite(mipi, true, 0, cabc_set3);
	mipi_lwrite(mipi, true, 0, cabc_set4);

	mipi_lwrite(mipi, true, 0, exit_sleep_power_control_2);
	msleep(10);
	mipi_lwrite(mipi, true, 0, exit_sleep_power_control_3);
	msleep(1);

	gpio_set_value(DSV_ONBST, 1);
	mdelay(20);

	// Power Supply Set
	mipi_lwrite(mipi, true, 0, otp_protection);
	mipi_lwrite(mipi, true, 0, sleep_out_for_cabc);
	mipi_lwrite(mipi, true, 0, gate_output_enabled_by_manual);
	mipi_dcs_swrite(mipi, true, 0, false, display_on[0]);

	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = true,
		.lanes = 0xf,
	});

	lm3530_lcd_backlight_set_level(0x65);

	return 0;
}

static int panel_lgit_off(struct panel *panel)
{
	struct panel_lgit *panel_lgit = to_panel_lgit(panel);
	struct mipi_adapter *mipi = panel_lgit->mipi;
	int ret;

	DRM_DEBUG_KMS("panel off\n");

	lm3530_lcd_backlight_set_level(0x02);
	lm3530_lcd_backlight_pwm_disable();

	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = false,
		.lanes = 0xf,
	});

	mipi_dcs_swrite(mipi, true, 0, false, display_off[0]);
	mdelay(20);
	mipi_dcs_swrite(mipi, true, 0, false, enter_sleep[0]);
	mdelay(5);

	gpio_set_value(DSV_ONBST, 0);
	mdelay(20);

	mipi_lwrite(mipi, true, 0, analog_boosting_power_control);
	mdelay(10);
	mipi_lwrite(mipi, true, 0, enter_sleep_power_control_3);
	mdelay(10);
	mipi_lwrite(mipi, true, 0, enter_sleep_power_control_2);
	mipi_lwrite(mipi, true, 0, deep_standby);
	mdelay(10);

	mipi_off(mipi);

	ret = panel_lgit_power_off(panel);
	if (ret)
		return ret;

	return 0;
}

static struct drm_display_mode *panel_lgit_mode(struct panel *panel)
{
	struct drm_display_mode *mode = drm_mode_create(panel->dev);

	snprintf(mode->name, sizeof(mode->name), "768x1280");

	mode->clock = 453770;

	mode->hdisplay = 768;
	mode->hsync_start = mode->hdisplay + 8;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 180;

	mode->vdisplay = 1280;
	mode->vsync_start = mode->vdisplay + 8;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 22;

	mode->flags = 0;

	return mode;
}

static const struct panel_funcs panel_lgit_funcs = {
		.destroy = panel_lgit_destroy,
		.on = panel_lgit_on,
		.off = panel_lgit_off,
		.mode = panel_lgit_mode,
};

struct panel *panel_lgit_init(struct drm_device *dev,
		// XXX uggg.. maybe we should just pass in a config structure
		// pre-populated with regulators, gpio's, etc??  the panel
		// needs the drm device, but we need the pdev to lookup the
		// regulators, etc, currently.. and for msm the pdev is different
		// device from the drm device..
		struct platform_device *pdev,
		struct mipi_adapter *mipi)
{
	struct panel_lgit *panel_lgit;
	struct panel *panel = NULL;
	int ret;

	panel_lgit = kzalloc(sizeof(*panel_lgit), GFP_KERNEL);
	if (!panel_lgit) {
		ret = -ENOMEM;
		goto fail;
	}

	panel_lgit->mipi = mipi;

	panel = &panel_lgit->base;
	ret = panel_init(dev, panel, &panel_lgit_funcs);
	if (ret)
		goto fail;

	/* Maybe GPIO/regulator/etc stuff should come from DT or similar..
	 * but we can sort that out when there is some other device that
	 * uses the same panel.
	 */

	ret = devm_gpio_request(&pdev->dev, DSV_ONBST, "DSV_ONBST_en");
	if (ret) {
		dev_err(dev->dev, "failed to request DSV_ONBST gpio: %d\n", ret);
		goto fail;
	}
	ret = gpio_direction_output(DSV_ONBST, 1);
	if (ret) {
		dev_err(dev->dev, "failed to set DSV_ONBST direction: %d\n", ret);
		goto fail;
	}

	ret = devm_gpio_request(&pdev->dev, DISP_RST, "disp_rst_n");
	if (ret) {
		dev_err(dev->dev, "failed to request disp_rst_n gpio: %d\n", ret);
		goto fail;
	}

	panel_lgit->ext_dsv_load = devm_regulator_get(&pdev->dev, "ext_dsv_load");
	if (IS_ERR(panel_lgit->ext_dsv_load))
		panel_lgit->ext_dsv_load = NULL;

	panel_lgit->reg_l8 = devm_regulator_get(&pdev->dev, "dsi_vci");
	if (IS_ERR(panel_lgit->reg_l8)) {
		ret = PTR_ERR(panel_lgit->reg_l8);
		dev_err(dev->dev, "failed to request dsi_vci regulator: %d\n", ret);
		goto fail;
	}

	panel_lgit->reg_lvs6 = devm_regulator_get(&pdev->dev, "dsi_iovcc");
	if (IS_ERR(panel_lgit->reg_lvs6)) {
		ret = PTR_ERR(panel_lgit->reg_lvs6);
		dev_err(dev->dev, "failed to request dsi_iovcc regulator: %d\n", ret);
		goto fail;
	}

	panel_lgit->reg_l2 = devm_regulator_get(&pdev->dev, "dsi_vdda");
	if (IS_ERR(panel_lgit->reg_l2)) {
		ret = PTR_ERR(panel_lgit->reg_l2);
		dev_err(dev->dev, "failed to request dsi_vdda regulator: %d\n", ret);
		goto fail;
	}

	ret = regulator_set_voltage(panel_lgit->reg_l8, 3000000, 3000000);
	if (ret) {
		dev_err(dev->dev, "set_voltage l8 failed: %d\n", ret);
		goto fail;
	}

	ret = regulator_set_voltage(panel_lgit->reg_l2, 1200000, 1200000);
	if (ret) {
		dev_err(dev->dev, "set_voltage l2 failed: %d\n", ret);
		goto fail;
	}

	return panel;
fail:
	if (panel)
		panel_lgit_destroy(panel);
	return ERR_PTR(ret);
}
