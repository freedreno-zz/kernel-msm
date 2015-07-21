/*
 * Copyright (C) 2015 Sony Mobile Communications Inc
 * Copyright (C) 2015 Red Hat
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

#define DEBUG 1
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct sharp_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static inline struct sharp_panel *to_sharp_panel(struct drm_panel *panel)
{
	return container_of(panel, struct sharp_panel, base);
}

/*
15: DCS_WRITE1
05: DCS_WRITE
39: DCS_LWRITE

dsi_lp_mode
somc,mdss-dsi-init-command:

  D
  T  L        W    D
  Y  A     A  A    L
  P  S  V  C  I    E
  E  T  C  K  T    N

  15 01 00 00 00   00 02   BB 10
  15 01 00 00 00   00 02   FF 24
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   C4 9A
  15 01 00 00 00   00 02   85 05
  15 01 00 00 00   00 02   93 02
  15 01 00 00 00   00 02   94 08
  15 01 00 00 00   00 02   92 95
  15 01 00 00 00   00 02   FF 10
  15 01 00 00 00   00 02   FF 20
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   58 82
  15 01 00 00 00   00 02   59 02
  15 01 00 00 00   00 02   5A 02
  15 01 00 00 00   00 02   5B 02
  15 01 00 00 00   00 02   5C 83
  15 01 00 00 00   00 02   5D 83
  15 01 00 00 00   00 02   5E 03
  15 01 00 00 00   00 02   5F 03
  15 01 00 00 00   00 02   6D 55
  15 01 00 00 00   00 02   FF 10
  15 01 00 00 00   00 02   FF 24
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   C4 02
  15 01 00 00 00   00 02   FF 10
  15 01 00 00 00   00 02   35 00            ; MIPI_DCS_SET_TEAR_ON
  39 01 00 00 00   00 03   44 03 00         ; MIPI_DCS_SET_TEAR_SCANLINE
  15 01 00 00 00   00 02   FF E0
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   B5 86
  15 01 00 00 00   00 02   B6 77
  15 01 00 00 00   00 02   B8 AD
  15 01 00 00 00   00 02   FF 20
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   10 04            ; MIPI_DCS_ENTER_SLEEP_MODE
  15 01 00 00 00   00 02   FF 10
  05 01 00 00 1E   00 01   11               ; MIPI_DCS_EXIT_SLEEP_MODE
 */
static int sharp_panel_init(struct sharp_panel *sharp)
{
	struct mipi_dsi_device *dsi = sharp->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
  
	ret = mipi_dsi_dcs_write(dsi, 0xBB, (u8[]) { 0x10 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x24 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFB, (u8[]) { 0x01 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xC4, (u8[]) { 0x9A }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x85, (u8[]) { 0x05 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x93, (u8[]) { 0x02 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x94, (u8[]) { 0x08 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x92, (u8[]) { 0x95 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x10 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x20 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFB, (u8[]) { 0x01 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x58, (u8[]) { 0x82 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x59, (u8[]) { 0x02 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5A, (u8[]) { 0x02 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5B, (u8[]) { 0x02 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5C, (u8[]) { 0x83 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5D, (u8[]) { 0x83 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5E, (u8[]) { 0x03 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x5F, (u8[]) { 0x03 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0x6D, (u8[]) { 0x55 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x10 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x24 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFB, (u8[]) { 0x01 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xC4, (u8[]) { 0x02 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x10 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		return ret;
	
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_TEAR_SCANLINE, (u8[]){ 0x03, 0x00 }, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0xE0 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFB, (u8[]) { 0x01 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xB5, (u8[]) { 0x86 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xB6, (u8[]) { 0x77 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xB8, (u8[]) { 0xAD }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x20 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFB, (u8[]) { 0x01 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xFF, (u8[]) { 0x10 }, 1);
	if (ret < 0)
		return ret;
	
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	msleep(30);

	return 0;
}

/*
dsi_lp_mode
qcom,mdss-dsi-on-command:
  05 01 00 00 28   00 01  29                ; MIPI_DCS_SET_DISPLAY_ON
 */
static int sharp_panel_on(struct sharp_panel *sharp)
{
	struct mipi_dsi_device *dsi = sharp->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	 if (ret < 0)
		return ret;

	msleep(40);

	return 0;
}

/*
dsi_hs_mode
qcom,mdss-dsi-off-command:
  15 01 00 00 00   00 02   FF 10
  05 01 00 00 00   00 01   28               ; MIPI_DCS_SET_DISPLAY_OFF
  05 01 00 00 64   00 01   10               ; MIPI_DCS_ENTER_SLEEP_MODE
 */
static int sharp_panel_off(struct sharp_panel *sharp)
{
	struct mipi_dsi_device *dsi = sharp->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0x10 }, 1);
	 if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	 if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	 if (ret < 0)
		return ret;

	msleep(100);

	return 0;
}


static int sharp_panel_disable(struct drm_panel *panel)
{
	struct sharp_panel *sharp = to_sharp_panel(panel);

	if (!sharp->enabled)
		return 0;

	dev_dbg(panel->dev, "disable\n");

	if (sharp->backlight) {
		sharp->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(sharp->backlight);
	}

	sharp->enabled = false;

	return 0;
}

static int sharp_panel_unprepare(struct drm_panel *panel)
{
	struct sharp_panel *sharp = to_sharp_panel(panel);
	int ret;

	if (!sharp->prepared)
		return 0;

//	sharp_wait_frames(sharp, 4);

	dev_dbg(panel->dev, "unprepare\n");

	ret = sharp_panel_off(sharp);
	 if (ret < 0) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	regulator_disable(sharp->supply);
	if (sharp->reset_gpio)
		gpiod_set_value(sharp->reset_gpio, 0);

	sharp->prepared = false;

	return 0;
}

static int sharp_panel_prepare(struct drm_panel *panel)
{
	struct sharp_panel *sharp = to_sharp_panel(panel);
	int ret;

	if (sharp->prepared)
		return 0;

	dev_dbg(panel->dev, "prepare\n");
	
	ret = regulator_enable(sharp->supply);
	if (ret < 0)
		return ret;
	
	msleep(20);
	
	if (sharp->reset_gpio) {
		gpiod_set_value(sharp->reset_gpio, 1);
		msleep(1);
		
		gpiod_set_value(sharp->reset_gpio, 0);
		msleep(1);
		
		gpiod_set_value(sharp->reset_gpio, 1);
		msleep(20);
	}

	ret = sharp_panel_init(sharp);
	if (ret < 0) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}

	ret = sharp_panel_on(sharp);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	sharp->prepared = true;

	/* wait for 6 frames before continuing */
//	sharp_wait_frames(sharp, 6);

	return 0;

poweroff:
	regulator_disable(sharp->supply);
	if (sharp->reset_gpio)
		gpiod_set_value(sharp->reset_gpio, 0);
	return ret;
}

static int sharp_panel_enable(struct drm_panel *panel)
{
	struct sharp_panel *sharp = to_sharp_panel(panel);

	if (sharp->enabled)
		return 0;

	dev_dbg(panel->dev, "enable\n");

	if (sharp->backlight) {
		sharp->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(sharp->backlight);
	}

	sharp->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
		.clock = 149506,
		.hdisplay = 1080,
		.hsync_start = 1080 + 56,
		.hsync_end = 1080 + 56 + 8,
		.htotal = 1080 + 56 + 8 + 8,
		.vdisplay = 1920,
		.vsync_start = 1920 + 233,
		.vsync_end = 1920 + 233 + 2,
		.vtotal = 1920 + 233 + 2 + 8,
		.vrefresh = 60,
};

static int sharp_panel_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
				default_mode.hdisplay, default_mode.vdisplay,
				default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 114;

	return 1;
}

static const struct drm_panel_funcs sharp_panel_funcs = {
		.disable = sharp_panel_disable,
		.unprepare = sharp_panel_unprepare,
		.prepare = sharp_panel_prepare,
		.enable = sharp_panel_enable,
		.get_modes = sharp_panel_get_modes,
};

static const struct of_device_id sharp_of_match[] = {
		{ .compatible = "sharp,novatek-1080p-vid", },
		{ }
};
MODULE_DEVICE_TABLE(of, sharp_of_match);

static int sharp_panel_add(struct sharp_panel *sharp)
{
	struct device *dev= &sharp->dsi->dev;
	struct device_node *np;
	int ret;

	sharp->mode = &default_mode;

	sharp->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(sharp->supply))
		return PTR_ERR(sharp->supply);

	sharp->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sharp->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(sharp->reset_gpio));
		sharp->reset_gpio = NULL;
	} else {
		gpiod_set_value(sharp->reset_gpio, 0);
	}

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		sharp->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!sharp->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&sharp->base);
	sharp->base.funcs = &sharp_panel_funcs;
	sharp->base.dev = &sharp->dsi->dev;

	ret = drm_panel_add(&sharp->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

	put_backlight:
	if (sharp->backlight)
		put_device(&sharp->backlight->dev);

	return ret;
}

static void sharp_panel_del(struct sharp_panel *sharp)
{
	if (sharp->base.dev)
		drm_panel_remove(&sharp->base);

	if (sharp->backlight)
		put_device(&sharp->backlight->dev);
}

static int sharp_panel_probe(struct mipi_dsi_device *dsi)
{
	struct sharp_panel *sharp;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			MIPI_DSI_MODE_VIDEO_HSE |
			MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_EOT_PACKET;

	sharp = devm_kzalloc(&dsi->dev, sizeof(*sharp), GFP_KERNEL);
	if (!sharp) {
		return -ENOMEM;
	}

	mipi_dsi_set_drvdata(dsi, sharp);

	sharp->dsi = dsi;

	ret = sharp_panel_add(sharp);
	if (ret < 0) {
		return ret;
	}

	return mipi_dsi_attach(dsi);
}

static int sharp_panel_remove(struct mipi_dsi_device *dsi)
{
	struct sharp_panel *sharp = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = sharp_panel_disable(&sharp->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&sharp->base);
	sharp_panel_del(sharp);

	return 0;
}

static void sharp_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct sharp_panel *sharp = mipi_dsi_get_drvdata(dsi);

	sharp_panel_disable(&sharp->base);
}

static struct mipi_dsi_driver sharp_panel_driver = {
	.driver = {
		.name = "panel-sharp-novatek-1080p-video",
		.of_match_table = sharp_of_match,
	},
	.probe = sharp_panel_probe,
	.remove = sharp_panel_remove,
	.shutdown = sharp_panel_shutdown,
};
module_mipi_dsi_driver(sharp_panel_driver);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Sharp Novatek 1080p video mode panel driver");
MODULE_LICENSE("GPL v2");
