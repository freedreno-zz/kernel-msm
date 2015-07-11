/*
 * Copyright (C) 2015 Red Hat
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

struct auo_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct backlight_device *backlight;
	struct regulator *supply;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static inline struct auo_panel *to_auo_panel(struct drm_panel *panel)
{
	return container_of(panel, struct auo_panel, base);
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

  15 01 00 00 00   00 02   FF E0
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   B5 86
  15 01 00 00 00   00 02   B6 77
  15 01 00 00 00   00 02   B8 AD
  15 01 00 00 00   00 02   FF 20
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   10 04            ; MIPI_DCS_ENTER_SLEEP_MODE
  15 01 00 00 00   00 02   FF 24
  15 01 00 00 00   00 02   FB 01
  15 01 00 00 00   00 02   C6 00
  15 01 00 00 00   00 02   C5 32
  15 01 00 00 00   00 02   92 92
  15 01 00 00 00   00 02   FF 10
  15 01 00 00 00   00 02   35 00            ; MIPI_DCS_SET_TEAR_ON
  39 01 00 00 00   00 03   44 03 00         ; MIPI_DCS_SET_TEAR_SCANLINE
  39 01 00 00 00   00 04   3B 03 30 06
  15 01 00 00 01   00 02   BB 10
  05 01 00 00 1E   00 01   11               ; MIPI_DCS_EXIT_SLEEP_MODE
 */
static int auo_panel_init(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0xe0 }, 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xfb, (u8[]){ 0x01 }, 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xb5, (u8[]){ 0x86 }, 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xb6, (u8[]){ 0x77 }, 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, 0xb8, (u8[]){ 0xad }, 1);
	if (ret)
		return ret;

	//00 02   FF 20
	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0x20 }, 1);
	if (ret)
		return ret;

	//00 02   FB 01
	ret = mipi_dsi_dcs_write(dsi, 0xfb, (u8[]){ 0x01 }, 1);
	if (ret)
		return ret;

	//00 02   10 04            ; MIPI_DCS_ENTER_SLEEP_MODE
	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		return ret;

	//00 02   FF 24
	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0x24 }, 1);
	if (ret)
		return ret;

	//00 02   FB 01
	ret = mipi_dsi_dcs_write(dsi, 0xfb, (u8[]){ 0x01 }, 1);
	if (ret)
		return ret;

	//00 02   C6 00
	ret = mipi_dsi_dcs_write(dsi, 0xc6, (u8[]){ 0x00 }, 1);
	if (ret)
		return ret;

	//00 02   C5 32
	ret = mipi_dsi_dcs_write(dsi, 0xc5, (u8[]){ 0x32 }, 1);
	if (ret)
		return ret;

	//00 02   92 92
	ret = mipi_dsi_dcs_write(dsi, 0x92, (u8[]){ 0x92 }, 1);
	if (ret)
		return ret;

	//00 02   FF 10
	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0x10 }, 1);
	if (ret)
		return ret;

	//00 02   35 00            ; MIPI_DCS_SET_TEAR_ON
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret)
		return ret;

	//00 03   44 03 00         ; MIPI_DCS_SET_TEAR_SCANLINE
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_TEAR_SCANLINE,
			(u8[]){ 0x03, 0x00 }, 2);
	if (ret)
		return ret;

	//00 04   3B 03 30 06
	ret = mipi_dsi_dcs_write(dsi, 0x3b, (u8[]){ 0x03, 0x30, 0x06 }, 3);
	if (ret)
		return ret;

	//00 02   BB 10
	ret = mipi_dsi_dcs_write(dsi, 0xbb, (u8[]){ 0x10 }, 1);
	if (ret)
		return ret;
	msleep(1);

	//00 01   11               ; MIPI_DCS_EXIT_SLEEP_MODE
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;
	msleep(30);

	return 0;
}

/*
dsi_lp_mode
qcom,mdss-dsi-on-command:
  05 01 00 00 28   00 01  29                ; MIPI_DCS_SET_DISPLAY_ON
 */
static int auo_panel_on(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
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
static int auo_panel_off(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_write(dsi, 0xff, (u8[]){ 0x10 }, 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(100);

	return 0;
}


static int auo_panel_disable(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);

	if (!auo->enabled)
		return 0;

	dev_dbg(panel->dev, "disable\n");

	if (auo->backlight) {
		auo->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(auo->backlight);
	}

	auo->enabled = false;

	return 0;
}

static int auo_panel_unprepare(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);
	int ret;

	if (!auo->prepared)
		return 0;

//	auo_wait_frames(auo, 4);

	dev_dbg(panel->dev, "unprepare\n");

	ret = auo_panel_off(auo);
	if (ret) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	regulator_disable(auo->supply);

	auo->prepared = false;

	return 0;
}

static int auo_panel_prepare(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);
	int ret;

	if (auo->prepared)
		return 0;

	dev_dbg(panel->dev, "prepare\n");

	ret = regulator_enable(auo->supply);
	if (ret < 0)
		return ret;

	ret = auo_panel_init(auo);
	if (ret < 0) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}

	ret = auo_panel_on(auo);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	auo->prepared = true;

	/* wait for 6 frames before continuing */
//	auo_wait_frames(auo, 6);

	return 0;

poweroff:
	regulator_disable(auo->supply);
	return ret;
}

static int auo_panel_enable(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);

	if (auo->enabled)
		return 0;

	dev_dbg(panel->dev, "enable\n");

	if (auo->backlight) {
		auo->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(auo->backlight);
	}

	auo->enabled = true;

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

static int auo_panel_get_modes(struct drm_panel *panel)
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

static const struct drm_panel_funcs auo_panel_funcs = {
		.disable = auo_panel_disable,
		.unprepare = auo_panel_unprepare,
		.prepare = auo_panel_prepare,
		.enable = auo_panel_enable,
		.get_modes = auo_panel_get_modes,
};

static const struct of_device_id auo_of_match[] = {
		{ .compatible = "auo,novatek-1080p-vid", },
		{ }
};
MODULE_DEVICE_TABLE(of, auo_of_match);

static int auo_panel_add(struct auo_panel *auo)
{
	struct device_node *np;
	int ret;

	auo->mode = &default_mode;

	auo->supply = devm_regulator_get(&auo->dsi->dev, "power");
	if (IS_ERR(auo->supply))
		return PTR_ERR(auo->supply);

	np = of_parse_phandle(auo->dsi->dev.of_node, "backlight", 0);
	if (np) {
		auo->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!auo->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&auo->base);
	auo->base.funcs = &auo_panel_funcs;
	auo->base.dev = &auo->dsi->dev;

	ret = drm_panel_add(&auo->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

	put_backlight:
	if (auo->backlight)
		put_device(&auo->backlight->dev);

	return ret;
}

static void auo_panel_del(struct auo_panel *auo)
{
	if (auo->base.dev)
		drm_panel_remove(&auo->base);

	if (auo->backlight)
		put_device(&auo->backlight->dev);
}

static int auo_panel_probe(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_HSE;

	auo = devm_kzalloc(&dsi->dev, sizeof(*auo), GFP_KERNEL);
	if (!auo) {
		return -ENOMEM;
	}

	mipi_dsi_set_drvdata(dsi, auo);

	auo->dsi = dsi;

	ret = auo_panel_add(auo);
	if (ret < 0) {
		return ret;
	}

	return mipi_dsi_attach(dsi);
}

static int auo_panel_remove(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo = mipi_dsi_get_drvdata(dsi);
	int ret;

	/* only detach from host for the DSI-LINK2 interface */
	if (!auo) {
		mipi_dsi_detach(dsi);
		return 0;
	}

	ret = auo_panel_disable(&auo->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&auo->base);
	auo_panel_del(auo);

	return 0;
}

static void auo_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo = mipi_dsi_get_drvdata(dsi);

	/* nothing to do for DSI-LINK2 */
	if (!auo)
		return;

	auo_panel_disable(&auo->base);
}

static struct mipi_dsi_driver auo_panel_driver = {
	.driver = {
		.name = "panel-auo-novatek-1080p-video",
		.of_match_table = auo_of_match,
	},
	.probe = auo_panel_probe,
	.remove = auo_panel_remove,
	.shutdown = auo_panel_shutdown,
};
module_mipi_dsi_driver(auo_panel_driver);

MODULE_AUTHOR("Rob Clark <robdclark@gmail.com>");
MODULE_DESCRIPTION("AUO Novatek 1080p video mode panel driver");
MODULE_LICENSE("GPL v2");
