/*
 * panel-jdi-novatek-1080p.c - Japan Display Inc. panel driver
 *
 * Copyright © 2015 AngeloGioacchino Del Regno <kholk11@gmail.com>
 * 
 * Derived from panel-auo-novatek-1080p.c © 2015 Red Hat
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
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

struct jdi_panel_desc {
	struct drm_panel 	base;
	struct mipi_dsi_device 	*dsi;

	struct backlight_device *backlight;
	struct regulator 	*supply;
	struct gpio_desc 	*reset_gpio;

	bool prepared;
	bool enabled;
	int state;

	const struct drm_display_mode *mode;
};

static void panel_dcs_send(struct jdi_panel_desc *panel, const u8 payload,
			   const void *data, int len)
{
	if (panel->state < 0) {
		WARN_ONCE(1, "%s: Error while sending commands\n", __func__);
		return;
	}

	panel->state = mipi_dsi_dcs_write(panel->dsi, payload, data, len);
}


#define panel_dcs_write(intf, payload, cmd...) \
({\
	static const u8 seq[] = { cmd };\
	panel_dcs_send(intf, payload, seq, ARRAY_SIZE(seq));\
})


static inline struct jdi_panel_desc *drm_to_panel_desc(struct drm_panel *panel)
{
	return container_of(panel, struct jdi_panel_desc, base);
}

static int jdi_panel_init(struct jdi_panel_desc *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xb0, 0x04 }, 2);
	if (ret < 0)
		return ret;

	panel_dcs_write(panel, 0x01, 0x00);
	usleep_range(10000, 11000);

	panel_dcs_write(panel, 0xFF, 0x01);
	panel_dcs_write(panel, 0x75, 0x00);
	panel_dcs_write(panel, 0x76, 0x30);
	panel_dcs_write(panel, 0x77, 0x00);
	panel_dcs_write(panel, 0x78, 0x43);
	panel_dcs_write(panel, 0x79, 0x00);
	panel_dcs_write(panel, 0x7A, 0x61);
	panel_dcs_write(panel, 0x7B, 0x00);
	panel_dcs_write(panel, 0x7C, 0x7D);
	panel_dcs_write(panel, 0x7D, 0x00);
	panel_dcs_write(panel, 0x7E, 0x98);
	panel_dcs_write(panel, 0x7F, 0x00);
	panel_dcs_write(panel, 0x80, 0xAE);
	panel_dcs_write(panel, 0x81, 0x00);
	panel_dcs_write(panel, 0x82, 0xC0);
	panel_dcs_write(panel, 0x83, 0x00);
	panel_dcs_write(panel, 0x84, 0xCD);
	panel_dcs_write(panel, 0x85, 0x00);
	panel_dcs_write(panel, 0x86, 0xDB);
	panel_dcs_write(panel, 0x87, 0x01);
	panel_dcs_write(panel, 0x88, 0x0A);
	panel_dcs_write(panel, 0x89, 0x01);
	panel_dcs_write(panel, 0x8A, 0x2A);
	panel_dcs_write(panel, 0x8B, 0x01);
	panel_dcs_write(panel, 0x8C, 0x66);
	panel_dcs_write(panel, 0x8D, 0x01);
	panel_dcs_write(panel, 0x8E, 0x95);
	panel_dcs_write(panel, 0x8F, 0x01);
	panel_dcs_write(panel, 0x90, 0xD8);
	panel_dcs_write(panel, 0x91, 0x02);
	panel_dcs_write(panel, 0x92, 0x1A);
	panel_dcs_write(panel, 0x93, 0x02);
	panel_dcs_write(panel, 0x94, 0x1C);
	panel_dcs_write(panel, 0x95, 0x02);
	panel_dcs_write(panel, 0x96, 0x5E);
	panel_dcs_write(panel, 0x97, 0x02);
	panel_dcs_write(panel, 0x98, 0x9A);
	panel_dcs_write(panel, 0x99, 0x02);
	panel_dcs_write(panel, 0x9A, 0xBF);
	panel_dcs_write(panel, 0x9B, 0x02);
	panel_dcs_write(panel, 0x9C, 0xED);
	panel_dcs_write(panel, 0x9D, 0x03);
	panel_dcs_write(panel, 0x9E, 0x0B);
	panel_dcs_write(panel, 0x9F, 0x03);
	panel_dcs_write(panel, 0xA0, 0x36);
	panel_dcs_write(panel, 0xA2, 0x03);
	panel_dcs_write(panel, 0xA3, 0x3B);
	panel_dcs_write(panel, 0xA4, 0x03);
	panel_dcs_write(panel, 0xA5, 0x40);
	panel_dcs_write(panel, 0xA6, 0x03);
	panel_dcs_write(panel, 0xA7, 0x45);
	panel_dcs_write(panel, 0xA9, 0x03);
	panel_dcs_write(panel, 0xAA, 0x54);
	panel_dcs_write(panel, 0xAB, 0x03);
	panel_dcs_write(panel, 0xAC, 0x70);
	panel_dcs_write(panel, 0xAD, 0x03);
	panel_dcs_write(panel, 0xAE, 0x8E);
	panel_dcs_write(panel, 0xAF, 0x03);
	panel_dcs_write(panel, 0xB0, 0xB2);
	panel_dcs_write(panel, 0xB1, 0x03);
	panel_dcs_write(panel, 0xB2, 0xC9);
	panel_dcs_write(panel, 0xB3, 0x00);
	panel_dcs_write(panel, 0xB4, 0x30);
	panel_dcs_write(panel, 0xB5, 0x00);
	panel_dcs_write(panel, 0xB6, 0x43);
	panel_dcs_write(panel, 0xB7, 0x00);
	panel_dcs_write(panel, 0xB8, 0x61);
	panel_dcs_write(panel, 0xB9, 0x00);
	panel_dcs_write(panel, 0xBA, 0x7D);
	panel_dcs_write(panel, 0xBB, 0x00);
	panel_dcs_write(panel, 0xBC, 0x98);
	panel_dcs_write(panel, 0xBD, 0x00);
	panel_dcs_write(panel, 0xBE, 0xAE);
	panel_dcs_write(panel, 0xBF, 0x00);
	panel_dcs_write(panel, 0xC0, 0xC0);
	panel_dcs_write(panel, 0xC1, 0x00);
	panel_dcs_write(panel, 0xC2, 0xCD);
	panel_dcs_write(panel, 0xC3, 0x00);
	panel_dcs_write(panel, 0xC4, 0xDB);
	panel_dcs_write(panel, 0xC5, 0x01);
	panel_dcs_write(panel, 0xC6, 0x0A);
	panel_dcs_write(panel, 0xC7, 0x01);
	panel_dcs_write(panel, 0xC8, 0x2A);
	panel_dcs_write(panel, 0xC9, 0x01);
	panel_dcs_write(panel, 0xCA, 0x66);
	panel_dcs_write(panel, 0xCB, 0x01);
	panel_dcs_write(panel, 0xCC, 0x95);
	panel_dcs_write(panel, 0xCD, 0x01);
	panel_dcs_write(panel, 0xCE, 0xD8);
	panel_dcs_write(panel, 0xCF, 0x02);
	panel_dcs_write(panel, 0xD0, 0x1A);
	panel_dcs_write(panel, 0xD1, 0x02);
	panel_dcs_write(panel, 0xD2, 0x1C);
	panel_dcs_write(panel, 0xD3, 0x02);
	panel_dcs_write(panel, 0xD4, 0x5E);
	panel_dcs_write(panel, 0xD5, 0x02);
	panel_dcs_write(panel, 0xD6, 0x9A);
	panel_dcs_write(panel, 0xD7, 0x02);
	panel_dcs_write(panel, 0xD8, 0xBF);
	panel_dcs_write(panel, 0xD9, 0x02);
	panel_dcs_write(panel, 0xDA, 0xED);
	panel_dcs_write(panel, 0xDB, 0x03);
	panel_dcs_write(panel, 0xDC, 0x0B);
	panel_dcs_write(panel, 0xDD, 0x03);
	panel_dcs_write(panel, 0xDE, 0x36);
	panel_dcs_write(panel, 0xDF, 0x03);
	panel_dcs_write(panel, 0xE0, 0x3B);
	panel_dcs_write(panel, 0xE1, 0x03);
	panel_dcs_write(panel, 0xE2, 0x40);
	panel_dcs_write(panel, 0xE3, 0x03);
	panel_dcs_write(panel, 0xE4, 0x45);
	panel_dcs_write(panel, 0xE5, 0x03);
	panel_dcs_write(panel, 0xE6, 0x54);
	panel_dcs_write(panel, 0xE7, 0x03);
	panel_dcs_write(panel, 0xE8, 0x70);
	panel_dcs_write(panel, 0xE9, 0x03);
	panel_dcs_write(panel, 0xEA, 0x8E);
	panel_dcs_write(panel, 0xEB, 0x03);
	panel_dcs_write(panel, 0xEC, 0xB2);
	panel_dcs_write(panel, 0xED, 0x03);
	panel_dcs_write(panel, 0xEE, 0xC9);
	panel_dcs_write(panel, 0xEF, 0x00);
	panel_dcs_write(panel, 0xF0, 0x30);
	panel_dcs_write(panel, 0xF1, 0x00);
	panel_dcs_write(panel, 0xF2, 0x43);
	panel_dcs_write(panel, 0xF3, 0x00);
	panel_dcs_write(panel, 0xF4, 0x61);
	panel_dcs_write(panel, 0xF5, 0x00);
	panel_dcs_write(panel, 0xF6, 0x7D);
	panel_dcs_write(panel, 0xF7, 0x00);
	panel_dcs_write(panel, 0xF8, 0x98);
	panel_dcs_write(panel, 0xF9, 0x00);
	panel_dcs_write(panel, 0xFA, 0xAE);
	panel_dcs_write(panel, 0xFB, 0x01);
	panel_dcs_write(panel, 0xFF, 0x02);
	panel_dcs_write(panel, 0x00, 0x00);
	panel_dcs_write(panel, 0x01, 0xC0);
	panel_dcs_write(panel, 0x02, 0x00);
	panel_dcs_write(panel, 0x03, 0xCD);
	panel_dcs_write(panel, 0x04, 0x00);
	panel_dcs_write(panel, 0x05, 0xDB);
	panel_dcs_write(panel, 0x06, 0x01);
	panel_dcs_write(panel, 0x07, 0x0A);
	panel_dcs_write(panel, 0x08, 0x01);
	panel_dcs_write(panel, 0x09, 0x2A);
	panel_dcs_write(panel, 0x0A, 0x01);
	panel_dcs_write(panel, 0x0B, 0x69);
	panel_dcs_write(panel, 0x0C, 0x01);
	panel_dcs_write(panel, 0x0D, 0x99);
	panel_dcs_write(panel, 0x0E, 0x01);
	panel_dcs_write(panel, 0x0F, 0xDE);
	panel_dcs_write(panel, 0x10, 0x02);
	panel_dcs_write(panel, 0x11, 0x20);
	panel_dcs_write(panel, 0x12, 0x02);
	panel_dcs_write(panel, 0x13, 0x22);
	panel_dcs_write(panel, 0x14, 0x02);
	panel_dcs_write(panel, 0x15, 0x64);
	panel_dcs_write(panel, 0x16, 0x02);
	panel_dcs_write(panel, 0x17, 0xA0);
	panel_dcs_write(panel, 0x18, 0x02);
	panel_dcs_write(panel, 0x19, 0xC4);
	panel_dcs_write(panel, 0x1A, 0x02);
	panel_dcs_write(panel, 0x1B, 0xF3);
	panel_dcs_write(panel, 0x1C, 0x03);
	panel_dcs_write(panel, 0x1D, 0x0F);
	panel_dcs_write(panel, 0x1E, 0x03);
	panel_dcs_write(panel, 0x1F, 0x36);
	panel_dcs_write(panel, 0x20, 0x03);
	panel_dcs_write(panel, 0x21, 0x3B);
	panel_dcs_write(panel, 0x22, 0x03);
	panel_dcs_write(panel, 0x23, 0x40);
	panel_dcs_write(panel, 0x24, 0x03);
	panel_dcs_write(panel, 0x25, 0x45);
	panel_dcs_write(panel, 0x26, 0x03);
	panel_dcs_write(panel, 0x27, 0x54);
	panel_dcs_write(panel, 0x28, 0x03);
	panel_dcs_write(panel, 0x29, 0x70);
	panel_dcs_write(panel, 0x2A, 0x03);
	panel_dcs_write(panel, 0x2B, 0x8E);
	panel_dcs_write(panel, 0x2D, 0x03);
	panel_dcs_write(panel, 0x2F, 0xB2);
	panel_dcs_write(panel, 0x30, 0x03);
	panel_dcs_write(panel, 0x31, 0xC9);
	panel_dcs_write(panel, 0x32, 0x00);
	panel_dcs_write(panel, 0x33, 0x30);
	panel_dcs_write(panel, 0x34, 0x00);
	panel_dcs_write(panel, 0x35, 0x43);
	panel_dcs_write(panel, 0x36, 0x00);
	panel_dcs_write(panel, 0x37, 0x61);
	panel_dcs_write(panel, 0x38, 0x00);
	panel_dcs_write(panel, 0x39, 0x7D);
	panel_dcs_write(panel, 0x3A, 0x00);
	panel_dcs_write(panel, 0x3B, 0x98);
	panel_dcs_write(panel, 0x3D, 0x00);
	panel_dcs_write(panel, 0x3F, 0xAE);
	panel_dcs_write(panel, 0x40, 0x00);
	panel_dcs_write(panel, 0x41, 0xC0);
	panel_dcs_write(panel, 0x42, 0x00);
	panel_dcs_write(panel, 0x43, 0xCD);
	panel_dcs_write(panel, 0x44, 0x00);
	panel_dcs_write(panel, 0x45, 0xDB);
	panel_dcs_write(panel, 0x46, 0x01);
	panel_dcs_write(panel, 0x47, 0x0A);
	panel_dcs_write(panel, 0x48, 0x01);
	panel_dcs_write(panel, 0x49, 0x2A);
	panel_dcs_write(panel, 0x4A, 0x01);
	panel_dcs_write(panel, 0x4B, 0x69);
	panel_dcs_write(panel, 0x4C, 0x01);
	panel_dcs_write(panel, 0x4D, 0x99);
	panel_dcs_write(panel, 0x4E, 0x01);
	panel_dcs_write(panel, 0x4F, 0xDE);
	panel_dcs_write(panel, 0x50, 0x02);
	panel_dcs_write(panel, 0x51, 0x20);
	panel_dcs_write(panel, 0x52, 0x02);
	panel_dcs_write(panel, 0x53, 0x22);
	panel_dcs_write(panel, 0x54, 0x02);
	panel_dcs_write(panel, 0x55, 0x64);
	panel_dcs_write(panel, 0x56, 0x02);
	panel_dcs_write(panel, 0x58, 0xA0);
	panel_dcs_write(panel, 0x59, 0x02);
	panel_dcs_write(panel, 0x5A, 0xC4);
	panel_dcs_write(panel, 0x5B, 0x02);
	panel_dcs_write(panel, 0x5C, 0xF3);
	panel_dcs_write(panel, 0x5D, 0x03);
	panel_dcs_write(panel, 0x5E, 0x0F);
	panel_dcs_write(panel, 0x5F, 0x03);
	panel_dcs_write(panel, 0x60, 0x36);
	panel_dcs_write(panel, 0x61, 0x03);
	panel_dcs_write(panel, 0x62, 0x3B);
	panel_dcs_write(panel, 0x63, 0x03);
	panel_dcs_write(panel, 0x64, 0x40);
	panel_dcs_write(panel, 0x65, 0x03);
	panel_dcs_write(panel, 0x66, 0x45);
	panel_dcs_write(panel, 0x67, 0x03);
	panel_dcs_write(panel, 0x68, 0x54);
	panel_dcs_write(panel, 0x69, 0x03);
	panel_dcs_write(panel, 0x6A, 0x70);
	panel_dcs_write(panel, 0x6B, 0x03);
	panel_dcs_write(panel, 0x6C, 0x8E);
	panel_dcs_write(panel, 0x6D, 0x03);
	panel_dcs_write(panel, 0x6E, 0xB2);
	panel_dcs_write(panel, 0x6F, 0x03);
	panel_dcs_write(panel, 0x70, 0xC9);
	panel_dcs_write(panel, 0x71, 0x00);
	panel_dcs_write(panel, 0x72, 0x30);
	panel_dcs_write(panel, 0x73, 0x00);
	panel_dcs_write(panel, 0x74, 0x43);
	panel_dcs_write(panel, 0x75, 0x00);
	panel_dcs_write(panel, 0x76, 0x61);
	panel_dcs_write(panel, 0x77, 0x00);
	panel_dcs_write(panel, 0x78, 0x7D);
	panel_dcs_write(panel, 0x79, 0x00);
	panel_dcs_write(panel, 0x7A, 0x98);
	panel_dcs_write(panel, 0x7B, 0x00);
	panel_dcs_write(panel, 0x7C, 0xAE);
	panel_dcs_write(panel, 0x7D, 0x00);
	panel_dcs_write(panel, 0x7E, 0xC0);
	panel_dcs_write(panel, 0x7F, 0x00);
	panel_dcs_write(panel, 0x80, 0xCD);
	panel_dcs_write(panel, 0x81, 0x00);
	panel_dcs_write(panel, 0x82, 0xDB);
	panel_dcs_write(panel, 0x83, 0x01);
	panel_dcs_write(panel, 0x84, 0x0A);
	panel_dcs_write(panel, 0x85, 0x01);
	panel_dcs_write(panel, 0x86, 0x2A);
	panel_dcs_write(panel, 0x87, 0x01);
	panel_dcs_write(panel, 0x88, 0x63);
	panel_dcs_write(panel, 0x89, 0x01);
	panel_dcs_write(panel, 0x8A, 0x90);
	panel_dcs_write(panel, 0x8B, 0x01);
	panel_dcs_write(panel, 0x8C, 0xD2);
	panel_dcs_write(panel, 0x8D, 0x02);
	panel_dcs_write(panel, 0x8E, 0x14);
	panel_dcs_write(panel, 0x8F, 0x02);
	panel_dcs_write(panel, 0x90, 0x16);
	panel_dcs_write(panel, 0x91, 0x02);
	panel_dcs_write(panel, 0x92, 0x58);
	panel_dcs_write(panel, 0x93, 0x02);
	panel_dcs_write(panel, 0x94, 0x95);
	panel_dcs_write(panel, 0x95, 0x02);
	panel_dcs_write(panel, 0x96, 0xBC);
	panel_dcs_write(panel, 0x97, 0x02);
	panel_dcs_write(panel, 0x98, 0xED);
	panel_dcs_write(panel, 0x99, 0x03);
	panel_dcs_write(panel, 0x9A, 0x0B);
	panel_dcs_write(panel, 0x9B, 0x03);
	panel_dcs_write(panel, 0x9C, 0x36);
	panel_dcs_write(panel, 0x9D, 0x03);
	panel_dcs_write(panel, 0x9E, 0x3B);
	panel_dcs_write(panel, 0x9F, 0x03);
	panel_dcs_write(panel, 0xA0, 0x40);
	panel_dcs_write(panel, 0xA2, 0x03);
	panel_dcs_write(panel, 0xA3, 0x45);
	panel_dcs_write(panel, 0xA4, 0x03);
	panel_dcs_write(panel, 0xA5, 0x54);
	panel_dcs_write(panel, 0xA6, 0x03);
	panel_dcs_write(panel, 0xA7, 0x70);
	panel_dcs_write(panel, 0xA9, 0x03);
	panel_dcs_write(panel, 0xAA, 0x8E);
	panel_dcs_write(panel, 0xAB, 0x03);
	panel_dcs_write(panel, 0xAC, 0xB2);
	panel_dcs_write(panel, 0xAD, 0x03);
	panel_dcs_write(panel, 0xAE, 0xC9);
	panel_dcs_write(panel, 0xAF, 0x00);
	panel_dcs_write(panel, 0xB0, 0x30);
	panel_dcs_write(panel, 0xB1, 0x00);
	panel_dcs_write(panel, 0xB2, 0x43);
	panel_dcs_write(panel, 0xB3, 0x00);
	panel_dcs_write(panel, 0xB4, 0x61);
	panel_dcs_write(panel, 0xB5, 0x00);
	panel_dcs_write(panel, 0xB6, 0x7D);
	panel_dcs_write(panel, 0xB7, 0x00);
	panel_dcs_write(panel, 0xB8, 0x98);
	panel_dcs_write(panel, 0xB9, 0x00);
	panel_dcs_write(panel, 0xBA, 0xAE);
	panel_dcs_write(panel, 0xBB, 0x00);
	panel_dcs_write(panel, 0xBC, 0xC0);
	panel_dcs_write(panel, 0xBD, 0x00);
	panel_dcs_write(panel, 0xBE, 0xCD);
	panel_dcs_write(panel, 0xBF, 0x00);
	panel_dcs_write(panel, 0xC0, 0xDB);
	panel_dcs_write(panel, 0xC1, 0x01);
	panel_dcs_write(panel, 0xC2, 0x0A);
	panel_dcs_write(panel, 0xC3, 0x01);
	panel_dcs_write(panel, 0xC4, 0x2A);
	panel_dcs_write(panel, 0xC5, 0x01);
	panel_dcs_write(panel, 0xC6, 0x63);
	panel_dcs_write(panel, 0xC7, 0x01);
	panel_dcs_write(panel, 0xC8, 0x90);
	panel_dcs_write(panel, 0xC9, 0x01);
	panel_dcs_write(panel, 0xCA, 0xD2);
	panel_dcs_write(panel, 0xCB, 0x02);
	panel_dcs_write(panel, 0xCC, 0x14);
	panel_dcs_write(panel, 0xCD, 0x02);
	panel_dcs_write(panel, 0xCE, 0x16);
	panel_dcs_write(panel, 0xCF, 0x02);
	panel_dcs_write(panel, 0xD0, 0x58);
	panel_dcs_write(panel, 0xD1, 0x02);
	panel_dcs_write(panel, 0xD2, 0x95);
	panel_dcs_write(panel, 0xD3, 0x02);
	panel_dcs_write(panel, 0xD4, 0xBC);
	panel_dcs_write(panel, 0xD5, 0x02);
	panel_dcs_write(panel, 0xD6, 0xED);
	panel_dcs_write(panel, 0xD7, 0x03);
	panel_dcs_write(panel, 0xD8, 0x0B);
	panel_dcs_write(panel, 0xD9, 0x03);
	panel_dcs_write(panel, 0xDA, 0x36);
	panel_dcs_write(panel, 0xDB, 0x03);
	panel_dcs_write(panel, 0xDC, 0x3B);
	panel_dcs_write(panel, 0xDD, 0x03);
	panel_dcs_write(panel, 0xDE, 0x40);
	panel_dcs_write(panel, 0xDF, 0x03);
	panel_dcs_write(panel, 0xE0, 0x45);
	panel_dcs_write(panel, 0xE1, 0x03);
	panel_dcs_write(panel, 0xE2, 0x54);
	panel_dcs_write(panel, 0xE3, 0x03);
	panel_dcs_write(panel, 0xE4, 0x70);
	panel_dcs_write(panel, 0xE5, 0x03);
	panel_dcs_write(panel, 0xE6, 0x8E);
	panel_dcs_write(panel, 0xE7, 0x03);
	panel_dcs_write(panel, 0xE8, 0xB2);
	panel_dcs_write(panel, 0xE9, 0x03);
	panel_dcs_write(panel, 0xEA, 0xC9);
	panel_dcs_write(panel, 0xFB, 0x01);
	panel_dcs_write(panel, 0xFF, 0x01);
	panel_dcs_write(panel, 0x0B, 0x4B);
	panel_dcs_write(panel, 0x0C, 0x4B);
	panel_dcs_write(panel, 0x0E, 0xA1);
	panel_dcs_write(panel, 0x15, 0x0B);
	panel_dcs_write(panel, 0x16, 0x0B);
	panel_dcs_write(panel, 0x1B, 0x1B);
	panel_dcs_write(panel, 0x1C, 0xF5);
	panel_dcs_write(panel, 0x01, 0x44);
	panel_dcs_write(panel, 0x5C, 0x82);
	panel_dcs_write(panel, 0x5E, 0x02);
	panel_dcs_write(panel, 0x60, 0x0F);
	panel_dcs_write(panel, 0x66, 0x01);
	panel_dcs_write(panel, 0x69, 0x99);
	panel_dcs_write(panel, 0x6D, 0x33);
	panel_dcs_write(panel, 0xFB, 0x01);
	panel_dcs_write(panel, 0xFF, 0x05);
	panel_dcs_write(panel, 0x35, 0x6B);
	panel_dcs_write(panel, 0x7E, 0x02);
	panel_dcs_write(panel, 0x7F, 0x18);
	panel_dcs_write(panel, 0x81, 0x05);
	panel_dcs_write(panel, 0x82, 0x05);
	panel_dcs_write(panel, 0xA6, 0x04);
	panel_dcs_write(panel, 0x84, 0x03);
	panel_dcs_write(panel, 0x85, 0x04);
	panel_dcs_write(panel, 0xC6, 0x00);
	panel_dcs_write(panel, 0xFB, 0x01);
	panel_dcs_write(panel, 0xFF, 0xFF);
	panel_dcs_write(panel, 0x4F, 0x03);
	panel_dcs_write(panel, 0xFB, 0x01);
	panel_dcs_write(panel, 0xFF, 0x00);
	panel_dcs_write(panel, 0xD3, 0x08);
	panel_dcs_write(panel, 0xD4, 0x1B);
	panel_dcs_write(panel, 0xD5, 0x50);
	panel_dcs_write(panel, 0xD6, 0x70);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;
	msleep(113);

	return 0;
}

static int jdi_panel_on(struct jdi_panel_desc *desc)
{
	struct mipi_dsi_device *dsi = desc->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	usleep_range(1000, 1100);

	return 0;
}

static int jdi_panel_off(struct jdi_panel_desc *desc)
{
	struct mipi_dsi_device *dsi = desc->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	msleep(70);

	return 0;
}

static int jdi_panel_disable(struct drm_panel *drm_panel)
{
	struct jdi_panel_desc *panel = drm_to_panel_desc(drm_panel);

	if (!panel->enabled)
		return 0;

	DRM_DEBUG("disabling panel\n");

	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(panel->backlight);
	}

	panel->enabled = false;

	return 0;
}

static int jdi_panel_unprepare(struct drm_panel *drm_panel)
{
	struct jdi_panel_desc *panel = drm_to_panel_desc(drm_panel);
	int ret;

	if (!panel->prepared)
		return 0;

	DRM_DEBUG("unpreparing panel\n");

	ret = jdi_panel_off(panel);
	if (ret) {
		dev_err(drm_panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	regulator_disable(panel->supply);
	if (panel->reset_gpio)
		gpiod_set_value(panel->reset_gpio, 0);

	panel->prepared = false;

	if (panel->state != 0) {
		dev_warn(drm_panel->dev, "Resetting previous error states.\n");
		panel->state = 0;
	}

	return 0;
}

static int jdi_panel_prepare(struct drm_panel *drm_panel)
{
	struct jdi_panel_desc *panel = drm_to_panel_desc(drm_panel);
	int ret;

	if (panel->prepared)
		return 0;

	DRM_DEBUG("preparing the panel...\n");

	if (panel->reset_gpio) {
		gpiod_set_value(panel->reset_gpio, 0);
		usleep_range(5000, 5100);
	}

	ret = regulator_enable(panel->supply);
	if (ret < 0)
		return ret;

	msleep(20);

	if (panel->reset_gpio) {
		gpiod_set_value(panel->reset_gpio, 1);
		msleep(10);
	}

	usleep_range(10000, 10500);

	ret = jdi_panel_init(panel);
	if (ret) {
		dev_err(drm_panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}

	ret = jdi_panel_on(panel);
	if (ret) {
		dev_err(drm_panel->dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	panel->prepared = true;

	return 0;

poweroff:
	regulator_disable(panel->supply);
	if (panel->reset_gpio)
		gpiod_set_value(panel->reset_gpio, 0);
	return ret;
}

static int jdi_panel_enable(struct drm_panel *drm_panel)
{
	struct jdi_panel_desc *panel = drm_to_panel_desc(drm_panel);

	if (panel->enabled)
		return 0;

	DRM_DEBUG("panel enable\n");

	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(panel->backlight);
	}

	panel->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
		.clock = 149506,
		.hdisplay = 1080,
		.hsync_start = 1080 + 112,
		.hsync_end = 1080 + 112 + 76,
		.htotal = 1080 + 112 + 76 + 4,
		.vdisplay = 1920,
		.vsync_start = 1920 + 27,
		.vsync_end = 1920 + 27 + 4,
		.vtotal = 1920 + 27 + 4 + 4,
		.vrefresh = 60,
};

static int jdi_panel_get_modes(struct drm_panel *drm_panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(drm_panel->drm, &default_mode);
	if (!mode) {
		dev_err(drm_panel->drm->dev, "failed to add mode %ux%ux@%u\n",
				default_mode.hdisplay, default_mode.vdisplay,
				default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(drm_panel->connector, mode);

	drm_panel->connector->display_info.width_mm = 64;
	drm_panel->connector->display_info.height_mm = 114;

	return 1;
}

static const struct drm_panel_funcs jdi_panel_funcs = {
		.disable = jdi_panel_disable,
		.unprepare = jdi_panel_unprepare,
		.prepare = jdi_panel_prepare,
		.enable = jdi_panel_enable,
		.get_modes = jdi_panel_get_modes,
};

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "jdi,novatek-1080p-vid", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static int jdi_panel_add(struct jdi_panel_desc *panel)
{
	struct device *dev = &panel->dsi->dev;
	struct device_node *np;
	int ret;

	panel->mode = &default_mode;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpio %ld\n",
			PTR_ERR(panel->reset_gpio));
		panel->reset_gpio = NULL;
	} else {
		gpiod_direction_output(panel->reset_gpio, 0);
	}

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		panel->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!panel->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&panel->base);
	panel->base.funcs = &jdi_panel_funcs;
	panel->base.dev = &panel->dsi->dev;

	ret = drm_panel_add(&panel->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

put_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return ret;
}

static void jdi_panel_del(struct jdi_panel_desc *panel)
{
	if (panel->base.dev)
		drm_panel_remove(&panel->base);

	if (panel->backlight)
		put_device(&panel->backlight->dev);
}

static int jdi_panel_probe(struct mipi_dsi_device *dsi)
{
	struct jdi_panel_desc *panel;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			MIPI_DSI_MODE_VIDEO_HSE |
			MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_EOT_PACKET;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	ret = jdi_panel_add(panel);
	if (ret < 0)
		return ret;

	return mipi_dsi_attach(dsi);
}

static int jdi_panel_remove(struct mipi_dsi_device *dsi)
{
	struct jdi_panel_desc *panel = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = jdi_panel_disable(&panel->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&panel->base);
	jdi_panel_del(panel);

	return 0;
}

static void jdi_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct jdi_panel_desc *panel = mipi_dsi_get_drvdata(dsi);

	jdi_panel_disable(&panel->base);
}

static struct mipi_dsi_driver jdi_panel_driver = {
	.driver = {
		.name = "panel-jdi-novatek-1080p",
		.of_match_table = jdi_of_match,
	},
	.probe = jdi_panel_probe,
	.remove = jdi_panel_remove,
	.shutdown = jdi_panel_shutdown,
};
module_mipi_dsi_driver(jdi_panel_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <kholk11@gmail.com>");
MODULE_DESCRIPTION("JDI Novatek 1080p panel driver");
MODULE_LICENSE("GPL v2");
