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

#ifndef __MIPI_H__
#define __MIPI_H__

#include <linux/types.h>

/* TODO get rid of msm specifics.. */
#include "dsi/dsi.xml.h"

/* This is something relatively generic that could be outside msm driver */

struct mipi_adapter;

struct mipi_bus_config {
	bool low_power;
	u32 lanes;     /* bitmask of used lanes */
	/* possibly some stuff that is in panel_config should be here instead */
};

struct mipi_panel_config {
	bool cmd_mode;

	/* TODO this needs to be more generic.. not just msm reg vals and bitfields: */
	enum dsi_dst_format format;
	enum dsi_traffic_mode traffic_mode;
	bool bllp_power_stop;
	bool eof_bllp_power_stop;
	bool hsa_power_stop;
	bool hbp_power_stop;
	bool hfp_power_stop;
	bool pulse_mode_hsa_he;
	enum dsi_rgb_swap rgb_swap;
	uint32_t interleave_max;
	enum dsi_cmd_trigger dma_trigger;
	enum dsi_cmd_trigger mdp_trigger;
	bool te;
	uint32_t dlane_swap;   /* board specific, rather than panel specific?? */
	uint32_t t_clk_pre;
	uint32_t t_clk_post;
	bool rx_eot_ignore;
	bool tx_eot_append;
	bool ecc_check;
	bool crc_check;

	// XXX this stuff is very msm specific (ie. it is directly register
	// values), so doesn't belong here.. but I need to know the meaning
	// of these registers to extract these out into something more
	// generic:
	struct {
		uint32_t regulator[5];
		uint32_t timing[12];
		uint32_t ctrl[4];
		uint32_t strength[4];
		uint32_t pll[21];
	} phy;
};

struct mipi_adapter_funcs {
	void (*destroy)(struct mipi_adapter *mipi);
	int (*set_bus_config)(struct mipi_adapter *mipi,
			const struct mipi_bus_config *bcfg);
	int (*set_panel_config)(struct mipi_adapter *mipi,
			const struct mipi_panel_config *pcfg);
	int (*on)(struct mipi_adapter *mipi);
	int (*off)(struct mipi_adapter *mipi);
	int (*write)(struct mipi_adapter *mipi, const u8 *data, size_t len);
	int (*read)(struct mipi_adapter *mipi, u8 *data, size_t len);
};

struct mipi_adapter {
	const struct mipi_adapter_funcs *funcs;
};

static inline int mipi_adapter_init(struct mipi_adapter *mipi,
		const struct mipi_adapter_funcs *funcs)
{
	mipi->funcs = funcs;
	return 0;
}

static inline void mipi_destroy(struct mipi_adapter *mipi)
{
	mipi->funcs->destroy(mipi);
}

static inline int mipi_set_bus_config(struct mipi_adapter *mipi,
		const struct mipi_bus_config *bcfg)
{
	return mipi->funcs->set_bus_config(mipi, bcfg);
}

static inline int mipi_set_panel_config(struct mipi_adapter *mipi,
		const struct mipi_panel_config *pcfg)
{
	return mipi->funcs->set_panel_config(mipi, pcfg);
}

static inline int mipi_on(struct mipi_adapter *mipi)
{
	return mipi->funcs->on(mipi);
}

static inline int mipi_off(struct mipi_adapter *mipi)
{
	return mipi->funcs->off(mipi);
}

static inline int mipi_write(struct mipi_adapter *mipi,
		const u8 *data, size_t len)
{
	return mipi->funcs->write(mipi, data, len);
}

static inline int mipi_read(struct mipi_adapter *mipi,
		u8 *data, size_t len)
{
	return mipi->funcs->read(mipi, data, len);
}

/* Helper functions/macros: */

#define DSI_HOST_HDR_SIZE    4
#define DSI_HDR_LAST         BIT(31)
#define DSI_HDR_LONG_PKT     BIT(30)
#define DSI_HDR_BTA          BIT(29)
#define DSI_HDR_VC(vc)       (((vc) & 0x03) << 22)
#define DSI_HDR_DTYPE(dtype) (((dtype) & 0x03f) << 16)
#define DSI_HDR_DATA2(data)  (((data) & 0x0ff) << 8)
#define DSI_HDR_DATA1(data)  ((data) & 0x0ff)
#define DSI_HDR_WC(wc)       ((wc) & 0x0ffff)

/* dcs read/write */
#define DTYPE_DCS_WRITE	     0x05 /* short write, 0 parameter */
#define DTYPE_DCS_WRITE1     0x15 /* short write, 1 parameter */
#define DTYPE_DCS_READ       0x06 /* read */
#define DTYPE_DCS_LWRITE     0x39 /* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE      0x03 /* short write, 0 parameter */
#define DTYPE_GEN_WRITE1     0x13 /* short write, 1 parameter */
#define DTYPE_GEN_WRITE2     0x23 /* short write, 2 parameter */
#define DTYPE_GEN_LWRITE     0x29 /* long write */
#define DTYPE_GEN_READ       0x04 /* long read, 0 parameter */
#define DTYPE_GEN_READ1      0x14 /* long read, 1 parameter */
#define DTYPE_GEN_READ2      0x24 /* long read, 2 parameter */

#define DTYPE_TEAR_ON        0x35 /* set tear on */
#define DTYPE_MAX_PKTSIZE    0x37 /* set max packet size */
#define DTYPE_NULL_PKT       0x09 /* null packet, no data */
#define DTYPE_BLANK_PKT      0x19 /* blanking packet, no data */

#define DTYPE_CM_ON          0x02 /* color mode off */
#define DTYPE_CM_OFF         0x12 /* color mode on */
#define DTYPE_PERIPHERAL_OFF 0x22
#define DTYPE_PERIPHERAL_ON  0x32

#define __mipi_lwrite(mipi, htype, last, vc, payload) ({            \
		u32 __buf[ALIGN(sizeof(payload) + DSI_HOST_HDR_SIZE, 4)/4]; \
		__buf[0]  = DSI_HDR_WC(sizeof(payload));                    \
		__buf[0] |= DSI_HDR_VC(vc);                                 \
		__buf[0] |= DSI_HDR_LONG_PKT;                               \
		__buf[0] |= DSI_HDR_DTYPE(htype);                           \
		if (last) __buf[0] |= DSI_HDR_LAST;                         \
		__buf[ARRAY_SIZE(__buf)-1] = 0xffffffff;                    \
		memcpy(&__buf[1], payload, sizeof(payload));                \
		mipi_write(mipi, (u8 *)__buf, sizeof(__buf));               \
	})

#define __mipi_swrite(mipi, htype, last, vc, ack, val) ({           \
		u32 __buf[1];                                               \
		__buf[0]  = DSI_HDR_VC(vc);                                 \
		__buf[0] |= DSI_HDR_DTYPE(htype);                           \
		if (ack)  __buf[0] |= DSI_HDR_BTA;                          \
		if (last) __buf[0] |= DSI_HDR_LAST;                         \
		__buf[0] |= DSI_HDR_DATA1(val);                             \
		__buf[0] |= DSI_HDR_DATA2(0);                               \
		mipi_write(mipi, (u8 *)__buf, sizeof(__buf));               \
	})

/* long-packet write: */
#define mipi_lwrite(mipi, last, vc, payload) \
	__mipi_lwrite(mipi, DTYPE_GEN_LWRITE, last, vc, payload)

/* dsc short-packet write: */
#define mipi_dcs_swrite(mipi, last, vc, ack, val) \
	__mipi_swrite(mipi, DTYPE_DCS_WRITE, last, vc, ack, val)

#endif /* __MIPI_H__ */
