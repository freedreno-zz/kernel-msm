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

#include "dsi.h"

struct dsi_mipi_adapter {
	struct mipi_adapter base;
	struct dsi *dsi;
	bool cmd_done;
	wait_queue_head_t cmd_event;

	/* TX buffer for DSI cmds..  we could just dma_remap_single(),
	 * except that it seems we have some alignment restrictions.
	 */
	void *buf;
	dma_addr_t pbuf;
};
#define to_dsi_mipi_adapter(x) container_of(x, struct dsi_mipi_adapter, base)

static bool cmd_done(struct dsi_mipi_adapter *dsi_mipi)
{
	struct dsi *dsi = dsi_mipi->dsi;

	if (!dsi_mipi->cmd_done) {
		uint32_t int_ctrl;

		int_ctrl = dsi_read(dsi, REG_DSI_INTR_CTRL);

		if (int_ctrl & DSI_IRQ_CMD_DMA_DONE) {
			dsi_mipi->cmd_done = true;
			dsi_write(dsi, REG_DSI_INTR_CTRL,
					DSI_IRQ_CMD_DMA_DONE);
		}
	}

	return dsi_mipi->cmd_done;
}

void dsi_mipi_irq(struct mipi_adapter *mipi)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);

	if (cmd_done(dsi_mipi))
		wake_up_all(&dsi_mipi->cmd_event);
}

static void dsi_mipi_destroy(struct mipi_adapter *mipi)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	if (dsi_mipi->buf) {
		struct dsi *dsi = dsi_mipi->dsi;
		dma_free_writecombine(&dsi->pdev->dev, PAGE_SIZE,
				dsi_mipi->buf, dsi_mipi->pbuf);
	}
	kfree(dsi_mipi);
}

static int dsi_mipi_set_bus_config(struct mipi_adapter *mipi,
		const struct mipi_bus_config *bcfg)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	struct dsi *dsi = dsi_mipi->dsi;
	uint32_t ctrl;

	dsi_write(dsi, REG_DSI_CMD_DMA_CTRL,
			COND(bcfg->low_power, DSI_CMD_DMA_CTRL_LOW_POWER) |
			DSI_CMD_DMA_CTRL_FROM_FRAME_BUFFER);

	ctrl = dsi_read(dsi, REG_DSI_CTRL);
	ctrl &= ~(DSI_CTRL_LANE0 | DSI_CTRL_LANE1 | DSI_CTRL_LANE2 | DSI_CTRL_LANE3);
	dsi_write(dsi, REG_DSI_CTRL,
			ctrl |
			COND(bcfg->lanes & 0x1, DSI_CTRL_LANE0) |
			COND(bcfg->lanes & 0x2, DSI_CTRL_LANE1) |
			COND(bcfg->lanes & 0x4, DSI_CTRL_LANE2) |
			COND(bcfg->lanes & 0x8, DSI_CTRL_LANE3));

	return 0;
}

static int dsi_mipi_set_panel_config(struct mipi_adapter *mipi,
		const struct mipi_panel_config *pcfg)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	struct dsi *dsi = dsi_mipi->dsi;
	struct dsi_phy *phy = dsi->phy;
	uint32_t ctrl;

	phy->funcs->config(phy, pcfg);

	if (pcfg->cmd_mode) {
		/* TODO */
	} else {
		dsi_write(dsi, REG_DSI_VID_CFG0,
				DSI_VID_CFG0_VIRT_CHANNEL(0) | /* seems always zero */
				DSI_VID_CFG0_DST_FORMAT(pcfg->format) |
				DSI_VID_CFG0_TRAFFIC_MODE(pcfg->traffic_mode) |
				COND(pcfg->bllp_power_stop, DSI_VID_CFG0_BLLP_POWER_STOP) |
				COND(pcfg->eof_bllp_power_stop, DSI_VID_CFG0_EOF_BLLP_POWER_STOP) |
				COND(pcfg->hsa_power_stop, DSI_VID_CFG0_HSA_POWER_STOP) |
				COND(pcfg->hbp_power_stop, DSI_VID_CFG0_HBP_POWER_STOP) |
				COND(pcfg->hfp_power_stop, DSI_VID_CFG0_HFP_POWER_STOP) |
				COND(pcfg->pulse_mode_hsa_he, DSI_VID_CFG0_PULSE_MODE_HSA_HE));
		dsi_write(dsi, REG_DSI_VID_CFG1,
				DSI_VID_CFG1_RGB_SWAP(pcfg->rgb_swap) |
				DSI_VID_CFG1_INTERLEAVE_MAX(pcfg->interleave_max));
	}

	dsi_write(dsi, REG_DSI_CMD_DMA_CTRL,
			DSI_CMD_DMA_CTRL_FROM_FRAME_BUFFER |
			DSI_CMD_DMA_CTRL_LOW_POWER);
	dsi_write(dsi, REG_DSI_TRIG_CTRL,
			DSI_TRIG_CTRL_DMA_TRIGGER(pcfg->dma_trigger) |
			DSI_TRIG_CTRL_MDP_TRIGGER(pcfg->mdp_trigger) |
			COND(pcfg->te, DSI_TRIG_CTRL_TE));
	dsi_write(dsi, REG_DSI_LANE_SWAP_CTRL, pcfg->dlane_swap);
	dsi_write(dsi, REG_DSI_CLKOUT_TIMING_CTRL,
			DSI_CLKOUT_TIMING_CTRL_T_CLK_PRE(pcfg->t_clk_pre) |
			DSI_CLKOUT_TIMING_CTRL_T_CLK_POST(pcfg->t_clk_post));
	dsi_write(dsi, REG_DSI_EOT_PACKET_CTRL,
			COND(pcfg->rx_eot_ignore, DSI_EOT_PACKET_CTRL_TX_EOT_APPEND) |
			COND(pcfg->tx_eot_append, DSI_EOT_PACKET_CTRL_RX_EOT_IGNORE));

	dsi_write(dsi, REG_DSI_ERR_INT_MASK0, 0x13ff3fe0);  /* ??? */
	dsi_write(dsi, REG_DSI_CLK_CTRL, 0x23f);

	ctrl = dsi_read(dsi, REG_DSI_CTRL);
	ctrl &= DSI_CTRL_ENABLE | DSI_CTRL_VID_MODE_EN | DSI_CTRL_CMD_MODE_EN |
			DSI_CTRL_LANE0 | DSI_CTRL_LANE1 | DSI_CTRL_LANE2 | DSI_CTRL_LANE3;
	dsi_write(dsi, REG_DSI_CTRL,
			ctrl |
			DSI_CTRL_CLK_EN |
			COND(pcfg->ecc_check, DSI_CTRL_ECC_CHECK) |
			COND(pcfg->crc_check, DSI_CTRL_CRC_CHECK));

	return 0;
}

static int dsi_mipi_on(struct mipi_adapter *mipi)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	struct dsi *dsi = dsi_mipi->dsi;
	struct dsi_phy *phy = dsi->phy;
	uint32_t ctrl;

	dsi_clk_prepare(dsi);
	dsi_clk_enable_ahb(dsi);

	phy->funcs->powerup(phy, 0/*dsi_bridge->pixclock*/);

	ctrl = dsi_read(dsi, REG_DSI_CTRL);
	dsi_write(dsi, REG_DSI_CTRL, ctrl | DSI_CTRL_ENABLE);
	dsi_write(dsi, REG_DSI_INTR_CTRL,
			DSI_IRQ_CMD_DMA_DONE | DSI_IRQ_MASK_CMD_DMA_DONE |
			DSI_IRQ_ERROR | DSI_IRQ_MASK_ERROR);

	phy->funcs->clk_enable(phy);

	return 0;
}

static int dsi_mipi_off(struct mipi_adapter *mipi)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	struct dsi *dsi = dsi_mipi->dsi;
	struct dsi_phy *phy = dsi->phy;

	phy->funcs->clk_disable(phy);

	dsi_write(dsi, REG_DSI_INTR_CTRL, 0);
	dsi_write(dsi, REG_DSI_CTRL, 0);

	phy->funcs->powerdown(phy);

	dsi_clk_disable_ahb(dsi);
	dsi_clk_unprepare(dsi);

	return 0;
}

static int dsi_mipi_write(struct mipi_adapter *mipi, const u8 *data, size_t len)
{
	struct dsi_mipi_adapter *dsi_mipi = to_dsi_mipi_adapter(mipi);
	struct dsi *dsi = dsi_mipi->dsi;
	uint32_t ctrl;
	int ret = len;

	len = ALIGN(len, 4);

	memcpy(dsi_mipi->buf, data, len);

	ctrl = dsi_read(dsi, REG_DSI_CTRL);
	if (ctrl & DSI_CTRL_VID_MODE_EN) {
		uint32_t cnt = 0xffff;
		dsi_write(dsi, REG_DSI_CTRL,
				(ctrl & ~DSI_CTRL_VID_MODE_EN) |
				DSI_CTRL_CMD_MODE_EN);
		/* wait for video mode to complete: */
		do {
			uint32_t status = dsi_read(dsi, REG_DSI_STATUS0);
			if (!(status & DSI_STATUS0_VIDEO_MODE_ENGINE_BUSY))
				break;
			udelay(100);
		} while (--cnt > 0);
	}

	dsi_mipi->cmd_done = false;

	dsi_write(dsi, REG_DSI_DMA_BASE, dsi_mipi->pbuf);
	dsi_write(dsi, REG_DSI_DMA_LEN, len);
	dsi_write(dsi, REG_DSI_TRIG_DMA, 1);
	wmb();

	if (!wait_event_timeout(dsi_mipi->cmd_event, cmd_done(dsi_mipi), HZ)) {
		dev_warn(dsi->dev->dev, "DSI timeout\n");
		ret = -ETIMEDOUT;
	}

	if (ctrl & DSI_CTRL_VID_MODE_EN)
		dsi_write(dsi, REG_DSI_CTRL, ctrl);

	return ret;
}

static int dsi_mipi_read(struct mipi_adapter *mipi, u8 *data, size_t len)
{
	// TODO;
	return WARN_ON(-1);
}

static const struct mipi_adapter_funcs dsi_mipi_funcs = {
		.destroy = dsi_mipi_destroy,
		.set_bus_config = dsi_mipi_set_bus_config,
		.set_panel_config = dsi_mipi_set_panel_config,
		.on = dsi_mipi_on,
		.off = dsi_mipi_off,
		.write = dsi_mipi_write,
		.read = dsi_mipi_read,
};

struct mipi_adapter *dsi_mipi_init(struct dsi *dsi)
{
	struct drm_device *dev = dsi->dev;
	struct dsi_mipi_adapter *dsi_mipi;
	struct mipi_adapter *mipi = NULL;
	int ret;

	dsi_mipi = kzalloc(sizeof(*dsi_mipi), GFP_KERNEL);
	if (!dsi_mipi) {
		ret = -ENOMEM;
		goto fail;
	}

	dsi_mipi->buf = dma_alloc_writecombine(&dsi->pdev->dev,
			PAGE_SIZE, &dsi_mipi->pbuf, GFP_KERNEL);
	if (!dsi_mipi->buf) {
		ret = -ENOMEM;
		goto fail;
	}

	mipi = &dsi_mipi->base;

	dsi_mipi->dsi = dsi;
	init_waitqueue_head(&dsi_mipi->cmd_event);

	ret = mipi_adapter_init(mipi, &dsi_mipi_funcs);
	if (ret) {
		dev_err(dev->dev, "failed to register dsi mipi: %d\n", ret);
		goto fail;
	}

	return mipi;

fail:
	if (mipi)
		dsi_mipi_destroy(mipi);
	return ERR_PTR(ret);
}
