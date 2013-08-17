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

static struct platform_device *dsi_pdev;

static irqreturn_t dsi_irq(int irq, void *dev_id)
{
	struct dsi *dsi = dev_id;
	uint32_t status;

	status = dsi_read(dsi, REG_DSI_INTR_CTRL);

	if (status & DSI_IRQ_ERROR) {
		uint32_t err;

		err = dsi_read(dsi, REG_DSI_ACK_ERR_STATUS);
		if (err) {
			DBG("err: %08x\n", err);
			dsi_write(dsi, REG_DSI_ACK_ERR_STATUS, err);
		}

		err = dsi_read(dsi, REG_DSI_TIMEOUT_STATUS);
		if (err & 0x00000111) {
			DBG("timeout: %08x\n", err);
			dsi_write(dsi, REG_DSI_TIMEOUT_STATUS, err);
		}

		err = dsi_read(dsi, REG_DSI_FIFO_STATUS);
		if (err & 0x44444489) {
			DBG("fifo: %08x\n", err);
			dsi_write(dsi, REG_DSI_FIFO_STATUS, err);
		}

		err = dsi_read(dsi, REG_DSI_STATUS0);
		if (err & 0x80000000) {
			DBG("status: %08x\n", err);
			dsi_write(dsi, REG_DSI_STATUS0, err);
		}

		err = dsi_read(dsi, REG_DSI_DLN0_PHY_ERR);
		if (err & 0x00011111) {
			DBG("phy: %08x\n", err);
			dsi_write(dsi, REG_DSI_DLN0_PHY_ERR, err);
		}
	}

	/* Process DSI MIPI bus: */
	dsi_mipi_irq(dsi->mipi);

	dsi_write(dsi, REG_DSI_INTR_CTRL, status);

	return IRQ_HANDLED;
}

void dsi_destroy(struct kref *kref)
{
	struct dsi *dsi = container_of(kref, struct dsi, refcount);
	struct dsi_phy *phy = dsi->phy;

	if (dsi->panel)
		panel_destroy(dsi->panel);

	if (phy)
		phy->funcs->destroy(phy);

	if (dsi->mipi)
		mipi_destroy(dsi->mipi);

	put_device(&dsi->pdev->dev);
}

/* initialize connector */
int dsi_init(struct drm_device *dev, struct drm_encoder *encoder)
{
	struct dsi *dsi = NULL;
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = dsi_pdev;
	struct dsi_platform_config *config;
	int ret;

	if (!pdev) {
		dev_err(dev->dev, "no dsi device\n");
		ret = -ENXIO;
		goto fail;
	}

	config = pdev->dev.platform_data;

	dsi = kzalloc(sizeof(*dsi), GFP_KERNEL);
	if (!dsi) {
		ret = -ENOMEM;
		goto fail;
	}

	kref_init(&dsi->refcount);

	get_device(&pdev->dev);

	dsi->dev = dev;
	dsi->pdev = pdev;
	dsi->encoder = encoder;

	if (config->phy_init)
		dsi->phy = config->phy_init(dsi);
	else
		dsi->phy = ERR_PTR(-ENXIO);

	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		dev_err(dev->dev, "failed to load phy: %d\n", ret);
		dsi->phy = NULL;
		goto fail;
	}

	dsi->mmio = msm_ioremap(pdev, "mipi_dsi", "DSI");
	if (IS_ERR(dsi->mmio)) {
		ret = PTR_ERR(dsi->mmio);
		goto fail;
	}

	dsi->sfpb = msm_ioremap(pdev, "mmss_sfpb", "SFPB");
	if (IS_ERR(dsi->sfpb))
		dsi->sfpb = NULL;

	dsi->cc = msm_ioremap(pdev, "mmss_cc", "MMSS_CC");
	if (IS_ERR(dsi->cc))
		dsi->cc = NULL;

	DBG("mmio=%p, sfpb=%p, cc=%p", dsi->mmio, dsi->sfpb, dsi->cc);

	dsi->amp_pclk = devm_clk_get(&pdev->dev, "arb_clk");
	if (IS_ERR(dsi->amp_pclk)) {
		ret = PTR_ERR(dsi->amp_pclk);
		dev_err(dev->dev, "failed to get 'amp_pclk': %d\n", ret);
		goto fail;
	}

	dsi->m_pclk = devm_clk_get(&pdev->dev, "master_iface_clk");
	if (IS_ERR(dsi->m_pclk)) {
		ret = PTR_ERR(dsi->m_pclk);
		dev_err(dev->dev, "failed to get 'm_pclk': %d\n", ret);
		goto fail;
	}

	dsi->s_pclk = devm_clk_get(&pdev->dev, "slave_iface_clk");
	if (IS_ERR(dsi->s_pclk)) {
		ret = PTR_ERR(dsi->s_pclk);
		dev_err(dev->dev, "failed to get 's_pclk': %d\n", ret);
		goto fail;
	}

	dsi->byte_div_clk = devm_clk_get(&pdev->dev, "byte_clk");
	if (IS_ERR(dsi->byte_div_clk)) {
		ret = PTR_ERR(dsi->byte_div_clk);
		dev_err(dev->dev, "failed to get 'byte_div_clk': %d\n", ret);
		goto fail;
	}

	dsi->esc_clk = devm_clk_get(&pdev->dev, "esc_clk");
	if (IS_ERR(dsi->esc_clk)) {
		ret = PTR_ERR(dsi->esc_clk);
		dev_err(dev->dev, "failed to get 'esc_clk': %d\n", ret);
		goto fail;
	}

	dsi->mipi = dsi_mipi_init(dsi);
	if (IS_ERR(dsi->mipi)) {
		ret = PTR_ERR(dsi->mipi);
		dev_err(dev->dev, "failed to get DSI adapter: %d\n", ret);
		dsi->mipi = NULL;
		goto fail;
	}

	dsi->panel = mipi_panel_init(dsi->dev, dsi->pdev, dsi->mipi);
	if (IS_ERR(dsi->panel)) {
		ret = PTR_ERR(dsi->panel);
		dev_err(dsi->dev->dev, "failed to load panel: %d\n", ret);
		goto fail;
	}

	dsi->bridge = dsi_bridge_init(dsi);
	if (IS_ERR(dsi->bridge)) {
		ret = PTR_ERR(dsi->bridge);
		dev_err(dev->dev, "failed to create HDMI bridge: %d\n", ret);
		dsi->bridge = NULL;
		goto fail;
	}

	dsi->connector = dsi_connector_init(dsi);
	if (IS_ERR(dsi->connector)) {
		ret = PTR_ERR(dsi->connector);
		dev_err(dev->dev, "failed to create HDMI connector: %d\n", ret);
		dsi->connector = NULL;
		goto fail;
	}

	dsi_write(dsi, REG_DSI_RESET, 1);
	dsi_write(dsi, REG_DSI_RESET, 0);

	dsi_write(dsi, REG_DSI_INTR_CTRL, 0);

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		ret = dsi->irq;
		dev_err(dev->dev, "failed to get irq: %d\n", ret);
		goto fail;
	}

	ret = devm_request_irq(&pdev->dev, dsi->irq, dsi_irq,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "dsi_isr", dsi);
	if (ret < 0) {
		dev_err(dev->dev, "failed to request IRQ%u: %d\n",
				dsi->irq, ret);
		goto fail;
	}

	encoder->bridge = dsi->bridge;

	priv->bridges[priv->num_bridges++]       = dsi->bridge;
	priv->connectors[priv->num_connectors++] = dsi->connector;

	return 0;

fail:
	if (dsi) {
		/* bridge/connector are normally destroyed by drm: */
		if (dsi->bridge)
			dsi->bridge->funcs->destroy(dsi->bridge);
		if (dsi->connector)
			dsi->connector->funcs->destroy(dsi->connector);
		dsi_destroy(&dsi->refcount);
	}

	return ret;
}

/*
 * The dsi device:
 */

static int dsi_dev_probe(struct platform_device *pdev)
{
	static struct dsi_platform_config config = {};
#ifdef CONFIG_OF
	/* TODO */
#else
	if (cpu_is_apq8064() || cpu_is_msm8960())
		config.phy_init = dsi_phy_8960_init;
	else if (cpu_is_msm8x60())
		config.phy_init = dsi_phy_8x60_init;
#endif
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.platform_data = &config;
	dsi_pdev = pdev;
	return 0;
}

static int dsi_dev_remove(struct platform_device *pdev)
{
	dsi_pdev = NULL;
	return 0;
}

static struct platform_driver dsi_driver = {
	.probe = dsi_dev_probe,
	.remove = dsi_dev_remove,
	.driver.name = "mipi_dsi",
};

void __init dsi_register(void)
{
	platform_driver_register(&dsi_driver);
}

void __exit dsi_unregister(void)
{
	platform_driver_unregister(&dsi_driver);
}
