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

#include <linux/kernel.h>
#include <linux/cpuset.h>
#include <linux/qcom_scm.h>

#include "msm_drv.h"
#include "ocmem.h"
#include "ocmem.xml.h"

enum region_mode {
	WIDE_MODE = 0x0,
	THIN_MODE,
	MODE_DEFAULT = WIDE_MODE,
};

struct ocmem_region {
	unsigned psgsc_ctrl;
	bool interleaved;
	unsigned int mode;
	unsigned int num_macros;
	enum ocmem_macro_state macro_state[4];
	unsigned long macro_size;
	unsigned long region_size;
};

struct ocmem_config {
	uint8_t  num_regions;
	uint8_t  num_macros;     // XXX always same as value from OCMEM_HW_PROFILE?
	uint32_t macro_size;
	uint32_t resource_type;  // XXX do we need??
};

struct ocmem {
	struct device *dev;
	const struct ocmem_config *config;
	struct resource *ocmem_mem;
	struct clk *core_clk;
	struct clk *iface_clk;
	void __iomem *mmio;

	unsigned num_ports;
	unsigned num_macros;
	bool interleaved;

	struct ocmem_region *regions;
};

struct ocmem *ocmem;


static inline void ocmem_write(struct ocmem *ocmem, u32 reg, u32 data)
{
	msm_writel(data, ocmem->mmio + reg);
}

static inline u32 ocmem_read(struct ocmem *ocmem, u32 reg)
{
	return msm_readl(ocmem->mmio + reg);
}

static int ocmem_clk_enable(struct ocmem *ocmem)
{
	int ret;

	ret = clk_prepare_enable(ocmem->core_clk);
	if (ret)
		return ret;

	if (ocmem->iface_clk) {
		ret = clk_prepare_enable(ocmem->iface_clk);
		if (ret)
			return ret;
	}

	return 0;
}

static void ocmem_clk_disable(struct ocmem *ocmem)
{
	if (ocmem->iface_clk)
		clk_disable_unprepare(ocmem->iface_clk);
	clk_disable_unprepare(ocmem->core_clk);
}

static int ocmem_dev_remove(struct platform_device *pdev)
{
	ocmem_clk_disable(ocmem);
	return 0;
}

static void update_ocmem(struct ocmem *ocmem)
{
	uint32_t region_mode_ctrl = 0x0;
	unsigned pos = 0;
	unsigned i = 0;

	for (i = 0; i < ocmem->config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];
		pos = i << 2;
		if (region->mode == THIN_MODE)
			region_mode_ctrl |= BIT(pos);
	}
	dev_dbg(ocmem->dev, "ocmem_region_mode_control %x\n", region_mode_ctrl);
	ocmem_write(ocmem, REG_OCMEM_REGION_MODE_CTL, region_mode_ctrl);
	/* Barrier to commit the region mode */
	mb();

	for (i = 0; i < ocmem->config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];

		ocmem_write(ocmem, REG_OCMEM_PSGSC_CTL(i),
				OCMEM_PSGSC_CTL_MACRO0_MODE(region->macro_state[0]) |
				OCMEM_PSGSC_CTL_MACRO1_MODE(region->macro_state[1]) |
				OCMEM_PSGSC_CTL_MACRO2_MODE(region->macro_state[2]) |
				OCMEM_PSGSC_CTL_MACRO3_MODE(region->macro_state[3]));
	}
}

inline unsigned long phys_to_offset(unsigned long addr)
{
	if ((addr < ocmem->ocmem_mem->start) ||
		(addr >= ocmem->ocmem_mem->end))
		return 0;
	return addr - ocmem->ocmem_mem->start;
}

static unsigned long device_address(enum ocmem_client client, unsigned long addr)
{
	/* TODO, gpu uses phys_to_offset, but others do not.. */
	return phys_to_offset(addr);
}

struct ocmem_buf *ocmem_allocate(enum ocmem_client client, unsigned long size)
{
	struct ocmem_buf *buf;
	unsigned long start;
	int i, j;

	if (!ocmem)
		return ERR_PTR(-ENXIO);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);

	/* TODO less hard-coded allocation that works for more than
	 * one user:
	 */
	start = 0;
	buf->len = size;
	buf->addr = device_address(client, start);
	for (i = 0; i < ocmem->config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];

		for (j = 0; j < region->num_macros; j++) {
			region->macro_state[j] = CORE_ON;
			if (size < region->macro_size)
				break;
			size -= region->macro_size;
		}
	}

	update_ocmem(ocmem);

	if (client == OCMEM_GRAPHICS) {
		ocmem_write(ocmem, REG_OCMEM_GFX_MPU_START, buf->addr);
		ocmem_write(ocmem, REG_OCMEM_GFX_MPU_END, buf->addr + buf->len);
	}

	return buf;
}

void ocmem_free(enum ocmem_client client, struct ocmem_buf *buf)
{
	kfree(buf);
}

static const struct ocmem_config ocmem_8974_config = {
	.num_regions = 3, .num_macros = 24, .macro_size = SZ_128K, .resource_type = 0x706d636f,
};

static const struct of_device_id dt_match[] = {
	{ .compatible = "qcom,msm-ocmem-8974", .data = &ocmem_8974_config },
	{}
};

static int ocmem_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct ocmem_config *config = NULL;
	uint32_t reg, num_banks, region_size;
	int i, j, ret;

	struct device_node *of_node = dev->of_node;
	const struct of_device_id *match;

	match = of_match_node(dt_match, dev->of_node);
	if (match)
		config = match->data;

	if (!config) {
		dev_err(dev, "unknown config: %s\n", of_node->name);
		return -ENXIO;
	}

	ocmem = devm_kzalloc(dev, sizeof(*ocmem), GFP_KERNEL);
	if (!ocmem)
		return -ENOMEM;

	ocmem->dev = dev;
	ocmem->config = config;

	ocmem->core_clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(ocmem->core_clk)) {
		dev_err(dev, "Unable to get the core clock\n");
		return PTR_ERR(ocmem->core_clk);
	}

	ocmem->iface_clk = devm_clk_get(dev, "iface_clk");
	if (IS_ERR_OR_NULL(ocmem->iface_clk))
		ocmem->iface_clk = NULL;

	ocmem->mmio = msm_ioremap(pdev, "ocmem_ctrl_physical", "OCMEM");
	if (IS_ERR(ocmem->mmio))
		return PTR_ERR(ocmem->mmio);

	ocmem->ocmem_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"ocmem_physical");
	if (!ocmem->ocmem_mem) {
		dev_err(dev, "could not get OCMEM region\n");
		return -ENXIO;
	}

	ret = ocmem_clk_enable(ocmem);
	if (ret)
		goto fail;

	if (qcom_scm_is_available() && qcom_scm_ocmem_secure_available()) {
		dev_info(dev, "configuring scm\n");
		ret = qcom_scm_ocmem_secure_cfg(0x5);
		if (ret)
			goto fail;
	}

	reg = ocmem_read(ocmem, REG_OCMEM_HW_PROFILE);
	ocmem->num_ports = FIELD(reg, OCMEM_HW_PROFILE_NUM_PORTS);
	ocmem->num_macros = FIELD(reg, OCMEM_HW_PROFILE_NUM_MACROS);
	ocmem->interleaved = !!(reg & OCMEM_HW_PROFILE_INTERLEAVING);

	num_banks = ocmem->num_ports / 2;
	region_size = config->macro_size * num_banks;

	dev_info(dev, "%u ports, %u regions, %u macros, %sinterleaved\n",
			ocmem->num_ports, config->num_regions, ocmem->num_macros,
			ocmem->interleaved ? "" : "not ");

	WARN_ON(ocmem->num_macros != config->num_macros);

	ocmem->regions = devm_kzalloc(dev, sizeof(struct ocmem_region) *
			config->num_regions, GFP_KERNEL);
	if (!ocmem->regions) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];

		if (WARN_ON(num_banks > ARRAY_SIZE(region->macro_state))) {
			ret = -EINVAL;
			goto fail;
		}

		region->mode = MODE_DEFAULT;
		region->num_macros = num_banks;

		if ((i == (config->num_regions - 1)) &&
				(reg & OCMEM_HW_PROFILE_LAST_REGN_HALFSIZE)) {
			region->macro_size = config->macro_size / 2;
			region->region_size = region_size / 2;
		} else {
			region->macro_size = config->macro_size;
			region->region_size = region_size;
		}

		for (j = 0; j < ARRAY_SIZE(region->macro_state); j++)
			region->macro_state[j] = CLK_OFF;
	}

	return 0;

fail:
	dev_err(dev, "probe failed\n");
	ocmem_dev_remove(pdev);
	return ret;
}

static struct platform_driver ocmem_driver = {
	.probe = ocmem_dev_probe,
	.remove = ocmem_dev_remove,
	.driver = {
		.name = "ocmem",
		.of_match_table = dt_match,
	},
};

void __init ocmem_register(void)
{
	platform_driver_register(&ocmem_driver);
}

void __exit ocmem_unregister(void)
{
	platform_driver_unregister(&ocmem_driver);
}
