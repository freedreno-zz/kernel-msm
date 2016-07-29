/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/soc/qcom/smd.h>

#include "remoteproc_internal.h"

#define PAS_INIT_IMAGE_CMD      1
#define PAS_MEM_SETUP_CMD       2
#define PAS_AUTH_AND_RESET_CMD  5
#define PAS_SHUTDOWN_CMD        6
#define PAS_IS_SUPPORTED_CMD    7

struct qproc {
	struct device *dev;
	struct rproc *rproc;

	int pas_id;

	int wdog_irq;
	int fatal_irq;
	int ready_irq;
	int handover_irq;
	int stop_ack_irq;

	struct gpio_desc *stop_gpio;

	const char *name;
	struct regulator *pll;

	unsigned proxy_clk_count;
	bool	no_scm_clks;
	struct clk *scm_core_clk;
	struct clk *scm_iface_clk;
	struct clk *scm_bus_clk;
	struct clk *scm_src_clk;

	struct clk **proxy_clks;

	struct completion start_done;
	struct completion stop_done;

	unsigned crash_reason;
	struct device_node *smd_edge_node;

	phys_addr_t reloc_phys;
	size_t reloc_size;
};

static int qproc_scm_clk_enable(struct qproc *qproc)
{
	int ret;

	if (qproc->no_scm_clks)
		return 0;

	ret = clk_prepare_enable(qproc->scm_core_clk);
	if (ret)
		goto bail;
	ret = clk_prepare_enable(qproc->scm_iface_clk);
	if (ret)
		goto disable_core;
	ret = clk_prepare_enable(qproc->scm_bus_clk);
	if (ret)
		goto disable_iface;

	ret = clk_prepare_enable(qproc->scm_src_clk);
	if (ret)
		goto disable_bus;

	return 0;

disable_bus:
	clk_disable_unprepare(qproc->scm_bus_clk);
disable_iface:
	clk_disable_unprepare(qproc->scm_iface_clk);
disable_core:
	clk_disable_unprepare(qproc->scm_core_clk);
bail:
	return ret;
}

static void qproc_scm_clk_disable(struct qproc *qproc)
{
	if (qproc->no_scm_clks)
		return;

	clk_disable_unprepare(qproc->scm_core_clk);
	clk_disable_unprepare(qproc->scm_iface_clk);
	clk_disable_unprepare(qproc->scm_bus_clk);
	clk_disable_unprepare(qproc->scm_src_clk);
}

/**
 * struct pil_mdt - Representation of <name>.mdt file in memory
 * @hdr: ELF32 header
 * @phdr: ELF32 program headers
 */
struct mdt_hdr {
	struct elf32_hdr hdr;
	struct elf32_phdr phdr[];
};

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int segment_is_loadable(const struct elf32_phdr *p)
{
	return (p->p_type == PT_LOAD) &&
	       !segment_is_hash(p->p_flags) &&
	       p->p_memsz;
}

static bool segment_is_relocatable(const struct elf32_phdr *p)
{
	return !!(p->p_flags & BIT(27));
}

/**
 * rproc_mdt_sanity_check() - sanity check mdt firmware header
 * @rproc: the remote processor handle
 * @fw: the mdt header firmware image
 */
static int qproc_sanity_check(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct elf32_hdr *ehdr;
	struct mdt_hdr *mdt;

	if (!fw) {
		dev_err(&rproc->dev, "failed to load %s\n", rproc->name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(&rproc->dev, "image is too small\n");
		return -EINVAL;
	}

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(&rproc->dev, "image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(&rproc->dev, "no loadable segments\n");
		return -EINVAL;
	}

	if (sizeof(struct elf32_phdr) * ehdr->e_phnum +
	    sizeof(struct elf32_hdr) > fw->size) {
		dev_err(&rproc->dev, "firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}

static struct resource_table * qproc_find_rsc_table(struct rproc *rproc,
						    const struct firmware *fw,
						    int *tablesz)
{
	static struct resource_table table = { .ver = 1, };

	*tablesz = sizeof(table);
	return &table;
}

static int qproc_load_segment(struct rproc *rproc, const char *fw_name,
				const struct elf32_phdr *phdr, phys_addr_t paddr)
{
	const struct firmware *fw;
	void *ptr;
	int ret = 0;

	ptr = ioremap_nocache(paddr, phdr->p_memsz);
	if (!ptr) {
		dev_err(&rproc->dev, "failed to ioremap segment area (%pa+0x%x)\n", &paddr, phdr->p_memsz);
		return -EBUSY;
	}

	if (phdr->p_filesz) {
		ret = request_firmware(&fw, fw_name, &rproc->dev);
		if (ret) {
			dev_err(&rproc->dev, "failed to load %s\n", fw_name);
			goto out;
		}

		memcpy_toio(ptr, fw->data, fw->size);

		release_firmware(fw);
	}

	if (phdr->p_memsz > phdr->p_filesz)
		memset_io(ptr + phdr->p_filesz, 0,
			  phdr->p_memsz - phdr->p_filesz);

out:
	iounmap(ptr);
	return ret;
}

static int qproc_load(struct rproc *rproc, const struct firmware *fw)
{
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	const struct mdt_hdr *mdt;
	phys_addr_t min_addr = (phys_addr_t)ULLONG_MAX;
	phys_addr_t max_addr = 0;
	phys_addr_t diff_addr;
	struct qproc *qproc = rproc->priv;
	char *fw_name;
	int ret;
	int i;
	size_t align = 0;
	bool relocatable = false;
	phys_addr_t paddr;

	ret = qproc_scm_clk_enable(qproc);
	if (ret)
		return ret;

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		if (phdr->p_paddr < min_addr) {
			min_addr = phdr->p_paddr;

			if (segment_is_relocatable(phdr)) {
				align = phdr->p_align;
				relocatable = true;
			}
		}

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = round_up(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	ret = qcom_scm_pas_init_image(qproc->pas_id, fw->data, fw->size);
	if (ret) {
		dev_err(qproc->dev, "Invalid firmware metadata\n");
		return -EINVAL;
	}

	diff_addr = max_addr - min_addr;
	dev_dbg(qproc->dev, "pas_mem_setup %pa, %pa\n", &min_addr, &diff_addr);

	ret = qcom_scm_pas_mem_setup(qproc->pas_id,
		relocatable ? qproc->reloc_phys : min_addr, max_addr - min_addr);
	if (ret) {
		dev_err(qproc->dev, "unable to setup memory for image\n");
		return -EINVAL;
	}

	fw_name = kzalloc(strlen(qproc->name) + 5, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		paddr = relocatable ?
				(phdr->p_paddr - min_addr + qproc->reloc_phys) :
				phdr->p_paddr;
		sprintf(fw_name, "%s.b%02d", qproc->name, i);
		ret = qproc_load_segment(rproc, fw_name, phdr, paddr);
		if (ret)
			break;
	}

	kfree(fw_name);

	qproc_scm_clk_disable(qproc);

	return 0;
}

const struct rproc_fw_ops qproc_fw_ops = {
	.find_rsc_table = qproc_find_rsc_table,
	.load = qproc_load,
	.sanity_check = qproc_sanity_check,
};

static int qproc_start(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;
	int ret;

	ret = regulator_enable(qproc->pll);
	if (ret) {
		dev_err(qproc->dev, "failed to enable pll supply\n");
		return ret;
	}

	ret = qproc_scm_clk_enable(qproc);
	if (ret)
		goto disable_regulator;

	ret = qcom_scm_pas_auth_and_reset(qproc->pas_id);
	if (ret) {
		dev_err(qproc->dev,
				"failed to authenticate image and release reset\n");
		goto unroll_clocks;
	}

	/* if ready irq not provided skip waiting */
	if (qproc->ready_irq < 0)
		goto done;

	ret = wait_for_completion_timeout(&qproc->start_done, msecs_to_jiffies(10000));
	if (ret == 0) {
		dev_err(qproc->dev, "start timed out\n");

		qcom_scm_pas_shutdown(qproc->pas_id);
		goto unroll_clocks;
	}

done:
	dev_info(qproc->dev, "start successful\n");

	return 0;

unroll_clocks:
	qproc_scm_clk_disable(qproc);

disable_regulator:
	regulator_disable(qproc->pll);

	return ret;
}

static int qproc_stop(struct rproc *rproc)
{
	return 0;
}

static const struct rproc_ops qproc_ops = {
	.start = qproc_start,
	.stop = qproc_stop,
};

static irqreturn_t qproc_wdog_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	rproc_report_crash(qproc->rproc, RPROC_WATCHDOG);
	return IRQ_HANDLED;
}

static irqreturn_t qproc_fatal_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;
	size_t len;
	char *msg;

	msg = qcom_smem_get(-1, qproc->crash_reason, &len);
	if (IS_ERR(msg) && len > 0 && msg[0])
		dev_err(qproc->dev, "fatal error received: %s\n", msg);

	rproc_report_crash(qproc->rproc, RPROC_FATAL_ERROR);

	return IRQ_HANDLED;
}

static irqreturn_t qproc_ready_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	complete(&qproc->start_done);

	return IRQ_HANDLED;
}

static irqreturn_t qproc_handover_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	qproc_scm_clk_disable(qproc);
	regulator_disable(qproc->pll);
	return IRQ_HANDLED;
}

static irqreturn_t qproc_stop_ack_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	complete(&qproc->stop_done);
	return IRQ_HANDLED;
}

static ssize_t qproc_boot_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct qproc *qproc = dev_get_drvdata(dev);
	int ret;

	ret = rproc_boot(qproc->rproc);
	return ret ? : size;
}

static ssize_t qproc_shutdown_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct qproc *qproc = dev_get_drvdata(dev);

	rproc_shutdown(qproc->rproc);
	return size;
}

static const struct device_attribute qproc_attrs[] = {
	__ATTR(boot, S_IWUSR, 0, qproc_boot_store),
	__ATTR(shutdown, S_IWUSR, 0, qproc_shutdown_store),
};

static int qproc_init_pas(struct qproc *qproc)
{
	char *key;
	int ret;

	key = "qcom,pas-id";
	ret = of_property_read_u32(qproc->dev->of_node, key, &qproc->pas_id);
	if (ret) {
		dev_err(qproc->dev, "Missing or incorrect %s\n", key);
		return -EINVAL;
	}

	if (!qcom_scm_pas_supported(qproc->pas_id)) {
		dev_err(qproc->dev, "PAS is not available for %d\n", qproc->pas_id);
		return -EIO;
	}

	return 0;
}

static int qproc_init_clocks(struct qproc *qproc)
{
	long rate;
	int ret;

	if (qproc->no_scm_clks)
		return 0;

	qproc->scm_core_clk = devm_clk_get(qproc->dev, "scm_core_clk");
	if (IS_ERR(qproc->scm_core_clk)) {
		if (PTR_ERR(qproc->scm_core_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_core_clk\n");
		return PTR_ERR(qproc->scm_core_clk);
	}

	qproc->scm_iface_clk = devm_clk_get(qproc->dev, "scm_iface_clk");
	if (IS_ERR(qproc->scm_iface_clk)) {
		if (PTR_ERR(qproc->scm_iface_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_iface_clk\n");
		return PTR_ERR(qproc->scm_iface_clk);
	}

	qproc->scm_bus_clk = devm_clk_get(qproc->dev, "scm_bus_clk");
	if (IS_ERR(qproc->scm_bus_clk)) {
		if (PTR_ERR(qproc->scm_bus_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_bus_clk\n");
		return PTR_ERR(qproc->scm_bus_clk);
	}

	qproc->scm_src_clk = devm_clk_get(qproc->dev, "scm_src_clk");
	if (IS_ERR(qproc->scm_src_clk)) {
		if (PTR_ERR(qproc->scm_src_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_src_clk\n");
		return PTR_ERR(qproc->scm_src_clk);
	}

	ret = clk_set_rate(qproc->scm_core_clk,
clk_round_rate(qproc->scm_core_clk, 19200000));
	ret = clk_set_rate(qproc->scm_bus_clk,
clk_round_rate(qproc->scm_bus_clk, 19200000));
	ret = clk_set_rate(qproc->scm_iface_clk,
clk_round_rate(qproc->scm_iface_clk, 19200000));
	rate = clk_round_rate(qproc->scm_core_clk, 80000000);
	ret = clk_set_rate(qproc->scm_src_clk, rate);
	if (ret) {
		dev_err(qproc->dev, "failed to set rate of scm_core_clk\n");
		return ret;
	}

	return 0;
}

static int qproc_init_regulators(struct qproc *qproc)
{
	int ret;
	u32 uA;
	u32 uV;

	qproc->pll = devm_regulator_get(qproc->dev, "qcom,pll");
	if (IS_ERR(qproc->pll)) {
		if (PTR_ERR(qproc->pll) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to aquire regulator\n");
		return PTR_ERR(qproc->pll);
	}

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,pll-uV", &uV);
	if (ret)
		dev_warn(qproc->dev, "failed to read qcom,pll_uV, skipping\n");
	else
		regulator_set_voltage(qproc->pll, uV, uV);

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,pll-uA", &uA);
	if (ret)
		dev_warn(qproc->dev, "failed to read qcom,pll_uA, skipping\n");
	else
		regulator_set_load(qproc->pll, uA);

	return 0;
}

static int qproc_request_irq(struct qproc *qproc, struct platform_device *pdev, const char *name, irq_handler_t thread_fn)
{
	int ret;

	ret = platform_get_irq_byname(pdev, name);
	if (ret < 0) {
		dev_err(&pdev->dev, "no %s IRQ defined\n", name);
		return ret;
	}

	ret = devm_request_threaded_irq(&pdev->dev, ret,
					NULL, thread_fn,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"qproc", qproc);
	if (ret)
		dev_err(&pdev->dev, "request %s IRQ failed\n", name);
	return ret;
}

static int qproc_probe(struct platform_device *pdev)
{
	struct qproc *qproc;
	struct rproc *rproc;
	char *fw_name;
	const char *name;
	const char *key;
	int ret;
	int i;
	struct device_node *np;
	struct resource r;


	key = "qcom,firmware-name";
	ret = of_property_read_string(pdev->dev.of_node, key, &name);
	if (ret) {
		dev_err(&pdev->dev, "missing or incorrect %s\n", key);
		return -EINVAL;
	}

	fw_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s.mdt", name);
	if (!fw_name)
		return -ENOMEM;

	rproc = rproc_alloc(&pdev->dev, pdev->name, &qproc_ops,
			    fw_name, sizeof(*qproc));
	if (!rproc) {
		dev_err(&pdev->dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc->fw_ops = &qproc_fw_ops;

	qproc = (struct qproc *)rproc->priv;
	qproc->dev = &pdev->dev;
	qproc->rproc = rproc;
	qproc->name = name;
	platform_set_drvdata(pdev, qproc);

	init_completion(&qproc->start_done);
	init_completion(&qproc->stop_done);

	ret = of_property_read_u32(pdev->dev.of_node, "qcom,crash-reason",
				   &qproc->crash_reason);
	if (ret)
		dev_info(&pdev->dev, "no crash reason id\n");

	qproc->smd_edge_node = of_parse_phandle(pdev->dev.of_node,
						"qcom,smd-edges", 0);

	qproc->no_scm_clks = of_device_is_compatible(pdev->dev.of_node,
						     "qcom,apq8064-tz-pil");

	ret = qproc_init_pas(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_clocks(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_regulators(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_request_irq(qproc, pdev, "wdog", qproc_wdog_interrupt);
	qproc->wdog_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "fatal", qproc_fatal_interrupt);
	qproc->fatal_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "ready", qproc_ready_interrupt);
	qproc->ready_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "handover", qproc_handover_interrupt);
	qproc->handover_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "stop-ack", qproc_stop_ack_interrupt);
	qproc->stop_ack_irq = ret;

	for (i = 0; i < ARRAY_SIZE(qproc_attrs); i++) {
		ret = device_create_file(&pdev->dev, &qproc_attrs[i]);
		if (ret) {
			dev_err(&pdev->dev, "unable to create sysfs file\n");
			goto remove_device_files;
		}
	}

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No memory region specified\n");
	} else {

		ret = of_address_to_resource(np, 0, &r);
		of_node_put(np);
		if (ret)
			return ret;

		qproc->reloc_phys = r.start;
		qproc->reloc_size = resource_size(&r);

		dev_info(&pdev->dev, "Found relocation area %lu@%pad\n",
				qproc->reloc_size, &qproc->reloc_phys);
	}

	ret = rproc_add(rproc);
	if (ret)
		goto remove_device_files;

	return 0;

remove_device_files:
	for (i--; i >= 0; i--)
		device_remove_file(&pdev->dev, &qproc_attrs[i]);

free_rproc:
	rproc_put(rproc);

	return ret;
}

static int qproc_remove(struct platform_device *pdev)
{
	struct qproc *qproc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(qproc_attrs); i++)
		device_remove_file(&pdev->dev, &qproc_attrs[i]);

	rproc_put(qproc->rproc);

	return 0;
}

static const struct of_device_id qproc_of_match[] = {
	{ .compatible = "qcom,tz-pil", },
	{ },
};

static struct platform_driver qproc_driver = {
	.probe = qproc_probe,
	.remove = qproc_remove,
	.driver = {
		.name = "qcom-tz-pil",
		.of_match_table = qproc_of_match,
	},
};

module_platform_driver(qproc_driver);
