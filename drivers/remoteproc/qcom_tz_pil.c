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
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>

#include "qcom_mdt_loader.h"
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

	struct qcom_smem_state *state;
	unsigned stop_bit;

	const char *name;
	struct regulator *pll;

	struct completion start_done;
	struct completion stop_done;

	unsigned crash_reason;
	struct device_node *smd_edge_node;
};

static int qproc_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qproc *qproc = rproc->priv;

	return qcom_mdt_load(rproc, qproc->pas_id, fw);
}

const struct rproc_fw_ops qproc_fw_ops = {
	.find_rsc_table = qcom_mdt_find_rsc_table,
	.sanity_check = qcom_mdt_sanity_check,
	.load = qproc_load,
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

	ret = qcom_scm_pas_auth_and_reset(qproc->pas_id);
	if (ret) {
		dev_err(qproc->dev,
				"failed to authenticate image and release reset\n");
		goto disable_regulator;
	}

	ret = wait_for_completion_timeout(&qproc->start_done, msecs_to_jiffies(10000));
	if (ret == 0) {
		dev_err(qproc->dev, "start timed out\n");

		qcom_scm_pas_shutdown(qproc->pas_id);
		goto disable_regulator;
	}

	return 0;

disable_regulator:
	regulator_disable(qproc->pll);

	return ret;
}

static int qproc_stop(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;
	int ret;

	qcom_smem_state_update_bits(qproc->state, BIT(qproc->stop_bit), BIT(qproc->stop_bit));

	ret = wait_for_completion_timeout(&qproc->stop_done, msecs_to_jiffies(1000));
	if (ret == 0)
		dev_err(qproc->dev, "timed out on wait\n");

	qcom_smem_state_update_bits(qproc->state, BIT(qproc->stop_bit), 0);

	ret = qcom_scm_pas_shutdown(qproc->pas_id);
	if (ret)
		dev_err(qproc->dev, "failed to shutdown: %d\n", ret);

	return ret;
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
	int ret;

	ret = qcom_smem_get(-1, qproc->crash_reason, (void**)&msg, &len);
	if (!ret && len > 0 && msg[0])
		dev_err(qproc->dev, "fatal error received: %s\n", msg);

	rproc_report_crash(qproc->rproc, RPROC_FATAL_ERROR);

	if (!ret)
		msg[0] = '\0';

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
	if (ret) {
		dev_err(&pdev->dev, "failed to read crash reason id\n");
		goto free_rproc;
	}

	qproc->smd_edge_node = of_parse_phandle(pdev->dev.of_node,
						"qcom,smd-edges", 0);

	ret = qproc_init_pas(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_regulators(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_request_irq(qproc, pdev, "wdog", qproc_wdog_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->wdog_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "fatal", qproc_fatal_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->fatal_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "ready", qproc_ready_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->ready_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "handover", qproc_handover_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->handover_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "stop-ack", qproc_stop_ack_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->stop_ack_irq = ret;

	qproc->state = qcom_smem_state_get(&pdev->dev, "stop", &qproc->stop_bit);
	if (IS_ERR(qproc->state))
		goto free_rproc;

	for (i = 0; i < ARRAY_SIZE(qproc_attrs); i++) {
		ret = device_create_file(&pdev->dev, &qproc_attrs[i]);
		if (ret) {
			dev_err(&pdev->dev, "unable to create sysfs file\n");
			goto remove_device_files;
		}
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

	qcom_smem_state_put(qproc->state);

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
