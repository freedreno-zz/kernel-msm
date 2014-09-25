/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/mfd/syscon.h>

#include <linux/delay.h>

#include <linux/soc/qcom/smem.h>

#define SMP2P_MAX_ENTRY 16
#define SMP2P_MAX_ENTRY_NAME 16

#define SMP2P_FEATURE_SSR_ACK 0x1

#define SMP2P_MAGIC 0x504d5324

struct smp2p_smem_item {
	u32 magic;
	u8 version;
	unsigned features:24;
	u16 local_pid;
	u16 remote_pid;
	u16 total_entries;
	u16 valid_entries;
	u32 flags;

	struct {
		u8 name[SMP2P_MAX_ENTRY_NAME];
		u32 value;
	} entries[SMP2P_MAX_ENTRY];
} __packed;

struct smp2p_entry {
	struct list_head node;
	struct qcom_smp2p *smp2p;

	const char *name;
	u32 *value;
	u32 last_value;

	struct irq_domain *domain;
	DECLARE_BITMAP(irq_enabled, 32);
	DECLARE_BITMAP(irq_rising, 32);
	DECLARE_BITMAP(irq_falling, 32);

	struct gpio_chip chip;
};

struct qcom_smp2p {
	struct device *dev;

	struct smp2p_smem_item *out;
	struct smp2p_smem_item *in;

	unsigned smem_items[2];

	unsigned valid_entries;

	unsigned local_pid;
	unsigned remote_pid;

	struct regmap *ipc_regmap;
	int ipc_offset;
	int ipc_bit;

	struct mutex lock;

	struct list_head inbound;
	struct list_head outbound;
};

static void qcom_smp2p_kick(struct qcom_smp2p *smp2p)
{
	wmb();
	regmap_write(smp2p->ipc_regmap, smp2p->ipc_offset, BIT(smp2p->ipc_bit));
}

static irqreturn_t qcom_smp2p_intr(int irq, void *data)
{
	struct smp2p_smem_item *in;
	struct smp2p_entry *entry;
	struct qcom_smp2p *smp2p = data;
	size_t size;
	int irq_pin;
	u32 status;
	u32 val;
	int ret;
	int i;

	mutex_lock(&smp2p->lock);

	if (!smp2p->in) {
		ret = qcom_smem_get(smp2p->smem_items[0], (void**)&in, &size);
		if (ret < 0 || size != ALIGN(sizeof(*in), 8))
			dev_err(smp2p->dev, "Unable to acquire remote smp2p item\n");
		else
			smp2p->in = in;
	}

	if (smp2p->in) {
		in = smp2p->in;

		for (i = smp2p->valid_entries; i < in->valid_entries; i++) {
			list_for_each_entry(entry, &smp2p->inbound, node) {
				if (!strcmp(in->entries[i].name, entry->name)) {
					entry->value = &in->entries[i].value;
					break;
				}
			}
		}
		smp2p->valid_entries = in->valid_entries;
	}

	list_for_each_entry(entry, &smp2p->inbound, node) {
		if (!entry->value)
			continue;

		val = *entry->value;

		status = val ^ entry->last_value;
		entry->last_value = val;

		for_each_set_bit(i, entry->irq_enabled, 32) {
			if (!(status & BIT(i)))
				continue;

			irq_pin = irq_find_mapping(entry->domain, i);

			if (val & BIT(i)) {
				if (test_bit(i, entry->irq_rising))
					handle_nested_irq(irq_pin);
			} else {
				if (test_bit(i, entry->irq_falling))
					handle_nested_irq(irq_pin);
			}

		}
	}

	mutex_unlock(&smp2p->lock);
	return IRQ_HANDLED;
}

static void smp2p_mask_irq(struct irq_data *irqd)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	clear_bit(irq, entry->irq_enabled);
}

static void smp2p_unmask_irq(struct irq_data *irqd)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	set_bit(irq, entry->irq_enabled);
}

static int smp2p_set_irq_type(struct irq_data *irqd, unsigned int type)
{
	struct smp2p_entry *entry = irq_data_get_irq_chip_data(irqd);
	irq_hw_number_t irq = irqd_to_hwirq(irqd);

	if (!(type & IRQ_TYPE_EDGE_BOTH))
		return -EINVAL;

	if (type & IRQ_TYPE_EDGE_RISING)
		set_bit(irq, entry->irq_rising);
	else
		clear_bit(irq, entry->irq_rising);

	if (type & IRQ_TYPE_EDGE_FALLING)
		set_bit(irq, entry->irq_falling);
	else
		clear_bit(irq, entry->irq_falling);

	return 0;
}

static struct irq_chip smp2p_irq_chip = {
	.name           = "smp2p",
	.irq_mask       = smp2p_mask_irq,
	.irq_unmask     = smp2p_unmask_irq,
	.irq_set_type	= smp2p_set_irq_type,
};

static int smp2p_irq_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	struct smp2p_entry *entry = d->host_data;

	irq_set_chip_and_handler(irq, &smp2p_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, entry);
	irq_set_nested_thread(irq, 1);

#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static const struct irq_domain_ops smp2p_irq_ops = {
	.map = smp2p_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int smp2p_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct smp2p_entry *entry = container_of(chip, struct smp2p_entry, chip);

	if (value)
		*entry->value |= BIT(offset);
	else
		*entry->value &= ~BIT(offset);

	qcom_smp2p_kick(entry->smp2p);

	return 0;
}

static void smp2p_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	smp2p_gpio_direction_output(chip, offset, value);
}

static int qcom_smp2p_outgoing_entry(struct qcom_smp2p *smp2p,
				     struct smp2p_entry *entry,
				     struct device_node *node)
{
	struct gpio_chip *chip;
	int ret;

	chip = &entry->chip;
	chip->base = -1;
	chip->ngpio = 32;
	chip->label = entry->name;
	chip->dev = smp2p->dev;
	chip->owner = THIS_MODULE;
	chip->of_node = node;

	chip->set = smp2p_gpio_set;
	chip->direction_output = smp2p_gpio_direction_output;

	ret = gpiochip_add(chip);
	if (ret)
		dev_err(smp2p->dev, "failed register gpiochip\n");

	return 0;
}

static int qcom_smp2p_alloc_outgoing(struct qcom_smp2p *smp2p)
{
	struct smp2p_smem_item *out;
	struct smp2p_entry *entry;
	int ret;

	ret = qcom_smem_alloc(smp2p->smem_items[1], sizeof(*out));
	if (ret < 0 && ret != -EEXIST) {
		if (ret != -EPROBE_DEFER)
			dev_err(smp2p->dev,
				"unable to allocate local smp2p item\n");
		return ret;
	}

	ret = qcom_smem_get(smp2p->smem_items[1], (void**)&out, NULL);
	if (ret < 0) {
		dev_err(smp2p->dev, "Unable to acquire local smp2p item\n");
		return ret;
	}

	smp2p->out = out;

	memset(out, 0, sizeof(*out));
	out->magic = SMP2P_MAGIC;
	out->local_pid = smp2p->local_pid;
	out->remote_pid = smp2p->remote_pid;
	out->total_entries = SMP2P_MAX_ENTRY;
	out->valid_entries = 0;

	wmb();
	out->version = 1;

	qcom_smp2p_kick(smp2p);

	list_for_each_entry(entry, &smp2p->outbound, node) {
		entry->value = &out->entries[out->valid_entries].value;
		strlcpy(out->entries[out->valid_entries].name, entry->name,
			SMP2P_MAX_ENTRY_NAME);

		out->valid_entries++;
	}

	qcom_smp2p_kick(smp2p);

	return 0;
}

static int smp2p_probe_ipc(struct qcom_smp2p *smp2p)
{
	struct device_node *syscon;
	struct device *dev = smp2p->dev;
	const char *key;
	int ret;

	syscon = of_parse_phandle(dev->of_node, "qcom,ipc", 0);
	if (!syscon) {
		dev_err(dev, "no qcom,ipc node\n");
		return -ENODEV;
	}

	smp2p->ipc_regmap = syscon_node_to_regmap(syscon);
	if (IS_ERR(smp2p->ipc_regmap))
		return PTR_ERR(smp2p->ipc_regmap);

	key = "qcom,ipc";
	ret = of_property_read_u32_index(dev->of_node, key, 1, &smp2p->ipc_offset);
	if (ret < 0) {
		dev_err(dev, "no offset in %s\n", key);
		return -EINVAL;
	}

	ret = of_property_read_u32_index(dev->of_node, key, 2, &smp2p->ipc_bit);
	if (ret < 0) {
		dev_err(dev, "no bit in %s\n", key);
		return -EINVAL;
	}

	return 0;
}

static int qcom_smp2p_probe(struct platform_device *pdev)
{
	struct smp2p_entry *entry;
	struct device_node *node;
	struct qcom_smp2p *smp2p;
	const char *key;
	int irq;
	int ret;

	smp2p = devm_kzalloc(&pdev->dev, sizeof(*smp2p), GFP_KERNEL);
	if (!smp2p)
		return -ENOMEM;

	smp2p->dev = &pdev->dev;
	mutex_init(&smp2p->lock);
	INIT_LIST_HEAD(&smp2p->inbound);
	INIT_LIST_HEAD(&smp2p->outbound);

	ret = smp2p_probe_ipc(smp2p);
	if (ret)
		return ret;

	key = "qcom,smem";
	ret = of_property_read_u32_array(pdev->dev.of_node, key,
					 smp2p->smem_items, 2);
	if (ret)
		return ret;

	key = "qcom,local-pid";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smp2p->local_pid);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read %s\n", key);
		return -EINVAL;
	}

	key = "qcom,remote-pid";
	ret = of_property_read_u32(pdev->dev.of_node, key, &smp2p->remote_pid);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read %s\n", key);
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to acquire smp2p interrupt\n");
		return irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq,
					NULL, qcom_smp2p_intr,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"smp2p", (void *)smp2p);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return ret;
	}

	for_each_available_child_of_node(pdev->dev.of_node, node) {
		entry = devm_kzalloc(&pdev->dev, sizeof(*entry), GFP_KERNEL);
		if (!entry) {
			ret = -ENOMEM;
			goto unwind_interfaces;
		}

		entry->smp2p = smp2p;

		ret = of_property_read_string(node, "qcom,entry-name", &entry->name);
		if (ret < 0)
			goto unwind_interfaces;

		if (of_property_read_bool(node, "qcom,outbound")) {
			ret = qcom_smp2p_outgoing_entry(smp2p, entry, node);
			if (ret < 0)
				goto unwind_interfaces;

			list_add(&entry->node, &smp2p->outbound);
		} else if (of_property_read_bool(node, "qcom,inbound")) {
			entry->domain = irq_domain_add_linear(node, 32, &smp2p_irq_ops, entry);
			if (!entry->domain) {
				ret = -ENOMEM;
				goto unwind_interfaces;
			}

			list_add(&entry->node, &smp2p->inbound);
		} else {
			dev_err(&pdev->dev, "neither inbound nor outbound\n");
			ret = -EINVAL;
			goto unwind_interfaces;
		}
	}

	ret = qcom_smp2p_alloc_outgoing(smp2p);
	if (ret < 0)
		goto unwind_interfaces;

	dev_info(smp2p->dev, "Qualcomm Shared Memory Point to Point initialized\n");

	return 0;

unwind_interfaces:
	list_for_each_entry(entry, &smp2p->inbound, node)
		irq_domain_remove(entry->domain);

	list_for_each_entry(entry, &smp2p->outbound, node)
		gpiochip_remove(&entry->chip);

	return ret;
}

static const struct of_device_id qcom_smp2p_of_match[] = {
	{ .compatible = "qcom,smp2p" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smp2p_of_match);

static struct platform_driver qcom_smp2p_driver = {
	.probe          = qcom_smp2p_probe,
	.driver  = {
		.name  = "qcom_smp2p",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smp2p_of_match,
	},
};

static int __init qcom_smp2p_init(void)
{
	return platform_driver_register(&qcom_smp2p_driver);
}
arch_initcall(qcom_smp2p_init);

static void __exit qcom_smp2p_exit(void)
{
	platform_driver_unregister(&qcom_smp2p_driver);
}
module_exit(qcom_smp2p_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Point to Point");
MODULE_LICENSE("GPLv2");
