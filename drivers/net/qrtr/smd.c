/*
 * Copyright (c) 2015, Sony Mobile Communications.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/printk.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/smd.h>

#include "qrtr.h"

struct qrtr_smd_dev {
	struct device *dev;
	struct qrtr_peer *peer;
	struct qcom_smd_channel *channel;
};

// #define dprint_hex_dump(...)
#define dprint_hex_dump print_hex_dump

static int qcom_smd_qrtr_callback(struct qcom_smd_device *sdev,
		const void *data, size_t len)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&sdev->dev);
	int rc;

	if (WARN_ON(qdev == NULL)) /* we're probably still probing */
		return -EAGAIN;

	dprint_hex_dump(KERN_ERR, "incoming: ", DUMP_PREFIX_OFFSET,
			16, 1, data, len, false);

	rc = qrtr_peer_post(qdev->peer, data, len);
	if (rc == -EINVAL) {
		dev_err(qdev->dev, "invalid ipcrouter packet: %d\n", rc);
		/* return 0 to let smd drop the packet */
		rc = 0;
	}

	return rc;
}

static int qcom_smd_qrtr_send(struct qrtr_peer *peer, struct sk_buff *skb)
{
	struct qrtr_smd_dev *qdev;
	int rc;

	qdev = qrtr_peer_get_privdata(peer);

	rc = skb_linearize(skb);
	if (rc) {
		kfree_skb(skb);
		return rc;
	}

	dprint_hex_dump(KERN_ERR, "outgoing: ", DUMP_PREFIX_OFFSET,
			16, 1, skb->data, skb->len, false);

	rc = qcom_smd_send(qdev->channel, skb->data, skb->len);
	kfree_skb(skb);

	return rc;
}

static int qcom_smd_qrtr_probe(struct qcom_smd_device *sdev)
{
	struct qrtr_smd_dev *qdev;

	qdev = devm_kzalloc(&sdev->dev, sizeof(*qdev), GFP_KERNEL);
	if (!qdev)
		return -ENOMEM;

	qdev->peer = qrtr_peer_create(qcom_smd_qrtr_send, QRTR_PEER_NID_AUTO);
	if (IS_ERR(qdev->peer))
		return PTR_ERR(qdev->peer);

	qdev->channel = sdev->channel;
	qdev->dev = &sdev->dev;
	qrtr_peer_set_privdata(qdev->peer, qdev);

	dev_set_drvdata(&sdev->dev, qdev);

	dev_info(&sdev->dev, "Qualcomm SMD QRTR driver probed\n");

	return 0;
}

static void qcom_smd_qrtr_remove(struct qcom_smd_device *sdev)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&sdev->dev);

	qrtr_peer_destroy(qdev->peer);

	dev_set_drvdata(&sdev->dev, NULL);
}

static const struct qcom_smd_id qcom_smd_qrtr_smd_match[] = {
	{ "IPCRTR" },
	{}
};

static struct qcom_smd_driver qcom_smd_qrtr_driver = {
	.probe = qcom_smd_qrtr_probe,
	.remove = qcom_smd_qrtr_remove,
	.callback = qcom_smd_qrtr_callback,
	.smd_match_table = qcom_smd_qrtr_smd_match,
	.driver = {
		.name = "qcom_smd_qrtr",
		.owner = THIS_MODULE,
	},
};

module_qcom_smd_driver(qcom_smd_qrtr_driver);

MODULE_DESCRIPTION("Qualcomm ipcrouter SMD interface driver");
MODULE_LICENSE("GPL v2");
