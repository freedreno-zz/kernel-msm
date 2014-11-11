/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
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
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/soc/qcom/smd.h>

#define NVBIN_REQUEST_TIMEOUT (5 * HZ)

/* message types */
#define WCNSS_CTRL_MSG_START		0x01000000
#define WCNSS_VERSION_REQ		(WCNSS_CTRL_MSG_START + 0)
#define WCNSS_VERSION_RSP		(WCNSS_CTRL_MSG_START + 1)
#define WCNSS_NVBIN_DNLD_REQ		(WCNSS_CTRL_MSG_START + 2)
#define WCNSS_NVBIN_DNLD_RSP		(WCNSS_CTRL_MSG_START + 3)
#define WCNSS_CALDATA_UPLD_REQ		(WCNSS_CTRL_MSG_START + 4)
#define WCNSS_CALDATA_UPLD_RSP		(WCNSS_CTRL_MSG_START + 5)
#define WCNSS_CALDATA_DNLD_REQ		(WCNSS_CTRL_MSG_START + 6)
#define WCNSS_CALDATA_DNLD_RSP		(WCNSS_CTRL_MSG_START + 7)

struct smd_msg_hdr {
	u32 type;
	u32 len;
} __packed;

struct wcnss_version {
	struct smd_msg_hdr hdr;
	u8 major;
	u8 minor;
	u8 version;
	u8 revision;
} __packed;

struct wcnss_ctrl {
	struct device *dev;
	struct qcom_smd_channel *channel;

	struct completion ack;
	struct mutex lock;
	int ack_status;

	struct work_struct download_nvbin;

	void __iomem *mmio;
};

#define NVBIN_FILE "wlan/prima/WCNSS_qcom_wlan_nv.bin"

struct wcnss_nvbin_download_req {
	/* Note: The length specified in wcn36xx_hal_nv_img_download_req_msg
	 * messages should be
	 * header.len = sizeof(wcn36xx_hal_nv_img_download_req_msg) +
	 * nv_img_buffer_size */
	struct smd_msg_hdr header;

	/* Fragment sequence number of the NV Image. Note that NV Image
	 * might not fit into one message due to size limitation of the SMD
	 * channel FIFO. UMAC can hence choose to chop the NV blob into
	 * multiple fragments starting with seqeunce number 0, 1, 2 etc.
	 * The last fragment MUST be indicated by marking the
	 * isLastFragment field to 1. Note that all the NV blobs would be
	 * concatenated together by HAL without any padding bytes in
	 * between.*/
	u16 frag_number;

	/* Is this the last fragment? When set to 1 it indicates that no
	 * more fragments will be sent by UMAC and HAL can concatenate all
	 * the NV blobs rcvd & proceed with the parsing. HAL would generate
	 * a WCN36XX_HAL_DOWNLOAD_NV_RSP to the WCN36XX_HAL_DOWNLOAD_NV_REQ
	 * after it receives each fragment */
	u16 last_fragment;

	/* NV Image size (number of bytes) */
	u32 nv_img_buffer_size;

	/* Following the 'nv_img_buffer_size', there should be
	 * nv_img_buffer_size bytes of NV Image i.e.
	 * u8[nv_img_buffer_size] */
	u8 payload[];
} __packed;

struct wcnss_nvbin_download_res {
	struct smd_msg_hdr header;
	u8 status;
} __packed;

struct nv_data {
	u32 is_valid;
	u8 table[];
} __packed;

#define NV_FRAGMENT_SIZE  3072

static int wcnss_ctrl_callback(struct qcom_smd_device *qsdev,
				 void *data,
				 size_t count)
{
	struct wcnss_nvbin_download_res *nvbin;
	struct wcnss_version *version;
	struct wcnss_ctrl *wcnss = dev_get_drvdata(&qsdev->dev);
	struct smd_msg_hdr *hdr = data;

	// print_hex_dump(KERN_DEBUG, "WCNSS_CTRL <<<: ", DUMP_PREFIX_OFFSET, 16, 1, data, count, true);

	switch (hdr->type) {
	case WCNSS_VERSION_RSP:
		if (count != sizeof(struct wcnss_version)) {
			dev_err(wcnss->dev, "invalid size of version response\n");
			break;
		}
		version = data;

		dev_info(wcnss->dev, "WCNSS Version %d.%d %d.%d\n", version->major, version->minor, version->version, version->revision);

		schedule_work(&wcnss->download_nvbin);
		break;
	case WCNSS_NVBIN_DNLD_RSP:
		nvbin = data;
		wcnss->ack_status = nvbin->status;
		complete(&wcnss->ack);
		break;
	}

	return 0;
}

static int wcnss_ctrl_send_version_req(struct wcnss_ctrl *wcnss)
{
	struct smd_msg_hdr msg;

	msg.type = WCNSS_VERSION_REQ;
	msg.len = sizeof(msg);

	// print_hex_dump(KERN_DEBUG, "WCNSS_CTRL >>>: ", DUMP_PREFIX_OFFSET, 16, 1, &msg, sizeof(msg), true);
	return qcom_smd_send(wcnss->channel, &msg, sizeof(msg));
}

static void wcnss_download_nv(struct work_struct *work)
{
	struct wcnss_ctrl *wcnss = container_of(work, struct wcnss_ctrl, download_nvbin);
	struct wcnss_nvbin_download_req *req;
	const struct firmware *fw;
	struct nv_data *nv;
	int left;
	int ret;
	u16 offset = 0;

	req = kzalloc(sizeof(*req) + NV_FRAGMENT_SIZE, GFP_KERNEL);
	if (!req)
		return;

	ret = request_firmware(&fw, NVBIN_FILE, wcnss->dev);
	if (ret) {
		dev_err(wcnss->dev, "Failed to load nv file %s: %d\n", NVBIN_FILE, ret);
		kfree(req);
		return;
	}

	nv = (struct nv_data *)fw->data;

	req->header.type = WCNSS_NVBIN_DNLD_REQ;
	req->header.len = sizeof(*req) + NV_FRAGMENT_SIZE;

	req->last_fragment = 0;
	req->nv_img_buffer_size = NV_FRAGMENT_SIZE;

	req->frag_number = 0;
	do {
		left = fw->size - offset - 4;
		if (left <= NV_FRAGMENT_SIZE) {
			req->last_fragment = 1;
			req->nv_img_buffer_size = left;
			req->header.len = sizeof(*req) + left;
		}

		memcpy(req->payload, nv->table + offset, req->nv_img_buffer_size);

		// print_hex_dump(KERN_DEBUG, "WCNSS_CTRL >>>: ", DUMP_PREFIX_OFFSET, 16, 1, req, req->header.len, true);
		ret = qcom_smd_send(wcnss->channel, (void*)req, req->header.len);
		if (ret) {
			dev_err(wcnss->dev, "failed to send smd packet\n");
			break;
		}

		req->frag_number++;
		offset += NV_FRAGMENT_SIZE;
	} while (offset < fw->size - 4);

	release_firmware(fw);

	left = wait_for_completion_timeout(&wcnss->ack, NVBIN_REQUEST_TIMEOUT);
	if (!left) {
		dev_err(wcnss->dev, "timeout waiting for nv upload ack\n");
	} else if (wcnss->ack_status != 1) {
		dev_err(wcnss->dev, "nv upload response failed err: %d\n", wcnss->ack_status);
	}
}

#define PRONTO_PMU_OFFSET		0x1004
#define PRONTO_SPARE_OFFSET		0x1088
#define NVBIN_DLND_BIT			BIT(25)

#define WCNSS_PMU_CFG_IRIS_XO_CFG          BIT(3)
#define WCNSS_PMU_CFG_IRIS_XO_EN           BIT(4)
#define WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP   BIT(5)
#define WCNSS_PMU_CFG_IRIS_XO_CFG_STS      BIT(6) /* 1: in progress, 0: done */

#define WCNSS_PMU_CFG_IRIS_RESET           BIT(7)
#define WCNSS_PMU_CFG_IRIS_RESET_STS       BIT(8) /* 1: in progress, 0: done */
#define WCNSS_PMU_CFG_IRIS_XO_READ         BIT(9)
#define WCNSS_PMU_CFG_IRIS_XO_READ_STS     BIT(10)

#define WCNSS_PMU_CFG_IRIS_XO_MODE         0x6
#define WCNSS_PMU_CFG_IRIS_XO_MODE_48      (3 << 1)

static void wcnss_power_on(struct wcnss_ctrl *wcnss)
{
	u32 val;

	val = readl(wcnss->mmio + PRONTO_SPARE_OFFSET);
	val |= NVBIN_DLND_BIT;
	writel(val, wcnss->mmio + PRONTO_SPARE_OFFSET);

	val = readl(wcnss->mmio + PRONTO_PMU_OFFSET);
	val |= WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP;
	val |= WCNSS_PMU_CFG_IRIS_XO_EN;
	writel(val, wcnss->mmio + PRONTO_PMU_OFFSET);

	val &= ~WCNSS_PMU_CFG_IRIS_XO_MODE;
	val |= WCNSS_PMU_CFG_IRIS_XO_MODE_48;
	writel(val, wcnss->mmio + PRONTO_PMU_OFFSET);

	val |= WCNSS_PMU_CFG_IRIS_XO_CFG;
	writel(val, wcnss->mmio + PRONTO_PMU_OFFSET);

	while (readl(wcnss->mmio + PRONTO_PMU_OFFSET) & WCNSS_PMU_CFG_IRIS_XO_CFG_STS)
		cpu_relax();

	val &= ~WCNSS_PMU_CFG_GC_BUS_MUX_SEL_TOP;
	val &= ~WCNSS_PMU_CFG_IRIS_XO_CFG;
	writel(val, wcnss->mmio + PRONTO_PMU_OFFSET);

	msleep(20);
}

static int wcnss_ctrl_probe(struct qcom_smd_device *sdev)
{
	struct wcnss_ctrl *wcnss;
	u32 mmio[2];
	int ret;

	wcnss = devm_kzalloc(&sdev->dev, sizeof(*wcnss), GFP_KERNEL);
	if (!wcnss)
		return -ENOMEM;

	/* Map the memory */
	ret = of_property_read_u32_array(sdev->dev.of_node, "qcom,wcnss_mmio", mmio, 2);
	if (ret) {
		dev_err(&sdev->dev, "failed to get mmio\n");
		return -ENOENT;
	}

	wcnss->mmio = devm_ioremap(&sdev->dev, mmio[0], mmio[1]);
	if (!wcnss->mmio) {
		dev_err(&sdev->dev, "failed to map io memory\n");
		return -ENOMEM;
	}

	wcnss->dev = &sdev->dev;
	mutex_init(&wcnss->lock);

	init_completion(&wcnss->ack);
	INIT_WORK(&wcnss->download_nvbin, wcnss_download_nv);

	wcnss->channel = sdev->channel;

	dev_set_drvdata(&sdev->dev, wcnss);

	wcnss_power_on(wcnss);

	return wcnss_ctrl_send_version_req(wcnss);
}

static const struct of_device_id wcnss_ctrl_of_match[] = {
	{ .compatible = "qcom,wcnss-ctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, wcnss_ctrl_of_match);

static struct qcom_smd_driver wcnss_ctrl_driver = {
	.probe = wcnss_ctrl_probe,
	.callback = wcnss_ctrl_callback,
	.driver  = {
		.name  = "wcnss_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = wcnss_ctrl_of_match,
	},
};

module_qcom_smd_driver(wcnss_ctrl_driver);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm WCNSS control driver");
MODULE_LICENSE("GPLv2");

