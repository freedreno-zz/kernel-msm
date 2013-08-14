/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/fb.h>
#include <linux/module.h>

#include "hdcp_tx.h"

#define SYSFS_BUF_SIZE 2048

enum SOURCE_ID {
	HDCP_V1_TX,
};

enum {
	MSG_ID_IDX,
	RET_CODE_IDX,
	HEADER_LEN,
};

struct hdcp_tx_state {
	atomic_t status;
	struct mutex lock;

	/* virtual sysfs device entry under graphics class for this channel */
	struct device *sysfs_dev;

	/* Reference to operations provided by HDMI Tx driver */
	struct hdmi_msm_apis *tx_ops;

	/*
	 * to store the MSG_ID of each request issued from the client,
	 * will be cleared once completing the read. Each transaction comprises
	 * of sysfs_write(..., MSG_ID) and following sysfs_read(..., *buf).
	 */
	u32 cur_msg;
};

static int hdcp_tx_topology_update(void);
static struct hdcp_tx_state *hdcp_state;

static ssize_t hdcp_tx_read_user_bin(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	int ret = 0;
	int sink = 0;

	if (!hdcp_state)
		return -EFAULT;

	count = min_t(loff_t, count, SYSFS_BUF_SIZE - off);
	switch (hdcp_state->cur_msg) {
	case CHECK_TOPOLOGY:
	case REQUEST_TOPOLOGY:
		buf[MSG_ID_IDX] = hdcp_state->cur_msg;

		ret = hdcp_state->tx_ops->
			request_topology_op((void *)(buf+HEADER_LEN), &sink);

		/* user task interrupted by signal */
		if (ret == (-ERESTARTSYS))
			break;

		if (ret < 0) {
			buf[RET_CODE_IDX] = HDCP_NOT_AUTHED;
			ret = HEADER_LEN;
		} else {
			if (sink == HDMI_PLUGOUT) {
				buf[RET_CODE_IDX] = HDMI_DISCONNECTED;
				ret = HEADER_LEN;
			} else {
				buf[RET_CODE_IDX] = HDCP_AUTHED;
				ret += HEADER_LEN;
				pr_debug("%s readir succeed! length = %d buf[2] = %d\n",
					__func__, ret, buf[2]);
			}
		}

		/* clear the flag once data is read back to user space*/
		hdcp_state->cur_msg = NONE;
		break;
	default:
		ret = -EINVAL;
	}

	pr_debug("%s reading bin sysfs dir, offset = %d, returned count = %d",
			__func__, (int)off, ret);
	return ret;
}

/*
 * for reading operation, one sysfs_write is needed to indicate what params
 * to be retrieved by ensuing sysfs_read().
 */
static ssize_t hdcp_tx_write_user_bin(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	int ret;
	int msgid = NONE;

	count = min_t(loff_t, count, SYSFS_BUF_SIZE - off);
	/*
	 * const char *buf--> msgID(u8) + params[] sent from HDCP manager
	 * in user space. Parse params[] based on msgID.
	 */
	msgid = buf[0];
	switch (msgid) {
	case CHECK_TOPOLOGY:
	case REQUEST_TOPOLOGY:
		hdcp_state->cur_msg = msgid;
		ret = count;
		break;
	/* more cases added here */
	default:
		ret = -EINVAL;
	}

	pr_debug("%s sysfs dir recv'd msgID =  %d, count of writing byte = %d\n",
			__func__, msgid, ret);

	return ret;
}

static struct bin_attribute hdcp_tx_bin_attr = {
	.attr = {
		.name = "hdmitx",
		.mode = S_IRUGO | S_IWUSR | S_IWGRP,
	},
	.size = SYSFS_BUF_SIZE, /*sizeof(HDCP_V2V1_MSG_TOPOLOGY) + 2,*/
	.read = hdcp_tx_read_user_bin,
	.write = hdcp_tx_write_user_bin,
};

static int hdcp_tx_create_sysfs_attrs(void)
{
	int ret = 0;

	/*
	 * Create non-parent sysfs directory device for hdcpmanager
	 * under graphics(fbmem) class.
	 */
	hdcp_state->sysfs_dev = device_create(fb_class, NULL, MKDEV(0, 0), NULL,
						"hdcpmanager");
	if (IS_ERR(hdcp_state->sysfs_dev)) {
		pr_err("%s failed to create sysfs dir hdcpmanager", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_bin_file(&hdcp_state->sysfs_dev->kobj,
						&hdcp_tx_bin_attr);
	if (ret) {
		device_destroy(fb_class, MKDEV(0, 0));
		pr_err("%s failed to create bin sysfs entry", __func__);
		ret = -EFAULT;
	}

	return ret;
}

static void hdcp_tx_destroy_sysfs_attrs(void)
{
	sysfs_remove_bin_file(&hdcp_state->sysfs_dev->kobj,
						&hdcp_tx_bin_attr);
	device_destroy(fb_class, MKDEV(0, 0));
}

/* Invoked from kthread when hdmi_msm completes HDCP authentication */
static int hdcp_tx_topology_update(void)
{
	int ret = 0;
	char a[16], b[16];
	char *envp[] = {
		[0] = "HDCP_MGR_EVENT=MSG_READY",
		[1] = a,
		[2] = b,
		NULL,
		/* more can be added as necessary. */
	};

	snprintf(envp[1], 16, "%d", (int)CHECK_TOPOLOGY);
	snprintf(envp[2], 16, "%d", (int)HDCP_V1_TX);
	kobject_uevent_env(&hdcp_state->sysfs_dev->kobj, KOBJ_CHANGE, envp);

	pr_debug("%s uevent is sending, %s msgID = %s srcID = %s\n", __func__,
			envp[0], envp[1], envp[2]);
	return ret;
}

static struct hdcp_tx_apis hdcp_ops = {
	.check_topology_op = &hdcp_tx_topology_update,
	/* extended here for new callbacks */
};

int hdcp_tx_init(void *input, void *output)
{
	int ret = 0;

	if (!hdcp_state)
		hdcp_state = kzalloc(sizeof(*hdcp_state), GFP_KERNEL);

	if (!hdcp_state)
		return -ENOMEM;

	/* Initialize struct hdcp_tx_state */
	atomic_set(&hdcp_state->status, 0);
	mutex_init(&hdcp_state->lock);
	hdcp_state->cur_msg = NONE;
	hdcp_state->tx_ops = (struct hdmi_msm_apis *)input;

	/* pass by value */
	*(struct hdcp_tx_apis *)output = hdcp_ops;

	/*1. create a non-parent binary sysfs dir kobject  */
	/*2. create attribute group and attached to created sysfs dir */
	if (hdcp_tx_create_sysfs_attrs()) {
		ret = -EFAULT;
		kfree(hdcp_state);
		hdcp_state = NULL;
	}

	return ret;
}
EXPORT_SYMBOL(hdcp_tx_init);

void hdcp_tx_deinit(void)
{
	hdcp_tx_destroy_sysfs_attrs();
	if (hdcp_state) {
		kfree(hdcp_state);
		hdcp_state = NULL;
	}
}
EXPORT_SYMBOL(hdcp_tx_deinit);

MODULE_VERSION("0.3");
MODULE_DESCRIPTION("HDCP MSM TX driver");
