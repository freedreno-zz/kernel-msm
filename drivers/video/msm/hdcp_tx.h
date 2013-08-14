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
#ifndef __HDCP_V1_TX_H__
#define __HDCP_V1_TX_H__

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/fb.h>

int hdcp_tx_init(void *input, void *output);
void hdcp_tx_deinit(void);

/*
 * APIs supported by hdmi_msm driver
 * @return  0--succeed. < 0--fail(maybe V1 downstream is not Auth'ed).
 * > 0-- length of message.
 */
struct hdmi_msm_apis {
	int (*request_topology_op)(void *output, int *sink);
};

/* APIs supported by hdcp_tx module for hdmi_msm callbacking. */
struct hdcp_tx_apis {
	int (*check_topology_op)(void);
};

enum MSG_ID {
	NONE = -1,
	CHECK_TOPOLOGY = 0,
	REQUEST_TOPOLOGY = 3,
};

enum RET_CODE {
	HDCP_NOT_AUTHED,
	HDCP_AUTHED,
	HDCP_DISABLE,
	HDMI_DISCONNECTED,
};

enum { HDMI_PLUGOUT = -1 };
/*
 * Message sent back to user space via sysfs sys/../hdcpmanager binaryentry.
 * Encoded as following:
 * msgID(u8) + retcode(u8) + params[up to 2k size].
 * Information all binary via bin sysfs entry.
 * params[] will be parsed according to msgID.
 */

struct HDCP_V2V1_RECEIVER_TOPOLOGY {
		u8	bksv[5];
};

struct HDCP_V2V1_REPEATER_TOPOLOGY {
		u8	bksv[5];
		u8	dev_count;
		u8	depth;
		u8	ksv_list[5 * 127];
		u32	max_cascade_exceeded;
		u32	max_dev_exceeded;
};

enum DS_TYPE {  /* type of downstream device */
	DS_UNKNOWN,
	DS_RECEIVER,
	DS_REPEATER,
};

/* parms[] data layout for msgID == CHECK_TOPOLOGY or REQUEST_TOPOLOGY */
struct HDCP_V2V1_MSG_TOPOLOGY {  /* how to parse sysfs params buffer */
		u32     ds_type;   /* indicates downstream's type */
		union topology {
				struct HDCP_V2V1_RECEIVER_TOPOLOGY  recv;
				struct HDCP_V2V1_REPEATER_TOPOLOGY reptor;
				} topology;
};

#endif

