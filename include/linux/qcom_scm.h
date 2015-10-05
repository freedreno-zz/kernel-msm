/* Copyright (c) 2010-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
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
#ifndef __QCOM_SCM_H
#define __QCOM_SCM_H

#include <linux/types.h>

struct cpumask;
extern int qcom_scm_set_cold_boot_addr(void *entry, const struct cpumask *cpus);
extern int qcom_scm_set_warm_boot_addr(void *entry, const struct cpumask *cpus);

#define QCOM_SCM_HDCP_MAX_REQ_CNT	5

struct qcom_scm_hdcp_req {
	u32 addr;
	u32 val;
};

extern bool qcom_scm_is_available(void);

extern bool qcom_scm_hdcp_available(void);
extern int qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt,
		u32 *resp);

enum qcom_scm_sec_dev_id {
	QCOM_SCM_MDSS_DEV_ID	= 1,
	QCOM_SCM_OCMEM_DEV_ID	= 5,
	QCOM_SCM_PCIE0_DEV_ID	= 11,
	QCOM_SCM_PCIE1_DEV_ID	= 12,
	QCOM_SCM_GFX_DEV_ID	= 18,
	QCOM_SCM_UFS_DEV_ID	= 19,
	QCOM_SCM_ICE_DEV_ID	= 20,
};

extern bool qcom_scm_restore_sec_config_available(void);
extern int qcom_scm_restore_sec_config(enum qcom_scm_sec_dev_id sec_id);

enum qcom_scm_ocmem_client {
	QCOM_SCM_OCMEM_UNUSED_ID = 0x0,
	QCOM_SCM_OCMEM_GRAPHICS_ID,
	QCOM_SCM_OCMEM_VIDEO_ID,
	QCOM_SCM_OCMEM_LP_AUDIO_ID,
	QCOM_SCM_OCMEM_SENSORS_ID,
	QCOM_SCM_OCMEM_OTHER_OS_ID,
	QCOM_SCM_OCMEM_DEBUG_ID,
};

extern bool qcom_scm_ocmem_lock_available(void);
extern int qcom_scm_ocmem_lock(enum qcom_scm_ocmem_client id, u32 offset, u32 size, u32 mode);
extern int qcom_scm_ocmem_unlock(enum qcom_scm_ocmem_client id, u32 offset, u32 size);

extern bool qcom_scm_pas_supported(u32 peripheral);
extern int qcom_scm_pas_init_image(u32 peripheral, const void *metadata, size_t size);
extern int qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size);
extern int qcom_scm_pas_auth_and_reset(u32 peripheral);
extern int qcom_scm_pas_shutdown(u32 peripheral);

#define QCOM_SCM_CPU_PWR_DOWN_L2_ON	0x0
#define QCOM_SCM_CPU_PWR_DOWN_L2_OFF	0x1

extern void qcom_scm_cpu_power_down(u32 flags);

#define QCOM_SCM_VERSION(major, minor) (((major) << 16) | ((minor) & 0xFF))

extern u32 qcom_scm_get_version(void);

#endif
