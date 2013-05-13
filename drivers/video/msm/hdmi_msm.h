/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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

#ifndef __HDMI_MSM_H__
#define __HDMI_MSM_H__


#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_CEC_SUPPORT
#include <linux/input.h>
#endif
#include <mach/msm_iomap.h>
#include "external_common.h"
/* #define PORT_DEBUG */

#ifndef DRVR_ONLY_CECT_NO_DAEMON
#define DRVR_ONLY_CECT_NO_DAEMON
#endif

#ifdef PORT_DEBUG
const char *hdmi_msm_name(uint32 offset);
void hdmi_outp(uint32 offset, uint32 value);
uint32 hdmi_inp(uint32 offset);

#define HDMI_OUTP_ND(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	hdmi_outp((offset), (value))
#define HDMI_INP_ND(offset)		inpdw(MSM_HDMI_BASE+(offset))
#define HDMI_INP(offset)		hdmi_inp((offset))
#else
#define HDMI_OUTP_ND(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	outpdw(MSM_HDMI_BASE+(offset), (value))
#define HDMI_INP_ND(offset)		inpdw(MSM_HDMI_BASE+(offset))
#define HDMI_INP(offset)		inpdw(MSM_HDMI_BASE+(offset))
#endif


#define HDMI_CEC_LOGICAL_ADDR_BROADCAST	15
#define HDMI_CEC_DEFAULT_LOGICAL_ADDR	HDMI_CEC_LOGICAL_ADDR_BROADCAST

/*
 * Ref. HDMI 1.4a
 * Supplement-1 CEC Section 6, 7
 */
struct hdmi_msm_cec_msg {
	uint8 sender_id;
	uint8 recvr_id;
	uint8 opcode;
	uint8 operand[15];
	uint8 frame_size;
	uint8 retransmit;
};

struct hdmi_msm_audio_config {
	uint8 sample_rate;
	uint8 channel_num;
	uint8 spkr_alloc;
	uint8 level_shift;
	uint8 down_mix;
};

#define QFPROM_BASE		((uint32)hdmi_msm_state->qfprom_io)
#define HDMI_BASE		((uint32)hdmi_msm_state->hdmi_io)
#define FLAG_SYSFS_CEC_WAKEUP_EN	0x2

struct hdmi_msm_state_type {
	boolean panel_power_on;
	boolean hpd_initialized;
#ifdef CONFIG_SUSPEND
	boolean pm_suspended;
#endif
	boolean full_auth_done;
	boolean hpd_during_auth;
	struct work_struct hpd_state_work;
	struct completion ddc_sw_done;

	bool hdcp_enable;
	boolean hdcp_activating;
	boolean reauth ;
	struct work_struct hdcp_reauth_work, hdcp_work;
	struct completion hdcp_success_done;
	struct timer_list hdcp_timer;

	struct hdmi_msm_audio_config hdmi_audio;
	u32 cec_wakeup_enabled;
	boolean irq_wakeup_enabled;
	boolean cec_initialized;

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_CEC_SUPPORT
	boolean cec_enabled;
	unsigned int first_monitor;
	int cec_logical_addr;
	struct completion cec_frame_wr_done;
	struct timer_list cec_read_timer;
#define CEC_STATUS_WR_ERROR	0x0001
#define CEC_STATUS_WR_DONE	0x0002
#define CEC_STATUS_WR_TMOUT	0x0004
	uint32 cec_frame_wr_status;

	struct hdmi_msm_cec_msg *cec_queue_start;
	struct hdmi_msm_cec_msg *cec_queue_wr;
	struct hdmi_msm_cec_msg *cec_queue_rd;
	boolean cec_queue_full;
	boolean fsm_reset_done;

	/*
	 * CECT 9-5-1
	 */
	struct completion cec_line_latch_wait;
	struct work_struct cec_latch_detect_work;

	struct input_dev *input;
	u8 last_key;
#define CEC_QUEUE_SIZE		16
#define CEC_QUEUE_END	 (hdmi_msm_state->cec_queue_start + CEC_QUEUE_SIZE)
#define RETRANSMIT_MAX_NUM	5
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_CEC_SUPPORT */

	int irq;
	struct msm_hdmi_platform_data *pd;
	struct clk *hdmi_app_clk;
	struct clk *hdmi_m_pclk;
	struct clk *hdmi_s_pclk;
	void __iomem *qfprom_io;
	void __iomem *hdmi_io;

	struct external_common_state_type common;
	boolean is_mhl_enabled;
	struct completion hpd_event_processed;
};

extern struct hdmi_msm_state_type *hdmi_msm_state;

uint32 hdmi_msm_get_io_base(void);

#ifdef CONFIG_FB_MSM_HDMI_COMMON
void hdmi_msm_set_mode(boolean power_on);
int hdmi_msm_clk(int on);
void hdmi_phy_reset(void);
void hdmi_msm_reset_core(void);
void hdmi_msm_init_phy(int video_format);
void hdmi_msm_powerdown_phy(void);
void hdmi_frame_ctrl_cfg(const struct hdmi_disp_mode_timing_type *timing);
void hdmi_msm_phy_status_poll(void);
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_CEC_SUPPORT
void hdmi_msm_cec_init(void);
void hdmi_msm_cec_write_logical_addr(int addr);
int hdmi_msm_cec_read_logical_addr(void);
void hdmi_msm_cec_msg_recv(void);
void hdmi_msm_cec_one_touch_play(void);
void hdmi_msm_cec_msg_send(struct hdmi_msm_cec_msg *msg);
void hdmi_msm_cec_init_input_event(void);
void hdmi_msm_cec_deinit_input_event(void);
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_CEC_SUPPORT */
void mhl_connect_api(boolean on);
u32 hdmi_msm_is_cec_wakeup_enabled(void);
#endif /* __HDMI_MSM_H__ */
