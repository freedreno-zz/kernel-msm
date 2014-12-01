/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
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

#include "hdmi.h"
#include <soc/qcom/scm.h>

#define TZ_HDCP_CMD_ID 0x00004401
#define HDCP_REG_ENABLE 0x01
#define HDCP_REG_DISABLE 0x00
#define HDCP_PORT_ADDR 0x74

#define HDCP_INT_CLR (BIT(1) | BIT(5) | BIT(7) | BIT(9) | BIT(13))
#define HDCP_INT_STATUS (BIT(0) | BIT(4) | BIT(8) | BIT(12))

#define AUTH_WORK_RETRIES_TIME (100)
#define AUTH_RETRIES_TIME (30)

/* QFPROM Registers for HDMI/HDCP */
#define QFPROM_RAW_FEAT_CONFIG_ROW0_LSB  (0x000000F8)
#define QFPROM_RAW_FEAT_CONFIG_ROW0_MSB  (0x000000FC)
#define HDCP_KSV_LSB                     (0x000060D8)
#define HDCP_KSV_MSB                     (0x000060DC)

enum DS_TYPE {  /* type of downstream device */
	DS_UNKNOWN,
	DS_RECEIVER,
	DS_REPEATER,
};

enum hdmi_hdcp_state {
	HDCP_STATE_INACTIVE,
	HDCP_STATE_AUTHENTICATING,
	HDCP_STATE_AUTHENTICATED,
	HDCP_STATE_AUTH_FAIL
};

struct hdmi_hdcp_reg_data {
	uint32_t reg_id;
	uint32_t off;
	char *name;
	uint32_t reg_val;
};

struct hdmi_hdcp_ctrl {
	struct hdmi *hdmi;
	uint32_t auth_retries;
	uint32_t tz_hdcp;
	enum hdmi_hdcp_state hdcp_state;
	struct mutex state_mutex;
	struct delayed_work hdcp_reauth_work;
	struct delayed_work hdcp_auth_part1_1_work;
	struct delayed_work hdcp_auth_part1_2_work;
	struct work_struct hdcp_auth_part1_3_work;
	struct delayed_work hdcp_auth_part2_1_work;
	struct delayed_work hdcp_auth_part2_2_work;
	struct delayed_work hdcp_auth_part2_3_work;
	struct delayed_work hdcp_auth_part2_4_work;
	struct work_struct hdcp_auth_prepare_work;
	uint32_t work_retry_cnt;
	uint32_t ksv_fifo_w_index;

	/*
	 * store aksv from qfprom
	 */
	uint8_t aksv[5];
	bool aksv_valid;
	uint32_t ds_type;
	uint8_t bksv[5];
	uint8_t dev_count;
	uint8_t depth;
	uint8_t ksv_list[5 * 127];
	bool max_cascade_exceeded;
	bool max_dev_exceeded;
};

static int hdmi_ddc_read(struct hdmi *hdmi, uint16_t addr, uint8_t offset,
	uint8_t *data, uint16_t data_len, bool no_align)
{
	int rc = 0;
	int retry = 5;
	uint8_t *buf = NULL;
	uint32_t request_len;
	struct i2c_msg msgs[] = {
		{
			.addr	= addr >> 1,
			.flags	= 0,
			.len	= 1,
			.buf	= &offset,
		}, {
			.addr	= addr >> 1,
			.flags	= I2C_M_RD,
			.len	= data_len,
			.buf	= data,
		}
	};

	DBG("Start DDC read");
retry:
	if (no_align) {
		rc = i2c_transfer(hdmi->i2c, msgs, 2);
	} else {
		request_len = 32 * ((data_len + 31) / 32);
		buf = kmalloc(request_len, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		msgs[1].buf = buf;
		rc = i2c_transfer(hdmi->i2c, msgs, 2);
	}

	retry--;
	if (rc == 2) {
		rc = 0;
		if (!no_align)
			memcpy(data, buf, data_len);
	} else if (retry > 0) {
		goto retry;
	} else {
		rc = -EIO;
	}

	kfree(buf);
	DBG("End DDC read %d", rc);

	return rc;
}

static int hdmi_ddc_write(struct hdmi *hdmi, uint16_t addr, uint8_t offset,
	uint8_t *data, uint16_t data_len)
{
	int rc = 0;
	int retry = 10;
	uint8_t *buf = NULL;
	struct i2c_msg msgs[] = {
		{
			.addr	= addr >> 1,
			.flags	= 0,
			.len	= 1,
		}
	};

	DBG("Start DDC write");
	buf = kmalloc(data_len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = offset;
	memcpy(&buf[1], data, data_len);
	msgs[0].buf = buf;
	msgs[0].len = data_len + 1;
retry:
	rc = i2c_transfer(hdmi->i2c, msgs, 1);

	retry--;
	if (rc == 1)
		rc = 0;
	else if (retry > 0)
		goto retry;
	else
		rc = -EIO;

	kfree(buf);
	DBG("End DDC write %d", rc);

	return rc;
}

static int hdmi_hdcp_scm_wr(struct hdmi_hdcp_ctrl *hdcp_ctrl, uint32_t *preg,
	uint32_t *pdata, uint32_t count)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	struct scm_hdcp_req scm_buf[SCM_HDCP_MAX_REG];
	uint32_t resp, phy_addr, idx = 0;
	int i, ret = 0;

	if (count == 0)
		return 0;

	if (!preg || !pdata) {
		pr_err("%s: Invalid pointer\n", __func__);
		return -EINVAL;
	}

	if (hdcp_ctrl->tz_hdcp) {
		phy_addr = (uint32_t)hdmi->mmio_phy_addr;

		while (count) {
			memset(scm_buf, 0, sizeof(scm_buf));
			for (i = 0; i < count && i < SCM_HDCP_MAX_REG; i++) {
				scm_buf[i].addr = phy_addr + preg[idx];
				scm_buf[i].val  = pdata[idx];
				idx++;
			}
			ret = scm_call(SCM_SVC_HDCP, SCM_CMD_HDCP,
				scm_buf, sizeof(scm_buf), &resp, sizeof(resp));

			if (ret || resp) {
				pr_err("%s: error: scm_call ret = %d, resp = %d\n",
					__func__, ret, resp);
				ret = -EINVAL;
				break;
			}

			count -= i;
		}
	} else {
		for (i = 0; i < count; i++)
			hdmi_write(hdmi, preg[i], pdata[i]);
	}

	return ret;
}

void hdmi_hdcp_irq(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi;
	uint32_t regval, hdcp_int_status;
	unsigned long flags;

	if (!hdcp_ctrl) {
		DBG("HDCP is disabled");
		return;
	}

	hdmi = hdcp_ctrl->hdmi;
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	regval = hdmi_read(hdmi, REG_HDMI_HDCP_INT_CTRL);
	hdcp_int_status = regval & HDCP_INT_STATUS;
	if (!hdcp_int_status) {
		spin_unlock_irqrestore(&hdmi->reg_lock, flags);
		return;
	}
	/* Clear Interrupts */
	regval |= hdcp_int_status << 1;
	/* Clear AUTH_FAIL_INFO as well */
	if (hdcp_int_status & BIT(4))
		regval |= BIT(7);
	hdmi_write(hdmi, REG_HDMI_HDCP_INT_CTRL, regval);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	DBG("hdcp irq %x", regval);
	if (hdcp_int_status & BIT(0)) {
		/* AUTH_SUCCESS_INT */
		pr_info("%s:AUTH_SUCCESS_INT received\n", __func__);
		if (HDCP_STATE_AUTHENTICATING == hdcp_ctrl->hdcp_state)
			queue_work(hdmi->workq,
				&hdcp_ctrl->hdcp_auth_part1_3_work);
	}

	if (hdcp_int_status & BIT(4)) {
		/* AUTH_FAIL_INT */
		regval = hdmi_read(hdmi, REG_HDMI_HDCP_LINK0_STATUS);
		pr_info("%s: AUTH_FAIL_INT rcvd, LINK0_STATUS=0x%08x\n",
			__func__, regval);
		if (HDCP_STATE_AUTHENTICATED == hdcp_ctrl->hdcp_state)
			queue_delayed_work(hdmi->workq,
				&hdcp_ctrl->hdcp_reauth_work, HZ/2);
		else if (HDCP_STATE_AUTHENTICATING == hdcp_ctrl->hdcp_state)
			queue_work(hdmi->workq,
				&hdcp_ctrl->hdcp_auth_part1_3_work);
	}

	if (hdcp_int_status & BIT(8)) {
		/* DDC_XFER_REQ_INT */
		pr_info("%s:DDC_XFER_REQ_INT received\n", __func__);
	}

	if (hdcp_int_status & BIT(12)) {
		/* DDC_XFER_DONE_INT */
		pr_info("%s:DDC_XFER_DONE received\n", __func__);
	}
} /* hdmi_hdcp_isr */

static int hdmi_hdcp_count_one(uint8_t *array, uint8_t len)
{
	int i, j, count = 0;

	for (i = 0; i < len; i++)
		for (j = 0; j < 8; j++)
			count += (((array[i] >> j) & 0x1) ? 1 : 0);

	return count;
}

static int hdmi_hdcp_read_validate_aksv(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t qfprom_aksv_lsb, qfprom_aksv_msb;

	/* Fetch aksv from QFPROM, this info should be public. */
	qfprom_aksv_lsb = hdmi_qfprom_read(hdmi, HDCP_KSV_LSB);
	qfprom_aksv_msb = hdmi_qfprom_read(hdmi, HDCP_KSV_MSB);

	hdcp_ctrl->aksv[0] =  qfprom_aksv_lsb        & 0xFF;
	hdcp_ctrl->aksv[1] = (qfprom_aksv_lsb >> 8)  & 0xFF;
	hdcp_ctrl->aksv[2] = (qfprom_aksv_lsb >> 16) & 0xFF;
	hdcp_ctrl->aksv[3] = (qfprom_aksv_lsb >> 24) & 0xFF;
	hdcp_ctrl->aksv[4] =  qfprom_aksv_msb        & 0xFF;

	/* check there are 20 ones in AKSV */
	if (hdmi_hdcp_count_one(hdcp_ctrl->aksv, 5) != 20) {
		pr_err("%s: AKSV QFPROM doesn't have 20 1's, 20 0's\n",
			__func__);
		pr_err("%s: QFPROM AKSV chk failed (AKSV=%02x%08x)\n",
			__func__, qfprom_aksv_msb,
			qfprom_aksv_lsb);
		return -EINVAL;
	}
	DBG("AKSV=%02x%08x", qfprom_aksv_msb, qfprom_aksv_lsb);

	return 0;
}

static void reset_hdcp_ddc_failures(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int hdcp_ddc_ctrl1_reg;
	int hdcp_ddc_status;
	int failure;
	int nack0;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;

	/* Check for any DDC transfer failures */
	hdcp_ddc_status = hdmi_read(hdmi, REG_HDMI_HDCP_DDC_STATUS);
	failure = (hdcp_ddc_status >> 16) & 0x1;
	nack0 = (hdcp_ddc_status >> 14) & 0x1;
	DBG("On Entry: HDCP_DDC_STATUS=0x%x, FAIL=%d, NACK0=%d",
		hdcp_ddc_status, failure, nack0);

	if (failure == 0x1) {
		/*
		 * Indicates that the last HDCP HW DDC transfer failed.
		 * This occurs when a transfer is attempted with HDCP DDC
		 * disabled (HDCP_DDC_DISABLE=1) or the number of retries
		 * matches HDCP_DDC_RETRY_CNT.
		 * Failure occurred,  let's clear it.
		 */
		DBG("DDC failure detected.HDCP_DDC_STATUS=0x%08x",
			hdcp_ddc_status);

		/* First, Disable DDC */
		hdmi_write(hdmi, REG_HDMI_HDCP_DDC_CTRL_0, BIT(0));

		/* ACK the Failure to Clear it */
		hdcp_ddc_ctrl1_reg = hdmi_read(hdmi, REG_HDMI_HDCP_DDC_CTRL_1);
		hdmi_write(hdmi, REG_HDMI_HDCP_DDC_CTRL_1,
			hdcp_ddc_ctrl1_reg | BIT(0));

		/* Check if the FAILURE got Cleared */
		hdcp_ddc_status = hdmi_read(hdmi, REG_HDMI_HDCP_DDC_STATUS);
		hdcp_ddc_status = (hdcp_ddc_status >> 16) & BIT(0);
		if (hdcp_ddc_status == 0x0)
			DBG("HDCP DDC Failure cleared");
		else
			pr_info("%s: Unable to clear HDCP DDC Failure\n",
				__func__);

		/* Re-Enable HDCP DDC */
		hdmi_write(hdmi, REG_HDMI_HDCP_DDC_CTRL_0, 0);
	}

	if (nack0 == 0x1) {
		DBG("Before: HDMI_DDC_SW_STATUS=0x%08x",
			hdmi_read(hdmi, REG_HDMI_DDC_SW_STATUS));
		/* Reset HDMI DDC software status */
		hdmi_write(hdmi, REG_HDMI_DDC_CTRL,
			hdmi_read(hdmi, REG_HDMI_DDC_CTRL) | BIT(3));
		msleep(20);
		hdmi_write(hdmi, REG_HDMI_DDC_CTRL,
			hdmi_read(hdmi, REG_HDMI_DDC_CTRL) & ~(BIT(3)));

		/* Reset HDMI DDC Controller */
		hdmi_write(hdmi, REG_HDMI_DDC_CTRL,
			hdmi_read(hdmi, REG_HDMI_DDC_CTRL) | BIT(1));
		msleep(20);
		hdmi_write(hdmi, REG_HDMI_DDC_CTRL,
			hdmi_read(hdmi, REG_HDMI_DDC_CTRL) & ~BIT(1));
		DBG("After: HDMI_DDC_SW_STATUS=0x%08x",
			hdmi_read(hdmi, REG_HDMI_DDC_SW_STATUS));
	}

	hdcp_ddc_status = hdmi_read(hdmi, REG_HDMI_HDCP_DDC_STATUS);

	failure = (hdcp_ddc_status >> 16) & BIT(0);
	nack0 = (hdcp_ddc_status >> 14) & BIT(0);
	DBG("On Exit: HDCP_DDC_STATUS=0x%x, FAIL=%d, NACK0=%d",
		hdcp_ddc_status, failure, nack0);
}

static void hdmi_hdcp_hw_ddc_clean(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	uint32_t hdcp_ddc_status, ddc_hw_status;
	uint32_t ddc_xfer_done, ddc_xfer_req, ddc_hw_done;
	uint32_t ddc_hw_not_ready;
	uint32_t timeout_count;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;

	if (hdmi_read(hdmi, REG_HDMI_DDC_HW_STATUS) == 0)
		return;

	/* Wait to be clean on DDC HW engine */
	timeout_count = 100;
	do {
		hdcp_ddc_status = hdmi_read(hdmi, REG_HDMI_HDCP_DDC_STATUS);
		ddc_hw_status = hdmi_read(hdmi, REG_HDMI_DDC_HW_STATUS);
		ddc_xfer_done = hdcp_ddc_status & BIT(10);
		ddc_xfer_req = hdcp_ddc_status & BIT(4);
		ddc_hw_done = ddc_hw_status & BIT(3);
		ddc_hw_not_ready = !ddc_xfer_done ||
			ddc_xfer_req || !ddc_hw_done;

		DBG("timeout count(%d):ddc hw%sready",
			timeout_count, ddc_hw_not_ready ? " not " : " ");
		DBG("hdcp_ddc_status[0x%x], ddc_hw_status[0x%x]",
			hdcp_ddc_status, ddc_hw_status);
		if (ddc_hw_not_ready)
			msleep(20);
	} while (ddc_hw_not_ready && --timeout_count);
}

/*
 * Only retries defined times then abort current authenticating process
 */
static int hdmi_msm_if_abort_reauth(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int rc = 0;

	if (++hdcp_ctrl->auth_retries == AUTH_RETRIES_TIME) {
		mutex_lock(&hdcp_ctrl->state_mutex);
		hdcp_ctrl->hdcp_state = HDCP_STATE_INACTIVE;
		mutex_unlock(&hdcp_ctrl->state_mutex);

		hdcp_ctrl->auth_retries = 0;
		rc = -ERANGE;
	}

	return rc;
}

static void hdmi_hdcp_reauth_work(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_reauth_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	unsigned long flags;
	int rc;

	DBG("HDCP REAUTH WORK");
	mutex_lock(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->hdcp_state = HDCP_STATE_AUTH_FAIL;
	mutex_unlock(&hdcp_ctrl->state_mutex);

	/*
	 * Disable HPD circuitry.
	 * This is needed to reset the HDCP cipher engine so that when we
	 * attempt a re-authentication, HW would clear the AN0_READY and
	 * AN1_READY bits in HDMI_HDCP_LINK0_STATUS register
	 */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	hdmi_write(hdmi, REG_HDMI_HPD_CTRL,
		hdmi_read(hdmi, REG_HDMI_HPD_CTRL) & ~BIT(28));

	/* Disable HDCP interrupts */
	hdmi_write(hdmi, REG_HDMI_HDCP_INT_CTRL, 0);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	hdmi_write(hdmi, REG_HDMI_HDCP_RESET, BIT(0));

	/* Wait to be clean on DDC HW engine */
	hdmi_hdcp_hw_ddc_clean(hdcp_ctrl);

	/* Disable encryption and disable the HDCP block */
	hdmi_write(hdmi, REG_HDMI_HDCP_CTRL, 0);

	/* Enable HPD circuitry */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	hdmi_write(hdmi, REG_HDMI_HPD_CTRL,
		hdmi_read(hdmi, REG_HDMI_HPD_CTRL) | BIT(28));
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	rc = hdmi_msm_if_abort_reauth(hdcp_ctrl);
	if (rc) {
		pr_err("%s: abort reauthentication!\n", __func__);
		return;
	}

	mutex_lock(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->hdcp_state = HDCP_STATE_AUTHENTICATING;
	mutex_unlock(&hdcp_ctrl->state_mutex);
	queue_work(hdmi->workq, &hdcp_ctrl->hdcp_auth_prepare_work);
}

static void hdmi_hdcp_auth_prepare_work(struct work_struct *work)
{
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(work,
		struct hdmi_hdcp_ctrl, hdcp_auth_prepare_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t qfprom_aksv_lsb, qfprom_aksv_msb;
	uint32_t link0_status;
	uint32_t regval;
	unsigned long flags;
	int ret;

	if (!hdcp_ctrl->aksv_valid) {
		ret = hdmi_hdcp_read_validate_aksv(hdcp_ctrl);
		if (ret) {
			pr_err("%s: ASKV validation failed\n", __func__);
			mutex_lock(&hdcp_ctrl->state_mutex);
			hdcp_ctrl->hdcp_state = HDCP_STATE_INACTIVE;
			mutex_unlock(&hdcp_ctrl->state_mutex);
			return;
		}
		hdcp_ctrl->aksv_valid = true;
	}

	spin_lock_irqsave(&hdmi->reg_lock, flags);
	/* disable HDMI Encrypt */
	regval = hdmi_read(hdmi, REG_HDMI_CTRL);
	regval &= ~HDMI_CTRL_ENCRYPTED;
	hdmi_write(hdmi, REG_HDMI_CTRL, regval);

	/* Enabling Software DDC */
	regval = hdmi_read(hdmi, REG_HDMI_DDC_ARBITRATION);
	regval &= ~HDMI_DDC_ARBITRATION_HW_ARBITRATION;
	hdmi_write(hdmi, REG_HDMI_DDC_ARBITRATION, regval);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	/*
	 * Write AKSV read from QFPROM to the HDCP registers.
	 * This step is needed for HDCP authentication and must be
	 * written before enabling HDCP.
	 */
	qfprom_aksv_lsb = hdcp_ctrl->aksv[3];
	qfprom_aksv_lsb = (qfprom_aksv_lsb << 8) | hdcp_ctrl->aksv[2];
	qfprom_aksv_lsb = (qfprom_aksv_lsb << 8) | hdcp_ctrl->aksv[1];
	qfprom_aksv_lsb = (qfprom_aksv_lsb << 8) | hdcp_ctrl->aksv[0];
	qfprom_aksv_msb = hdcp_ctrl->aksv[4];
	hdmi_write(hdmi, REG_HDMI_HDCP_SW_LOWER_AKSV, qfprom_aksv_lsb);
	hdmi_write(hdmi, REG_HDMI_HDCP_SW_UPPER_AKSV, qfprom_aksv_msb);

	/*
	 * HDCP setup prior to enabling HDCP_CTRL.
	 * Setup seed values for random number An.
	 */
	hdmi_write(hdmi, REG_HDMI_HDCP_ENTROPY_CTRL0, 0xB1FFB0FF);
	hdmi_write(hdmi, REG_HDMI_HDCP_ENTROPY_CTRL1, 0xF00DFACE);

	/* Disable the RngCipher state */
	hdmi_write(hdmi, REG_HDMI_HDCP_DEBUG_CTRL,
		hdmi_read(hdmi, REG_HDMI_HDCP_DEBUG_CTRL) & ~(BIT(2)));
	DBG("HDCP_DEBUG_CTRL=0x%08x",
		hdmi_read(hdmi, REG_HDMI_HDCP_DEBUG_CTRL));

	/*
	 * Ensure that all register writes are completed before
	 * enabling HDCP cipher
	 */
	wmb();

	/*
	 * Enable HDCP
	 * This needs to be done as early as possible in order for the
	 * hardware to make An available to read
	 */
	hdmi_write(hdmi, REG_HDMI_HDCP_CTRL, BIT(0));

	/*
	 * If we had stale values for the An ready bit, it should most
	 * likely be cleared now after enabling HDCP cipher
	 */
	link0_status = hdmi_read(hdmi, REG_HDMI_HDCP_LINK0_STATUS);
	DBG("After enabling HDCP Link0_Status=0x%08x", link0_status);
	if (!(link0_status & (BIT(8) | BIT(9))))
		DBG("An not ready after enabling HDCP");

	/* Clear any DDC failures from previous tries before enable HDCP*/
	reset_hdcp_ddc_failures(hdcp_ctrl);

	DBG("Queuing work to start HDCP authentication");
	hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
	queue_delayed_work(hdmi->workq,
		&hdcp_ctrl->hdcp_auth_part1_1_work, HZ/2);
}

static void hdmi_hdcp_auth_fail(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t regval;
	unsigned long flags;

	DBG("hdcp auth failed, queue reauth work");
	/* clear HDMI Encrypt */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	regval = hdmi_read(hdmi, REG_HDMI_CTRL);
	regval &= ~HDMI_CTRL_ENCRYPTED;
	hdmi_write(hdmi, REG_HDMI_CTRL, regval);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);
	queue_delayed_work(hdmi->workq, &hdcp_ctrl->hdcp_reauth_work, HZ/2);
}

static void hdmi_hdcp_auth_done(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t regval;
	unsigned long flags;

	/*
	 * Disable software DDC before going into part3 to make sure
	 * there is no Arbitration between software and hardware for DDC
	 */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	regval = hdmi_read(hdmi, REG_HDMI_DDC_ARBITRATION);
	regval |= HDMI_DDC_ARBITRATION_HW_ARBITRATION;
	hdmi_write(hdmi, REG_HDMI_DDC_ARBITRATION, regval);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	/*
	 * Ensure that the state did not change during authentication.
	 * If it did, it means that deauthenticate/reauthenticate was
	 * called. In that case, this function doesn't need to enable encryption
	 */
	mutex_lock(&hdcp_ctrl->state_mutex);
	if (HDCP_STATE_AUTHENTICATING == hdcp_ctrl->hdcp_state) {
		hdcp_ctrl->hdcp_state = HDCP_STATE_AUTHENTICATED;
		hdcp_ctrl->auth_retries = 0;

		/* enable HDMI Encrypt */
		spin_lock_irqsave(&hdmi->reg_lock, flags);
		regval = hdmi_read(hdmi, REG_HDMI_CTRL);
		regval |= HDMI_CTRL_ENCRYPTED;
		hdmi_write(hdmi, REG_HDMI_CTRL, regval);
		spin_unlock_irqrestore(&hdmi->reg_lock, flags);
		mutex_unlock(&hdcp_ctrl->state_mutex);
	} else {
		mutex_unlock(&hdcp_ctrl->state_mutex);
		DBG("HDCP state changed during authentication");
	}
}

/*
 * hdcp authenticating part 1: 1st
 * Wait Key/An ready
 * Read BCAPS from sink
 * Write BCAPS and AKSV into HDCP engine
 * Write An and AKSV to sink
 * Read BKSV from sink and write into HDCP engine
 */
static int hdmi_hdcp_check_key_an_ready(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t link0_status, an_ready, keys_state;

	link0_status = hdmi_read(hdmi, REG_HDMI_HDCP_LINK0_STATUS);
	/* Wait for HDCP keys to be checked and validated */
	keys_state = (link0_status >> 28) & 0x7;
	if (keys_state != HDCP_KEYS_STATE_VALID) {
		DBG("Keys not ready(%d). s=%d, l0=%0x08x",
			hdcp_ctrl->work_retry_cnt,
			keys_state, link0_status);

		return -EAGAIN;
	}

	/* Wait for An0 and An1 bit to be ready */
	an_ready = (link0_status & BIT(8)) && (link0_status & BIT(9));
	if (!an_ready) {
		DBG("An not ready(%d). l0_status=0x%08x",
			hdcp_ctrl->work_retry_cnt, link0_status);

		return -EAGAIN;
	}

	return 0;
}

static int hdmi_hdcp_send_aksv_an(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int rc = 0;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t link0_aksv_0, link0_aksv_1;
	uint32_t link0_an_0, link0_an_1;
	uint8_t aksv[5];
	uint8_t an[8];

	/* Read An0 and An1 */
	link0_an_0 = hdmi_read(hdmi, REG_HDMI_HDCP_RCVPORT_DATA5);
	link0_an_1 = hdmi_read(hdmi, REG_HDMI_HDCP_RCVPORT_DATA6);

	/* Read AKSV */
	link0_aksv_0 = hdmi_read(hdmi, REG_HDMI_HDCP_RCVPORT_DATA3);
	link0_aksv_1 = hdmi_read(hdmi, REG_HDMI_HDCP_RCVPORT_DATA4);

	DBG("Link ASKV=%08x%08x", link0_aksv_0, link0_aksv_1);
	/* Copy An and AKSV to byte arrays for transmission */
	aksv[0] =  link0_aksv_0        & 0xFF;
	aksv[1] = (link0_aksv_0 >> 8)  & 0xFF;
	aksv[2] = (link0_aksv_0 >> 16) & 0xFF;
	aksv[3] = (link0_aksv_0 >> 24) & 0xFF;
	aksv[4] =  link0_aksv_1        & 0xFF;

	an[0] =  link0_an_0        & 0xFF;
	an[1] = (link0_an_0 >> 8)  & 0xFF;
	an[2] = (link0_an_0 >> 16) & 0xFF;
	an[3] = (link0_an_0 >> 24) & 0xFF;
	an[4] =  link0_an_1        & 0xFF;
	an[5] = (link0_an_1 >> 8)  & 0xFF;
	an[6] = (link0_an_1 >> 16) & 0xFF;
	an[7] = (link0_an_1 >> 24) & 0xFF;

	/* Write An to offset 0x18 */
	rc = hdmi_ddc_write(hdmi, HDCP_PORT_ADDR, 0x18, an, 8);
	if (rc) {
		pr_err("%s:An write failed\n", __func__);
		return rc;
	}
	DBG("Link0-An=%08x%08x", link0_an_1, link0_an_0);

	/* Write AKSV to offset 0x10 */
	rc = hdmi_ddc_write(hdmi, HDCP_PORT_ADDR, 0x10, aksv, 5);
	if (rc) {
		pr_err("%s:AKSV write failed\n", __func__);
		return rc;
	}
	DBG("Link0-AKSV=%02x%08x", link0_aksv_1 & 0xFF, link0_aksv_0);

	return 0;
}

static int hdmi_hdcp_recv_bksv(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int rc = 0;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t link0_bksv_0, link0_bksv_1;
	uint8_t *bksv = NULL;
	uint32_t reg[2], data[2];

	bksv = hdcp_ctrl->bksv;

	/* Read BKSV at offset 0x00 */
	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x00, bksv, 5, true);
	if (rc) {
		pr_err("%s:BKSV read failed\n", __func__);
		return rc;
	}

	/* check there are 20 ones in BKSV */
	if (hdmi_hdcp_count_one(bksv, 5) != 20) {
		pr_err(": BKSV doesn't have 20 1's and 20 0's\n");
		pr_err(": BKSV chk fail. BKSV=%02x%02x%02x%02x%02x\n",
			bksv[4], bksv[3], bksv[2], bksv[1], bksv[0]);
		return -EINVAL;
	}

	link0_bksv_0 = bksv[3];
	link0_bksv_0 = (link0_bksv_0 << 8) | bksv[2];
	link0_bksv_0 = (link0_bksv_0 << 8) | bksv[1];
	link0_bksv_0 = (link0_bksv_0 << 8) | bksv[0];
	link0_bksv_1 = bksv[4];
	DBG(":BKSV=%02x%08x", link0_bksv_1, link0_bksv_0);

	/* Write BKSV read from sink to HDCP registers */
	reg[0] = REG_HDMI_HDCP_RCVPORT_DATA0;
	data[0] = link0_bksv_0;
	reg[1] = REG_HDMI_HDCP_RCVPORT_DATA1;
	data[1] = link0_bksv_1;
	rc = hdmi_hdcp_scm_wr(hdcp_ctrl, reg, data, 2);

	return rc;
}

static int hdmi_hdcp_recv_bcaps(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int rc = 0;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t reg, data;
	uint8_t bcaps;

	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x40, &bcaps, 1, true);
	if (rc) {
		pr_err("%s:BCAPS read failed\n", __func__);
		return rc;
	}
	DBG("BCAPS=%02x", bcaps);

	/* receiver (0), repeater (1) */
	hdcp_ctrl->ds_type = (bcaps & BIT(6)) ? DS_REPEATER : DS_RECEIVER;

	/* Write BCAPS to the hardware */
	reg = REG_HDMI_HDCP_RCVPORT_DATA12;
	data = (uint32_t)bcaps;
	rc = hdmi_hdcp_scm_wr(hdcp_ctrl, &reg, &data, 1);

	return rc;
}

static void hdmi_hdcp_auth_part1_1_work(struct work_struct *work)
{
	int rc = 0;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part1_1_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	unsigned long flags;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		pr_err("%s: invalid state. returning\n", __func__);
		return;
	}

	hdcp_ctrl->work_retry_cnt--;

	/*
	 * Wait for AKSV key and An ready
	 */
	rc = hdmi_hdcp_check_key_an_ready(hdcp_ctrl);
	if ((rc == -EAGAIN) && hdcp_ctrl->work_retry_cnt) {
		queue_delayed_work(hdmi->workq,
			&hdcp_ctrl->hdcp_auth_part1_1_work, HZ/50);
		return;
	} else if (rc) {
		pr_err("%s: Key/An not ready, abort\n", __func__);
		goto error;
	}

	/*
	 * Read BCAPS and send to HDCP engine
	 */
	rc = hdmi_hdcp_recv_bcaps(hdcp_ctrl);
	if (rc) {
		pr_err("%s: read bcaps error, abort\n", __func__);
		goto error;
	}

	/*
	 * 1.1_Features turned off by default.
	 * No need to write AInfo since 1.1_Features is disabled.
	 */
	hdmi_write(hdmi, REG_HDMI_HDCP_RCVPORT_DATA4, 0);

	/*
	 * Send AKSV and An to sink
	 */
	rc = hdmi_hdcp_send_aksv_an(hdcp_ctrl);
	if (rc) {
		pr_err("%s:An/Aksv write failed\n", __func__);
		goto error;
	}

	/* Read BKSV and send to HDCP engine*/
	rc = hdmi_hdcp_recv_bksv(hdcp_ctrl);
	if (rc) {
		pr_err("%s:BKSV Process failed\n", __func__);
		goto error;
	}

	/* Enable HDCP interrupts and ack/clear any stale interrupts */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	hdmi_write(hdmi, REG_HDMI_HDCP_INT_CTRL, 0xE6);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	/*
	 * HDCP Compliance Test case 1A-01:
	 * Wait here at least 100ms before reading R0'
	 */
	hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
	queue_delayed_work(hdmi->workq,
		&hdcp_ctrl->hdcp_auth_part1_2_work, HZ/8);

	return;
error:
	pr_err("%s: Authentication Part I failed %d\n", __func__, rc);
	hdmi_hdcp_auth_fail(hdcp_ctrl);
}

/*
 * hdcp authenticating part 1: 2nd
 * read R0' from sink and pass it to HDCP engine
 */
static void hdmi_hdcp_auth_part1_2_work(struct work_struct *work)
{
	int rc = 0;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part1_2_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint8_t buf[2];

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		pr_err("%s: invalid state. returning\n", __func__);
		return;
	}

	/* Read R0' at offset 0x08 */
	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x08, buf, 2, true);
	if (rc) {
		pr_err("%s:R0' read failed\n", __func__);
		goto error;
	}
	DBG("R0'=%02x%02x", buf[1], buf[0]);

	/* Write R0' to HDCP registers and check to see if it is a match */
	hdmi_write(hdmi, REG_HDMI_HDCP_RCVPORT_DATA2_0,
		(((uint32_t)buf[1]) << 8) | buf[0]);

	return;
error:
	pr_err("%s: Authentication Part I:2 failed %d\n", __func__, rc);
	hdmi_hdcp_auth_fail(hdcp_ctrl);
}

/*
 * hdcp authenticating part 1: 3rd
 * Wait for authenticating result: R0/R0' are matched or not
 */
static void hdmi_hdcp_auth_part1_3_work(struct work_struct *work)
{
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(work,
		struct hdmi_hdcp_ctrl, hdcp_auth_part1_3_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	bool is_match;
	uint32_t link0_status;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		pr_err("%s: invalid state. returning\n", __func__);
		return;
	}

	link0_status = hdmi_read(hdmi, REG_HDMI_HDCP_LINK0_STATUS);
	is_match = link0_status & BIT(12);
	if (!is_match) {
		pr_err("%s: Authentication Part I failed\n", __func__);
		hdmi_hdcp_auth_fail(hdcp_ctrl);
	} else {
		/* Enable HDCP Encryption */
		hdmi_write(hdmi, REG_HDMI_HDCP_CTRL, BIT(0) | BIT(8));
		DBG("Authentication Part I successful");
		if (hdcp_ctrl->ds_type == DS_REPEATER) {
			/* queue HDCP PartII work */
			hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
			queue_delayed_work(hdmi->workq,
				&hdcp_ctrl->hdcp_auth_part2_1_work, 0);
		} else {
			hdmi_hdcp_auth_done(hdcp_ctrl);
		}
	}
}

/*
 * hdcp authenticating part 2: 1st
 * wait until sink (repeater)'s ksv fifo ready
 * read bstatus from sink and write to HDCP engine
 */
static int hdmi_hdcp_recv_bstatus(struct hdmi_hdcp_ctrl *hdcp_ctrl,
	uint8_t bcaps)
{
	int rc;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint16_t bstatus;
	bool max_devs_exceeded = false, max_cascade_exceeded = false;
	uint32_t repeater_cascade_depth = 0, down_stream_devices = 0;
	uint32_t reg, data;
	uint8_t buf[2];

	memset(buf, 0, sizeof(buf));

	/* Read BSTATUS at offset 0x41 */
	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x41, buf, 2, false);
	if (rc) {
		pr_err("%s: BSTATUS read failed\n", __func__);
		goto error;
	}
	bstatus = buf[1];
	bstatus = (bstatus << 8) | buf[0];

	/* Write BSTATUS and BCAPS to HDCP registers */
	reg = REG_HDMI_HDCP_RCVPORT_DATA12;
	data = bcaps | (bstatus << 8);
	rc = hdmi_hdcp_scm_wr(hdcp_ctrl, &reg, &data, 1);
	if (rc) {
		pr_err("%s: BSTATUS write failed\n", __func__);
		goto error;
	}

	down_stream_devices = bstatus & 0x7F;
	repeater_cascade_depth = (bstatus >> 8) & 0x7;
	max_devs_exceeded = (bstatus & BIT(7)) ? true : false;
	max_cascade_exceeded = (bstatus & BIT(11)) ? true : false;

	if (down_stream_devices == 0) {
		/*
		 * If no downstream devices are attached to the repeater
		 * then part II fails.
		 * todo: The other approach would be to continue PART II.
		 */
		pr_err("%s: No downstream devices\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	/*
	 * HDCP Compliance 1B-05:
	 * Check if no. of devices connected to repeater
	 * exceed max_devices_connected from bit 7 of Bstatus.
	 */
	if (max_devs_exceeded) {
		pr_err("%s: no. of devs connected exceeds max allowed",
			__func__);
		rc = -EINVAL;
		goto error;
	}

	/*
	 * HDCP Compliance 1B-06:
	 * Check if no. of cascade connected to repeater
	 * exceed max_cascade_connected from bit 11 of Bstatus.
	 */
	if (max_cascade_exceeded) {
		pr_err("%s: no. of cascade conn exceeds max allowed",
			__func__);
		rc = -EINVAL;
		goto error;
	}

error:
	hdcp_ctrl->dev_count = down_stream_devices;
	hdcp_ctrl->max_cascade_exceeded = max_cascade_exceeded;
	hdcp_ctrl->max_dev_exceeded = max_devs_exceeded;
	hdcp_ctrl->depth = repeater_cascade_depth;
	return rc;
}

static void hdmi_hdcp_auth_part2_1_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part2_1_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint8_t bcaps;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		pr_err("%s: invalid state. returning", __func__);
		return;
	}

	hdcp_ctrl->work_retry_cnt--;

	/*
	 * Wait until READY bit is set in BCAPS, as per HDCP specifications
	 * maximum permitted time to check for READY bit is five seconds.
	 */
	/* Read BCAPS at offset 0x40 */
	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x40, &bcaps, 1, false);
	if (rc) {
		pr_err("%s: BCAPS read failed\n", __func__);
		goto error;
	}

	if (!(bcaps & BIT(5))) {
		if (hdcp_ctrl->work_retry_cnt) {
			DBG("ksv fifo (%d) is not ready",
				hdcp_ctrl->work_retry_cnt);
			queue_delayed_work(hdmi->workq,
				&hdcp_ctrl->hdcp_auth_part2_1_work, HZ/20);
			return;
		} else {
			pr_err("%s: timeout to wait ksv ready\n",
				__func__);
			rc = -ETIMEDOUT;
			goto error;
		}
	}

	rc = hdmi_hdcp_recv_bstatus(hdcp_ctrl, bcaps);
	if (rc) {
		pr_err("%s: bstatus error\n", __func__);
		goto error;
	}

	hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
	queue_delayed_work(hdmi->workq,
		&hdcp_ctrl->hdcp_auth_part2_2_work, 0);

	return;
error:
	pr_err("%s: Authentication Part II failed %d\n", __func__, rc);
	hdmi_hdcp_auth_fail(hdcp_ctrl);
}

/*
 * hdcp authenticating part 2: 2nd
 * read ksv fifo from sink
 * transfer V' from sink to HDCP engine
 * reset SHA engine
 */
int hdmi_hdcp_read_v(struct hdmi *hdmi, char *name,
	uint32_t off, uint32_t *val)
{
	int rc = 0;
	uint8_t buf[4];

	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, off, buf, 4, true);
	if (rc) {
		pr_err("%s: Read %s failed\n", __func__,
			name);
		return rc;
	}

	if (val)
		*val = (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);

	DBG("%s: buf[0]=%x, buf[1]=%x, buf[2]=%x, buf[3]=%x", name,
		buf[0], buf[1], buf[2], buf[3]);

	return rc;
}

static int hdmi_hdcp_transfer_v_h(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	int rc = 0;
	struct hdmi_hdcp_reg_data reg_data[]  = {
		{REG_HDMI_HDCP_RCVPORT_DATA7,  0x20, "V' H0"},
		{REG_HDMI_HDCP_RCVPORT_DATA8,  0x24, "V' H1"},
		{REG_HDMI_HDCP_RCVPORT_DATA9,  0x28, "V' H2"},
		{REG_HDMI_HDCP_RCVPORT_DATA10, 0x2C, "V' H3"},
		{REG_HDMI_HDCP_RCVPORT_DATA11, 0x30, "V' H4"},
	};
	struct hdmi_hdcp_reg_data *rd;
	uint32_t size = ARRAY_SIZE(reg_data);
	uint32_t reg[ARRAY_SIZE(reg_data)], data[ARRAY_SIZE(reg_data)];
	int i;

	for (i = 0; i < size; i++) {
		rd = &reg_data[i];
		rc = hdmi_hdcp_read_v(hdmi, rd->name,
			rd->off, &data[i]);
		if (rc)
			goto error;

		reg[i] = reg_data[i].reg_id;
	}

	rc = hdmi_hdcp_scm_wr(hdcp_ctrl, reg, data, size);

error:
	return rc;
}

static int hdmi_hdcp_recv_ksv_fifo(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int rc;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint8_t *ksv_fifo = NULL;
	uint32_t ksv_bytes;

	ksv_fifo = hdcp_ctrl->ksv_list;
	ksv_bytes = 5 * hdcp_ctrl->dev_count;
	memset(ksv_fifo, 0,
		sizeof(hdcp_ctrl->ksv_list));

	rc = hdmi_ddc_read(hdmi, HDCP_PORT_ADDR, 0x43,
		ksv_fifo, ksv_bytes, true);
	if (rc)
		pr_err("%s: KSV FIFO read failed\n", __func__);

	return rc;
}

static int hdmi_hdcp_reset_sha_engine(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	uint32_t reg[2], data[2];
	uint32_t rc  = 0;

	reg[0] = REG_HDMI_HDCP_SHA_CTRL;
	data[0] = HDCP_REG_ENABLE;
	reg[1] = REG_HDMI_HDCP_SHA_CTRL;
	data[1] = HDCP_REG_DISABLE;

	rc = hdmi_hdcp_scm_wr(hdcp_ctrl, reg, data, 2);

	return rc;
}

static void hdmi_hdcp_auth_part2_2_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part2_2_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		DBG("invalid state. returning");
		return;
	}

	hdcp_ctrl->work_retry_cnt--;

	/*
	 * Read KSV FIFO over DDC
	 * Key Selection vector FIFO Used to pull downstream KSVs
	 * from HDCP Repeaters.
	 * All bytes (DEVICE_COUNT * 5) must be read in a single,
	 * auto incrementing access.
	 * All bytes read as 0x00 for HDCP Receivers that are not
	 * HDCP Repeaters (REPEATER == 0).
	 */
	rc = hdmi_hdcp_recv_ksv_fifo(hdcp_ctrl);
	if (rc && hdcp_ctrl->work_retry_cnt) {
		/*
		 * HDCP Compliance Test case 1B-01:
		 * Wait until all the ksv bytes have been
		 * read from the KSV FIFO register.
		 */
		queue_delayed_work(hdmi->workq,
			&hdcp_ctrl->hdcp_auth_part2_2_work, HZ/40);
		return;
	} else if (rc) {
		pr_err("%s: error to recv ksv fifo\n", __func__);
		goto error;
	}

	rc = hdmi_hdcp_transfer_v_h(hdcp_ctrl);
	if (rc) {
		pr_err("%s: transfer V failed\n", __func__);
		goto error;
	}

	/* reset SHA engine before write ksv fifo */
	rc = hdmi_hdcp_reset_sha_engine(hdcp_ctrl);
	if (rc) {
		pr_err("%s: fail to reset sha engine\n", __func__);
		goto error;
	}

	hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
	hdcp_ctrl->ksv_fifo_w_index = 0;
	queue_delayed_work(hdmi->workq,
		&hdcp_ctrl->hdcp_auth_part2_3_work, 0);

	return;
error:
	pr_err("%s: Authentication Part II failed\n", __func__);
	hdmi_hdcp_auth_fail(hdcp_ctrl);
}

/*
 * Write KSV FIFO to HDCP_SHA_DATA.
 * This is done 1 byte at time starting with the LSB.
 * Once 64 bytes have been written, we need to poll for
 * HDCP_SHA_BLOCK_DONE before writing any further
 * If the last byte is written, we need to poll for
 * HDCP_SHA_COMP_DONE to wait until HW finish
 */
static int hdmi_hdcp_write_ksv_fifo(struct hdmi_hdcp_ctrl *hdcp_ctrl)
{
	int i;
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t ksv_bytes, last_byte = 0;
	uint8_t *ksv_fifo = NULL;
	uint32_t reg_val, data, reg;
	uint32_t rc  = 0;

	ksv_bytes  = 5 * hdcp_ctrl->dev_count;

	/* Check if need to wait for HW completion */
	if (hdcp_ctrl->ksv_fifo_w_index) {
		reg_val = hdmi_read(hdmi, REG_HDMI_HDCP_SHA_STATUS);
		DBG("HDCP_SHA_STATUS=%08x", reg_val);
		if (hdcp_ctrl->ksv_fifo_w_index == ksv_bytes) {
			/* check COMP_DONE if last write */
			if (reg_val & BIT(4)) {
				DBG("COMP_DONE");
				return 0;
			} else {
				return -EAGAIN;
			}
		} else {
			/* check BLOCK_DONE if not last write */
			if (!(reg_val & BIT(0)))
				return -EAGAIN;
			else
				DBG("BLOCK_DONE");
		}
	}

	ksv_bytes  -= hdcp_ctrl->ksv_fifo_w_index;
	if (ksv_bytes <= 64)
		last_byte = 1;
	else
		ksv_bytes = 64;

	ksv_fifo = hdcp_ctrl->ksv_list;
	ksv_fifo += hdcp_ctrl->ksv_fifo_w_index;

	for (i = 0; i < ksv_bytes; i++) {
		/* Write KSV byte and set DONE bit[0] for last byte*/
		reg_val = ksv_fifo[i] << 16;
		if ((i == (ksv_bytes - 1)) && last_byte)
			reg_val |= BIT(0);

		reg = REG_HDMI_HDCP_SHA_DATA;
		data = reg_val;
		rc = hdmi_hdcp_scm_wr(hdcp_ctrl, &reg, &data, 1);

		if (rc)
			return rc;
	}

	hdcp_ctrl->ksv_fifo_w_index += ksv_bytes;

	/*
	 *return -EAGAIN to notify caller to wait for COMP_DONE or BLOCK_DONE
	 */
	return -EAGAIN;
}

/*
 * hdcp authenticating part 2: 3rd
 * write ksv fifo into HDCP engine
 */
static void hdmi_hdcp_auth_part2_3_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part2_3_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		DBG("invalid state. returning");
		return;
	}

	hdcp_ctrl->work_retry_cnt--;

	rc = hdmi_hdcp_write_ksv_fifo(hdcp_ctrl);
	if ((rc == -EAGAIN) && hdcp_ctrl->work_retry_cnt) {
		queue_delayed_work(hdmi->workq,
			&hdcp_ctrl->hdcp_auth_part2_3_work, HZ/50);
		return;
	} else if (rc) {
		pr_err("%s: error write ksv fifo\n", __func__);
		goto error;
	}

	hdcp_ctrl->work_retry_cnt = AUTH_WORK_RETRIES_TIME;
	queue_delayed_work(hdmi->workq,
		&hdcp_ctrl->hdcp_auth_part2_4_work, HZ/50);

	return;
error:
	pr_err("%s: Authentication Part II failed\n", __func__);
	hdmi_hdcp_auth_fail(hdcp_ctrl);
} /* hdmi_hdcp_authentication_part2 */

/*
 * hdcp authenticating part 2: 4th
 * Wait for V_MATCHES
 */
static void hdmi_hdcp_auth_part2_4_work(struct work_struct *work)
{
	int rc = 0;
	struct delayed_work *dw = to_delayed_work(work);
	struct hdmi_hdcp_ctrl *hdcp_ctrl = container_of(dw,
		struct hdmi_hdcp_ctrl, hdcp_auth_part2_4_work);
	struct hdmi *hdmi = hdcp_ctrl->hdmi;
	uint32_t link0_status;

	if (HDCP_STATE_AUTHENTICATING != hdcp_ctrl->hdcp_state) {
		DBG("invalid state. returning");
		return;
	}

	hdcp_ctrl->work_retry_cnt--;

	link0_status = hdmi_read(hdmi, REG_HDMI_HDCP_LINK0_STATUS);
	if (!(link0_status & BIT(20))) {
		if (hdcp_ctrl->work_retry_cnt) {
			queue_delayed_work(hdmi->workq,
				&hdcp_ctrl->hdcp_auth_part2_4_work, HZ/50);
			return;
		} else {
			rc = -ETIMEDOUT;
			pr_err("%s: HDCP V Match timedout", __func__);
		}
	}

	if (rc) {
		pr_err("%s: Authentication Part II failed\n", __func__);
		hdmi_hdcp_auth_fail(hdcp_ctrl);
	} else {
		pr_info("%s: Authentication Part II successful\n",
			__func__);
		hdmi_hdcp_auth_done(hdcp_ctrl);
	}
}

int hdmi_hdcp_on(struct hdmi *hdmi)
{
	struct hdmi_hdcp_ctrl *hdcp_ctrl = hdmi->hdcp_ctrl;
	uint32_t regval;
	unsigned long flags;

	if (!hdcp_ctrl) {
		pr_warn("%s:hdcp_ctrl is NULL\n", __func__);
		return 0;
	}

	if (HDCP_STATE_INACTIVE != hdcp_ctrl->hdcp_state) {
		DBG("still active or activating. returning");
		return 0;
	}

	/* clear HDMI Encrypt */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	regval = hdmi_read(hdmi, REG_HDMI_CTRL);
	regval &= ~HDMI_CTRL_ENCRYPTED;
	hdmi_write(hdmi, REG_HDMI_CTRL, regval);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	mutex_lock(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->hdcp_state = HDCP_STATE_AUTHENTICATING;
	mutex_unlock(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->auth_retries = 0;
	queue_work(hdmi->workq, &hdcp_ctrl->hdcp_auth_prepare_work);

	return 0;
}

void hdmi_hdcp_off(struct hdmi *hdmi)
{
	struct hdmi_hdcp_ctrl *hdcp_ctrl = hdmi->hdcp_ctrl;
	unsigned long flags;
	uint32_t regval;
	int rc = 0;

	if (!hdcp_ctrl) {
		pr_err("%s:hdcp_ctrl is NULL\n", __func__);
		return;
	}

	if (HDCP_STATE_INACTIVE == hdcp_ctrl->hdcp_state) {
		DBG("hdcp inactive. returning");
		return;
	}

	/*
	 * Disable HPD circuitry.
	 * This is needed to reset the HDCP cipher engine so that when we
	 * attempt a re-authentication, HW would clear the AN0_READY and
	 * AN1_READY bits in HDMI_HDCP_LINK0_STATUS register
	 */
	spin_lock_irqsave(&hdmi->reg_lock, flags);
	hdmi_write(hdmi, REG_HDMI_HPD_CTRL,
		hdmi_read(hdmi, REG_HDMI_HPD_CTRL) & ~BIT(28));

	/*
	 * Disable HDCP interrupts.
	 * Also, need to set the state to inactive here so that any ongoing
	 * reauth works will know that the HDCP session has been turned off.
	 */
	hdmi_write(hdmi, REG_HDMI_HDCP_INT_CTRL, 0);
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	/*
	 * Cancel any pending auth/reauth attempts.
	 * If one is ongoing, this will wait for it to finish.
	 * No more reauthentication attempts will be scheduled since we
	 * set the current state to inactive.
	 */
	rc = cancel_work_sync(&hdcp_ctrl->hdcp_auth_prepare_work);
	if (rc)
		DBG("Deleted hdcp auth prepare work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_reauth_work);
	if (rc)
		DBG("Deleted hdcp reauth work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part1_1_work);
	if (rc)
		DBG("Deleted hdcp auth part1_1 work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part1_2_work);
	if (rc)
		DBG("Deleted hdcp auth part1_2 work");
	rc = cancel_work_sync(&hdcp_ctrl->hdcp_auth_part1_3_work);
	if (rc)
		DBG("Deleted hdcp auth part1_3 work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part2_1_work);
	if (rc)
		DBG("Deleted hdcp auth part2_1 work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part2_2_work);
	if (rc)
		DBG("Deleted hdcp auth part2_2 work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part2_3_work);
	if (rc)
		DBG("Deleted hdcp auth part2_3 work");
	rc = cancel_delayed_work_sync(&hdcp_ctrl->hdcp_auth_part2_4_work);
	if (rc)
		DBG("Deleted hdcp auth part2_4 work");

	/* set state to inactive after all work cancelled */
	mutex_lock(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->hdcp_state = HDCP_STATE_INACTIVE;
	mutex_unlock(&hdcp_ctrl->state_mutex);

	hdmi_write(hdmi, REG_HDMI_HDCP_RESET, BIT(0));

	/* Disable encryption and disable the HDCP block */
	hdmi_write(hdmi, REG_HDMI_HDCP_CTRL, 0);

	spin_lock_irqsave(&hdmi->reg_lock, flags);
	regval = hdmi_read(hdmi, REG_HDMI_CTRL);
	regval &= ~HDMI_CTRL_ENCRYPTED;
	hdmi_write(hdmi, REG_HDMI_CTRL, regval);

	/* Enable HPD circuitry */
	hdmi_write(hdmi, REG_HDMI_HPD_CTRL,
		hdmi_read(hdmi, REG_HDMI_HPD_CTRL) | BIT(28));
	spin_unlock_irqrestore(&hdmi->reg_lock, flags);

	DBG("HDCP: Off");
} /* hdmi_hdcp_off */

struct hdmi_hdcp_ctrl *hdmi_hdcp_init(struct hdmi *hdmi)
{
	uint32_t scm_buf = TZ_HDCP_CMD_ID;
	struct hdmi_hdcp_ctrl *hdcp_ctrl;
	uint32_t ret  = 0;
	uint32_t resp = 0;

	if (!hdmi->qfprom_mmio) {
		pr_err("%s: HDCP is not supported without qfprom\n",
			__func__);
		ret = -EINVAL;
		goto fail;
	}

	hdcp_ctrl = kzalloc(sizeof(*hdcp_ctrl), GFP_KERNEL);
	if (!hdcp_ctrl) {
		ret = -ENOMEM;
		goto fail;
	}

	INIT_WORK(&hdcp_ctrl->hdcp_auth_prepare_work,
		hdmi_hdcp_auth_prepare_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_reauth_work, hdmi_hdcp_reauth_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part1_1_work,
		hdmi_hdcp_auth_part1_1_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part1_2_work,
		hdmi_hdcp_auth_part1_2_work);
	INIT_WORK(&hdcp_ctrl->hdcp_auth_part1_3_work,
		hdmi_hdcp_auth_part1_3_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part2_1_work,
		hdmi_hdcp_auth_part2_1_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part2_2_work,
		hdmi_hdcp_auth_part2_2_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part2_3_work,
		hdmi_hdcp_auth_part2_3_work);
	INIT_DELAYED_WORK(&hdcp_ctrl->hdcp_auth_part2_4_work,
		hdmi_hdcp_auth_part2_4_work);

	hdcp_ctrl->hdmi = hdmi;
	hdcp_ctrl->hdcp_state = HDCP_STATE_INACTIVE;
	mutex_init(&hdcp_ctrl->state_mutex);
	hdcp_ctrl->aksv_valid = false;

	ret = scm_call(SCM_SVC_INFO, SCM_CMD_HDCP, &scm_buf,
		sizeof(scm_buf), &resp, sizeof(resp));
	if (ret) {
		pr_err("%s: error: scm_call ret = %d, resp = %d\n",
			__func__, ret, resp);
		goto fail;
	} else {
		DBG("tz_hdcp = %d", resp);
		hdcp_ctrl->tz_hdcp = resp;
	}

	return hdcp_ctrl;
fail:
	kfree(hdcp_ctrl);
	return ERR_PTR(ret);
} /* hdmi_hdcp_init */

void hdmi_hdcp_destroy(struct hdmi *hdmi)
{
	if (hdmi && hdmi->hdcp_ctrl) {
		kfree(hdmi->hdcp_ctrl);
		hdmi->hdcp_ctrl = NULL;
	}
}
