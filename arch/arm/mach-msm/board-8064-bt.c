/* Copyright (c) 2011-2013, Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/marimba.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_xo.h>
#include <mach/socinfo.h>

#include "board-8064.h"

int gpio_bt_sys_reset_en;
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.id = -1,
};

#if defined CONFIG_BT_HCIUART_ATH3K
static struct resource bluesleep_resources[] = {
	{
		.name   = "gpio_host_wake",
		.start  = 7,
		.end    = 7,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "gpio_ext_wake",
		.start  = 9,
		.end    = 9,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "host_wake",
		.start  = MSM_GPIO_TO_INT(7),
		.end    = MSM_GPIO_TO_INT(7),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name		= "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};
#endif

#if defined(CONFIG_BT)

static unsigned bt_config_pcm_on[] = {
	/*PCM_DOUT*/
	GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(44, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(46, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned bt_config_pcm_off[] = {
	/*PCM_DOUT*/
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static int config_pcm(int mode)
{
	int pin, rc = 0;

	if (mode == BT_PCM_ON) {
		pr_debug("%s mode : BT_PCM_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_on[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
		}
	} else if (mode == BT_PCM_OFF) {
		pr_debug("%s mode : BT_PCM_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
		}

	}

	return rc;
}

#if defined(CONFIG_MARIMBA_CORE)

#define BAHAMA_SLAVE_ID_FM_ADDR  0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

struct bt_vreg_info {
	const char *name;
	unsigned int pmapp_id;
	unsigned int min_level;
	unsigned int max_level;
	unsigned int is_pin_controlled;
	struct regulator *reg;
};

struct bahama_config_register {
	u8 reg;
	u8 value;
	u8 mask;
};
static struct bt_vreg_info bt_vregs[] = {
	{"bha_vddxo",  2, 1800000, 1800000, 0, NULL},
	{"bha_vddpx", 21, 1800000, 1800000, 0, NULL},
	{"bha_vddpa", 21, 2900000, 3300000, 0, NULL}
};

static struct msm_xo_voter *bt_clock;

static int bahama_bt(int on)
{
	int rc = 0;
	int i;

	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	int version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x8E, 0x15, 0xFF },
		{ 0x8F, 0x15, 0xFF },
		{ 0x90, 0x15, 0xFF },

		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_variant_register bt_bahama[2][3] = {
	{
		{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
		{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
		{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
	},
	{
		{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
		{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
		{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
	}
	};

	u8 offset = 0; /* index into bahama configs */
	on = on ? 1 : 0;
	version = marimba_read_bahama_ver(&config);
	if (version < 0 || version == BAHAMA_VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev,
			"%s : Bahama version read Error, version = %d\n",
			__func__, version);
		return -EIO;
	}
	if (version == BAHAMA_VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][version + offset].set;

	dev_dbg(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %x write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
		value = 0;
		if (marimba_read_bit_mask(&config,
				(p+i)->reg, &value,
				sizeof((p+i)->value), (p+i)->mask) < 0)
			dev_err(&msm_bt_power_device.dev,
				"%s marimba_read_bit_mask- error",
				__func__);
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x read value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT Status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);
	return rc;
}

static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		if (IS_ERR_OR_NULL(bt_vregs[i].reg)) {
			bt_vregs[i].reg =
				regulator_get(&msm_bt_power_device.dev,
						bt_vregs[i].name);
			if (IS_ERR(bt_vregs[i].reg)) {
				rc = PTR_ERR(bt_vregs[i].reg);
				dev_err(&msm_bt_power_device.dev,
					"%s: invalid regulator handle for %s: %d\n",
						__func__, bt_vregs[i].name, rc);
				goto reg_disable;
			}
		}
		rc = on ? regulator_set_voltage(bt_vregs[i].reg,
					bt_vregs[i].min_level,
						bt_vregs[i].max_level) : 0;
		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not set voltage for %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_disable;
		}

		rc = on ? regulator_enable(bt_vregs[i].reg) : 0;
		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not %sable regulator %s: %d\n",
					__func__, "en", bt_vregs[i].name, rc);
			goto reg_disable;
		}

		rc = on ? 0 : regulator_disable(bt_vregs[i].reg);

		if (rc) {
			dev_err(&msm_bt_power_device.dev,
				"%s: could not %sable regulator %s: %d\n",
					__func__, "dis", bt_vregs[i].name, rc);
		}
	}

	return rc;
reg_disable:
	pr_err("bluetooth_switch_regulators - FAIL!!!!\n");
	if (on) {
		while (i) {
			i--;
			regulator_disable(bt_vregs[i].reg);
			regulator_put(bt_vregs[i].reg);
			bt_vregs[i].reg = NULL;
		}
	}
	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		bt_vregs[i].reg = regulator_get(&msm_bt_power_device.dev,
						bt_vregs[i].name);
		if (IS_ERR(bt_vregs[i].reg)) {
			rc = PTR_ERR(bt_vregs[i].reg);
			pr_err("%s: could not get regulator %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_fail;
		}
		rc = regulator_set_voltage(bt_vregs[i].reg,
					bt_vregs[i].min_level,
						bt_vregs[i].max_level);
		if (rc) {
			pr_err("%s: could not set voltage for %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_fail;
		}
		rc = regulator_enable(bt_vregs[i].reg);
		if (rc) {
			pr_err("%s: could not enable regulator %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto reg_fail;
		}
	}
	return rc;
reg_fail:
	pr_err("msm_bahama_setup_power FAILED !!!\n");

	while (i) {
		i--;
		regulator_disable(bt_vregs[i].reg);
		regulator_put(bt_vregs[i].reg);
		bt_vregs[i].reg = NULL;
	}
	return rc;
}

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		rc = regulator_disable(bt_vregs[i].reg);

		if (rc < 0) {
			pr_err("%s: could not disable regulator %s: %d\n",
					__func__, bt_vregs[i].name, rc);
			goto out;
		}

		regulator_put(bt_vregs[i].reg);
		bt_vregs[i].reg = NULL;
	}
out:
	return rc;
}

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {
		int i;
		struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};
		if (marimba_read_bahama_ver(&config) == BAHAMA_VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					pr_err("%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				pr_debug("%s: reg 0x%02x value 0x%02x mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	pr_debug("core type: %d\n", type);
	return rc;
}

static int bahama_bluetooth_power(int on)
{
	int rc = 0;
	const char *id = "BTPW";
	int bt_state = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	if (on) {
		pr_debug("%s: Powering up BT module on WCN2243.\n", __func__);
		rc = bluetooth_switch_regulators(on);
		if (rc < 0) {
			pr_err("%s: bluetooth_switch_regulators rc = %d",
					__func__, rc);
			goto exit;
		}
		/* UART GPIO configuration to be done by by UART module*/
		/*Setup BT clocks*/
		pr_debug("%s: Voting for the 19.2MHz clock\n", __func__);
		bt_clock = msm_xo_get(MSM_XO_TCXO_A2, id);
		if (IS_ERR(bt_clock)) {
			rc = PTR_ERR(bt_clock);
			pr_err("%s: failed to get the handle for A2(%d)\n",
					__func__, rc);
			goto fail_power;
		}
		rc = msm_xo_mode_vote(bt_clock, MSM_XO_MODE_ON);
		if (rc < 0) {
			pr_err("%s: Failed to vote for TCXO_A2 ON\n", __func__);
			goto fail_xo_vote;
		}
		msleep(20);

		/*I2C config for Bahama*/
		pr_debug("%s: BT Turn On sequence in-progress.\n", __func__);
		rc = bahama_bt(1);
		if (rc < 0) {
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
			goto fail_xo_vote;
		}
		msleep(20);

		/*setup BT PCM lines*/
		pr_debug("%s: Configuring PCM lines.\n", __func__);
		rc = config_pcm(BT_PCM_ON);
		if (rc < 0) {
			pr_err("%s: config_pcm , rc = %d\n",
				__func__, rc);
			goto fail_i2c;
		}
		pr_debug("%s: BT Turn On complete.\n", __func__);
		/* TO DO - Enable PIN CTRL */
		/*
		rc = msm_xo_mode_vote(bt_clock, MSM_XO_MODE_PIN_CTRL);
		if (rc < 0) {
			pr_err("%s: Failed to vote for TCXO_A2 in PIN_CTRL\n",
				__func__);
			goto fail_xo_vote;
		} */
	} else {
		pr_debug("%s: Powering down BT module on WCN2243.\n", __func__);
		bt_state = marimba_get_bt_status(&config);
		if (!bt_state) {
			pr_err("%s: BT is already turned OFF.\n", __func__);
			return 0;
		}

		rc = config_pcm(BT_PCM_OFF);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s, rc =%d\n",
				__func__, rc);
		}
fail_i2c:
		rc = bahama_bt(0);
		if (rc < 0)
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
fail_xo_vote:
		pr_debug("%s: Voting off the 19.2MHz clk\n", __func__);
		msm_xo_put(bt_clock);
fail_power:
		pr_debug("%s: Switching off voltage regulators.\n", __func__);
		rc = bluetooth_switch_regulators(0);
		if (rc < 0) {
			pr_err("%s: switch_regulators : rc = %d",\
				__func__, rc);
			goto exit;
		}
		pr_debug("%s: BT Power Off complete.\n", __func__);
	}
	return rc;
exit:
	pr_err("%s: failed with rc = %d", __func__, rc);
	return rc;
}

static struct marimba_platform_data marimba_pdata = {
	.slave_id[SLAVE_ID_BAHAMA_FM]	= BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST] = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.bahama_setup			= msm_bahama_setup_power,
	.bahama_shutdown		= msm_bahama_shutdown_power,
	.bahama_core_config		= msm_bahama_core_config,
	.fm			        = NULL,
};

static struct i2c_board_info bahama_devices[] = {
{
	I2C_BOARD_INFO("marimba", 0xc),
	.platform_data = &marimba_pdata,
},
};
#endif

struct regulator *vreg_s4;
struct regulator *vreg_gpio_8;

static int bt_vreg_disable(struct regulator *vreg)
{
	int rc;

	rc = regulator_disable(vreg);
	if (rc < 0) {
		pr_err("%s: Failed to disable reg. %p with %d\n", __func__, vreg
			, rc);
		return rc;
	}
	regulator_put(vreg);
	vreg = NULL;
	return rc;
}

static int atheros_bluetooth_power(int on)
{
	int rc = 0;

	if (on) {
		printk(KERN_INFO "%s: Powering up BT module on On AR3002\n",
			__func__);
		/* Voting for 1.8V s4 regulator */
		pr_debug("%s: Voting for the 1.8V s4 regulator\n", __func__);
		vreg_s4 = regulator_get(&msm_bluesleep_device.dev, "8921_l8");
		if (IS_ERR(vreg_s4)) {
			rc = PTR_ERR(vreg_s4);
			pr_err("%s: Failed to get s4 regulator: %d\n", __func__,
				rc);
			goto out;
		}
		if (regulator_count_voltages(vreg_s4) > 0) {
			pr_debug("%s: Setting volt. levels for s4 regulator\n",
				 __func__);
			rc = regulator_set_voltage(vreg_s4, 1800000, 1800000);
			if (rc) {
				pr_err("%s: Failed to set volt. for s4 regulator: %d\n",
					__func__, rc);
				goto free_vreg_s4;
			}
			pr_debug("%s: Enabling the s4 regulator\n", __func__);
			rc = regulator_enable(vreg_s4);
			if (rc) {
				pr_err("%s: Failed to enable s4 regulator : %d\n",
					__func__, rc);
				goto free_vreg_s4;
			}
		}

		/* Voting for the ATH_CHIP_PWD_L GPIO line */
		printk(KERN_INFO "%s: Voting On ath_pwd_l gpio-regulator\n",
			__func__);
		vreg_gpio_8 = regulator_get(&msm_bluesleep_device.dev, "bt_en");
		if (IS_ERR(vreg_gpio_8)) {
			rc = PTR_ERR(vreg_gpio_8);
			pr_err("%s: Failed to vote ath_pwd_l gpio-regulator: %d\n",
				__func__, rc);
			goto free_vreg_s4;
		}
		pr_debug("%s: Enabling ath_pwd_l gpio-regulator\n", __func__);
		rc = regulator_enable(vreg_gpio_8);
		if (rc) {
			pr_err("%s: Failed to enable ath_pwd_l gpio-regulator: %d\n",
				 __func__, rc);
			goto free_vreg_gpio_8;
		}
		pr_debug("%s: Brining BT out of reset\n", __func__);
		pr_debug("%s: Configuring BT_SYS_RST_EN GPIO%d\n", __func__,
			gpio_bt_sys_reset_en);
		rc = gpio_request(gpio_bt_sys_reset_en, "bt sys_rst_n");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
				__func__, gpio_bt_sys_reset_en, rc);
			goto free_vreg_gpio_8;
		}
		rc = gpio_direction_output(gpio_bt_sys_reset_en, 0);
		if (rc) {
			pr_err("%s: Unable to set direction\n", __func__);
			goto free_gpio;
		}
		msleep(100);
		gpio_direction_output(gpio_bt_sys_reset_en, 1);
		msleep(100);
		/*setup BT PCM lines*/
		pr_debug("%s: Configuring PCM lines.\n", __func__);
		rc = config_pcm(BT_PCM_ON);
		if (rc < 0) {
			pr_err("%s: config_pcm , rc = %d\n", __func__, rc);
			goto free_gpio;
		}
		goto out;
	} else {
		printk(KERN_INFO "%s: Powering down BT module on AR3002\n",
			__func__);
		gpio_set_value(gpio_bt_sys_reset_en, 0);
		rc = gpio_direction_input(gpio_bt_sys_reset_en);
		msleep(100);
		pr_debug("%s: Releasing the PCM lines.\n", __func__);
		rc = config_pcm(BT_PCM_OFF);
		if (rc < 0)
			pr_err("%s: Disabling PCM lines failed with rc =%d\n",
				__func__, rc);
	}
free_gpio:
	gpio_free(gpio_bt_sys_reset_en);
free_vreg_gpio_8:
	printk(KERN_INFO "%s: Voting off ath_pwd_l gpio-regulator\n", __func__);
	bt_vreg_disable(vreg_gpio_8);
free_vreg_s4:
	bt_vreg_disable(vreg_s4);
out:
	return rc;
}

int bluetooth_power(int on)
{
	int rc = 0;

#if defined(CONFIG_MARIMBA_CORE)
	int conn_chip = 0;

	conn_chip = adie_get_detected_connectivity_type();
	if (conn_chip == BAHAMA_ID) {
		printk(KERN_INFO "%s: Chip type is WCN2243\n", __func__);
		rc = bahama_bluetooth_power(on);
		if (rc < 0) {
			if (on)
				pr_err("%s: BT power on failed with ret : %d\n",
					 __func__, rc);
			else
				pr_err("%s: BT power off failed with ret: %d\n",
					 __func__, rc);
		} else {
			if (on)
				pr_debug("%s: BT powered on\n", __func__);
			else
				pr_debug("%s: BT powered off\n", __func__);
		}
	} else
#endif
	{
		printk(KERN_INFO "%s: Chip type is AR3002\n", __func__);
		rc = atheros_bluetooth_power(on);
		if (rc < 0) {
			if (on)
				pr_err("%s: BT power on failed with ret : %d\n",
					 __func__, rc);
			else
				pr_err("%s: BT power off failed with ret: %d\n",
					 __func__, rc);
		} else {
			if (on)
				printk(KERN_INFO
					"%s: BT Powered ON", __func__);
			else
				printk(KERN_INFO
					"%s: BT Powered Off", __func__);
		}
	}
	return rc;
}

void __init apq8064_bt_power_init(void)
{
#if defined(CONFIG_MARIMBA_CORE)
	int rc = 0;
	uint32_t hrd_version = socinfo_get_version();
#endif
	struct device *dev;

	dev = &msm_bt_power_device.dev;
	dev->platform_data = bluetooth_power;

#if defined(CONFIG_MARIMBA_CORE)
	pr_debug("%s: Registering WCN2243 specific information\n", __func__);
	/* I2C registration for WCN2243 */
	if (machine_is_mpq8064_hrd()
		&& (SOCINFO_VERSION_MAJOR(hrd_version) == 2)) {
		rc = i2c_register_board_info(
				APQ_8064_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
	} else {
		rc = i2c_register_board_info(
				APQ_8064_GSBI5_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
	}
	if (rc < 0)
		pr_err("%s: I2C Register failed\n", __func__);
	else
		pr_debug("%s: I2C Registration successful\n", __func__);
#endif

	if (platform_device_register(&msm_bt_power_device) < 0)
		pr_err("%s: Platform dev. registration failed\n", __func__);
	else
		pr_debug("%s: Platform dev. registration success\n", __func__);

#if defined CONFIG_BT_HCIUART_ATH3K
	pr_debug("%s: Registering AR3002 specific information\n", __func__);
	if (machine_is_apq8064_dma() || machine_is_apq8064_bueller()) {
		printk(KERN_ERR "gpio_bt_sys_reset_en=%d\n", QCA6234_BT_RST_N);
		gpio_bt_sys_reset_en = QCA6234_BT_RST_N;
	} else
		gpio_bt_sys_reset_en = PM8921_MPP_PM_TO_SYS(10);
	if (platform_device_register(&msm_bluesleep_device) < 0)
		pr_err("%s: Bluesleep registration failed\n", __func__);
	else
		pr_debug("%s: Bluesleep registration success\n", __func__);
#endif
	return;
}
#endif
