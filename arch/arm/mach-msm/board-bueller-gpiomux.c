/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8064.h"

static struct gpiomux_setting audio_auxpcm[] = {
/* Suspended state */
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
/* Active state */
	{
		.func = GPIOMUX_FUNC_1,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
};

static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gsbi4_suspended_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi4_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi5_suspended_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi5_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi4_uart_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi7_spi_func1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting gsbi7_spi_func2_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi7_spi_func3_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};


static struct gpiomux_setting hsic_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hsic_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting hsic_wakeup_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hsic_wakeup_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config apq8064_hsic_configs[] = {
	{
		.gpio = 88,               /*HSIC_STROBE */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	{
		.gpio = 89,               /* HSIC_DATA */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	{
		.gpio = 47,              /* wake up */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_wakeup_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_wakeup_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config apq8064_hdmi_configs[] __initdata = {
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 72,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config mpq8064_audio_auxpcm_configs[] __initdata = {
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 44,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
};

static struct msm_gpiomux_config mpq8064_gsbi4_i2c_configs[] __initdata = {
	{
		.gpio      = 12,			/* GSBI4 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi4_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi4_active_cfg,
		},
	},
	{
		.gpio      = 13,			/* GSBI4 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi4_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi4_active_cfg,
		},
	},
};


static struct gpiomux_setting i2s_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting i2s_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting spdif_enable_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting dit4192_int_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting dit4192_ubit_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting dit4192_bls_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting dit4192_rst_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config bueller_dit4192_configs[] __initdata = {
	{
		.gpio	= 5,		/* spdif buffer enable */
		.settings = {
			[GPIOMUX_ACTIVE]    = &spdif_enable_cfg,
			[GPIOMUX_SUSPENDED] = &spdif_enable_cfg,
		},
	},
	{
		.gpio	= 27,		/* i2s ws */
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2s_act_cfg,
			[GPIOMUX_SUSPENDED] = &i2s_sus_cfg,
		},
	},
	{
		.gpio	= 28,		/* i2s sclk */
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2s_act_cfg,
			[GPIOMUX_SUSPENDED] = &i2s_sus_cfg,
		},
	},
	{
		.gpio	= 32,		/* i2s dout0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2s_act_cfg,
			[GPIOMUX_SUSPENDED] = &i2s_sus_cfg,
		},
	},
	{
		.gpio	= 33,		/* i2s mclk */
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2s_act_cfg,
			[GPIOMUX_SUSPENDED] = &i2s_sus_cfg,
		},
	},
	{
		.gpio	= 29,		/* buf flip int */
		.settings = {
			[GPIOMUX_ACTIVE]    = &dit4192_int_cfg,
			[GPIOMUX_SUSPENDED] = &dit4192_int_cfg,
		},
	},
	{
		.gpio	= 30,		/* ubit */
		.settings = {
			[GPIOMUX_ACTIVE]    = &dit4192_ubit_cfg,
			[GPIOMUX_SUSPENDED] = &dit4192_ubit_cfg,
		},
	},
	{
		.gpio	= 31,		/* bls */
		.settings = {
			[GPIOMUX_ACTIVE]    = &dit4192_bls_cfg,
			[GPIOMUX_SUSPENDED] = &dit4192_bls_cfg,
		},
	},
	{
		.gpio	= 35,		/* rst */
		.settings = {
			[GPIOMUX_ACTIVE]    = &dit4192_rst_cfg,
			[GPIOMUX_SUSPENDED] = &dit4192_rst_cfg,
		},
	},
};


static struct msm_gpiomux_config mpq8064_gsbi5_i2c_configs[] __initdata = {
	{
		.gpio      = 53,			/* GSBI5 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi5_active_cfg,
		},
	},
	{
		.gpio      = 54,			/* GSBI5 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi5_active_cfg,
		},
	},
};

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct gpiomux_setting sdc2_clk_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdc2_cmd_data_0_3_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdc2_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdc2_data_1_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config apq8064_sdc2_configs[] __initdata = {
	{
		.gpio      = 59,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_clk_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_suspended_cfg,
		},
	},
	{
		.gpio      = 57,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_suspended_cfg,
		},

	},
	{
		.gpio      = 62,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_suspended_cfg,
		},
	},
	{
		.gpio      = 61,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_data_1_suspended_cfg,
		},
	},
	{
		.gpio      = 60,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_suspended_cfg,
		},
	},
	{
		.gpio      = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc2_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc2_suspended_cfg,
		},
	},
};
#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct gpiomux_setting sdc4_clk_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdc4_cmd_data_0_3_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdc4_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdc4_data_1_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config apq8064_sdc4_configs[] __initdata = {
	{
		.gpio      = 68,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_clk_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspended_cfg,
		},
	},
	{
		.gpio      = 67,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspended_cfg,
		},

	},
	{
		.gpio      = 66,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspended_cfg,
		},
	},
	{
		.gpio      = 65,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_data_1_suspended_cfg,
		},
	},
	{
		.gpio      = 64,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspended_cfg,
		},
	},
	{
		.gpio      = 63,
		.settings = {
			[GPIOMUX_ACTIVE] = &sdc4_cmd_data_0_3_active_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspended_cfg,
		},
	},
};
#endif

static struct gpiomux_setting gsbi6_uartdm_active = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi6_uartdm_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting bt_host_wake_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting bt_ext_wake_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting ath_chip_pwd_l_suspended = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config mpq8064_uartdm_configs[] __initdata = {
	{ /* UARTDM_TX */
		.gpio      = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gsbi6_uartdm_active,
			[GPIOMUX_SUSPENDED] = &gsbi6_uartdm_suspended,
		},
	},
	{ /* UARTDM_RX */
		.gpio      = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gsbi6_uartdm_active,
			[GPIOMUX_SUSPENDED] = &gsbi6_uartdm_suspended,
		},
	},
	{ /* UARTDM_CTS */
		.gpio      = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gsbi6_uartdm_active,
			[GPIOMUX_SUSPENDED] = &gsbi6_uartdm_suspended,
		},
	},
	{ /* UARTDM_RFR */
		.gpio      = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gsbi6_uartdm_active,
			[GPIOMUX_SUSPENDED] = &gsbi6_uartdm_suspended,
		},
	},
};

static struct msm_gpiomux_config apq8064_bt_wifi_reset_configs[] __initdata = {
	{ /* ATH_CHIP_PWD_L */
		.gpio      = 8,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ath_chip_pwd_l_suspended,
		},
	}
};

static struct msm_gpiomux_config mpq8064_gsbi7_spi_configs[] __initdata = {
	{
		.gpio      = 85,        /* GSBI7_0 SPI CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi7_spi_func3_cfg,
		},
	},
	{
		.gpio      = 84,        /* GSBI7_1 SPI CS */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi7_spi_func2_cfg,
		},
	},
	{
		.gpio      = 83,        /* GSBI7_2 SPI MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi7_spi_func1_cfg,
		},
	},
	{
		.gpio      = 82,        /* GSBI7_3 SPI_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi7_spi_func2_cfg,
		},
	},
};

static struct msm_gpiomux_config mpq8064_gsbi4_uart_configs[] __initdata = {
	{
		.gpio      = 10,        /* GSBI4 UART TX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi4_uart_cfg,
		},
	},
	{
		.gpio      = 11,        /* GSBI4 UART RX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi4_uart_cfg,
		},
	},
};

static struct msm_gpiomux_config mpq8064_bluesleep_configs[] __initdata = {
	{
		.gpio      = 7,        /* HOST_WAKE */
		.settings = {
			[GPIOMUX_SUSPENDED] = &bt_host_wake_suspended,
		},
	},
	{
		.gpio      = 9,        /* EXT_WAKE */
		.settings = {
			[GPIOMUX_SUSPENDED] = &bt_ext_wake_suspended,
		},
	},
};

void __init apq8064_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return;
	}

	msm_gpiomux_install(mpq8064_gsbi4_i2c_configs,
		ARRAY_SIZE(mpq8064_gsbi4_i2c_configs));
	msm_gpiomux_install(mpq8064_gsbi5_i2c_configs,
		ARRAY_SIZE(mpq8064_gsbi5_i2c_configs));
	msm_gpiomux_install(mpq8064_gsbi4_uart_configs,
		ARRAY_SIZE(mpq8064_gsbi4_uart_configs));

	msm_gpiomux_install(mpq8064_audio_auxpcm_configs,
		ARRAY_SIZE(mpq8064_audio_auxpcm_configs));


	msm_gpiomux_install(bueller_dit4192_configs,
			ARRAY_SIZE(bueller_dit4192_configs));

	msm_gpiomux_install(apq8064_hdmi_configs,
			ARRAY_SIZE(apq8064_hdmi_configs));

	msm_gpiomux_install(mpq8064_gsbi7_spi_configs,
				ARRAY_SIZE(mpq8064_gsbi7_spi_configs));

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	 msm_gpiomux_install(apq8064_sdc2_configs,
			     ARRAY_SIZE(apq8064_sdc2_configs));
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	 msm_gpiomux_install(apq8064_sdc4_configs,
			     ARRAY_SIZE(apq8064_sdc4_configs));
#endif

	msm_gpiomux_install(mpq8064_uartdm_configs,
		ARRAY_SIZE(mpq8064_uartdm_configs));

	apq8064_hsic_configs[2].gpio = APQ8064_DMA_HSIC_WAKEUP_GPIO;
	msm_gpiomux_install(apq8064_hsic_configs,
				ARRAY_SIZE(apq8064_hsic_configs));
	msm_gpiomux_install(mpq8064_bluesleep_configs,
			ARRAY_SIZE(mpq8064_bluesleep_configs));
	msm_gpiomux_install(apq8064_bt_wifi_reset_configs,
			ARRAY_SIZE(apq8064_bt_wifi_reset_configs));
}
