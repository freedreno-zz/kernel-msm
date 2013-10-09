/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_wintek.h"


static struct msm_panel_info pinfo;


struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {

/* DSI_BIT_CLK at 500MHz, 2 lane, RGB888 */

	/* regulator */
	{0x0F, 0x0a, 0x04, 0x00, 0x20},

	/* timing   */
	{
		0x60,   /* DSI1_DSIPHY_TIMING_CTRL_0 */
		0x29,   /* DSI1_DSIPHY_TIMING_CTRL_1 */
		0x0c,   /* DSI1_DSIPHY_TIMING_CTRL_2 */
		0x00,   /* DSI1_DSIPHY_TIMING_CTRL_3 */
		0x35,   /* DSI1_DSIPHY_TIMING_CTRL_4 */
		0x3f,   /* DSI1_DSIPHY_TIMING_CTRL_5 */
		0x11,   /* DSI1_DSIPHY_TIMING_CTRL_6 */
		0x2d,   /* DSI1_DSIPHY_TIMING_CTRL_7 */
		0x16,   /* DSI1_DSIPHY_TIMING_CTRL_8 */
		0x03,   /* DSI1_DSIPHY_TIMING_CTRL_9 */
		0x04,   /* DSI1_DSIPHY_TIMING_CTRL_10 */
		0x00	/* DSI1_DSIPHY_TIMING_CTRL_11 */
	},

	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},

	/* strength */
	{
		0xff,   /* DSI1_DSIPHY_STRENGTH_CTRL_0 */
		0x00,   /* DSI1_DSIPHY_STRENGTH_CTRL_1 */
		0x06,   /* DSI1_DSIPHY_STRENGTH_CTRL_2 */
		0x00	/* Not used on msm8960 */
	},

	/* pll control */
	{
		0x01,   /* DSI_PHY_PLL_CTRL_0 */
		0x38,   /* DSI_PHY_PLL_CTRL_1 */
		0x31,   /* DSI_PHY_PLL_CTRL_2 */
		0xda,   /* DSI_PHY_PLL_CTRL_3 */
		0x00,   /* DSI_PHY_PLL_CTRL_4 (reserved) */
		0x40,   /* DSI_PHY_PLL_CTRL_5 */
		0x03,   /* DSI_PHY_PLL_CTRL_6 */
		0x62,   /* DSI_PHY_PLL_CTRL_7 */
		0x40,   /* DSI_PHY_PLL_CTRL_8 */
		0x07,   /* DSI_PHY_PLL_CTRL_9 */
		0x07,   /* DSI_PHY_PLL_CTRL_10 */
		0x00,   /* DSI_PHY_PLL_CTRL_11 */
		0x1a,   /* DSI_PHY_PLL_CTRL_12 */
		0x00,   /* DSI_PHY_PLL_CTRL_13 */
		0x00,   /* DSI_PHY_PLL_CTRL_14 */
		0x02,   /* DSI_PHY_PLL_CTRL_15 */
		0x00,   /* DSI_PHY_PLL_CTRL_16 */
		0x20,   /* DSI_PHY_PLL_CTRL_17 */
		0x00,   /* DSI_PHY_PLL_CTRL_18 */
		0x02,   /* DSI_PHY_PLL_CTRL_19 */
		0x00	/* Not used on msm8960 */
	},


};


static int __init mipi_video_wintek_wvga_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_video_wintek_wvga"))
		return 0;

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 16;
	pinfo.lcdc.h_front_porch = 20;
	pinfo.lcdc.h_pulse_width = 11;

	pinfo.lcdc.v_back_porch = 8;
	pinfo.lcdc.v_front_porch = 10;
	pinfo.lcdc.v_pulse_width = 8;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = CONFIG_FB_MSM_MIPI_WINTEK_VIDEO_WVGA_BL_LEVELS;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x16;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	ret = mipi_wintek_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_WXGA);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_wintek_wvga_init);
