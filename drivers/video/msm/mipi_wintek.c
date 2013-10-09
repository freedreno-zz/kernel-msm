
#include <mach/gpio.h>
#include <mach/irqs.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_wintek.h"
#include "mdp4.h"

#define BACKLIGHT_MAX   255
#define BACKLIGHT_GPIO  (24 - 1 + NR_GPIO_IRQS)

static struct dsi_buf wintek_tx_buf;
static struct dsi_buf wintek_rx_buf;
static int mipi_wintek_lcd_init(void);


/* DTYPE_DCS_WRITE */
static char sw_reset[2]	   = {0x01, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char exit_sleep[2]  = {0x11, 0x00};
static char display_off[2] = {0x28, 0x00};
static char display_on[2]  = {0x29, 0x00};
static char mem_write[2]   = {0x2c, 0x00};


/* DTYPE_DCS_WRITE1 */
static char rgb_888[2]		= {0x3A, 0x77};
static char set_num_of_lanes[2] = {0xae, 0x03};
static char mem_access_ctrl[2]  = {0x36, 0xc8};
static char display_inv[2]	= {0xb4, 0x00};
static char tearing_on[2]	= {0x35, 0x00};
static char pwr_seq[2]		= {0xfa, 0x04};
static char dither[2]		= {0xea, 0x00};
#if CONFIG_FB_MSM_MIPI_WINTEK_VIDEO_WVGA_BL_LEVELS > 1
static char brightness_val[2]   = {0x51, 0xf0};
static char brightness_ctl[2]   = {0x53, 0x24};
#endif

/* DTYPE_DCS_LWRITE */
static char power_ctrl1[4]	= {0xc1, 0x15, 0x56, 0x16};
static char display_ctrl[5]	= {0xb6, 0x02, 0xe2, 0xff, 0x04};
static char frame_ctrl[4]	= {0xb1, 0x00, 0x10, 0x1a};
static char vcom_ctrl[3]	= {0xc5, 0x00, 0x48};
static char gamma_pos[17]	= {0xe0, 0x00, 0x07, 0x0d, 0x10, 0x13,
0x19, 0x0f, 0x0c, 0x05, 0x08, 0x06, 0x13, 0x0f, 0x30, 0x20, 0x1f};
static char gamma_neg[17]	= {0xe1, 0x1f, 0x20, 0x30, 0x00, 0x2d,
0x06, 0x08, 0x15, 0x0c, 0x0f, 0x19, 0x13, 0x10, 0x0d, 0x07, 0x00};



static char vert_scr_defl[7] = {0x33, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00};
static char vert_scrl_start[3] = {0x37, 0x00, 0x00};



static struct dsi_cmd_desc wintek_video_on_cmds[] = {
{DTYPE_DCS_WRITE,  1, 0, 0, 50,  sizeof(sw_reset),	   sw_reset},
{DTYPE_DCS_WRITE,  1, 0, 0, 10,  sizeof(display_off),	   display_off},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(tearing_on),	   tearing_on},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(mem_access_ctrl),  mem_access_ctrl},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(pwr_seq),  	   pwr_seq},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(power_ctrl1),	   power_ctrl1},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(display_ctrl),	   display_ctrl},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(frame_ctrl),	   frame_ctrl},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(display_inv),	   display_inv},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(vcom_ctrl),	   vcom_ctrl},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(gamma_pos),	   gamma_pos},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(gamma_neg),	   gamma_neg},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(dither),  	   dither},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(set_num_of_lanes), set_num_of_lanes},
{DTYPE_DCS_WRITE1, 1, 0, 0, 10,  sizeof(rgb_888),	   rgb_888},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(vert_scr_defl),    vert_scr_defl},
{DTYPE_DCS_LWRITE, 1, 0, 0, 10,  sizeof(vert_scrl_start),  vert_scrl_start},
{DTYPE_DCS_WRITE,  1, 0, 0, 120, sizeof(exit_sleep),	   exit_sleep},
{DTYPE_DCS_WRITE,  1, 0, 0, 10,  sizeof(display_on),	   display_on},
#if CONFIG_FB_MSM_MIPI_WINTEK_VIDEO_WVGA_BL_LEVELS > 1
{DTYPE_DCS_WRITE1,  1, 0, 0, 10, sizeof(brightness_ctl),   brightness_ctl},
{DTYPE_DCS_WRITE1,  1, 0, 0, 10, sizeof(brightness_val),   brightness_val},
#endif
{DTYPE_DCS_WRITE,  1, 0, 0, 10,  sizeof(mem_write),	   mem_write},

};


static struct dsi_cmd_desc wintek_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,   sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  sizeof(enter_sleep), enter_sleep}
};


uint32 mipi_wintek_set_brightness(struct msm_fb_data_type *mfd,
	unsigned char level)
{
	char brightness[2] = {0x51, level};

	struct dsi_cmd_desc wintek_set_brightness_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0x11,  sizeof(brightness), brightness},
	};

	mipi_dsi_cmds_tx(mfd, &wintek_tx_buf, wintek_set_brightness_cmds,
	ARRAY_SIZE(wintek_set_brightness_cmds));
	return 0;
}


static int mipi_wintek_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	pinfo = &mfd->panel_info;
	mipi  = &mfd->panel_info.mipi;

	mipi_dsi_cmds_tx(mfd, &wintek_tx_buf, wintek_video_on_cmds,
					 ARRAY_SIZE(wintek_video_on_cmds));

	return 0;
}

static int mipi_wintek_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &wintek_tx_buf, wintek_display_off_cmds,
					 ARRAY_SIZE(wintek_display_off_cmds));
	return 0;
}



static void mipi_wintek_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	int lcd_level;

	mipi  = &mfd->panel_info.mipi;

	if (CONFIG_FB_MSM_MIPI_WINTEK_VIDEO_WVGA_BL_LEVELS == 1) {
		if (mfd->bl_level > 0)
			gpio_set_value_cansleep(BACKLIGHT_GPIO, 1);
		else
			gpio_set_value_cansleep(BACKLIGHT_GPIO, 0);
	} else {
	lcd_level = (BACKLIGHT_MAX / mfd->panel_info.bl_max) * mfd->bl_level;

		mutex_lock(&mfd->dma->ov_mutex);
		mdp4_dsi_cmd_dma_busy_wait(mfd);
		mdp4_dsi_blt_dmap_busy_wait(mfd);
		mipi_dsi_mdp_busy_wait(mfd);
		mipi_wintek_set_brightness(mfd, lcd_level);
		mutex_unlock(&mfd->dma->ov_mutex);
	}
	return;
}


static int __devinit mipi_wintek_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;

		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;
	}
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_wintek_lcd_probe,
	.driver = {
		.name   = "mipi_wintek",
	},
};

static struct msm_fb_panel_data wintek_panel_data = {
	.on		= mipi_wintek_lcd_on,
	.off		= mipi_wintek_lcd_off,
	.set_backlight = mipi_wintek_set_backlight,
};


static int ch_used[3];

int mipi_wintek_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_wintek_lcd_init();
	if (ret) {
		pr_err("mipi_wintek_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_wintek", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	wintek_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &wintek_panel_data,
		sizeof(wintek_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}
	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_wintek_lcd_init(void)
{
	mipi_dsi_buf_alloc(&wintek_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&wintek_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
