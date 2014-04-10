/*
 *  drivers/leds/leds-bueller.c
 *
 *  Author: Andrew Price <andprice@lab126.com>
 *  Description: Bueller LED driver.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/string.h>

#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/pwm.h>

/* Uncomment to enable changing the LUT for breathing from sysfs and to display
	debugging print statements in driver */
//#define DEBUG

#ifdef DEBUG
#define dev_debug(dev, format, ...)	\
do { dev_err(dev, format, ##__VA_ARGS__); } while(0)
#else
#define dev_debug(dev, format, ...)	\
do { } while(0)
#endif

#define WHITE_LED_ID			0
#define AMBER_LED_ID			1
#define LPG_AMBER				7
#define LPG_WHITE				5

enum led_delay {
	LED_DELAY_OFF,
	LED_DELAY_ON,
};

struct bueller_led_data {
	struct led_classdev cdev;
	u8 id;
	u8 is_breathing;
	u8 need_lut_update;
	struct device *dev;
	struct work_struct work;
	struct pwm_device	*pwm_dev;
	int pwm_channel;
	u32 pwm_period_us;
	struct pm8xxx_pwm_duty_cycles pwm_duty_cycles;
#ifdef DEBUG
	int pwm_pause_lo;
	int pwm_pause_hi;
#endif
};

/**************************************
 * LED Class function implementations *
 **************************************/
static void bueller_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct bueller_led_data *led = container_of(cdev, struct bueller_led_data, cdev);

	led->cdev.brightness = value;
	schedule_work(&led->work);
}

/***********************************
 * Bueller LED device descriptions *
 ***********************************/
#define PWM_DUTY_MS_WHITE				30
#define PWM_DUTY_MS_AMBER				7
#define PWM_PERIOD_US					1024
#define PWM_LED_FLAGS					( PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_REVERSE)
#define PWM_DUTY_CYCLE_STEP_SIZE		4
/*
 * Note: There is a bug in LPG module that results in incorrect
 * behavior of pattern when LUT index 0 is used. So effectively
 * there are 63 usable LUT entries.
 */
#define PWM_DUTY_PCTS_USED_DEFAULT	63
#define PWM_DUTY_PCTS_MAX_VALS		63

/* tables changed to allow for breathing from a brightness of ~10% to 100% */
static int led_pwm_duty_pcts_white[PWM_DUTY_PCTS_MAX_VALS] = {
	4, 4, 4, 4, 4, 4, 4, 4,
	4, 5, 5, 5, 5, 5, 6, 6,
	7, 7, 7, 8, 9, 9, 10, 11,
	11, 12, 13, 14, 15, 16, 18, 19,
	20, 22, 24, 25, 27, 30, 32, 34,
	36, 39, 42, 45, 47, 50, 53, 56,
	59, 63, 66, 69, 72, 75, 78, 81,
	84, 87, 89, 92, 95, 97, 100,
};
static int led_pwm_duty_pcts_amber[PWM_DUTY_PCTS_MAX_VALS] = {
	4, 4, 4, 4, 4, 4, 4, 4,
	4, 5, 5, 5, 5, 5, 6, 6,
	7, 7, 7, 8, 9, 9, 10, 11,
	11, 12, 13, 14, 15, 16, 18, 19,
	20, 22, 24, 25, 27, 30, 32, 34,
	36, 39, 42, 45, 47, 50, 53, 56,
	59, 63, 66, 69, 72, 75, 78, 81,
	84, 87, 89, 92, 95, 97, 100,
};

static struct bueller_led_data led_white = {
	.cdev = {
		.name = "bueller-status",
		.brightness_set = bueller_brightness_set,
		.max_brightness = LED_FULL,
	},
	.id = WHITE_LED_ID,
	.is_breathing = 0,
	.need_lut_update = 0,
	.dev = NULL,
	.pwm_dev = NULL,
	.pwm_channel = LPG_WHITE,
	.pwm_period_us = PWM_PERIOD_US,
	.pwm_duty_cycles = {
		.duty_pcts = (int *)&led_pwm_duty_pcts_white,
		.num_duty_pcts = PWM_DUTY_PCTS_USED_DEFAULT,
		.duty_ms = PWM_DUTY_MS_WHITE,
		.start_idx = 1,
	},
#ifdef DEBUG
	.pwm_pause_lo = 0,
	.pwm_pause_hi = 0,
#endif
};

static struct bueller_led_data led_amber = {
	.cdev = {
		.name = "bueller-error",
		.brightness_set = bueller_brightness_set,
		.max_brightness = LED_FULL,
	},
	.id = AMBER_LED_ID,
	.is_breathing = 0,
	.need_lut_update = 0,
	.dev = NULL,
	.pwm_dev = NULL,
	.pwm_channel = LPG_AMBER,
	.pwm_period_us = PWM_PERIOD_US,
	.pwm_duty_cycles = {
		.duty_pcts = (int *)&led_pwm_duty_pcts_amber,
		.num_duty_pcts = PWM_DUTY_PCTS_USED_DEFAULT,
		.duty_ms = PWM_DUTY_MS_AMBER,
		.start_idx = 1,
	},
#ifdef DEBUG
	.pwm_pause_lo = 0,
	.pwm_pause_hi = 0,
#endif
};

/*************************************************
 * QC specific functions to control LED hardware *
 *************************************************/
#define REG_ADDR_DRV_VIB			0x004A
#define REG_ADDR_LED_CNTRL_0		0x0131

#define LED_CNTRL_MAX_DRV			0x20 /* 8 mA */
#define LED_CNTRL_PWM_2				0x02 /* Driven by PWM 2 (LPG 5) */
#define LED_CNTRL_MIN_DRV			0x00
#define DRV_VIB_MAX_DRV				0x68 /* 1.3 V */
#define DRV_VIB_DTEST_3				0x03 /* Driven by DTEST3 (LPG 7) */
#define DRV_VIB_MIN_DRV				0x00
static int init_pmic_resources(void)
{
	int ret = 0;

	/* dev_debug(led_white.dev, "%s: configuring white LED current sink.\n", __func__);*/
	/* Initially set the pins to off and manual mode */
	/*ret = pm8xxx_writeb(led_white.dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MIN_DRV);
	if (ret) {
		dev_err(led_white.dev, "pm8xxx_writeb() failed, ret=%d.\n", ret);
		return ret;
	}*/
	/* white led is set to breathing state in lk as required to indicate boot status*/
	/* we are here, so boot is fine till this point and we will inform kernel here */
	led_white.is_breathing = 1;
	led_white.need_lut_update = 1;

	dev_debug(led_amber.dev, "%s: configuring amber LED vib motor.\n", __func__);
	ret = pm8xxx_writeb(led_amber.dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MIN_DRV);
	if (ret) {
		dev_err(led_amber.dev, "pm8xxx_writeb() failed, ret=%d.\n", ret);
		return ret;
	}

	/* Request PWMs to drive LED pins */
	dev_debug(led_white.dev, "%s: requesting PWMs.\n", __func__);

	led_white.pwm_dev = pwm_request(LPG_WHITE, "led-status-pwm");
	if (IS_ERR_OR_NULL(led_white.pwm_dev)) {
		dev_err(led_white.dev, "could not acquire PWM Channel %d\n", LPG_WHITE);
		led_white.pwm_dev = NULL;
		return -ENODEV;
	}

	led_amber.pwm_dev = pwm_request(LPG_AMBER, "led-error-pwm");
	if (IS_ERR_OR_NULL(led_amber.pwm_dev)) {
		dev_err(led_amber.dev, "could not acquire PWM Channel %d\n", LPG_AMBER);
		led_amber.pwm_dev = NULL;
		return -ENODEV;
	}

	return 0;
}

static void free_pmic_resources(void)
{
	dev_debug(led_white.dev, "%s: freeing resources.\n", __func__);

	/* Set the pins to off manual mode */
	pm8xxx_writeb(led_white.dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MIN_DRV);
	pm8xxx_writeb(led_amber.dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MIN_DRV);

	/* Release the PWM resources */
	pwm_free(led_white.pwm_dev);
	pwm_free(led_amber.pwm_dev);
}

static void bueller_led_work(struct work_struct *work)
{
	struct bueller_led_data *led = container_of(work, struct bueller_led_data, work);

#ifdef DEBUG
	int pwm_flags = PWM_LED_FLAGS;
	if (led->pwm_pause_lo != 0)
		pwm_flags |= PM_PWM_LUT_PAUSE_LO_EN;
	if (led->pwm_pause_hi != 0)
		pwm_flags |= PM_PWM_LUT_PAUSE_HI_EN;
#endif
	/* Breathing takes precedence over all other activity */
	if (led->is_breathing)
	{
		/* Turn on the current sinks under PWM control */
		if (led->id == WHITE_LED_ID)
			pm8xxx_writeb(led->dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MAX_DRV | LED_CNTRL_PWM_2);
		else
			pm8xxx_writeb(led->dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MAX_DRV | DRV_VIB_DTEST_3);

		/* Enable PWM with LUT */
		if (led->need_lut_update) {
			dev_debug(led_white.dev, "%s: configuring LUTs for breathing.\n", __func__);
			pwm_disable(led->pwm_dev);
			pm8xxx_pwm_lut_enable(led->pwm_dev, 0);
			pm8xxx_pwm_lut_config ( led->pwm_dev,
				led->pwm_period_us,
				led->pwm_duty_cycles.duty_pcts,
				led->pwm_duty_cycles.duty_ms,
				led->pwm_duty_cycles.start_idx,
				led->pwm_duty_cycles.num_duty_pcts,
#ifdef DEBUG
				led->pwm_pause_lo,
				led->pwm_pause_hi,
				pwm_flags );
#else
				0,
				0,
				PWM_LED_FLAGS );
#endif
		led->need_lut_update = 0;
		}
		pm8xxx_pwm_lut_enable(led->pwm_dev, 1);
		dev_debug(led->dev, "%s: LED is BREATHING.\n", __func__);
	}
	else
	{
		/* Disable PWM and LUT */
		pwm_disable(led->pwm_dev);
		pm8xxx_pwm_lut_enable(led->pwm_dev, 0);

		if (led->cdev.brightness == LED_OFF) {
			/* Turn off the current sink */
			if (led->id == WHITE_LED_ID)
				pm8xxx_writeb(led->dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MIN_DRV);
			else
				pm8xxx_writeb(led->dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MIN_DRV);

			dev_debug(led->dev, "%s: LED is OFF.\n", __func__);
		} else if (led->cdev.brightness == LED_FULL) {
			/* Turn on the current sinks to max */
			if (led->id == WHITE_LED_ID)
				pm8xxx_writeb(led->dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MAX_DRV);
			else
				pm8xxx_writeb(led->dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MAX_DRV);

			dev_debug(led->dev, "%s: LED is ON FULL.\n", __func__);
		} else {
			/* Turn on the current sinks under PWM control */
			if (led->id == WHITE_LED_ID)
				pm8xxx_writeb(led->dev->parent, REG_ADDR_LED_CNTRL_0, LED_CNTRL_MAX_DRV | LED_CNTRL_PWM_2);
			else
				pm8xxx_writeb(led->dev->parent, REG_ADDR_DRV_VIB, DRV_VIB_MAX_DRV | DRV_VIB_DTEST_3);

			/* We want an intermediate brightness so turn on PWM */
			pwm_config(led->pwm_dev,
				led->cdev.brightness*PWM_DUTY_CYCLE_STEP_SIZE,
				led->pwm_period_us );
			pwm_enable(led->pwm_dev);

			dev_debug(led->dev, "%s: LED is ON %d.\n", __func__, led->cdev.brightness);
		}
	}
}

/****************************************
 * Functions to store driver attributes *
 ****************************************/
static inline ssize_t led_store_delay(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t size, enum led_delay delay)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (delay == LED_DELAY_OFF) {
			led_blink_set(led_cdev, &led_cdev->blink_delay_on, &state);
		}
		else if (delay == LED_DELAY_ON) {
			led_blink_set(led_cdev, &state, &led_cdev->blink_delay_off);
		}
	}
	dev_debug(dev, "%s: on_time = %lu, off_time = %lu.\n", __func__,
		led_cdev->blink_delay_on, led_cdev->blink_delay_off);

	return ret;
}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return led_store_delay(dev, attr, buf, size, LED_DELAY_ON);
}

static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return led_store_delay(dev, attr, buf, size, LED_DELAY_OFF);
}

static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_on);
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", led_cdev->blink_delay_off);
}

static ssize_t led_breathe_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (state == 0) {
			led->is_breathing = 0;
			led->need_lut_update = 0;
		} else {
			if (led->is_breathing == 0)
				led->need_lut_update = 1;
			led->is_breathing = 1;
		}
	}
	schedule_work(&led->work);

	return ret;
}

static ssize_t led_breathe_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);

	return sprintf(buf, "%d\n", led->is_breathing);
}
#ifdef DEBUG
/* The LUT is a series of comma delimitted values between 0 and 100 inclusive.
   The max LUT size is 63 entries. Size is calculated automatically. */
static ssize_t led_lut_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	ssize_t ret = -EINVAL;
	char *temp_str;
	int *array = NULL, num = 0, vals = 0;

	if (led->id == WHITE_LED_ID)
		array = &led_pwm_duty_pcts_white[0];
	else
		array = &led_pwm_duty_pcts_amber[0];

	temp_str = strsep((char **)&buf, ",");
	while (temp_str != NULL) {
		num = simple_strtoul(temp_str, NULL, 10);
		if (num > 100) {
			dev_debug(dev, "LUT entry must be a percentage between 0 and 100.\n");
			num = 100;
		}
		*array = num;
		array++;
		vals++;
		temp_str = strsep((char **)&buf, ",");
		if (vals >= PWM_DUTY_PCTS_MAX_VALS)
			temp_str = NULL;
	}
	led->pwm_duty_cycles.num_duty_pcts = vals;
	schedule_work(&led->work);

	return ret;
}

static ssize_t led_lut_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	int i = 0, size = 0;

	size = sprintf(buf, "LUT Size:%d\nLUT:\n", led->pwm_duty_cycles.num_duty_pcts);
	for (i = 0; i < led->pwm_duty_cycles.num_duty_pcts; i++) {
		size += sprintf(&buf[size], "%d,", led->pwm_duty_cycles.duty_pcts[i]);
		if ((i+1) % 10 == 0)
			size += sprintf(&buf[size], "\n");
	}
	size += sprintf(&buf[size], "\n");

	return size;
}

static ssize_t led_lut_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long time = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		led->pwm_duty_cycles.duty_ms = time;
	}
	schedule_work(&led->work);

	return ret;
}

static ssize_t led_lut_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);

	return sprintf(buf, "%d\n", led->pwm_duty_cycles.duty_ms);
}

static ssize_t led_pause_lo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long time = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		led->pwm_pause_lo = time;
	}
	schedule_work(&led->work);

	return ret;
}

static ssize_t led_pause_lo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);

	return sprintf(buf, "%d\n", led->pwm_pause_lo);
}

static ssize_t led_pause_hi_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long time = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		led->pwm_pause_hi = time;
	}
	schedule_work(&led->work);

	return ret;
}

static ssize_t led_pause_hi_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct bueller_led_data *led = container_of(led_cdev, struct bueller_led_data, cdev);

	return sprintf(buf, "%d\n", led->pwm_pause_hi);
}
#endif

static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);
static DEVICE_ATTR(breathe, 0644, led_breathe_show, led_breathe_store);
#ifdef DEBUG
static DEVICE_ATTR(lut, 0644, led_lut_show, led_lut_store);
static DEVICE_ATTR(lut_time, 0644, led_lut_time_show, led_lut_time_store);
static DEVICE_ATTR(pause_lo, 0644, led_pause_lo_show, led_pause_lo_store);
static DEVICE_ATTR(pause_hi, 0644, led_pause_hi_show, led_pause_hi_store);
#endif
/*****************************************
 * Bueller LED platform driver functions *
 *****************************************/
static int __devinit bueller_led_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_debug(&pdev->dev, "%s: Bueller LED probe start.\n", __func__);
	/* WHITE LED */
	led_white.dev = &pdev->dev;
	INIT_WORK(&(led_white.work), bueller_led_work);

	/* Register the device. */
	ret = led_classdev_register(&pdev->dev, &(led_white.cdev));
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register bueller-status LED (ret=%d).\n", ret);
		goto failwhiteregister;
	}

	/* Create the device attribute files for WHITE LED */
	ret = device_create_file(led_white.cdev.dev, &dev_attr_delay_on);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitedelayon;
	}
	ret = device_create_file(led_white.cdev.dev, &dev_attr_delay_off);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitedelayoff;
	}
	ret = device_create_file(led_white.cdev.dev, &dev_attr_breathe);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitebreathe;
	}
#ifdef DEBUG
	ret = device_create_file(led_white.cdev.dev, &dev_attr_lut);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitelut;
	}
	ret = device_create_file(led_white.cdev.dev, &dev_attr_lut_time);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhiteluttime;
	}
	ret = device_create_file(led_white.cdev.dev, &dev_attr_pause_lo);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitepauselo;
	}
	ret = device_create_file(led_white.cdev.dev, &dev_attr_pause_hi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failwhitepausehi;
	}
#endif
	/* AMBER LED */
	led_amber.dev = &pdev->dev;
	INIT_WORK(&(led_amber.work), bueller_led_work);

	/* Register the device. */
	ret = led_classdev_register(&pdev->dev, &(led_amber.cdev));
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register bueller-error LED (ret=%d).\n", ret);
		goto failamberregister;
	}

	/* Create the device attribute files for AMBER LED */
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_delay_on);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberdelayon;
	}
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_delay_off);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberdelayoff;
	}
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_breathe);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberbreathe;
	}
#ifdef DEBUG
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_lut);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberlut;
	}
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_lut_time);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberluttime;
	}
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_pause_lo);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberpauselo;
	}
	ret = device_create_file(led_amber.cdev.dev, &dev_attr_pause_hi);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create attribute file (ret=%d).\n", ret);
		goto failamberpausehi;
	}
#endif
	/* Initialize PMIC resources (current sink and PWM) */
	ret = init_pmic_resources();
	if (ret < 0)  {
		dev_err(&pdev->dev, "failed to obtain pmic resources (ret=%d).\n", ret);
		goto failpmicinit;
	}
	dev_debug(&pdev->dev, "%s: Bueller LED probe finish success.\n", __func__);

	return ret;

failpmicinit:
#ifdef DEBUG
	device_remove_file(led_amber.cdev.dev, &dev_attr_pause_hi);
failamberpausehi:
	device_remove_file(led_amber.cdev.dev, &dev_attr_pause_lo);
failamberpauselo:
	device_remove_file(led_amber.cdev.dev, &dev_attr_lut_time);
failamberluttime:
	device_remove_file(led_amber.cdev.dev, &dev_attr_lut);
failamberlut:
	device_remove_file(led_amber.cdev.dev, &dev_attr_breathe);
#else
	device_remove_file(led_amber.cdev.dev, &dev_attr_breathe);
#endif
failamberbreathe:
	device_remove_file(led_amber.cdev.dev, &dev_attr_delay_off);
failamberdelayoff:
	device_remove_file(led_amber.cdev.dev, &dev_attr_delay_on);
failamberdelayon:
	led_classdev_unregister(&(led_amber.cdev));
failamberregister:
#ifdef DEBUG
	device_remove_file(led_white.cdev.dev, &dev_attr_pause_hi);
failwhitepausehi:
	device_remove_file(led_white.cdev.dev, &dev_attr_pause_lo);
failwhitepauselo:
	device_remove_file(led_white.cdev.dev, &dev_attr_lut_time);
failwhiteluttime:
	device_remove_file(led_white.cdev.dev, &dev_attr_lut);
failwhitelut:
	device_remove_file(led_white.cdev.dev, &dev_attr_breathe);
#else
	device_remove_file(led_white.cdev.dev, &dev_attr_breathe);
#endif
failwhitebreathe:
	device_remove_file(led_white.cdev.dev, &dev_attr_delay_off);
failwhitedelayoff:
	device_remove_file(led_white.cdev.dev, &dev_attr_delay_on);
failwhitedelayon:
	led_classdev_unregister(&(led_white.cdev));
failwhiteregister:
	dev_debug(&pdev->dev, "%s: Bueller LED probe finish error.\n", __func__);

	return ret;
}

static int __devexit bueller_led_remove(struct platform_device *pdev)
{
	dev_debug(&pdev->dev, "%s: Bueller LED remove.\n", __func__);
	cancel_work_sync(&(led_white.work));
	cancel_work_sync(&(led_amber.work));
	free_pmic_resources();
	led_classdev_unregister(&(led_white.cdev));
	led_classdev_unregister(&(led_amber.cdev));
	return 0;
}

static struct platform_driver bueller_led_driver = {
	.probe = bueller_led_probe,
	.remove = __devexit_p(bueller_led_remove),
	.driver = {
		.name = "bueller-leds",
		.owner = THIS_MODULE,
	},
};

static int __init bueller_led_init(void)
{
	return platform_driver_register(&bueller_led_driver);
}

static void __exit bueller_led_exit(void)
{
	platform_driver_unregister(&bueller_led_driver);
}

module_init(bueller_led_init);
module_exit(bueller_led_exit);

MODULE_ALIAS("platform:bueller-leds");
MODULE_AUTHOR("Andrew Price <andprice@lab126.com>");
MODULE_DESCRIPTION("Bueller LED driver");
MODULE_LICENSE("GPL");
