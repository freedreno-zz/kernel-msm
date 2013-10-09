/*
 * drivers/input/touchscreen/sitronix_i2c_touch.c
 *
 * Touchscreen driver for Sitronix (I2C bus)
 *
 * Copyright (C) 2011 Sitronix Technology Co., Ltd.
 *	Rudy Huang <rudy_huang@sitronix.com.tw>
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifdef CONFIG_ARCH_SC8810
#include <linux/init.h>
#include <mach/ldo.h>
#include <mach/eic.h>
#endif // CONFIG_ARCH_SC8810
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif // CONFIG_HAS_EARLYSUSPEND
#include "sitronix_i2c_touch.h"
#ifdef SITRONIX_FW_UPGRADE_FEATURE
#include <linux/cdev.h>
#include <asm/uaccess.h>
#ifdef SITRONIX_PERMISSION_THREAD
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE
#include <linux/i2c.h>
#include <linux/input.h>
#ifdef SITRONIX_SUPPORT_MT_SLOT
#include <linux/input/mt.h>
#endif // SITRONIX_SUPPORT_MT_SLOT
#include <linux/interrupt.h>
#include <linux/slab.h> // to be compatible with linux kernel 3.2.15
#include <linux/gpio.h>
#include <mach/gpio.h>
#if defined(SITRONIX_PERMISSION_THREAD) || defined(SITRONIX_MONITOR_THREAD)
#include <linux/kthread.h>
#endif // SITRONIX_MONITOR_THREAD

#define DRIVER_AUTHOR           "Sitronix, Inc."
#define DRIVER_NAME             "sitronix"
#define DRIVER_DESC             "Sitronix I2C touch"
#define DRIVER_DATE             "20120911"
#define DRIVER_MAJOR            2
#define DRIVER_MINOR         	9
#define DRIVER_PATCHLEVEL       10

MODULE_AUTHOR("Rudy Huang <rudy_huang@sitronix.com.tw>");
MODULE_DESCRIPTION("Sitronix I2C multitouch panels");
MODULE_LICENSE("GPL");

char sitronix_sensor_key_status = 0;
struct sitronix_sensor_key_t sitronix_sensor_key_array[] = {
#ifdef CONFIG_ARCH_MSM8960
	{KEY_MENU},   // bit 0
	{KEY_HOME},   // bit 1
	{KEY_BACK},   // bit 2
	{KEY_SEARCH}, // bit 3
#else
	{KEY_BACK}, // bit 0
	{KEY_HOME}, // bit 1
	{KEY_MENU}, // bit 2
#endif	
};

#ifdef SITRONIX_AA_KEY
char sitronix_aa_key_status = 0;

#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#define SITRONIX_TOUCH_RESOLUTION_X 480 /* max of X value in display area */
#define SITRONIX_TOUCH_RESOLUTION_Y 854 /* max of Y value in display area */
#define SITRONIX_TOUCH_GAP_Y	10  /* Gap between bottom of display and top of touch key */
#define SITRONIX_TOUCH_MAX_Y 915  /* resolution of y axis of touch ic */
struct sitronix_AA_key sitronix_aa_key_array[] = {
	{15, 105, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_MENU}, /* MENU */
	{135, 225, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_HOME},
	{255, 345, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_BACK}, /* KEY_EXIT */
	{375, 465, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_SEARCH},
};
#else
#define SCALE_KEY_HIGH_Y 15
struct sitronix_AA_key sitronix_aa_key_array[] = {
	{0, 0, 0, 0, KEY_MENU}, /* MENU */
	{0, 0, 0, 0, KEY_HOME},
	{0, 0, 0, 0, KEY_BACK}, /* KEY_EXIT */
	{0, 0, 0, 0, KEY_SEARCH},
};
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#endif // SITRONIX_AA_KEY
struct sitronix_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *keyevent_input;
	int use_irq;
	struct hrtimer timer;
#ifndef SITRONIX_INT_POLLING_MODE
	struct work_struct  work;
#else
	struct delayed_work work;
#endif // SITRONIX_INT_POLLING_MODE
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif // CONFIG_HAS_EARLYSUSPEND
	uint8_t fw_revision[4];
	int resolution_x;
	int resolution_y;
	uint8_t max_touches;
	uint8_t touch_protocol_type;
	uint8_t pixel_length;
	uint8_t chip_id;
#ifdef SITRONIX_MONITOR_THREAD
	uint8_t enable_monitor_thread;
#endif // SITRONIX_MONITOR_THREAD
	uint8_t Num_X;
	uint8_t Num_Y;
	uint8_t sensing_mode;
	int suspend_state;
};

static int i2cErrorCount = 0;

#ifdef SITRONIX_MONITOR_THREAD
static struct task_struct * SitronixMonitorThread = NULL;
static int gMonitorThreadSleepInterval = 300; // 0.3 sec
static atomic_t iMonitorThreadPostpone = ATOMIC_INIT(0);

static uint8_t PreCheckData[4] ;
static int StatusCheckCount = 0;
static int sitronix_ts_monitor_thread(void *data);
static int sitronix_ts_delay_monitor_thread_start = DELAY_MONITOR_THREAD_START_PROBE; 
#endif // SITRONIX_MONITOR_THREAD

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h);
static void sitronix_ts_late_resume(struct early_suspend *h);
#endif // CONFIG_HAS_EARLYSUSPEND

static struct sitronix_ts_data *sitronix_ts_gpts = NULL;
static int sitronix_ts_irq_on = 0;
#ifdef SITRONIX_SYSFS
static bool sitronix_ts_sysfs_created = false;
static bool sitronix_ts_sysfs_using = false;
#endif // SITRONIX_SYSFS

static void sitronix_ts_reset_ic(void);

#ifdef CONFIG_ARCH_SC8810
extern int sprd_3rdparty_gpio_tp_rst ;
extern int sprd_3rdparty_gpio_tp_irq ;
extern int sprd_3rdparty_tp_ldo_id ;
extern int sprd_3rdparty_tp_ldo_level;

static void sitronix_ts_pwron(void)
{
	LDO_SetVoltLevel(LDO_LDO_SIM2, LDO_VOLT_LEVEL0);
	LDO_TurnOnLDO(LDO_LDO_SIM2);
	msleep(20);
}

static int  sitronix_ts_config_pins(void)
{
	int irq;
	sitronix_ts_pwron();
	gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
	irq = sprd_alloc_eic_irq(EIC_ID_2);

	sitronix_ts_reset_ic();

	return irq;
}

static int  sitronix_ts_hw_init(void)
{
	int irq;
	irq = sitronix_ts_config_pins();
	return irq;
}
#endif // CONFIG_ARCH_SC8810

#ifdef SITRONIX_FW_UPGRADE_FEATURE
#ifdef SITRONIX_PERMISSION_THREAD
SYSCALL_DEFINE3(fchmodat, int, dfd, const char __user *, filename, mode_t, mode);
static struct task_struct * SitronixPermissionThread = NULL;
static int sitronix_ts_delay_permission_thread_start = 1000;

static int sitronix_ts_permission_thread(void *data)
{
	int ret = 0;
	int retry = 0;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);

	DbgMsg("%s start\n", __FUNCTION__);
	do{
		DbgMsg("delay %d ms\n", sitronix_ts_delay_permission_thread_start);
		msleep(sitronix_ts_delay_permission_thread_start);
		ret = sys_fchmodat(AT_FDCWD, "/dev/"SITRONIX_I2C_TOUCH_DEV_NAME , 0666);
		if(ret < 0)
			pr_debug("fail to execute sys_fchmodat, ret = %d\n", ret);
		if(retry++ > 10)
			break;
	}while(ret == -ENOENT);
	set_fs(fs);
	DbgMsg("%s exit\n", __FUNCTION__);
	return 0;
}
#endif // SITRONIX_PERMISSION_THREAD

int      sitronix_release(struct inode *, struct file *);
int      sitronix_open(struct inode *, struct file *);
ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos);
long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static struct cdev sitronix_cdev;
static struct class *sitronix_class;
static int sitronix_major = 0;

int  sitronix_open(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_open);

int  sitronix_release(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_release);

ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret;
	char *tmp;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
	UpgradeMsg("writing %zu bytes.\n", count);

	ret = i2c_master_send(sitronix_ts_gpts->client, tmp, count);
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_write);

ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *tmp;
	int ret;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	UpgradeMsg("reading %zu bytes.\n", count);

	ret = i2c_master_recv(sitronix_ts_gpts->client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_read);

static int sitronix_ts_resume(struct i2c_client *client);
static int sitronix_ts_suspend(struct i2c_client *client, pm_message_t mesg);
void sitronix_ts_reprobe(void);
long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	uint8_t temp[4];

	if (_IOC_TYPE(cmd) != SMT_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SMT_IOC_MAXNR) return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user *)arg,\
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ,(void __user *)arg,\
				  _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {
		case IOCTL_SMT_GET_DRIVER_REVISION:
			UpgradeMsg("IOCTL_SMT_GET_DRIVER_REVISION\n");
			temp[0] = SITRONIX_TOUCH_DRIVER_VERSION;
			if(copy_to_user((uint8_t __user *)arg, &temp[0], 1)){
				UpgradeMsg("fail to get driver version\n");
				retval = -EFAULT;
			}
			break;
		case IOCTL_SMT_GET_FW_REVISION:
			UpgradeMsg("IOCTL_SMT_GET_FW_REVISION\n");
			if(copy_to_user((uint8_t __user *)arg, &sitronix_ts_gpts->fw_revision[0], 4))
					retval = -EFAULT;
			break;
		case IOCTL_SMT_ENABLE_IRQ:
			UpgradeMsg("IOCTL_SMT_ENABLE_IRQ\n");
			if(!sitronix_ts_irq_on){
				sitronix_ts_irq_on = 1;
				enable_irq(sitronix_ts_gpts->client->irq);
#ifdef SITRONIX_MONITOR_THREAD
				if(sitronix_ts_gpts->enable_monitor_thread == 1){
					atomic_set(&iMonitorThreadPostpone,1);
					SitronixMonitorThread = kthread_run(sitronix_ts_monitor_thread,"Sitronix","Monitorthread");
					if(IS_ERR(SitronixMonitorThread))
						SitronixMonitorThread = NULL;
				}
#endif // SITRONIX_MONITOR_THREAD
			}
			break;
		case IOCTL_SMT_DISABLE_IRQ:
			UpgradeMsg("IOCTL_SMT_DISABLE_IRQ\n");
			if(sitronix_ts_irq_on){
				sitronix_ts_irq_on = 0;
				disable_irq_nosync(sitronix_ts_gpts->client->irq);
#ifdef SITRONIX_MONITOR_THREAD
				if(sitronix_ts_gpts->enable_monitor_thread == 1){
					if(SitronixMonitorThread){
						kthread_stop(SitronixMonitorThread);
						SitronixMonitorThread = NULL;
					}
				}
#endif // SITRONIX_MONITOR_THREAD
			}
			break;
		case IOCTL_SMT_RESUME:
			UpgradeMsg("IOCTL_SMT_RESUME\n");
			sitronix_ts_resume(sitronix_ts_gpts->client);
			break;
		case IOCTL_SMT_SUSPEND:
			UpgradeMsg("IOCTL_SMT_SUSPEND\n");
			sitronix_ts_suspend(sitronix_ts_gpts->client, PMSG_SUSPEND);
			break;
		case IOCTL_SMT_HW_RESET:
			UpgradeMsg("IOCTL_SMT_HW_RESET\n");
			sitronix_ts_reset_ic();
			break;
		case IOCTL_SMT_REPROBE:
			UpgradeMsg("IOCTL_SMT_REPROBE\n");
			sitronix_ts_reprobe();
			break;
		default:
			retval = -ENOTTY;
	}

	return retval;
}
EXPORT_SYMBOL(sitronix_ioctl);
#endif // SITRONIX_FW_UPGRADE_FEATURE

static void sitronix_ts_reset_ic(void)
{
	printk("%s\n", __FUNCTION__);

#ifdef CONFIG_ARCH_MSM
	gpio_set_value(SITRONIX_RESET_GPIO, 0);
	mdelay(1);
	gpio_set_value(SITRONIX_RESET_GPIO, 1);
#elif defined(CONFIG_ARCH_SC8810)
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(3);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
	msleep(10);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst,1);
#else
	gpio_request(SITRONIX_RESET_GPIO, "Multitouch Reset");
	gpio_direction_output(SITRONIX_RESET_GPIO, 1);
	gpio_set_value(SITRONIX_RESET_GPIO, 0);
	mdelay(1);
	gpio_set_value(SITRONIX_RESET_GPIO, 1);
	//gpio_free(SITRONIX_RESET_GPIO);
#endif // CONFIG_ARCH_MSM

	mdelay(SITRONIX_TS_CHANGE_MODE_DELAY);
}

static int sitronix_i2c_read_bytes(struct i2c_client *client, u8 addr, u8 *rxbuf, int len)
{
	int ret = 0;
	u8 txbuf = addr;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &txbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = rxbuf,
		},
	};
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)

	if(rxbuf == NULL)
		return -1;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	ret = i2c_transfer(client->adapter, &msg[0], 2);
#elif defined(SITRONIX_I2C_SINGLE_MESSAGE)
	ret = i2c_master_send(client, &txbuf, 1);
	if (ret < 0){
		printk("write 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	ret = i2c_master_recv(client, rxbuf, len);
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)
	if (ret < 0){
		DbgMsg("read 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	return 0;
}

static int sitronix_i2c_write_bytes(struct i2c_client *client, u8 *txbuf, int len)
{
	int ret = 0;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	struct i2c_msg msg[1] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len,
			.buf = txbuf,
		},
	};
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)

	if(txbuf == NULL)
		return -1;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	ret = i2c_transfer(client->adapter, &msg[0], 1);
#elif defined(SITRONIX_I2C_SINGLE_MESSAGE)
	ret = i2c_master_send(client, txbuf, len);
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)
	if (ret < 0){
		printk("write 0x%x error (%d)\n", *txbuf, ret);
		return ret;
	}
	return 0;
}

static int sitronix_get_fw_revision(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[4];

	ret = sitronix_i2c_read_bytes(ts->client, FIRMWARE_REVISION_3, buffer, 4);
	if (ret < 0){
		printk("read fw revision error (%d)\n", ret);
		return ret;
	}else{
		memcpy(ts->fw_revision, buffer, 4);
		printk("fw revision (hex) = %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	}
	return 0;
}
static int sitronix_get_max_touches(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];

	ret = sitronix_i2c_read_bytes(ts->client, MAX_NUM_TOUCHES, buffer, 1);
	if (ret < 0){
		printk("read max touches error (%d)\n", ret);
		return ret;
	}else{
		ts->max_touches = buffer[0];
		if (ts->max_touches > SITRONIX_MAX_SUPPORTED_POINT)
			ts->max_touches = SITRONIX_MAX_SUPPORTED_POINT;
		printk("max touches = %d \n",ts->max_touches);
	}
	return 0;
}

static int sitronix_get_protocol_type(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];

	ret = sitronix_i2c_read_bytes(ts->client, I2C_PROTOCOL, buffer, 1);
	if (ret < 0){
		printk("read i2c protocol error (%d)\n", ret);
		return ret;
	}else{
		ts->touch_protocol_type = buffer[0] & I2C_PROTOCOL_BMSK;
		printk("i2c protocol = %d \n", ts->touch_protocol_type);
		ts->sensing_mode = (buffer[0] & (ONE_D_SENSING_CONTROL_BMSK << ONE_D_SENSING_CONTROL_SHFT)) >> ONE_D_SENSING_CONTROL_SHFT;
		printk("sensing mode = %d \n", ts->sensing_mode);
	}
	return 0;
}

static int sitronix_get_resolution(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[3];

	ret = sitronix_i2c_read_bytes(ts->client, XY_RESOLUTION_HIGH, buffer, 3);
	if (ret < 0){
		printk("read resolution error (%d)\n", ret);
		return ret;
	}else{
		ts->resolution_x = ((buffer[0] & (X_RES_H_BMSK << X_RES_H_SHFT)) << 4) | buffer[1];
		ts->resolution_y = ((buffer[0] & Y_RES_H_BMSK) << 8) | buffer[2];
		printk("resolution = %d x %d\n", ts->resolution_x, ts->resolution_y);
	}
	return 0;
}

static int sitronix_ts_get_CHIP_ID(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[3];

	DbgMsg("%s\n", __FUNCTION__);

	ret = sitronix_i2c_read_bytes(ts->client, CHIP_ID, buffer, 3);
	if (ret < 0){
		printk("read Chip ID error (%d)\n", ret);
		return ret;
	}else{
		if(buffer[0] == 0){
			if(buffer[1] + buffer[2] > 32)
				ts->chip_id = 2;
			else
				ts->chip_id = 0;
		}else
			ts->chip_id = buffer[0];
		ts->Num_X = buffer[1];
		ts->Num_Y = buffer[2];
		printk("Chip ID = %d\n", ts->chip_id);
		printk("Num_X = %d\n", ts->Num_X);
		printk("Num_Y = %d\n", ts->Num_Y);
	}

	return 0;
}

static int sitronix_ts_set_powerdown_bit(struct sitronix_ts_data *ts, int value)
{
	int ret = 0;
	uint8_t buffer[2];

	DbgMsg("%s, value = %d\n", __FUNCTION__, value);
	ret = sitronix_i2c_read_bytes(ts->client, DEVICE_CONTROL_REG, buffer, 1);
	if (ret < 0){
		printk("read device control status error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("dev status = %d \n", buffer[0]);
	}

	if(value == 0)
		buffer[1] = buffer[0] & 0xfd;
	else
		buffer[1] = buffer[0] | 0x2;

	buffer[0] = DEVICE_CONTROL_REG;
	ret = sitronix_i2c_write_bytes(ts->client, buffer, 2);
	if (ret < 0){
		printk("write power down error (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int sitronix_ts_get_touch_info(struct sitronix_ts_data *ts)
{
	int ret = 0;
	ret = sitronix_get_resolution(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_ts_get_CHIP_ID(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_fw_revision(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_protocol_type(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_max_touches(ts);
	if(ret < 0)
		return ret;

	if((ts->fw_revision[0] == 0) && (ts->fw_revision[1] == 0)){
		if(ts->touch_protocol_type == SITRONIX_RESERVED_TYPE_0){
			ts->touch_protocol_type = SITRONIX_B_TYPE;
			printk("i2c protocol (revised) = %d \n", ts->touch_protocol_type);
		}
	}
	if(ts->touch_protocol_type == SITRONIX_A_TYPE)
		ts->pixel_length = PIXEL_DATA_LENGTH_A;
	else if(ts->touch_protocol_type == SITRONIX_B_TYPE){
		ts->pixel_length = PIXEL_DATA_LENGTH_B;
		ts->max_touches = 2;
		printk("max touches (revised) = %d \n", ts->max_touches);
	}

#ifdef SITRONIX_MONITOR_THREAD
	if(ts->chip_id == 3)
		ts->enable_monitor_thread = 1;
	else
		ts->enable_monitor_thread = 0;
#endif // SITRONIX_MONITOR_THREAD

	return 0;
}

static int sitronix_ts_get_device_status(struct sitronix_ts_data *ts, uint8_t *dev_status)
{
	int ret = 0;
	uint8_t buffer[8];

	DbgMsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(ts->client, STATUS_REG, buffer, 8);
	if (ret < 0){
		printk("read status reg error (%d)\n", ret);
		return ret;
	}else{
		printk("status reg = %d \n", buffer[0]);
	}

	*dev_status = buffer[0] & 0xf;

	return 0;
}

#ifdef SITRONIX_IDENTIFY_ID
static int sitronix_ts_Enhance_Function_control(struct sitronix_ts_data *ts, uint8_t *value)
{
	int ret = 0;
	uint8_t buffer[4];

	DbgMsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(ts->client, 0xF0, buffer, 1);
	if (ret < 0){
		printk("read Enhance Functions status error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("Enhance Functions status = %d \n", buffer[0]);
	}

	*value = buffer[0] & 0x4;

	return 0;
}

static int sitronix_ts_FW_Bank_Select(struct sitronix_ts_data *ts, uint8_t value)
{
	int ret = 0;
	uint8_t buffer[1];

	DbgMsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(ts->client, 0xF1, buffer, 1);
	if (ret < 0){
		printk("read FW Bank Select status error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("FW Bank Select status = %d \n", buffer[0]);
	}

	buffer[1] = ((buffer[0] & 0xfc) | value);
	buffer[0] = 0xF1;
	ret = sitronix_i2c_write_bytes(ts->client, buffer, 2);
	if (ret < 0){
		printk("send FW Bank Select command error (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int sitronix_get_id_info(struct sitronix_ts_data *ts, uint8_t *id_info)
{
	int ret = 0;
	uint8_t buffer[4];

	ret = sitronix_i2c_read_bytes(ts->client, 0x0C, buffer, 4);
	if (ret < 0){
		printk("read id info error (%d)\n", ret);
		return ret;
	}else{
		memcpy(id_info, buffer, 4);
	}
	return 0;
}

static int sitronix_ts_identify(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t id[4];
	uint8_t Enhance_Function = 0;

	ret = sitronix_ts_FW_Bank_Select(ts, 1);
	if(ret < 0)
		return ret;
	ret = sitronix_ts_Enhance_Function_control(ts, &Enhance_Function);
	if(ret < 0)
		return ret;
	if(Enhance_Function == 0x4){
		ret = sitronix_get_id_info(ts, &id[0]);
		if(ret < 0)
			return ret;
		printk("id (hex) = %x %x %x %x\n", id[0], id[1], id[2], id[3]);
		if((id[0] == 1)&&(id[1] == 2)&&(id[2] == 0xb)&&(id[3] == 1)){
			return 0;
		}else{
			printk("Error: It is not Sitronix IC\n");
			return -1;
		}
	}else{
		printk("Error: Can not get ID of Sitronix IC\n");
		return -1;
	}
}
#endif // SITRONIX_IDENTIFY_ID

#ifdef SITRONIX_MONITOR_THREAD
static int sitronix_set_raw_data_type(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1] = {0};

	ret = sitronix_i2c_read_bytes(ts->client, DEVICE_CONTROL_REG, buffer, 1);
	if (ret < 0){
		DbgMsg("read DEVICE_CONTROL_REG error (%d)\n", ret);
		return ret;
	}else{
		DbgMsg("read DEVICE_CONTROL_REG status = %d \n", buffer[0]);
	}
	if(ts->sensing_mode == SENSING_BOTH_NOT){
		buffer[1] = ((buffer[0] & 0xf3) | (0x01 << 2));
	}else{
		buffer[1] = (buffer[0] & 0xf3);
	}
	buffer[0] = DEVICE_CONTROL_REG;
	ret = sitronix_i2c_write_bytes(ts->client, buffer, 2);
	if (ret < 0){
		DbgMsg("write DEVICE_CONTROL_REG error (%d)\n", ret);
		return ret;
	}
	return 0;
}

static int sitronix_ts_monitor_thread(void *data)
{
	int ret = 0;
	uint8_t buffer[4] = { 0, 0, 0, 0 };
	int result = 0;
	int once = 1;
	uint8_t raw_data_ofs = 0;

	DbgMsg("%s:\n", __FUNCTION__);

	printk("delay %d ms\n", sitronix_ts_delay_monitor_thread_start);
	msleep(sitronix_ts_delay_monitor_thread_start);
	while(!kthread_should_stop()){
		DbgMsg("%s:\n", "Sitronix_ts_monitoring");
		if(atomic_read(&iMonitorThreadPostpone)){
		 		atomic_set(&iMonitorThreadPostpone,0);
		}else{
			if(once == 1){
				ret = sitronix_set_raw_data_type(sitronix_ts_gpts);
				if (ret < 0)
					goto exit_i2c_invalid;

				if((sitronix_ts_gpts->sensing_mode == SENSING_BOTH) || (sitronix_ts_gpts->sensing_mode == SENSING_X_ONLY)){
					raw_data_ofs = 0x40;
				}else if(sitronix_ts_gpts->sensing_mode == SENSING_Y_ONLY){
					raw_data_ofs = 0x40 + sitronix_ts_gpts->Num_X * 2; 
				}else{
					raw_data_ofs = 0x40;
				}

				once = 0;
			}
			if(raw_data_ofs != 0x40){
				ret = sitronix_i2c_read_bytes(sitronix_ts_gpts->client, 0x40, buffer, 1);
				if (ret < 0){
					DbgMsg("read raw data error (%d)\n", ret);
					result = 0;
					goto exit_i2c_invalid;
				}
			}
			ret = sitronix_i2c_read_bytes(sitronix_ts_gpts->client, raw_data_ofs, buffer, 4);
			if (ret < 0){
				DbgMsg("read raw data error (%d)\n", ret);
				result = 0;
				goto exit_i2c_invalid;
			}else{
				DbgMsg("%dD data h%x-%x = %d, %d, %d, %d \n", (sitronix_ts_gpts->sensing_mode == SENSING_BOTH_NOT ? 2:1), raw_data_ofs, raw_data_ofs + 3, buffer[0], buffer[1], buffer[2], buffer[3]);
				result = 1;
				if ((PreCheckData[0] == buffer[0]) && (PreCheckData[1] == buffer[1]) && 
				(PreCheckData[2] == buffer[2]) && (PreCheckData[3] == buffer[3]))
					StatusCheckCount ++;
				else
					StatusCheckCount =0;
				PreCheckData[0] = buffer[0];
				PreCheckData[1] = buffer[1];
				PreCheckData[2] = buffer[2];
				PreCheckData[3] = buffer[3];
				if (3 <= StatusCheckCount){
					DbgMsg("IC Status doesn't update! \n");
					result = -1;
					StatusCheckCount = 0;
				}
			}
			if (-1 == result){
				printk("Chip abnormal, reset it!\n");
				sitronix_ts_reset_ic();
		   		i2cErrorCount = 0;
		   		StatusCheckCount = 0;
				ret = sitronix_set_raw_data_type(sitronix_ts_gpts);
				if (ret < 0)
					goto exit_i2c_invalid;
			}
exit_i2c_invalid:
			if(0 == result){
				i2cErrorCount ++;
				if ((2 <= i2cErrorCount)){
					printk("I2C abnormal, reset it!\n");
					sitronix_ts_reset_ic();
					sitronix_set_raw_data_type(sitronix_ts_gpts);
		    			i2cErrorCount = 0;
		    			StatusCheckCount = 0;
		    		}
		    	}else
		    		i2cErrorCount = 0;
		}
		msleep(gMonitorThreadSleepInterval);
	}
	DbgMsg("%s exit\n", __FUNCTION__);
	return 0;
}
#endif // SITRONIX_MONITOR_THREAD

static inline void sitronix_ts_pen_down(struct input_dev *input_dev, int id, u16 x, u16 y)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#ifndef SITRONIX_SWAP_XY
	input_report_abs(input_dev,  ABS_MT_POSITION_X, x);
	input_report_abs(input_dev,  ABS_MT_POSITION_Y, y);
#else
	input_report_abs(input_dev,  ABS_MT_POSITION_X, y);
	input_report_abs(input_dev,  ABS_MT_POSITION_Y, x);
#endif // SITRONIX_SWAP_XY
#else
	input_report_abs(input_dev,  ABS_MT_TRACKING_ID, id);
#ifndef SITRONIX_SWAP_XY
	input_report_abs(input_dev,  ABS_MT_POSITION_X, x);
	input_report_abs(input_dev,  ABS_MT_POSITION_Y, y);
#else
	input_report_abs(input_dev,  ABS_MT_POSITION_X, y);
	input_report_abs(input_dev,  ABS_MT_POSITION_Y, x);
#endif // SITRONIX_SWAP_XY
	input_report_abs(input_dev,  ABS_MT_TOUCH_MAJOR, 10);
	input_report_abs(input_dev,  ABS_MT_WIDTH_MAJOR, 10);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
	input_report_abs(input_dev, ABS_MT_PRESSURE, 10);
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
	input_mt_sync(input_dev);
#endif // SITRONIX_SUPPORT_MT_SLOT
	DbgMsg("[%d](%d, %d)+\n", id, x, y);
}

static inline void sitronix_ts_pen_up(struct input_dev *input_dev, int id)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#endif // SITRONIX_SUPPORT_MT_SLOT
	DbgMsg("[%d]-\n", id);
}

static inline void sitronix_ts_handle_sensor_key(struct input_dev *input_dev, struct sitronix_sensor_key_t *key_array, char *pre_key_status, char cur_key_status, int key_count)
{
	int i = 0;
	for(i = 0; i < key_count; i++){
		if(cur_key_status & (1 << i)){
			DbgMsg("sensor key[%d] down\n", i);
			input_report_key(input_dev, key_array[i].code, 1);
			input_sync(input_dev);
		}else{
			if(*pre_key_status & (1 << i)){
				DbgMsg("sensor key[%d] up\n", i);
				input_report_key(input_dev, key_array[i].code, 0);
				input_sync(input_dev);
			}
		}
	}
	*pre_key_status = cur_key_status;
}

#ifdef SITRONIX_AA_KEY
static inline void sitronix_ts_handle_aa_key(struct input_dev *input_dev, struct sitronix_AA_key *key_array, char *pre_key_status, char cur_key_status, int key_count)
{
	int i = 0;
	for(i = 0; i < key_count; i++){
		if(cur_key_status & (1 << i)){
			DbgMsg("aa key[%d] down\n", i);
			input_report_key(input_dev, key_array[i].code, 1);
			input_sync(input_dev);
		}else{
			if(*pre_key_status & (1 << i)){
				DbgMsg("aa key[%d] up\n", i);
				input_report_key(input_dev, key_array[i].code, 0);
				input_sync(input_dev);
			}
		}
	}
	*pre_key_status = cur_key_status;
}
#endif // SITRONIX_AA_KEY

static void sitronix_ts_work_func(struct work_struct *work)
{
	int i;
#ifdef SITRONIX_AA_KEY
	int j;
	char aa_key_status = 0;
#endif // SITRONIX_AA_KEY
	int ret;
#ifndef SITRONIX_INT_POLLING_MODE
	struct sitronix_ts_data *ts = container_of(work, struct sitronix_ts_data, work);
#else
	struct sitronix_ts_data *ts = container_of(to_delayed_work(work), struct sitronix_ts_data, work);
#endif // SITRONIX_INT_POLLING_MODE
	u16 x, y;
	uint8_t buffer[1+ SITRONIX_MAX_SUPPORTED_POINT * PIXEL_DATA_LENGTH_A] = {0};
	uint8_t PixelCount = 0;

	DbgMsg("%s\n",  __FUNCTION__);
	if(ts->suspend_state){
		goto exit_invalid_data;
	}

	ret = sitronix_i2c_read_bytes(ts->client, KEYS_REG, buffer, 1 + ts->max_touches * ts->pixel_length);
	if (ret < 0) {
		printk("read finger error (%d)\n", ret);
   		i2cErrorCount++;
		goto exit_invalid_data;
	}

	for(i = 0; i < ts->max_touches; i++){
		if(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x80){
			x = (u16)(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x70) << 4 | buffer[1 + i * ts->pixel_length + X_COORD_L];
			y = (u16)(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x07) << 8 | buffer[1 + i * ts->pixel_length + Y_COORD_L];
#ifndef SITRONIX_AA_KEY
			PixelCount++;
			sitronix_ts_pen_down(ts->input_dev, i, x, y);
#else
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
			if(y < SITRONIX_TOUCH_RESOLUTION_Y){
#else
			if(y < (ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y)){
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
				PixelCount++;
				sitronix_ts_pen_down(ts->input_dev, i, x, y);
				//DbgMsg("AREA_DISPLAY\n");
			}else{
				for(j = 0; j < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); j++){
					if((x >= sitronix_aa_key_array[j].x_low) &&
					(x <= sitronix_aa_key_array[j].x_high) &&
					(y >= sitronix_aa_key_array[j].y_low) &&
					(y <= sitronix_aa_key_array[j].y_high)){
						aa_key_status |= (1 << j);
						//DbgMsg("AREA_KEY [%d]\n", j);
						break;
					}
				}
			}
#endif // SITRONIX_AA_KEY
		}else{
			sitronix_ts_pen_up(ts->input_dev, i);
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, PixelCount > 0);
	input_sync(ts->input_dev);

	sitronix_ts_handle_sensor_key(ts->keyevent_input, sitronix_sensor_key_array, &sitronix_sensor_key_status, buffer[0], (sizeof(sitronix_sensor_key_array)/sizeof(struct sitronix_sensor_key_t)));
#ifdef SITRONIX_AA_KEY
	sitronix_ts_handle_aa_key(ts->keyevent_input, sitronix_aa_key_array, &sitronix_aa_key_status, aa_key_status, (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)));
#endif // SITRONIX_AA_KEY

exit_invalid_data:
#ifdef SITRONIX_INT_POLLING_MODE
	if(PixelCount > 0){
#ifdef SITRONIX_MONITOR_THREAD
		if(ts->enable_monitor_thread == 1){
			atomic_set(&iMonitorThreadPostpone,1);
		}
#endif // SITRONIX_MONITOR_THREAD
		schedule_delayed_work(&ts->work, msecs_to_jiffies(INT_POLLING_MODE_INTERVAL));
	}else{
#ifdef CONFIG_HARDIRQS_SW_RESEND
		printk("Please not set HARDIRQS_SW_RESEND to prevent kernel from sending SW IRQ\n");
#endif // CONFIG_HARDIRQS_SW_RESEND
		if (ts->use_irq){
			sitronix_ts_irq_on = 1;
			enable_irq(ts->client->irq);
		}
	}
#endif // SITRONIX_INT_POLLING_MODE
#if defined(SITRONIX_LEVEL_TRIGGERED)
	if (ts->use_irq){
		sitronix_ts_irq_on = 1;
		enable_irq(ts->client->irq);
	}
#endif // defined(SITRONIX_LEVEL_TRIGGERED)
	if ((2 <= i2cErrorCount)){
		printk("I2C abnormal in work_func(), reset it!\n");
		sitronix_ts_reset_ic();
   		i2cErrorCount = 0;
#ifdef SITRONIX_MONITOR_THREAD
		if(ts->enable_monitor_thread == 1){
			StatusCheckCount = 0;
			sitronix_set_raw_data_type(sitronix_ts_gpts);
		}
#endif // SITRONIX_MONITOR_THREAD
	}
;
}


static irqreturn_t sitronix_ts_irq_handler(int irq, void *dev_id)
{
	struct sitronix_ts_data *ts = dev_id;

	DbgMsg("%s\n", __FUNCTION__);
#if defined(SITRONIX_LEVEL_TRIGGERED) || defined(SITRONIX_INT_POLLING_MODE)
	sitronix_ts_irq_on = 0;
	disable_irq_nosync(ts->client->irq);
#endif // defined(SITRONIX_LEVEL_TRIGGERED) || defined(SITRONIX_INT_POLLING_MODE)
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		atomic_set(&iMonitorThreadPostpone,1);
	}
#endif // SITRONIX_MONITOR_THREAD
#ifndef SITRONIX_INT_POLLING_MODE
	schedule_work(&ts->work);
#else
	schedule_delayed_work(&ts->work, msecs_to_jiffies(0));
#endif // SITRONIX_INT_POLLING_MODE
	return IRQ_HANDLED;
}

#ifdef SITRONIX_SYSFS
static ssize_t sitronix_ts_reprobe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sitronix_ts_sysfs_using = true;
	sitronix_ts_reprobe();
	sitronix_ts_sysfs_using = false;
	return count;
}

static DEVICE_ATTR(reprobe, 0222, NULL, sitronix_ts_reprobe_store);

static struct attribute *sitronix_ts_attrs_v0[] = {
	&dev_attr_reprobe.attr,
	NULL,
};

static struct attribute_group sitronix_ts_attr_group_v0 = {
	.name = "sitronix_ts_attrs",
	.attrs = sitronix_ts_attrs_v0,
};

static int sitronix_ts_create_sysfs_entry(struct i2c_client *client)
{
	int err;

	err = sysfs_create_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);
	if (err) {
		dev_warn(&client->dev, "%s(%u): sysfs_create_group() failed!\n", __FUNCTION__, __LINE__);
	}
	return err;
}

static void sitronix_ts_destroy_sysfs_entry(struct i2c_client *client)
{
	sysfs_remove_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);

	return;
}
#endif // SITRONIX_SYSFS
static int sitronix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	struct sitronix_ts_data *ts = NULL;
	int ret = 0;
	uint16_t max_x = 0, max_y = 0;
	struct sitronix_i2c_touch_platform_data *pdata;
	uint8_t dev_status = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->input_dev = NULL;
	ts->keyevent_input = NULL;

#ifndef SITRONIX_INT_POLLING_MODE
	INIT_WORK(&ts->work, sitronix_ts_work_func);
#else
	INIT_DELAYED_WORK(&ts->work, sitronix_ts_work_func);
#endif // SITRONIX_INT_POLLING_MODE
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

#ifdef CONFIG_ARCH_MSM
	gpio_tlmm_config(GPIO_CFG(SITRONIX_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif // CONFIG_ARCH_MSM
#ifdef CONFIG_ARCH_SC8810
	client->irq = sitronix_ts_hw_init();
#else
	sitronix_ts_reset_ic();
#endif // CONFIG_ARCH_SC8810

	sitronix_ts_gpts = ts;

	ret = sitronix_ts_get_device_status(ts, &dev_status);
	if((ret < 0) || (dev_status == 0x6)){
		ret = -EPERM;
		goto err_device_info_error;
	}

	ret = sitronix_ts_get_touch_info(ts);
	if(ret < 0)
		goto err_device_info_error;

#ifdef SITRONIX_IDENTIFY_ID
	ret = sitronix_ts_identify(ts);
	if(ret < 0)
		goto err_device_info_error;
#endif // SITRONIX_IDENTIFY_ID

#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		//== Add thread to monitor chip
		atomic_set(&iMonitorThreadPostpone,1);
		SitronixMonitorThread = kthread_run(sitronix_ts_monitor_thread,"Sitronix","Monitorthread");
		if(IS_ERR(SitronixMonitorThread))
			SitronixMonitorThread = NULL;
	}
#endif // SITRONIX_MONITOR_THREAD

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL){
		printk("Can not allocate memory for input device.");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sitronix-i2c-touch-mt";
	ts->input_dev->dev.parent = &client->dev;

#ifdef SITRONIX_SUPPORT_MT_SLOT
	ts->input_dev->id.bustype = BUS_I2C;
#endif // SITRONIX_SUPPORT_MT_SLOT

	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	ts->keyevent_input = input_allocate_device();
	if (ts->keyevent_input == NULL){
		printk("Can not allocate memory for key input device.");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	ts->keyevent_input->name  = "sitronix-i2c-touch-key";
	ts->keyevent_input->dev.parent = &client->dev;
	set_bit(EV_KEY, ts->keyevent_input->evbit);
	for(i = 0; i < (sizeof(sitronix_sensor_key_array)/sizeof(struct sitronix_sensor_key_t)); i++){
		set_bit(sitronix_sensor_key_array[i].code, ts->keyevent_input->keybit);
	}

#ifndef SITRONIX_AA_KEY
	max_x = ts->resolution_x;
	max_y = ts->resolution_y;
#else
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
	for(i = 0; i < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); i++){
		set_bit(sitronix_aa_key_array[i].code, ts->keyevent_input->keybit);
	}
	max_x = SITRONIX_TOUCH_RESOLUTION_X;
	max_y = SITRONIX_TOUCH_RESOLUTION_Y;
#else
	for(i = 0; i < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); i++){
		sitronix_aa_key_array[i].x_low = ((ts->resolution_x / (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)) ) * i ) + 15;
		sitronix_aa_key_array[i].x_high = ((ts->resolution_x / (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)) ) * (i + 1)) - 15;
		sitronix_aa_key_array[i].y_low = ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y;
		sitronix_aa_key_array[i].y_high = ts->resolution_y;
		DbgMsg("key[%d] %d, %d, %d, %d\n", i, sitronix_aa_key_array[i].x_low, sitronix_aa_key_array[i].x_high, sitronix_aa_key_array[i].y_low, sitronix_aa_key_array[i].y_high);
		set_bit(sitronix_aa_key_array[i].code, ts->keyevent_input->keybit);
	}
	max_x = ts->resolution_x;
	max_y = ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y;
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#endif // SITRONIX_AA_KEY
	ret = input_register_device(ts->keyevent_input);
	if(ret < 0){
		printk("Can not register key input device.");
		goto err_input_register_device_failed;
	}

#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_init_slots(ts->input_dev, ts->max_touches);
#else
	__set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	__set_bit(ABS_MT_TOOL_TYPE, ts->input_dev->absbit);
	__set_bit(ABS_MT_BLOB_ID, ts->input_dev->absbit);
	__set_bit(ABS_MT_TRACKING_ID, ts->input_dev->absbit);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0,  255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0,  255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touches, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
#endif // SITRONIX_SUPPORT_MT_SLOT
#ifndef SITRONIX_SWAP_XY
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
#else
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_x, 0, 0);
#endif // SITRONIX_SWAP_XY

	ret = input_register_device(ts->input_dev);
	if(ret < 0){
		printk("Can not register input device.\n");
		goto err_input_register_device_failed;
	}

	ts->suspend_state = 0;
#ifdef CONFIG_ARCH_MSM
	gpio_tlmm_config(GPIO_CFG(SITRONIX_INT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif // CONFIG_ARCH_MSM
	if (client->irq){
		dev_info(&client->dev, "irq = %d\n", client->irq);
#ifdef SITRONIX_LEVEL_TRIGGERED
		ret = request_irq(client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_LOW | IRQF_DISABLED, client->name, ts);
#else
		ret = request_irq(client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_FALLING | IRQF_DISABLED, client->name, ts);
#endif // SITRONIX_LEVEL_TRIGGERED
		if (ret == 0){
			sitronix_ts_irq_on = 1;
			ts->use_irq = 1;
		}else
			dev_err(&client->dev, "request_irq failed\n");
	}

#ifdef SITRONIX_SYSFS
	if(!sitronix_ts_sysfs_created){
		ret = sitronix_ts_create_sysfs_entry(client);
		if(ret < 0)
			goto err_create_sysfs_failed;
		sitronix_ts_sysfs_created = true;
	}
#endif // SITRONIX_SYSFS
#ifdef CONFIG_HAS_EARLYSUSPEND
        ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        ts->early_suspend.suspend = sitronix_ts_early_suspend;
        ts->early_suspend.resume = sitronix_ts_late_resume;
        register_early_suspend(&ts->early_suspend);
#endif // CONFIG_HAS_EARLYSUSPEND
	return 0;

#ifdef SITRONIX_SYSFS
err_create_sysfs_failed:
	input_unregister_device(ts->input_dev);
	input_unregister_device(ts->keyevent_input);
#endif // SITRONIX_SYSFS
err_input_register_device_failed:
	input_free_device(ts->input_dev);
	input_free_device(ts->keyevent_input);
err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		if(SitronixMonitorThread){
		      kthread_stop(SitronixMonitorThread);
		      SitronixMonitorThread = NULL;
		}
	}
#endif // SITRONIX_MONITOR_THREAD
err_device_info_error:
	return ret;
}

static int sitronix_ts_remove(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif // CONFIG_HAS_EARLYSUSPEND
#ifdef SITRONIX_SYSFS
	if(!sitronix_ts_sysfs_using){
		sitronix_ts_destroy_sysfs_entry(client);
		sitronix_ts_sysfs_created = false;
	}
#endif // SITRONIX_SYSFS
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		if(SitronixMonitorThread){
		      kthread_stop(SitronixMonitorThread);
		      SitronixMonitorThread = NULL;
		}
	}
#endif // SITRONIX_MONITOR_THREAD
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	if(ts->input_dev)
		input_unregister_device(ts->input_dev);
	if(ts->keyevent_input)
		input_unregister_device(ts->keyevent_input);

	kfree(ts);
	return 0;
}

static int sitronix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	DbgMsg("%s\n", __FUNCTION__);
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		if(SitronixMonitorThread){
			kthread_stop(SitronixMonitorThread);
			SitronixMonitorThread = NULL;
		}
		sitronix_ts_delay_monitor_thread_start = DELAY_MONITOR_THREAD_START_RESUME;
	}
#endif // SITRONIX_MONITOR_THREAD
	if(ts->use_irq){
		sitronix_ts_irq_on = 0;
		disable_irq_nosync(ts->client->irq);
	}
	ts->suspend_state = 1;

	ret = sitronix_ts_set_powerdown_bit(ts, 1);
	if(ts->chip_id == 2){
#ifdef CONFIG_ARCH_MSM
		gpio_tlmm_config(GPIO_CFG(SITRONIX_INT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(SITRONIX_INT_GPIO, 1);
#else
		gpio_direction_output(irq_to_gpio(client->irq), 1);
#endif // CONFIG_ARCH_MSM
	}
	DbgMsg("%s return\n", __FUNCTION__);

	return 0;
}

static int sitronix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	DbgMsg("%s\n", __FUNCTION__);

	if(ts->chip_id == 2){
#ifdef CONFIG_ARCH_MSM
		gpio_set_value(SITRONIX_INT_GPIO, 0);
		gpio_tlmm_config(GPIO_CFG(SITRONIX_INT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#else
		ret = irq_to_gpio(client->irq);
		gpio_set_value(ret, 0);
		gpio_direction_input(ret);
#endif // CONFIG_ARCH_MSM
	}else{
		ret = sitronix_ts_set_powerdown_bit(ts, 0);
	}

	ts->suspend_state = 0;
	if(ts->use_irq){
		sitronix_ts_irq_on = 1;
		enable_irq(ts->client->irq);
	}
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		atomic_set(&iMonitorThreadPostpone,1);
		SitronixMonitorThread = kthread_run(sitronix_ts_monitor_thread,"Sitronix","Monitorthread");
		if(IS_ERR(SitronixMonitorThread))
			SitronixMonitorThread = NULL;
	}
#endif // SITRONIX_MONITOR_THREAD
	DbgMsg("%s return\n", __FUNCTION__);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	DbgMsg("%s\n", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void sitronix_ts_late_resume(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	DbgMsg("%s\n", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_resume(ts->client);
}
#endif // CONFIG_HAS_EARLYSUSPEND

static const struct i2c_device_id sitronix_ts_id[] = {
	{ SITRONIX_I2C_TOUCH_DRV_NAME, 0 },
	{ }
};

static struct i2c_driver sitronix_ts_driver = {
	.probe		= sitronix_ts_probe,
	.remove		= sitronix_ts_remove,
	.id_table	= sitronix_ts_id,
	.driver = {
		.name	= SITRONIX_I2C_TOUCH_DRV_NAME,
	},
};

#ifdef SITRONIX_FW_UPGRADE_FEATURE
static struct file_operations nc_fops = {
	.owner =        THIS_MODULE,
	.write		= sitronix_write,
	.read		= sitronix_read,
	.open		= sitronix_open,
	.unlocked_ioctl = sitronix_ioctl,
	.release	= sitronix_release,
};
#endif // SITRONIX_FW_UPGRADE_FEATURE
void sitronix_ts_reprobe(void)
{
	int retval = 0;
	i2c_del_driver(&sitronix_ts_driver);
	retval = i2c_add_driver(&sitronix_ts_driver);
	if(retval < 0)
		printk("fail to reprobe driver!\n");
}

static int __devinit sitronix_ts_init(void)
{
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	int result;
	int err = 0;
	dev_t devno = MKDEV(sitronix_major, 0);
#endif // SITRONIX_FW_UPGRADE_FEATURE
	printk("Sitronix touch driver %d.%d.%d\n", DRIVER_MAJOR, DRIVER_MINOR, DRIVER_PATCHLEVEL);
	printk("Release date: %s\n", DRIVER_DATE);
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	result  = alloc_chrdev_region(&devno, 0, 1, SITRONIX_I2C_TOUCH_DEV_NAME);
	if(result < 0){
		printk("fail to allocate chrdev (%d) \n", result);
		return 0;
	}
	sitronix_major = MAJOR(devno);
        cdev_init(&sitronix_cdev, &nc_fops);
	sitronix_cdev.owner = THIS_MODULE;
	sitronix_cdev.ops = &nc_fops;
        err =  cdev_add(&sitronix_cdev, devno, 1);
	if(err){
		printk("fail to add cdev (%d) \n", err);
		return 0;
	}

	sitronix_class = class_create(THIS_MODULE, SITRONIX_I2C_TOUCH_DEV_NAME);
	if (IS_ERR(sitronix_class)) {
		result = PTR_ERR(sitronix_class);
		unregister_chrdev(sitronix_major, SITRONIX_I2C_TOUCH_DEV_NAME);
		printk("fail to create class (%d) \n", result);
		return result;
	}
	device_create(sitronix_class, NULL, MKDEV(sitronix_major, 0), NULL, SITRONIX_I2C_TOUCH_DEV_NAME);
#ifdef SITRONIX_PERMISSION_THREAD
	SitronixPermissionThread = kthread_run(sitronix_ts_permission_thread,"Sitronix","Permissionthread");
	if(IS_ERR(SitronixPermissionThread))
		SitronixPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE
	return i2c_add_driver(&sitronix_ts_driver);
}

static void __exit sitronix_ts_exit(void)
{
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	dev_t dev_id = MKDEV(sitronix_major, 0);
#endif // SITRONIX_FW_UPGRADE_FEATURE
	i2c_del_driver(&sitronix_ts_driver);
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	cdev_del(&sitronix_cdev);

	device_destroy(sitronix_class, dev_id); //delete device node under /dev
	class_destroy(sitronix_class); //delete class created by us
	unregister_chrdev_region(dev_id, 1);
#ifdef SITRONIX_PERMISSION_THREAD
	if(SitronixPermissionThread)
		SitronixPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE
}

module_init(sitronix_ts_init);
module_exit(sitronix_ts_exit);

MODULE_DESCRIPTION("Sitronix Multi-Touch Driver");
MODULE_LICENSE("GPL");
