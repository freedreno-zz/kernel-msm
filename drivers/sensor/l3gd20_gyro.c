/*
 *  Copyright (C) 2011, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/sensor/l3gd20_gyro.h>
#include <linux/sensor/sensors_core.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#define DEBUG 1

#undef LOGGING_GYRO
#undef DEBUG_REGISTER

/* l3gd20_gyro chip id */
#define DEVICE_ID	0xD4
/* l3gd20_gyro gyroscope registers */
#define WHO_AM_I	0x0F
#define CTRL_REG1	0x20  /* power control reg */
#define CTRL_REG2	0x21  /* trigger & filter setting control reg */
#define CTRL_REG3	0x22  /* interrupt control reg */
#define CTRL_REG4	0x23  /* data control reg */
#define CTRL_REG5	0x24  /* fifo en & filter en control reg */
#define OUT_TEMP	0x26  /* Temperature data */
#define STATUS_REG	0x27
#define AXISDATA_REG	0x28
#define OUT_Y_L		0x2A
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG	0x2F
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define BYPASS_MODE	0x00
#define FIFO_MODE	0x20
#define FIFO_EMPTY	0x20
#define AC		(1 << 7) /* register auto-increment bit */

/* odr settings */
#define FSS_MASK	0x1F
#define ODR_MASK	0xF0
#define ODR95_BW12_5	0x00  /* ODR = 95Hz; BW = 12.5Hz */
#define ODR95_BW25	0x11  /* ODR = 95Hz; BW = 25Hz   */
#define ODR190_BW12_5	0x40  /* ODR = 190Hz; BW = 12.5Hz */
#define ODR190_BW25	0x50  /* ODR = 190Hz; BW = 25Hz   */
#define ODR190_BW50	0x60  /* ODR = 190Hz; BW = 50Hz   */
#define ODR190_BW70	0x70  /* ODR = 190Hz; BW = 70Hz   */
#define ODR380_BW20	0x80  /* ODR = 380Hz; BW = 20Hz   */
#define ODR380_BW25	0x90  /* ODR = 380Hz; BW = 25Hz   */
#define ODR380_BW50	0xA0  /* ODR = 380Hz; BW = 50Hz   */
#define ODR380_BW100	0xB0  /* ODR = 380Hz; BW = 100Hz  */
#define ODR760_BW30	0xC0  /* ODR = 760Hz; BW = 30Hz   */
#define ODR760_BW35	0xD0  /* ODR = 760Hz; BW = 35Hz   */
#define ODR760_BW50	0xE0  /* ODR = 760Hz; BW = 50Hz   */
#define ODR760_BW100	0xF0  /* ODR = 760Hz; BW = 100Hz  */

/* full scale selection */
#define DPS250		250
#define DPS500		500
#define DPS2000		2000
#define FS_MASK		0x30
#define FS_250DPS	0x00
#define FS_500DPS	0x10
#define FS_2000DPS	0x20
#define DEFAULT_DPS	DPS500
#define FS_DEFULAT_DPS	FS_500DPS

/* self tset settings */
#define MIN_ST		175
#define MAX_ST		875
#define FIFO_TEST_WTM	0x1F
#define MIN_ZERO_RATE	-1714
#define MAX_ZERO_RATE	1714

/* max and min entry */
#define MAX_ENTRY	20
#define MAX_DELAY	(MAX_ENTRY * 10000000LL) /* 200ms */

/* default register setting for device init */
static const char default_ctrl_regs_bypass[] = {
	0x3F,	/* 95HZ, BW25, PM-normal, xyz enable */
	0x00,	/* normal mode */
	0x08,	/* fifo wtm interrupt off */
	0x90,	/* block data update, 500d/s */
	0x00,	/* fifo disable */
};
static const char default_ctrl_regs_fifo[] = {
	0x3F,	/* 95HZ, BW25, PM-normal, xyz enable */
	0x00,	/* normal mode */
	0x0c,	/* fifo wtm interrupt on */
	0x90,	/* block data update, 500d/s */
	0x40,	/* fifo enable */
};

static const struct odr_delay {
	u8 odr; /* odr reg setting */
	u64 delay_ns; /* odr in ns */
} odr_delay_table[] = {
	{ ODR760_BW100, 1315789LL },/* 760Hz */
	{ ODR380_BW100, 2631578LL },/* 380Hz */
	{ ODR190_BW70, 5263157LL }, /* 190Hz */
	{ ODR95_BW25, 10526315LL }, /* 95Hz */
};

/*
 * l3gd20_gyro gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */
struct gyro_t {
	s16 x;
	s16 y;
	s16 z;
};
struct negative_sign {
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
} singd = { 1, 1, 1};
struct l3gd20_gyro_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct mutex lock;
	struct workqueue_struct *l3gd20_gyro_wq;
	struct work_struct work;
	struct hrtimer timer;
	atomic_t opened;
	bool enable;
	bool drop_next_event;
	bool fifo_test;		/* is self_test or not? */
	bool interruptible;	/* interrupt or polling? */
	int entries;		/* number of fifo entries */
	int dps;		/* scale selection */
	/* fifo data entries */
	u8 fifo_data[sizeof(struct gyro_t) * 32];
	u8 ctrl_regs[5];	/* saving register settings */
	u32 time_to_read;	/* time needed to read one entry */
	ktime_t polling_delay;	/* polling time for timer */
	struct gyro_t xyz_data;

	struct device *dev;
	struct completion data_ready;
#ifdef LOGGING_GYRO
	int count;
#endif
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator l3gd20_vreg[] = {
	{NULL, "vdd2v9", 2400000, 3300000},
	{NULL, "vdd1v8", 1800000, 1900000},
};

static int l3gd20_config_regulator(struct l3gd20_gyro_data *acc, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(l3gd20_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			l3gd20_vreg[i].vreg =
				regulator_get(&acc->client->dev,
				l3gd20_vreg[i].name);
			if (IS_ERR(l3gd20_vreg[i].vreg)) {
				rc = PTR_ERR(l3gd20_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				goto error_vdd;
			}

			if (regulator_count_voltages(
				l3gd20_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					l3gd20_vreg[i].vreg,
					l3gd20_vreg[i].min_uV,
					l3gd20_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(l3gd20_vreg[i].vreg);
					goto error_vdd;
				}
			}

			rc = regulator_enable(l3gd20_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					l3gd20_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						l3gd20_vreg[i].vreg, 0,
						l3gd20_vreg[i].max_uV);
				}
				regulator_put(l3gd20_vreg[i].vreg);
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (regulator_count_voltages(l3gd20_vreg[i].vreg) > 0) {
			regulator_set_voltage(l3gd20_vreg[i].vreg, 0,
						l3gd20_vreg[i].max_uV);
		}
		regulator_disable(l3gd20_vreg[i].vreg);
		regulator_put(l3gd20_vreg[i].vreg);
	}
	return rc;
}




static int l3gd20_gyro_restart_fifo(struct l3gd20_gyro_data *data)
{
	int res = 0;

	res = i2c_smbus_write_byte_data(data->client,
			FIFO_CTRL_REG, BYPASS_MODE);
	if (res < 0) {
		pr_err("%s : failed to set bypass_mode\n", __func__);
		return res;
	}

	res = i2c_smbus_write_byte_data(data->client,
			FIFO_CTRL_REG, FIFO_MODE | (data->entries - 1));

	if (res < 0)
		pr_err("%s : failed to set fifo_mode\n", __func__);

	return res;
}

/* gyroscope data readout */
static int l3gd20_gyro_read_values(struct i2c_client *client,
				struct gyro_t *data, int total_read)
{
	int err;
	int len = sizeof(*data) * (total_read ? (total_read - 1) : 1);
	struct l3gd20_gyro_data *gyro_data = i2c_get_clientdata(client);
	struct i2c_msg msg[2];
	u8 reg_buf;

	msg[0].addr = client->addr;
	msg[0].buf = &reg_buf;
	msg[0].flags = 0;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = gyro_data->fifo_data;

	if (total_read > 1) {
		reg_buf = AXISDATA_REG | AC;
		msg[1].len = len;

		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2)
			return (err < 0) ? err : -EIO;
	}

	reg_buf = AXISDATA_REG | AC;
	msg[1].len = sizeof(*data);
	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	data->x = (gyro_data->fifo_data[1] << 8) | gyro_data->fifo_data[0];
	data->y = (gyro_data->fifo_data[3] << 8) | gyro_data->fifo_data[2];
	data->z = (gyro_data->fifo_data[5] << 8) | gyro_data->fifo_data[4];

	return 0;
}

static int l3gd20_gyro_report_values\
	(struct l3gd20_gyro_data *gyro_data)
{
	int res;
	struct gyro_t data;

	res = l3gd20_gyro_read_values(gyro_data->client, &data,
		gyro_data->entries);
	if (res < 0)
		return res;

	input_report_rel(gyro_data->input_dev, REL_RX, data.x);
	input_report_rel(gyro_data->input_dev, REL_RY, data.y);
	input_report_rel(gyro_data->input_dev, REL_RZ, data.z);
	input_sync(gyro_data->input_dev);

	l3gd20_gyro_restart_fifo(gyro_data);

#ifdef LOGGING_GYRO
	printk(KERN_INFO "%s, x = %d, y = %d, z = %d, count = %d\n"
		, __func__, data.x, data.y, data.z, gyro_data->count);
#endif

	return res;
}

static enum hrtimer_restart l3gd20_gyro_timer_func(struct hrtimer *timer)
{
	struct l3gd20_gyro_data *gyro_data\
		= container_of(timer, struct l3gd20_gyro_data, timer);
	
	queue_work(gyro_data->l3gd20_gyro_wq, &gyro_data->work);
	hrtimer_forward_now(&gyro_data->timer, gyro_data->polling_delay);
	return HRTIMER_RESTART;
}

static void l3gd20_gyro_work_func(struct work_struct *work)
{
	int res, retry = 3;
	struct l3gd20_gyro_data *gyro_data\
		= container_of(work, struct l3gd20_gyro_data, work);
	s32 status = 0;

	do {
		status = i2c_smbus_read_byte_data(gyro_data->client,
					STATUS_REG);
		if (status & 0x08)
			break;
	} while (retry--);

	if (status & 0x08) {
		res = l3gd20_gyro_read_values(gyro_data->client,
			&gyro_data->xyz_data, 0);
		if (res < 0)
			pr_err("%s, reading data fail(res = %d)\n",
				__func__, res);
	}

	//else
	//	pr_warn("%s, use last data.\n", __func__);

	input_report_rel(gyro_data->input_dev, REL_RX, gyro_data->xyz_data.x);
	input_report_rel(gyro_data->input_dev, REL_RY, gyro_data->xyz_data.y);
	input_report_rel(gyro_data->input_dev, REL_RZ, gyro_data->xyz_data.z);
	input_sync(gyro_data->input_dev);

#ifdef LOGGING_GYRO
	printk(KERN_INFO "%s, x = %d, y = %d, z = %d\n"
		, __func__, gyro_data->xyz_data.x, gyro_data->xyz_data.y,
		gyro_data->xyz_data.z);
#endif
}

static irqreturn_t l3gd20_gyro_interrupt_thread(int irq\
	, void *l3gd20_gyro_data_p)
{
	int res;
	struct l3gd20_gyro_data *data = l3gd20_gyro_data_p;
	
	if (unlikely(data->fifo_test)) {
		disable_irq_nosync(irq);
		
		complete(&data->data_ready);
		return IRQ_HANDLED;
	}
	
#ifdef LOGGING_GYRO
	data->count++;
#endif
	res = l3gd20_gyro_report_values(data);
	if (res < 0)
		pr_err("%s: failed to report gyro values\n", __func__);

	return IRQ_HANDLED;
}

static ssize_t l3gd20_gyro_selftest_dps_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->dps);
}

#ifdef DEBUG_REGISTER
static ssize_t register_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	struct i2c_msg msg[2];
	u8 reg1, reg2, reg3, reg4, reg5, fifo_ctrl, fifo_src;
	u8 reg_buf;
	int err;

	msg[0].addr = data->client->addr;
	msg[0].buf = &reg_buf;
	msg[0].flags = 0;
	msg[0].len = 1;

	reg_buf = CTRL_REG1;
	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &reg1;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = CTRL_REG2;
	msg[1].buf = &reg2;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = CTRL_REG3;
	msg[1].buf = &reg3;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = CTRL_REG4;
	msg[1].buf = &reg4;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = CTRL_REG5;
	msg[1].buf = &reg5;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = FIFO_CTRL_REG;
	msg[1].buf = &fifo_ctrl;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	reg_buf = FIFO_SRC_REG;
	msg[1].buf = &fifo_src;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2)
		return (err < 0) ? err : -EIO;

	return sprintf(buf, "0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n"\
		, reg1, reg2, reg3, reg4, reg5, fifo_ctrl, fifo_src);
}
#endif

static ssize_t l3gd20_gyro_selftest_dps_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	int new_dps = 0;
	int err;
	u8 ctrl;

	sscanf(buf, "%d", &new_dps);

	/* check out dps validity */
	if (new_dps != DPS250 && new_dps != DPS500 && new_dps != DPS2000) {
		pr_err("%s: wrong dps(%d)\n", __func__, new_dps);
		return -1;
	}

	ctrl = (data->ctrl_regs[3] & ~FS_MASK);

	if (new_dps == DPS250)
		ctrl |= FS_250DPS;
	else if (new_dps == DPS500)
		ctrl |= FS_500DPS;
	else if (new_dps == DPS2000)
		ctrl |= FS_2000DPS;
	else
		ctrl |= FS_DEFULAT_DPS;

	/* apply new dps */
	mutex_lock(&data->lock);
	data->ctrl_regs[3] = ctrl;

	err = i2c_smbus_write_byte_data(data->client, CTRL_REG4, ctrl);
	if (err < 0) {
		pr_err("%s: updating dps failed\n", __func__);
		mutex_unlock(&data->lock);
		return err;
	}
	mutex_unlock(&data->lock);

	data->dps = new_dps;
	pr_err("%s: %d dps stored\n", __func__, data->dps);

	return count;
}

static ssize_t l3gd20_gyro_show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data  = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->enable);
}

static ssize_t l3gd20_gyro_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct l3gd20_gyro_data *data  = dev_get_drvdata(dev);
	bool new_enable;

	if (sysfs_streq(buf, "1"))
		new_enable = true;
	else if (sysfs_streq(buf, "0"))
		new_enable = false;
	else {
		pr_debug("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	printk(KERN_INFO "%s, %d enable : %d.\n", __func__, __LINE__,\
		new_enable);

	if (new_enable == data->enable)
		return size;

	mutex_lock(&data->lock);
	if (new_enable) {
		/* turning on */
		err = i2c_smbus_write_i2c_block_data(data->client,
			CTRL_REG1 | AC, sizeof(data->ctrl_regs),
						data->ctrl_regs);
		if (err < 0) {
			err = -EIO;
			goto unlock;
		}

		if (data->interruptible) {
			enable_irq(data->client->irq);
			
			err = l3gd20_gyro_restart_fifo(data);
			if (err < 0) {
				err = -EIO;
				goto turn_off;
			}
		}

		if (!data->interruptible) {
			hrtimer_start(&data->timer,
				data->polling_delay, HRTIMER_MODE_REL);
		}
	} else {
		if (data->interruptible)
			disable_irq(data->client->irq);
		else {
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->work);
		}
		/* turning off */
		err = i2c_smbus_write_byte_data(data->client,
						CTRL_REG1, 0x00);
		if (err < 0)
			goto unlock;
	}
	data->enable = new_enable;

	printk(KERN_INFO "%s, %d l3gd20_gyro enable is done.\n",\
		__func__, __LINE__);

turn_off:
	if (err < 0)
		i2c_smbus_write_byte_data(data->client,
						CTRL_REG1, 0x00);
unlock:
	mutex_unlock(&data->lock);

	return err ? err : size;
}

static u64 l3gd20_gyro_get_delay_ns(struct l3gd20_gyro_data *k3)
{
	u64 delay = -1;

	delay = k3->time_to_read * k3->entries;
	delay = ktime_to_ns(ns_to_ktime(delay));

	return delay;
}

static int l3gd20_gyro_set_delay_ns(struct l3gd20_gyro_data *k3\
	, u64 delay_ns)
{
	int res = 0;
	int odr_value = ODR95_BW25;
	int i = 0, odr_table_size = ARRAY_SIZE(odr_delay_table) - 1;
	u8 ctrl;
	u64 new_delay = 0;

	mutex_lock(&k3->lock);
	if (!k3->interruptible) {
		hrtimer_cancel(&k3->timer);
		cancel_work_sync(&k3->work);
	} else
		disable_irq(k3->client->irq);

	/* round to the nearest supported ODR that is equal or above than
	 * the requested value
	 * 10ms(entries = 1), 20ms(entries = 2),
	 * 67ms(entries = 6), 200ms(entries = 20)
	 */
	if (delay_ns == 10000000LL && k3->interruptible)
		delay_ns = odr_delay_table[odr_table_size].delay_ns;

	if (delay_ns >= MAX_DELAY) {/* > max delay */
		pr_info("%s, %d (delay_ns >= MAX_DELAY), delay_ns = %lld\n",
			__func__, __LINE__, delay_ns);
		k3->entries = MAX_ENTRY;
		odr_value = odr_delay_table[odr_table_size].odr;
		k3->time_to_read = odr_delay_table[odr_table_size].delay_ns;
		new_delay = MAX_DELAY;
	} else if (delay_ns <= odr_delay_table[0].delay_ns) { /* < min delay */
		pr_info("%s, %d (delay_ns < MIN_DELAY), delay_ns=%lld\n",
						__func__, __LINE__, delay_ns);
		k3->entries = 1;
		odr_value = odr_delay_table[0].odr;
		k3->time_to_read = odr_delay_table[0].delay_ns;
		new_delay = odr_delay_table[0].delay_ns;
	} else {
		for (i = odr_table_size; i >= 0; i--) {
			if (delay_ns >= odr_delay_table[i].delay_ns) {
				pr_info("%s, %d (delay_ns < MAX_DELAY), delay_ns=%lld\n",
					__func__, __LINE__, delay_ns);
				new_delay = delay_ns;
				do_div(delay_ns, odr_delay_table[i].delay_ns);
				k3->entries = delay_ns;
				odr_value = odr_delay_table[i].odr;
				k3->time_to_read = odr_delay_table[i].delay_ns;
				break;
			}
		}
	}

	if (k3->interruptible)
		pr_info("%s, k3->entries=%d, odr_value=0x%x\n", __func__,
			k3->entries, odr_value);
	else
		pr_info("%s, odr_value=0x%x\n", __func__, odr_value);

	if (odr_value != (k3->ctrl_regs[0] & ODR_MASK)) {
		ctrl = (k3->ctrl_regs[0] & ~ODR_MASK);
		ctrl |= odr_value;
		k3->ctrl_regs[0] = ctrl;
		res = i2c_smbus_write_byte_data(k3->client, CTRL_REG1, ctrl);
	}

	if (k3->interruptible) {
		enable_irq(k3->client->irq);

		/* (re)start fifo */
		l3gd20_gyro_restart_fifo(k3);
	}

	if (!k3->interruptible) {
		pr_info("%s, %d delay_ns=%lld\n",
					__func__, __LINE__, new_delay);
		k3->polling_delay = ns_to_ktime(new_delay);
		if (k3->enable)
			hrtimer_start(&k3->timer,
				k3->polling_delay, HRTIMER_MODE_REL);
	}

	mutex_unlock(&k3->lock);

	return res;
}

static struct gyro_t s_data;

static ssize_t l3gd20_get_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *gyro_data  = dev_get_drvdata(dev);
	s32 status = 0;

	int res, retry = 3;

	do {
		status = i2c_smbus_read_byte_data(gyro_data->client,
					STATUS_REG);
		if (status & 0x08)
			break;
	} while (retry--);

	if (status & 0x08) {
		res = l3gd20_gyro_read_values(gyro_data->client,
			&s_data, 0);
		if (res < 0)
			printk("%s, reading data fail(res = %d)\n",
				__func__, res);
	} 

	//else
	//	printk("%s, use last data.\n", __func__);

	return sprintf(buf, "%d %d %d\n", s_data.x, s_data.y, s_data.z);

}

static ssize_t l3gd20_set_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t l3gd20_gyro_show_delay(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data  = dev_get_drvdata(dev);
	u64 delay;

	if (!data->interruptible)
		return sprintf(buf, "%lld\n", ktime_to_ns(data->polling_delay));

	delay = l3gd20_gyro_get_delay_ns(data);

	return sprintf(buf, "%lld\n", delay);
}

static ssize_t l3gd20_gyro_set_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int res;
	struct l3gd20_gyro_data *data  = dev_get_drvdata(dev);
	u64 delay_ns;

	res = strict_strtoll(buf, 10, &delay_ns);
	if (res < 0)
		return res;

	l3gd20_gyro_set_delay_ns(data, delay_ns);

	return size;
}

static DEVICE_ATTR(get_value, S_IRUGO | S_IWUSR | S_IWGRP,
			l3gd20_get_value, l3gd20_set_value);

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
			l3gd20_gyro_show_enable, l3gd20_gyro_set_enable);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
			l3gd20_gyro_show_delay, l3gd20_gyro_set_delay);

/*************************************************************************/
/* l3gd20_gyro Sysfs															 */
/*************************************************************************/

/* Device Initialization  */
static int device_init(struct l3gd20_gyro_data *data)
{
	int err;
	u8 buf[5];

	buf[0] = 0x6f;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	err = i2c_smbus_write_i2c_block_data(data->client,
					CTRL_REG1 | AC, sizeof(buf), buf);
	if (err < 0)
		pr_err("%s: CTRL_REGs i2c writing failed\n", __func__);

	return err;
}
static ssize_t l3gd20_gyro_power_off(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	int err;

	err = i2c_smbus_write_byte_data(data->client,
					CTRL_REG1, 0x00);

	printk(KERN_INFO "[%s] result of power off = %d\n", __func__, err);
	l3gd20_config_regulator(data,false);

	return sprintf(buf, "%d\n", (err < 0 ? 0 : 1));
}

static ssize_t l3gd20_gyro_power_on(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	int err;
	
	l3gd20_config_regulator(data,true);
	msleep(300);
	err = device_init(data);
	if (err < 0) {
		pr_err("%s: device_init() failed\n", __func__);
		return 0;
	}

	printk(KERN_INFO "[%s] result of device init = %d\n", __func__, err);

	return sprintf(buf, "%d\n", 1);
}

static ssize_t l3gd20_gyro_get_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	char temp;

	temp = i2c_smbus_read_byte_data(data->client, OUT_TEMP);
	if (temp < 0) {
		pr_err("%s: STATUS_REGS i2c reading failed\n", __func__);
		return 0;
	}

	printk(KERN_INFO "[%s] read temperature : %d\n", __func__, temp);

	return sprintf(buf, "%d\n", temp);
}

static int l3gd20_gyro_fifo_self_test(struct l3gd20_gyro_data *data)
{
	struct gyro_t raw_data;
	int err;
	int i, j;
	s16 raw[3] = { 0, };
	u8 reg[5];
	u8 fifo_pass = 2;
	u8 status_reg;

	data->fifo_test = true;

	/* fifo mode, enable interrupt, 500dps */
	reg[0] = 0x6F;
	reg[1] = 0x00;
	reg[2] = 0x04;
	reg[3] = 0x90;
	reg[4] = 0x40;

	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_i2c_block_data(data->client,
			CTRL_REG1 | AC,	sizeof(reg), reg);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s: CTRL_REGs i2c writing failed\n", __func__);
		goto exit;
	}

	/* Power up, wait for 800ms for stable output */
	msleep(800);

	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_byte_data(data->client,
				FIFO_CTRL_REG, BYPASS_MODE);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s : failed to set bypass_mode\n", __func__);
		goto exit;
	}

	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_byte_data(data->client,
				FIFO_CTRL_REG, FIFO_MODE | FIFO_TEST_WTM);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s: failed to set fifo_mode\n", __func__);
		goto exit;
	}

	/* if interrupt mode */
	if (!data->enable && data->interruptible) {
		enable_irq(data->client->irq);
		err = wait_for_completion_timeout(&data->data_ready, 5*HZ);
		msleep(200);
		if (err <= 0) {
			disable_irq(data->client->irq);
			if (!err)
				pr_err("%s: wait timed out\n", __func__);
			goto exit;
		}
	/* if polling mode */
	} else
		msleep(200);

	/* check out watermark status */
	status_reg = i2c_smbus_read_byte_data(data->client, FIFO_SRC_REG);
	if (!(status_reg & 0x80)) {
		pr_err("%s: Watermark level is not enough\n", __func__);
		goto exit;
	}

	/* read fifo entries */
	err = l3gd20_gyro_read_values(data->client,
				&raw_data, FIFO_TEST_WTM + 2);
	if (err < 0) {
		pr_err("%s: l3gd20_gyro_read_values() failed\n", __func__);
		goto exit;
	}

	/* print out fifo data */
	printk(KERN_INFO "[gyro_self_test] fifo data\n");
	for (i = 0; i < sizeof(raw_data) * (FIFO_TEST_WTM + 1);
		i += sizeof(raw_data)) {
		raw[0] = (data->fifo_data[i+1] << 8)
				| data->fifo_data[i];
		raw[1] = (data->fifo_data[i+3] << 8)
				| data->fifo_data[i+2];
		raw[2] = (data->fifo_data[i+5] << 8)
				| data->fifo_data[i+4];
		pr_err("%2dth: %8d %8d %8d\n", i/6, raw[0], raw[1], raw[2]);

		for (j = 0; j < 3; j++) {
			if (raw[j] < MIN_ZERO_RATE || raw[j] > MAX_ZERO_RATE) {
				pr_err("%s: %dth data(%d) is out of zero-rate",
					__func__, i/6, raw[j]);
				pr_err("%s: fifo test failed\n", __func__);
				fifo_pass = 0;
				goto exit;
			}
		}
	}

	fifo_pass = 1;

exit:
	data->fifo_test = false;

	/* 1: success, 0: fail, 2: retry */
	return fifo_pass;
}

static int l3gd20_gyro_bypass_self_test\
	(struct l3gd20_gyro_data *gyro_data, int NOST[3], int ST[3])
{
	int differ_x = 0, differ_y = 0, differ_z = 0;
	int err, i, j, sample_num = 6;
	s16 raw[3] = {0,};
	s32 temp = 0;
	u8 data[6] = {0,};
	u8 reg[5];
	u8 bZYXDA = 0;
	u8 bypass_pass = 2;

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	/* Set BDU=1, Set ODR=200Hz, Cut-Off Frequency=50Hz, FS=2000dps */
	reg[0] = 0x6f;
	reg[1] = 0x00;
	reg[2] = 0x00;
	reg[3] = 0xA0;
	reg[4] = 0x02;

	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_i2c_block_data(gyro_data->client,
			CTRL_REG1 | AC,	sizeof(reg), reg);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s: CTRL_REGs i2c writing failed\n", __func__);
		goto exit;
	}

	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_byte_data(gyro_data->client,
				FIFO_CTRL_REG, BYPASS_MODE);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s : failed to set bypass_mode\n", __func__);
		goto exit;
	}

	/* Power up, wait for 800ms for stable output */
	msleep(800);

	/* Read 5 samples output before self-test on */
	for (i = 0; i < 5; i++) {
		/* check ZYXDA ready bit */
		for (j = 0; j < 10; j++) {
			temp = i2c_smbus_read_byte_data(gyro_data->client,
							STATUS_REG);
			if (temp >= 0) {
				bZYXDA = temp & 0x08;
				if (!bZYXDA) {
					usleep_range(10000, 20000);
					pr_err("%s: %d,%d: no_data_ready",
							__func__, i, j);
					continue;
				} else
					break;
			}
		}
		if (temp < 0) {
			pr_err("%s: STATUS_REGS i2c reading failed\n",
								__func__);
			goto exit;
		}

		for (j = 0; j < 6; j++) {
			data[j] = i2c_smbus_read_byte_data(gyro_data->client,
							AXISDATA_REG+j);
		}
		if (i == 0)
			continue;

		raw[0] = (data[1] << 8) | data[0];
		raw[1] = (data[3] << 8) | data[2];
		raw[2] = (data[5] << 8) | data[4];

		NOST[0] += raw[0];
		NOST[1] += raw[1];
		NOST[2] += raw[2];

		printk(KERN_INFO "[gyro_self_test] raw[0] = %d\n", raw[0]);
		printk(KERN_INFO "[gyro_self_test] raw[1] = %d\n", raw[1]);
		printk(KERN_INFO "[gyro_self_test] raw[2] = %d\n\n", raw[2]);
	}

	for (i = 0; i < 3; i++)
		printk(KERN_INFO "[gyro_self_test] "
			"SUM of NOST[%d] = %d\n", i, NOST[i]);

	/* calculate average of NOST and covert from ADC to DPS */
	for (i = 0; i < 3; i++) {
		NOST[i] = (NOST[i] / 5) * 70 / 1000;
		printk(KERN_INFO "[gyro_self_test] "
			"AVG of NOST[%d] = %d\n", i, NOST[i]);
	}
	printk(KERN_INFO "\n");

	/* Enable Self Test */
	reg[0] = 0xA2;
	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_byte_data(gyro_data->client,
						CTRL_REG4, reg[0]);
		if (err >= 0)
			break;
	}
	if (temp < 0) {
		pr_err("%s: CTRL_REG4 i2c writing failed\n", __func__);
		goto exit;
	}

	msleep(100);

	/* Read 5 samples output after self-test on */
	/* The first data is useless on the l3gd20 selftest. */
	for (i = 0; i < sample_num; i++) {
		/* check ZYXDA ready bit */
		for (j = 0; j < 10; j++) {
			temp = i2c_smbus_read_byte_data(gyro_data->client,
							STATUS_REG);
			if (temp >= 0) {
				bZYXDA = temp & 0x08;
				if (!bZYXDA) {
					usleep_range(10000, 20000);
					pr_err("%s: %d,%d: no_data_ready",
							__func__, i, j);
					continue;
				} else
					break;
			}

		}
		if (temp < 0) {
			pr_err("%s: STATUS_REGS i2c reading failed\n",
								__func__);
			goto exit;
		}

		for (j = 0; j < 6; j++) {
			data[j] = i2c_smbus_read_byte_data(gyro_data->client,
							AXISDATA_REG+j);
		}
		if (i == 0)
			continue;

		raw[0] = (data[1] << 8) | data[0];
		raw[1] = (data[3] << 8) | data[2];
		raw[2] = (data[5] << 8) | data[4];

		ST[0] += raw[0];
		ST[1] += raw[1];
		ST[2] += raw[2];

		printk(KERN_INFO "[gyro_self_test] raw[0] = %d\n", raw[0]);
		printk(KERN_INFO "[gyro_self_test] raw[1] = %d\n", raw[1]);
		printk(KERN_INFO "[gyro_self_test] raw[2] = %d\n\n", raw[2]);
	}

	for (i = 0; i < 3; i++)
		printk(KERN_INFO "[gyro_self_test] "
			"SUM of ST[%d] = %d\n", i, ST[i]);

	/* calculate average of ST and convert from ADC to dps */
	for (i = 0; i < 3; i++)	{
		/* When FS=2000, 70 mdps/digit */
		ST[i] = (ST[i] / 5) * 70 / 1000;
		printk(KERN_INFO "[gyro_self_test] "
			"AVG of ST[%d] = %d\n", i, ST[i]);
	}

	/* check whether pass or not */
	if (ST[0] >= NOST[0]) /* for x */
		differ_x = ST[0] - NOST[0];
	else
		differ_x = NOST[0] - ST[0];

	if (ST[1] >= NOST[1]) /* for y */
		differ_y = ST[1] - NOST[1];
	else
		differ_y = NOST[1] - ST[1];

	if (ST[2] >= NOST[2]) /* for z */
		differ_z = ST[2] - NOST[2];
	else
		differ_z = NOST[2] - ST[2];

	printk(KERN_INFO "[gyro_self_test] differ x:%d, y:%d, z:%d\n",
						differ_x, differ_y, differ_z);

	if ((MIN_ST <= differ_x && differ_x <= MAX_ST)
		&& (MIN_ST <= differ_y && differ_y <= MAX_ST)
		&& (MIN_ST <= differ_z && differ_z <= MAX_ST))
		bypass_pass = 1;
	else
		bypass_pass = 0;

exit:
	return bypass_pass;
}

static ssize_t l3gd20_gyro_self_test(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyro_data *data = dev_get_drvdata(dev);
	int NOST[3] = { 0, }, ST[3] = { 0, };
	int err;
	int i;
	u8 backup_regs[5];
	u8 fifo_pass = 2;
	u8 bypass_pass = 2;

	/* before starting self-test, backup register */
	for (i = 0; i < 10; i++) {
		err = i2c_smbus_read_i2c_block_data(data->client,
			CTRL_REG1 | AC,	sizeof(backup_regs), backup_regs);
		if (err >= 0)
			break;
	}
	if (err < 0) {
		pr_err("%s: CTRL_REGs i2c reading failed\n", __func__);
		goto exit;
	}

	for (i = 0; i < 5; i++)
		printk(KERN_INFO "[gyro_self_test] "
			"backup reg[%d] = %2x\n", i, backup_regs[i]);

	/* fifo self test */
	printk(KERN_INFO "\n[gyro_self_test] fifo self-test\n");

	fifo_pass = l3gd20_gyro_fifo_self_test(data);
	if (fifo_pass)
		printk(KERN_INFO "[gyro_self_test] fifo self-test success\n");
	else if (!fifo_pass)
		printk(KERN_INFO "[gyro_self_test] fifo self-test fail\n");
	else
		printk(KERN_INFO "[gyro_self_test] fifo self-test restry\n");

	/* bypass self test */
	printk(KERN_INFO "\n[gyro_self_test] bypass self-test\n");

	bypass_pass = l3gd20_gyro_bypass_self_test(data, NOST, ST);
	if (bypass_pass)
		printk(KERN_INFO "[gyro_self_test] bypass self-test success\n\n");
	else if (!fifo_pass)
		printk(KERN_INFO "[gyro_self_test] bypass self-test fail\n\n");
	else
		printk(KERN_INFO "[gyro_self_test] bypass self-test restry\n\n");

	/* restore backup register */
	for (i = 0; i < 10; i++) {
		err = i2c_smbus_write_i2c_block_data(data->client,
			CTRL_REG1 | AC, sizeof(backup_regs),
			backup_regs);
		if (err >= 0)
			break;
	}
	if (err < 0)
		pr_err("%s: CTRL_REGs i2c writing failed\n", __func__);

exit:
	if (!data->enable) {
		/* If gyro is not enabled, make it go to the power down mode. */
		err = i2c_smbus_write_byte_data(data->client,
						CTRL_REG1, 0x00);
		if (err < 0)
			pr_err("%s: CTRL_REGs i2c writing failed\n", __func__);
	}

	if (fifo_pass == 2 && bypass_pass == 2)
		printk(KERN_INFO "[gyro_self_test] self-test result : retry\n");
	else
		printk(KERN_INFO "[gyro_self_test] self-test result : %s\n",
			fifo_pass & bypass_pass ? "pass" : "fail");

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
		NOST[0], NOST[1], NOST[2], ST[0], ST[1], ST[2],
		fifo_pass & bypass_pass, fifo_pass);
}

static DEVICE_ATTR(power_off, 0664,
	l3gd20_gyro_power_off, NULL);
static DEVICE_ATTR(power_on, 0664,
	l3gd20_gyro_power_on, NULL);
static DEVICE_ATTR(temperature, 0664,
	l3gd20_gyro_get_temp, NULL);
static DEVICE_ATTR(selftest, 0664,
	l3gd20_gyro_self_test, NULL);
static DEVICE_ATTR(selftest_dps, 0664,
	l3gd20_gyro_selftest_dps_show, l3gd20_gyro_selftest_dps_store);
#ifdef DEBUG_REGISTER
static DEVICE_ATTR(reg_data, 0664,
	register_data_show, NULL);
#endif

/*************************************************************************/
/* End of l3gd20_gyro Sysfs							 */
/*************************************************************************/
static int l3gd20_gyro_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
	int ret;
	int err = 0;
	int error;
	struct l3gd20_gyro_data *data;
	struct input_dev *input_dev;
	const struct l3gd20_gyro_platform_data *pdata = client->dev.platform_data;
	printk(">>Probing GYRO<<\n");	
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;
	data->drop_next_event = 0;

	l3gd20_config_regulator(data,true);
	msleep(300);

	/* read chip id */
	ret = i2c_smbus_read_byte_data(client, WHO_AM_I);
	if (ret != DEVICE_ID) {
		if (ret < 0) {
			pr_err("%s: i2c for reading chip id failed\n",
								__func__);
			err = ret;
		} else {
			pr_err("%s : Device identification failed\n",
								__func__);
			err = -ENODEV;
		}
		goto err_read_reg;
	}
	l3gd20_config_regulator(data,false);
	mutex_init(&data->lock);
	atomic_set(&data->opened, 0);
	init_completion(&data->data_ready);

	/* allocate gyro input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device;
	}

	data->input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = L3GD20_GYR_INPUT_NAME;
	/* X */
	input_set_capability(input_dev, EV_REL, REL_RX);
	/* Y */
	input_set_capability(input_dev, EV_REL, REL_RY);
	/* Z */
	input_set_capability(input_dev, EV_REL, REL_RZ);

	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(data->input_dev);
		goto err_input_register_device;
	}

	i2c_set_clientdata(client, data);
	dev_set_drvdata(&input_dev->dev, data);
	
	if (gpio_is_valid(pdata->gpio_int1)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_int1,
							"l3gd20_gyro_irq_gpio");
		if (error) {
			pr_err("%s: unable to request gpio [%d]\n", __func__,
						pdata->gpio_int1);
			//goto err_power_on;
		}
		error = gpio_direction_input(pdata->gpio_int1);
		if (error) {
			pr_err("%s: unable to set_direction for gpio [%d]\n",
					__func__, pdata->gpio_int1);
			//goto err_irq_gpio_req;
		}
	}
	
	//if (data->client->irq > 0) {
    if (false) {
		memcpy(&data->ctrl_regs, &default_ctrl_regs_fifo,
			sizeof(default_ctrl_regs_fifo));
		data->interruptible = true;
		data->entries = 1;
		err = request_threaded_irq(data->client->irq, NULL,
			l3gd20_gyro_interrupt_thread\
			, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,\
				"l3gd20_gyro", data);
		if (err < 0) {
			pr_err("%s: can't allocate irq.\n", __func__);
			goto err_request_irq;
		}
		disable_irq(data->client->irq);
	} else {
		memcpy(&data->ctrl_regs, &default_ctrl_regs_bypass,
			sizeof(default_ctrl_regs_bypass));
		data->ctrl_regs[2] = 0x00;
		hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		data->polling_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
		//data->polling_delay = ns_to_ktime(1 * NSEC_PER_MSEC);

		data->timer.function = l3gd20_gyro_timer_func;
		
		data->l3gd20_gyro_wq \
		= create_singlethread_workqueue("l3gd20_gyro_wq");
		if (!data->l3gd20_gyro_wq) {
			err = -ENOMEM;
			pr_err("%s: could not create workqueue\n", __func__);
			goto err_create_workqueue;
		}
		INIT_WORK(&data->work, l3gd20_gyro_work_func);
	}

	if (device_create_file(&input_dev->dev,
				&dev_attr_enable) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
				dev_attr_enable.attr.name);
		goto err_device_create_file;
	}

	if (device_create_file(&input_dev->dev,
				&dev_attr_poll_delay) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
				dev_attr_poll_delay.attr.name);
		goto err_device_create_file2;
	}

	if (device_create_file(&input_dev->dev,
				&dev_attr_get_value) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
				dev_attr_get_value.attr.name);
		goto err_device_create_file2;
	}

	/* create device node for l3gd20_gyro digital gyroscope */
	data->dev = sensors_classdev_register("gyro_sensor");

	if (IS_ERR(data->dev)) {
		pr_err("%s: Failed to create device(gyro)\n", __func__);
		err = PTR_ERR(data->dev);
		goto err_device_create;
	}

	if (device_create_file(data->dev, &dev_attr_power_on) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_power_on.attr.name);
		goto err_device_create_file3;
	}

	if (device_create_file(data->dev, &dev_attr_power_off) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_power_off.attr.name);
		goto err_device_create_file4;
	}

	if (device_create_file(data->dev, &dev_attr_temperature) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_temperature.attr.name);
		goto err_device_create_file5;
	}

	if (device_create_file(data->dev, &dev_attr_selftest) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_selftest.attr.name);
		goto err_device_create_file6;
	}

	if (device_create_file(data->dev, &dev_attr_selftest_dps) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_selftest_dps.attr.name);
		goto err_device_create_file7;
	}

#ifdef DEBUG_REGISTER
	if (device_create_file(data->dev, &dev_attr_reg_data) < 0) {
		pr_err("%s: Failed to create device file(%s)!\n", __func__,
			dev_attr_reg_data.attr.name);
		goto err_device_create_file8;
	}
#endif

	dev_set_drvdata(data->dev, data);

	return 0;

#ifdef DEBUG_REGISTER
err_device_create_file8:
	device_remove_file(data->dev, &dev_attr_selftest_dps);
#endif
err_device_create_file7:
	device_remove_file(data->dev, &dev_attr_selftest);
err_device_create_file6:
	device_remove_file(data->dev, &dev_attr_temperature);
err_device_create_file5:
	device_remove_file(data->dev, &dev_attr_power_off);
err_device_create_file4:
	device_remove_file(data->dev, &dev_attr_power_on);
err_device_create_file3:
	sensors_classdev_unregister(data->dev);
err_device_create:
	device_remove_file(&input_dev->dev, &dev_attr_poll_delay);
err_device_create_file2:
	device_remove_file(&input_dev->dev, &dev_attr_enable);
err_device_create_file:
	if (data->interruptible)
		free_irq(data->client->irq, data);
	else
		destroy_workqueue(data->l3gd20_gyro_wq);
	input_unregister_device(data->input_dev);
err_create_workqueue:
err_request_irq:
err_input_register_device:
err_input_allocate_device:
	mutex_destroy(&data->lock);
err_read_reg:
	kfree(data);
exit:
	return err;
}

static int l3gd20_gyro_remove(struct i2c_client *client)
{
	int err = 0;
	struct l3gd20_gyro_data *data = i2c_get_clientdata(client);

	device_remove_file(data->dev, &dev_attr_selftest_dps);
	device_remove_file(data->dev, &dev_attr_selftest);
	device_remove_file(data->dev, &dev_attr_temperature);
	device_remove_file(data->dev, &dev_attr_power_on);
	device_remove_file(data->dev, &dev_attr_power_off);
#ifdef DEBUG_REGISTER
	device_remove_file(data->dev, &dev_attr_reg_data);
#endif
	sensors_classdev_unregister(data->dev);

	if (data->interruptible) {
		if (data->enable)
			disable_irq(data->client->irq);
		free_irq(data->client->irq, data);
	} else {
		hrtimer_cancel(&data->timer);
	    cancel_work_sync(&data->work);
		destroy_workqueue(data->l3gd20_gyro_wq);
	}

	if (data->enable)
		err = i2c_smbus_write_byte_data(data->client,
					CTRL_REG1, 0x00);

	device_remove_file(&data->input_dev->dev, &dev_attr_enable);
	device_remove_file(&data->input_dev->dev, &dev_attr_poll_delay);
	input_unregister_device(data->input_dev);
	mutex_destroy(&data->lock);
	kfree(data);

	return err;
}

static int l3gd20_gyro_suspend(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyro_data *data = i2c_get_clientdata(client);

	if (data->enable) {
		mutex_lock(&data->lock);
		if (!data->interruptible) {
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->work);
		} else
			disable_irq(data->client->irq);

		err = i2c_smbus_write_byte_data(data->client,
						CTRL_REG1, 0x00);
		mutex_unlock(&data->lock);
	}

	return err;
}

static int l3gd20_gyro_resume(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyro_data *data = i2c_get_clientdata(client);

	if (data->enable) {
		mutex_lock(&data->lock);
		err = i2c_smbus_write_i2c_block_data(client,
				CTRL_REG1 | AC, sizeof(data->ctrl_regs),
							data->ctrl_regs);

		if (data->interruptible) {
			enable_irq(data->client->irq);

			l3gd20_gyro_restart_fifo(data);
		}

		if (!data->interruptible)
			hrtimer_start(&data->timer,
				data->polling_delay, HRTIMER_MODE_REL);

		mutex_unlock(&data->lock);
	}

	return err;
}

static const struct dev_pm_ops l3gd20_gyro_pm_ops = {
	.suspend = l3gd20_gyro_suspend,
	.resume = l3gd20_gyro_resume
};

static const struct i2c_device_id l3gd20_gyro_id[] = {
	{ L3GD20_GYRO_DEV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, l3gd20_gyro_id);

static struct i2c_driver l3gd20_gyro_driver = {
	.probe = l3gd20_gyro_probe,
	.remove = __devexit_p(l3gd20_gyro_remove),
	.id_table = l3gd20_gyro_id,
	.driver = {
		.pm = &l3gd20_gyro_pm_ops,
		.owner = THIS_MODULE,
		.name = L3GD20_GYRO_DEV_NAME,
	},
};

static int __init l3gd20_gyro_init(void)
{
	return i2c_add_driver(&l3gd20_gyro_driver);
}

static void __exit l3gd20_gyro_exit(void)
{
	i2c_del_driver(&l3gd20_gyro_driver);
}

module_init(l3gd20_gyro_init);
module_exit(l3gd20_gyro_exit);

MODULE_DESCRIPTION("l3gd20 digital gyroscope driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
