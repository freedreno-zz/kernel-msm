/*
 * lsm303dlhc_m.c
 * ST 3-Axis Magnetometer Driver
 *
 * Copyright (C) 2010 STMicroelectronics
 * Author: Carmine Iascone (carmine.iascone at st.com)
 * Author: Matteo Dameno (matteo.dameno at st.com)
 *
 * Copyright (C) 2010 STEricsson
 * Author: Mian Yousaf Kaukab <mian.yousaf.kaukab at stericsson.com>
 * Updated:Preetham Rao Kaskurthi <preetham.rao at stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG 1
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
#include <linux/input.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#endif

#include <linux/sensor/lsm303dlhc.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/sensor/sensors_core.h>
/* lsm303dlhc magnetometer registers */
#define WHO_AM_I	0x0F

/* Magnetometer registers */
#define CRA_REG_M	0x00  /* Configuration register A */
#define CRB_REG_M	0x01  /* Configuration register B */
#define MR_REG_M	0x02  /* Mode register */
#define SR_REG_M    0x09  /* Status register */

/* Output register start address*/
#define OUT_X_M		0x03
#define OUT_Y_M		0x05
#define OUT_Z_M		0x07

/* Magnetometer X-Y gain  */
#define XY_GAIN_1_3	1100 /* XY gain at 1.3G */
#define XY_GAIN_1_9	 855 /* XY gain at 1.9G */
#define XY_GAIN_2_5	 670 /* XY gain at 2.5G */
#define XY_GAIN_4_0	 450 /* XY gain at 4.0G */
#define XY_GAIN_4_7	 400 /* XY gain at 4.7G */
#define XY_GAIN_5_6	 330 /* XY gain at 5.6G */
#define XY_GAIN_8_1	 230 /* XY gain at 8.1G */

/* Magnetometer Z gain  */
#define Z_GAIN_1_3	980 /* Z gain at 1.3G */
#define Z_GAIN_1_9	760 /* Z gain at 1.9G */
#define Z_GAIN_2_5	600 /* Z gain at 2.5G */
#define Z_GAIN_4_0	400 /* Z gain at 4.0G */
#define Z_GAIN_4_7	355 /* Z gain at 4.7G */
#define Z_GAIN_5_6	295 /* Z gain at 5.6G */
#define Z_GAIN_8_1	205 /* Z gain at 8.1G */

/* Control A regsiter. */
#define LSM303DLH_M_CRA_TEMP_EN_BIT 7
#define LSM303DLH_M_CRA_TEMP_EN_MASK (0x01 <<  LSM303DLH_M_CRA_TEMP_EN_BIT)

#define LSM303DLH_M_CRA_DO_BIT 2
#define LSM303DLH_M_CRA_DO_MASK (0x7 <<  LSM303DLH_M_CRA_DO_BIT)
#define LSM303DLH_M_CRA_MS_BIT 0
#define LSM303DLH_M_CRA_MS_MASK (0x3 <<  LSM303DLH_M_CRA_MS_BIT)

/* Control B regsiter. */
#define LSM303DLH_M_CRB_GN_BIT 5
#define LSM303DLH_M_CRB_GN_MASK (0x7 <<  LSM303DLH_M_CRB_GN_BIT)

/* Control Mode regsiter. */
#define LSM303DLH_M_MR_MD_BIT 0
#define LSM303DLH_M_MR_MD_MASK (0x3 <<  LSM303DLH_M_MR_MD_BIT)

/* Control Status regsiter. */
#define LSM303DLH_M_SR_RDY_BIT 0
#define LSM303DLH_M_SR_RDY_MASK (0x1 <<  LSM303DLH_M_SR_RDY_BIT)
#define LSM303DLH_M_SR_LOC_BIT 1
#define LSM303DLH_M_SR_LCO_MASK (0x1 <<  LSM303DLH_M_SR_LOC_BIT)

/* Magnetometer gain setting */
#define LSM303DLH_M_RANGE_1_3G  0x01
#define LSM303DLH_M_RANGE_1_9G  0x02
#define LSM303DLH_M_RANGE_2_5G  0x03
#define LSM303DLH_M_RANGE_4_0G  0x04
#define LSM303DLH_M_RANGE_4_7G  0x05
#define LSM303DLH_M_RANGE_5_6G  0x06
#define LSM303DLH_M_RANGE_8_1G  0x07

/* Magnetometer capturing mode  */
#define LSM303DLH_M_MODE_CONTINUOUS 0
#define LSM303DLH_M_MODE_SINGLE 1
#define LSM303DLH_M_MODE_SLEEP 3

/* Magnetometer output data rate */
#define LSM303DLH_M_RATE_00_75 0x00
#define LSM303DLH_M_RATE_01_50 0x01
#define LSM303DLH_M_RATE_03_00 0x02
#define LSM303DLH_M_RATE_07_50 0x03
#define LSM303DLH_M_RATE_15_00 0x04
#define LSM303DLH_M_RATE_30_00 0x05
#define LSM303DLH_M_RATE_75_00 0x06
#define LSM303DLH_M_RATE_220_00 0x07

/* Multiple byte transfer enable */
#define MULTIPLE_I2C_TR 0x80

#define I2C_WR_ERR "i2c_smbus_write_byte_data failed error"

/*
 * magnetometer local data
 */
struct lsm303dlhc_m_data {
	struct i2c_client *client;
	/* lock for sysfs operations */
	struct mutex lock;

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
	struct input_dev *input_dev;
#endif
	struct regulator *regulator;
	struct lsm303dlhc_platform_data pdata;

	short gain[3];
	short data[3];
	unsigned char mode;
	unsigned char rate;
	unsigned char range;
#ifdef DEBUG
	u8 reg_addr;
#endif
	struct device *dev;
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator lis3dh_mag_vreg[] = {
	{NULL, "vdd2v9", 2400000, 3300000},
	{NULL, "vdd1v8", 1800000, 1900000},
};

static int lis3dh_acc_config_regulator(struct lsm303dlhc_m_data *acc, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(lis3dh_mag_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			lis3dh_mag_vreg[i].vreg =
				regulator_get(&acc->client->dev,
				lis3dh_mag_vreg[i].name);
			if (IS_ERR(lis3dh_mag_vreg[i].vreg)) {
				rc = PTR_ERR(lis3dh_mag_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				goto error_vdd;
			}

			if (regulator_count_voltages(
				lis3dh_mag_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					lis3dh_mag_vreg[i].vreg,
					lis3dh_mag_vreg[i].min_uV,
					lis3dh_mag_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(lis3dh_mag_vreg[i].vreg);
					goto error_vdd;
				}
			}

			rc = regulator_enable(lis3dh_mag_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					lis3dh_mag_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						lis3dh_mag_vreg[i].vreg, 0,
						lis3dh_mag_vreg[i].max_uV);
				}
				regulator_put(lis3dh_mag_vreg[i].vreg);
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (regulator_count_voltages(lis3dh_mag_vreg[i].vreg) > 0) {
			regulator_set_voltage(lis3dh_mag_vreg[i].vreg, 0,
						lis3dh_mag_vreg[i].max_uV);
		}
		regulator_disable(lis3dh_mag_vreg[i].vreg);
		regulator_put(lis3dh_mag_vreg[i].vreg);
	}
	return rc;
}


static int lsm303dlhc_m_set_mode(struct lsm303dlhc_m_data *ddata,
		unsigned char mode);
static int lsm303dlhc_m_write(struct lsm303dlhc_m_data *ddata,
		u8 reg, u8 val, char *msg)
{
	int ret = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (ret < 0)
		dev_err(&ddata->client->dev, "%s %d Register (%s)\n",\
					I2C_WR_ERR, ret, msg);
	return ret;
}

static int lsm303dlhc_m_restore(struct lsm303dlhc_m_data *ddata)
{
	int ret = 0;

	ret = lsm303dlhc_m_write(ddata, CRB_REG_M, ddata->range, "SET RANGE");

	if (ret < 0)
		goto fail;

	ret = lsm303dlhc_m_write(ddata, CRA_REG_M, ddata->rate, "SET RATE");

	if (ret < 0)
		goto fail;

	ret = lsm303dlhc_m_set_mode(ddata, ddata->mode);

	if (ret < 0)
		goto fail;

fail:
	return ret;
}

static int lsm303dlhc_m_read_multi(struct lsm303dlhc_m_data *ddata, u8 reg,
		u8 count, u8 *val, char *msg)
{
	int ret = i2c_smbus_read_i2c_block_data(ddata->client,
		   reg | MULTIPLE_I2C_TR, count, val);
	if (ret < 0)
		dev_err(&ddata->client->dev, "%s %d Register (%s)\n",\
					I2C_WR_ERR, ret, msg);
	return ret;
}

static ssize_t lsm303dlhc_m_show_rate(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlhc_m_data *ddata = platform_get_drvdata(pdev);

	return snprintf(buf, 64, "%d\n", ddata->rate >> LSM303DLH_M_CRA_DO_BIT);
}

/* set lsm303dlhc magnetometer bandwidth */
static ssize_t lsm303dlhc_m_store_rate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlhc_m_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	unsigned char data;
	int error;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
				"device is switched off,make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	mutex_lock(&ddata->lock);

	data = ((val << LSM303DLH_M_CRA_DO_BIT) & LSM303DLH_M_CRA_DO_MASK);
	ddata->rate = data;

	error = lsm303dlhc_m_write(ddata, CRA_REG_M, data, "SET RATE");

	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static int lsm303dlhc_m_xyz_read(struct lsm303dlhc_m_data *ddata)
{
	unsigned char xyz_data[6];
	int ret = lsm303dlhc_m_read_multi(ddata, OUT_X_M,
			6, xyz_data, "OUT_X_M");
	if (ret < 0)
		return -EINVAL;

	/* MSB is at lower address */
	ddata->data[0] = (short)
		(((xyz_data[0]) << 8) | xyz_data[1]);
	ddata->data[1] = (short)
		(((xyz_data[2]) << 8) | xyz_data[3]);
	ddata->data[2] = (short)
		(((xyz_data[4]) << 8) | xyz_data[5]);

	/* taking orientation of x,y,z axis into account*/

	ddata->data[ddata->pdata.axis_map_x] = ddata->pdata.negative_x ?
		-ddata->data[ddata->pdata.axis_map_x] :
		ddata->data[ddata->pdata.axis_map_x];
	ddata->data[ddata->pdata.axis_map_y] = ddata->pdata.negative_y ?
		-ddata->data[ddata->pdata.axis_map_y] :
		ddata->data[ddata->pdata.axis_map_y];
	ddata->data[ddata->pdata.axis_map_z] = ddata->pdata.negative_z ?
		-ddata->data[ddata->pdata.axis_map_z] :
		ddata->data[ddata->pdata.axis_map_z];

	return ret;
}

static ssize_t lsm303dlhc_m_gain(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);

	return snprintf(buf, 64, "%8x:%8x:%8x\n",
			ddata->gain[ddata->pdata.axis_map_x],
			ddata->gain[ddata->pdata.axis_map_y],
			ddata->gain[ddata->pdata.axis_map_z]);
}

static ssize_t lsm303dlhc_m_values(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	int ret = 0;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
				"device is switched off,make it ON using MODE");
		return ret;
	}

	mutex_lock(&ddata->lock);

	ret = lsm303dlhc_m_xyz_read(ddata);
	if (ret < 0) {
		mutex_unlock(&ddata->lock);
		return -EINVAL;
	}

	mutex_unlock(&ddata->lock);

	/* taking orientation of x,y,z axis into account*/

	return snprintf(buf, 64, "%8x:%8x:%8x\n",
			ddata->data[ddata->pdata.axis_map_x],
			ddata->data[ddata->pdata.axis_map_y],
			ddata->data[ddata->pdata.axis_map_z]);
}

static int lsm303dlhc_m_set_mode(struct lsm303dlhc_m_data *ddata,
		unsigned char mode)
{
	int ret = -1;
	mode = (mode << LSM303DLH_M_MR_MD_BIT);
	ret = i2c_smbus_write_byte_data(ddata->client, MR_REG_M, mode);

	if (ret < 0)
		dev_err(&ddata->client->dev, "%s %d Register (%s)\n",\
					I2C_WR_ERR, ret, "MODE CONTROL");

	return ret;
}

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE

static irqreturn_t lsm303dlhc_m_gpio_irq(int irq, void *device_data)
{
	struct lsm303dlhc_m_data *ddata = device_data;
	int ret;
	ret = lsm303dlhc_m_xyz_read(ddata);

	if (ret < 0) {
		dev_err(&ddata->client->dev,
				"reading data of xyz failed error %d\n", ret);
		return IRQ_NONE;
	}

	/* taking orientation of x,y,z axis into account*/

	input_report_abs(ddata->input_dev, ABS_X,
			ddata->data[ddata->pdata.axis_map_x]);
	input_report_abs(ddata->input_dev, ABS_Y,
		   ddata->data[ddata->pdata.axis_map_y]);
	input_report_abs(ddata->input_dev, ABS_Z,
		   ddata->data[ddata->pdata.axis_map_z]);
	input_sync(ddata->input_dev);

	return IRQ_HANDLED;

}
#endif

static ssize_t lsm303dlhc_m_show_range(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);

	return snprintf(buf, 64, "%d\n",\
				ddata->range >> LSM303DLH_M_CRB_GN_BIT);
}

static ssize_t lsm303dlhc_m_store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	short xy_gain;
	short z_gain;
	unsigned long range;

	int error;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
				"device is switched off,make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &range);
	if (error)
		return error;

	switch (range) {
	case LSM303DLH_M_RANGE_1_3G:
		xy_gain = XY_GAIN_1_3;
		z_gain = Z_GAIN_1_3;
		break;
	case LSM303DLH_M_RANGE_1_9G:
		xy_gain = XY_GAIN_1_9;
		z_gain = Z_GAIN_1_9;
		break;
	case LSM303DLH_M_RANGE_2_5G:
		xy_gain = XY_GAIN_2_5;
		z_gain = Z_GAIN_2_5;
		break;
	case LSM303DLH_M_RANGE_4_0G:
		xy_gain = XY_GAIN_4_0;
		z_gain = Z_GAIN_4_0;
		break;
	case LSM303DLH_M_RANGE_4_7G:
		xy_gain = XY_GAIN_4_7;
		z_gain = Z_GAIN_4_7;
		break;
	case LSM303DLH_M_RANGE_5_6G:
		xy_gain = XY_GAIN_5_6;
		z_gain = Z_GAIN_5_6;
		break;
	case LSM303DLH_M_RANGE_8_1G:
		xy_gain = XY_GAIN_8_1;
		z_gain = Z_GAIN_8_1;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&ddata->lock);

	ddata->gain[ddata->pdata.axis_map_x] = xy_gain;
	ddata->gain[ddata->pdata.axis_map_y] = xy_gain;
	ddata->gain[ddata->pdata.axis_map_z] = z_gain;

	range <<= LSM303DLH_M_CRB_GN_BIT;
	range &= LSM303DLH_M_CRB_GN_MASK;

	ddata->range = range;

	error = lsm303dlhc_m_write(ddata, CRB_REG_M, range, "SET RANGE");
	mutex_unlock(&ddata->lock);

	if (error < 0)
		return error;

	return count;
}

static ssize_t lsm303dlhc_m_show_mode(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);

	return snprintf(buf, 64, "%d\n", ddata->mode);
}

static ssize_t lsm303dlhc_m_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	unsigned long mode;
	int error;

	error = strict_strtoul(buf, 0, &mode);
	if (error)
		return error;

	/*  if same mode as existing, return */
	if (ddata->mode == mode) {
		printk(KERN_INFO "%s---mode are the same\n", __func__);
		return 0xff;
	}

	/* turn on the supplies if already off */
	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		regulator_enable(lis3dh_mag_vreg[0].vreg);
		regulator_enable(lis3dh_mag_vreg[1].vreg);

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
		enable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif
	}

	mutex_lock(&ddata->lock);

	error = lsm303dlhc_m_set_mode(ddata, mode);

	ddata->mode = (mode >> LSM303DLH_M_MR_MD_BIT);
	mutex_unlock(&ddata->lock);

	if (error < 0)
		return error;

	if (mode == LSM303DLH_M_MODE_SLEEP) {

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
		disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

		/*
		 * No need to store context here, it is not like
		 * suspend/resume but fall back to default values
		 */
		ddata->rate = LSM303DLH_M_RATE_00_75;
		ddata->range = LSM303DLH_M_RANGE_1_3G;
		ddata->range <<= LSM303DLH_M_CRB_GN_BIT;
		ddata->range &= LSM303DLH_M_CRB_GN_MASK;
		ddata->gain[ddata->pdata.axis_map_x] = XY_GAIN_1_3;
		ddata->gain[ddata->pdata.axis_map_y] = XY_GAIN_1_3;
		ddata->gain[ddata->pdata.axis_map_z] = Z_GAIN_1_3;
		regulator_disable(lis3dh_mag_vreg[0].vreg);
		regulator_disable(lis3dh_mag_vreg[1].vreg);
	}

	return count;
}


/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{

	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	int rc;
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&ddata->lock);
	x[0] = ddata->reg_addr;
	mutex_unlock(&ddata->lock);
	x[1] = val;
	rc = i2c_smbus_write_byte_data(ddata->client, ddata->reg_addr, x[1]);
	if (rc < 0)
		printk(KERN_INFO "%s----ERR-reg_addr:=%x value:=%s\n",\
					__func__, ddata->reg_addr, buf);
	else
		printk(KERN_INFO "%s---OK--reg_addr:=%x value:=%s\n",\
					__func__, ddata->reg_addr, buf);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	ssize_t ret;
	u8 data;

	mutex_lock(&ddata->lock);
	data = i2c_smbus_read_byte_data(ddata->client, ddata->reg_addr);
	mutex_unlock(&ddata->lock);

	/* TODO: error need to be managed */
	printk(KERN_INFO "%s-----reg_addr:=%02x value:=%02x\n",\
				__func__, ddata->reg_addr, data);
	ret = snprintf(buf, 8, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303dlhc_m_data *ddata = dev_get_drvdata(dev);
	unsigned long val;
	int rc;
	u8 data;
	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&ddata->lock);
	ddata->reg_addr = val;
	mutex_unlock(&ddata->lock);
	data = val;

	rc = i2c_smbus_write_byte_data(ddata->client, ddata->reg_addr, data);

	if (rc < 0)
		printk(KERN_INFO "%s----ERR-reg_addr:=%x\n",\
						__func__, ddata->reg_addr);
	else
		printk(KERN_INFO "%s---OK--reg_addr:=%x\n",\
						__func__, ddata->reg_addr);
	return size;
}


static DEVICE_ATTR(gain, S_IRUGO, lsm303dlhc_m_gain, NULL);

static DEVICE_ATTR(data, S_IRUGO, lsm303dlhc_m_values, NULL);

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR | S_IWGRP,
		lsm303dlhc_m_show_mode, lsm303dlhc_m_store_mode);

static DEVICE_ATTR(range, S_IRUGO | S_IWUSR | S_IWGRP,
		lsm303dlhc_m_show_range, lsm303dlhc_m_store_range);

static DEVICE_ATTR(rate, S_IRUGO | S_IWUSR | S_IWGRP,
		lsm303dlhc_m_show_rate, lsm303dlhc_m_store_rate);
#ifdef DEBUG
static DEVICE_ATTR(reg_value, S_IRUGO | S_IWUSR | S_IWGRP,\
					attr_reg_get, attr_reg_set);
static DEVICE_ATTR(reg_addr,  S_IWUSR | S_IWGRP , NULL, attr_addr_set);
#endif

static struct attribute *lsm303dlhc_m_attributes[] = {
	&dev_attr_gain.attr,
	&dev_attr_data.attr,
	&dev_attr_mode.attr,
	&dev_attr_range.attr,
	&dev_attr_rate.attr,
#ifdef DEBUG
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
#endif
	NULL
};

static const struct attribute_group lsm303dlhc_m_attr_group = {
	.attrs = lsm303dlhc_m_attributes,
};

static int __devinit lsm303dlhc_m_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret;
	struct lsm303dlhc_m_data *ddata = NULL;
	unsigned char version[3];
	ddata = kzalloc(sizeof(struct lsm303dlhc_m_data), GFP_KERNEL);
	if (ddata == NULL) {
		ret = -ENOMEM;
		goto err_op_failed;
	}
	printk(KERN_INFO "%s >> Probing magnetometer <<\n", __func__);
	ddata->client = client;
	i2c_set_clientdata(client, ddata);

	/* copy platform specific data */
	memcpy(&ddata->pdata, client->dev.platform_data, sizeof(ddata->pdata));

	ddata->mode = LSM303DLH_M_MODE_SLEEP;
	ddata->rate = LSM303DLH_M_RATE_00_75;
	ddata->range = LSM303DLH_M_RANGE_1_3G;
	ddata->range <<= LSM303DLH_M_CRB_GN_BIT;
	ddata->range &= LSM303DLH_M_CRB_GN_MASK;
	ddata->gain[ddata->pdata.axis_map_x] = XY_GAIN_1_3;
	ddata->gain[ddata->pdata.axis_map_y] = XY_GAIN_1_3;
	ddata->gain[ddata->pdata.axis_map_z] = Z_GAIN_1_3;
	dev_set_name(&client->dev, ddata->pdata.name_m);
	lis3dh_acc_config_regulator(ddata, true);
	msleep(300);
	ret = lsm303dlhc_m_read_multi(ddata, WHO_AM_I, 1, version, "WHO_AM_I");
	if (ret < 0)
		goto exit_free_regulator;

	dev_info(&client->dev, "magnetometer, WHO_AM_I: %02x\n", version[0]);
	mutex_init(&ddata->lock);
	ddata->dev = sensors_classdev_register("magnetometer");
	ret = sysfs_create_group(&ddata->dev->kobj, &lsm303dlhc_m_attr_group);
	if (ret)
		goto exit_free_regulator;

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE

	ddata->input_dev = input_allocate_device();
	if (!ddata->input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto exit_free_regulator;
	}
	dev_set_drvdata(ddata->dev, ddata);
	set_bit(EV_ABS, ddata->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_X, -32768, 32767, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Y, -32768, 32767, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Z, -32768, 32767, 0, 0);

	ddata->input_dev->name = "magnetometer";

	ret = input_register_device(ddata->input_dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register input device: %s\n",
				ddata->input_dev->name);
		goto err_input_register_failed;
	}

	/* register interrupt */
	ret = request_threaded_irq(gpio_to_irq(ddata->pdata.irq_m), NULL,
		   lsm303dlhc_m_gpio_irq,
		   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "lsm303dlhc_m",
		   ddata);
	if (ret) {
		dev_err(&client->dev, "request irq EGPIO_PIN_1 failed\n");
		goto err_input_failed;
	}

	disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

	return ret;

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
err_input_failed:
	input_unregister_device(ddata->input_dev);
err_input_register_failed:
	input_free_device(ddata->input_dev);
#endif
exit_free_regulator:
	lis3dh_acc_config_regulator(ddata, false);
err_op_failed:
	dev_err(&client->dev, " lsm303dlhc_m_probe failed %x", ret);
	kfree(ddata);
	return ret;
}

static int __devexit lsm303dlhc_m_remove(struct i2c_client *client)
{
	struct lsm303dlhc_m_data *ddata;

	ddata = i2c_get_clientdata(client);

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
	input_unregister_device(ddata->input_dev);
	input_free_device(ddata->input_dev);
#endif

	sysfs_remove_group(&ddata->dev->kobj, &lsm303dlhc_m_attr_group);

	/* safer to make device off */
	if (ddata->mode != LSM303DLH_M_MODE_SLEEP) {
		lsm303dlhc_m_set_mode(ddata, LSM303DLH_M_MODE_SLEEP);
		regulator_disable(lis3dh_mag_vreg[0].vreg);
		regulator_disable(lis3dh_mag_vreg[1].vreg);
	}
	regulator_put(lis3dh_mag_vreg[0].vreg);
	regulator_put(lis3dh_mag_vreg[1].vreg);

	i2c_set_clientdata(client, NULL);
	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303dlhc_m_suspend(struct device *dev)
{
	struct lsm303dlhc_m_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP)
		return 0;

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
	disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

	ret = lsm303dlhc_m_set_mode(ddata, LSM303DLH_M_MODE_SLEEP);
	if (ret < 0)
		return ret;
	regulator_disable(lis3dh_mag_vreg[0].vreg);
	regulator_disable(lis3dh_mag_vreg[1].vreg);
	return ret;
}

static int lsm303dlhc_m_resume(struct device *dev)
{
	struct lsm303dlhc_m_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	/* in correct mode, no need to change it */
	if (ddata->mode == LSM303DLH_M_MODE_SLEEP)
		return 0;

#ifdef CONFIG_SENSORS_LSM303DLHC_INPUT_DEVICE
	enable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif
	regulator_enable(lis3dh_mag_vreg[0].vreg);
	regulator_enable(lis3dh_mag_vreg[1].vreg);

	ret = lsm303dlhc_m_restore(ddata);

	if (ret < 0)
		return ret;

	return ret;
}
static const struct dev_pm_ops lsm303dlhc_m_dev_pm_ops = {
	.suspend = lsm303dlhc_m_suspend,
	.resume  = lsm303dlhc_m_resume,
};
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303dlhc_m_id[] = {
	{ "lsm303dlhc_m", 0 },
	{ },
};

static struct i2c_driver lsm303dlhc_m_driver = {
	.probe		= lsm303dlhc_m_probe,
	.remove		= lsm303dlhc_m_remove,
	.id_table	= lsm303dlhc_m_id,
	.driver = {
		.name = LSM303DHLC_MAG_DEV_NAME,
	#ifdef CONFIG_PM
		.pm = &lsm303dlhc_m_dev_pm_ops,
	#endif
	},
};

static int __init lsm303dlhc_m_init(void)
{
	return i2c_add_driver(&lsm303dlhc_m_driver);
}

static void __exit lsm303dlhc_m_exit(void)
{
	i2c_del_driver(&lsm303dlhc_m_driver);
}

module_init(lsm303dlhc_m_init);
module_exit(lsm303dlhc_m_exit);

MODULE_DESCRIPTION("lSM303DLH 3-Axis Magnetometer Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("STMicroelectronics");
