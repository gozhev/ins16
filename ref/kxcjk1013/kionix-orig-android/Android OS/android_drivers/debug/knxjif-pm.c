/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input/knxjif.h>

#define NAME			"knxjif"
#define G_MAX			8000
/* KNXJIF ID VALUES */
#define KXTF9_ID_VAL		0x01
#define KXTI9_ID_VAL		0x04
#define KXTIK_ID_VAL		0x05
#define KXTJ9_AA_ID_VAL		0x07
#define KXTJ9_ID_VAL		0x08
#define KXTJ2_ID_VAL		0x09
#define KXCJ9_ID_VAL		0x0A
#define KXCJK_ID_VAL		0x11
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
#define DRDYE			(1 << 5)
#define WUFE			(1 << 1)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F			3
#define ODR200F			4
#define ODR400F			5
#define ODR800F			6
/* INTERRUPT CONTROL REGISTER 1 BITS */
#define KNXJIF_IEL		(1 << 3)
#define KNXJIF_IEA		(1 << 4)
#define KNXJIF_IEN		(1 << 5)
/* SLEEP/WAKE CONSTANTS */
#define SLEEP_THRESH_INIT	50	/* Counts */
#define WAKE_THRESH_INIT	100	/* Counts */
#define WAKE_TIMER_INIT		4	/* @ 12.5Hz */
#define WAKE_RESET_TIMER_INIT	2	/* @ 12.5Hz */
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3

/* KLP -- try to make everything be contained in the module */
#ifdef MODULE
static int i2c_bus =    2;   /* panda=4, odroid=2, zoom=2 */
static int gpio_line = 	156; /* panda=137, odroid=270, zoom=156 */

module_param(i2c_bus, int, S_IRUGO);
module_param(gpio_line, int, S_IRUGO);

struct i2c_client *knxjif_i2c_client;

struct knxjif_platform_data knxjif_data = {
	.min_interval	= 1,
	.init_interval	= 200,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.g_range	= KNXJIF_G_2G,
	.res_12bit	= RES_12BIT,
};

static struct i2c_board_info knxjif_board_info = {
	I2C_BOARD_INFO("knxjif", 0),
	.platform_data = &knxjif_data,
};

/* list of all the supported accels */
static const unsigned short knxjif_addr_list[] = {0x0f, I2C_CLIENT_END};

/* try to read the WHO_AM_I register */
static int __knxjif_probe_i2c(struct i2c_adapter *adapter, unsigned short addr)
{
	struct i2c_msg msgs[2];
	u8 reg_addr = 0x0f, value;
	int ret;

	msgs[0].addr = msgs[1].addr = addr;
	msgs[0].len = msgs[1].len = 1;
	msgs[0].flags = msgs[1].flags = 0;
	msgs[1].flags |= I2C_M_RD;
	msgs[0].buf = &reg_addr;
	msgs[1].buf = &value;

	ret = i2c_transfer(adapter, msgs, 2);

	pr_warn("knxjif probing %d %d %d\n", (int)addr, (int)ret, (int)value);

	return (ret == 2 &&
			(value == KXTF9_ID_VAL || value == KXTI9_ID_VAL ||
			value == KXTIK_ID_VAL || value == KXTJ9_AA_ID_VAL ||
			value == KXTJ9_ID_VAL || value == KXTJ2_ID_VAL ||
			value == KXCJ9_ID_VAL || value == KXCJK_ID_VAL));
}

int __init knxjif_init_i2c(void)
{
	struct i2c_adapter *adapter;
	int res;

	adapter = i2c_get_adapter(i2c_bus);

	if (!adapter) {
		pr_warn("knxjif: i2c_get_adapter(%d) failed\n", i2c_bus);
		goto fail;
	}

	res = gpio_request(gpio_line, "knxjif");
	if (0 > res) {
		pr_warn("knxjif: request gpio failed\n");
		goto fail2;
	}

	res = gpio_direction_input(gpio_line);
	if (0 > res) {
		pr_warn("knxjif: gpio direction input failed\n");
		goto fail2;
	}

	knxjif_board_info.irq = gpio_to_irq(gpio_line);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
	knxjif_i2c_client = i2c_new_probed_device(adapter,
			&knxjif_board_info,
			knxjif_addr_list,
			__knxjif_probe_i2c);
#else
	knxjif_i2c_client = i2c_new_probed_device(adapter,
			&knxjif_board_info,
			knxjif_addr_list);
#endif

	if (!knxjif_i2c_client) {
		pr_warn("knxjif: i2c_new_probed_device() failed\n");
		goto fail1;
	}

	i2c_put_adapter(adapter);

	return 0;

fail2:
	gpio_free(gpio_line);
fail1:
	i2c_put_adapter(adapter);
fail:
	return -1;
}

void knxjif_cleanup_i2c(void)
{
	if (knxjif_i2c_client) {
		i2c_unregister_device(knxjif_i2c_client);
		knxjif_i2c_client = NULL;
		gpio_free(gpio_line);
	}
}
#endif /* MODULE */
/* KLP end */

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
	u16 sleep;	/* Sleep timer (set per ODR) */
	u8 reset;	/* Sleep reset timer (set per ODR) */
} knxjif_odr_table[] = {
	{ 3,	ODR800F,	320,	160 },	/* 400ms, 200ms */
	{ 5,	ODR400F,	160,	80 },	/* 400ms, 200ms */
	{ 10,	ODR200F,	80,	40 },	/* 400ms, 200ms */
	{ 20,	ODR100F,	40,	20 },	/* 400ms, 200ms */
	{ 40,	ODR50F,		20,	10 },	/* 400ms, 200ms */
	{ 80,	ODR25F,		10,	5 },	/* 400ms, 200ms */
	{ 0,	ODR12_5F,	5,	3 },	/* 400ms, 240ms */
};

struct sleep_vars {
	u8 thresh;
	u8 count;
	u8 timer;
	u8 reset_count;
	u8 reset_timer;
};

struct knxjif_data {
	struct i2c_client *client;
	struct knxjif_platform_data pdata;
	struct input_dev *input_dev;
	unsigned int last_poll_interval;
	u8 device_id;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	s16 last_axis_data[3];
	struct sleep_vars wake;
	struct sleep_vars sleep;
	u8 sleep_flag;
};

static int knxjif_i2c_read(struct knxjif_data *knx, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = knx->client->addr,
			.flags = knx->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = knx->client->addr,
			.flags = knx->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(knx->client->adapter, msgs, 2);
}

static int knxjif_update_odr(struct knxjif_data *knx, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(knxjif_odr_table); i++) {
		knx->data_ctrl = knxjif_odr_table[i].mask;
		knx->sleep.timer = knxjif_odr_table[i].sleep;
		knx->sleep.reset_timer = knxjif_odr_table[i].reset;
		if (poll_interval < knxjif_odr_table[i].cutoff)
			break;
		/* KXTF9 does not support 12.5Hz ODR */
		if (knx->device_id == KXTF9_ID_VAL && knx->data_ctrl == ODR25F)
			break;
	}

	if (!knx->sleep_flag) {
		err = i2c_smbus_write_byte_data(knx->client, CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(knx->client, DATA_CTRL, knx->data_ctrl);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(knx->client, CTRL_REG1, knx->ctrl_reg1);
		if (err < 0)
			return err;
	}

	return 0;
}

static int knxjif_update_g_range(struct knxjif_data *knx, u8 new_g_range)
{
	switch (new_g_range) {
	case KNXJIF_G_2G:
		knx->shift = 4;
		break;
	case KNXJIF_G_4G:
		knx->shift = 3;
		break;
	case KNXJIF_G_8G:
		knx->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	knx->ctrl_reg1 &= 0xe7;
	knx->ctrl_reg1 |= new_g_range;

	return 0;
}

static void knxjif_exit_sleep(struct knxjif_data *knx)
{
	knx->sleep_flag = 0;
	knxjif_update_g_range(knx, knx->pdata.g_range);
	knxjif_update_odr(knx, knx->last_poll_interval);

	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&knx->client->dev, "Sleep mode exited\n");
	/*** <end> DEBUG OUTPUT - REMOVE ***/
}

static void knxjif_enter_sleep(struct knxjif_data *knx)
{
	knxjif_update_g_range(knx, KNXJIF_G_2G);
	knxjif_update_odr(knx, 1000);

	i2c_smbus_write_byte_data(knx->client, CTRL_REG1, 0);
	i2c_smbus_write_byte_data(knx->client, CTRL_REG1,
					(knx->ctrl_reg1 & ~RES_12BIT));

	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&knx->client->dev, "Sleep mode entered\n");
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	knx->sleep_flag = 1;
}

static void knxjif_calculate_sleep(struct knxjif_data *knx, s16 *xyz)
{
	s16 delta[3];
	int i;

	for (i = 0; i < 3; i++) {
		delta[i] = knx->last_axis_data[i] - xyz[i];
		if (delta[i] < 0)
			delta[i] = -delta[i];
	}

	if (!knx->sleep_flag) {	/* Check for sleep events */
		/* Determine whether delta is over the threshold for any axis */
		if (delta[0] > knx->sleep.thresh ||
			delta[1] > knx->sleep.thresh ||
			delta[2] > knx->sleep.thresh) {
			knx->sleep.reset_count++;
		} else {
			knx->sleep.reset_count = 0;
			knx->sleep.count++;
		}

		/* Sleep timer has been reached */
		if (knx->sleep.count == knx->sleep.timer) {
			knx->sleep.count = 0;
			knx->sleep.reset_count = 0;
			knxjif_enter_sleep(knx);
		}

		/* Sleep reset timer has been reached */
		if (knx->sleep.reset_count == knx->sleep.reset_timer) {
			knx->sleep.count = 0;
			knx->sleep.reset_count = 0;
		}
	}

	if (knx->sleep_flag) {	/* Check for wake events */
		/* Determine whether delta is over the threshold for any axis */
		if (delta[0] > knx->wake.thresh ||
			delta[1] > knx->wake.thresh ||
			delta[2] > knx->wake.thresh) {
			knx->wake.count++;
			knx->wake.reset_count = 0;
		} else {
			knx->wake.reset_count++;
		}

		/* Wake timer has been reached */
		if (knx->wake.count == knx->wake.timer) {
			knx->wake.count = 0;
			knx->wake.reset_count = 0;
			knxjif_exit_sleep(knx);
		}

		/* Wake reset timer has been reached */
		if (knx->wake.reset_count == knx->wake.reset_timer) {
			knx->wake.count = 0;
			knx->wake.reset_count = 0;
		}
	}

	for (i = 0; i < 3; i++)
		knx->last_axis_data[i] = xyz[i];
}

static void knxjif_report_acceleration_data(struct knxjif_data *knx)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 xyz[3];
	int err, i;

	err = knxjif_i2c_read(knx, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&knx->client->dev, "accelerometer data read failed\n");

	xyz[0] = le16_to_cpu(acc_data[knx->pdata.axis_map_x]);
	xyz[1] = le16_to_cpu(acc_data[knx->pdata.axis_map_y]);
	xyz[2] = le16_to_cpu(acc_data[knx->pdata.axis_map_z]);

	for (i = 0; i < 3; i++)
		xyz[i] >>= knx->shift;

	knxjif_calculate_sleep(knx, xyz);

	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&knx->client->dev, "x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	input_report_abs(knx->input_dev, ABS_X, knx->pdata.negate_x ?
							-xyz[0] : xyz[0]);
	input_report_abs(knx->input_dev, ABS_Y, knx->pdata.negate_y ?
							-xyz[1] : xyz[1]);
	input_report_abs(knx->input_dev, ABS_Z, knx->pdata.negate_z ?
							-xyz[2] : xyz[2]);
	input_sync(knx->input_dev);
}

static irqreturn_t knxjif_isr(int irq, void *dev)
{
	struct knxjif_data *knx = dev;
	int err;

	/* DRDY is the only possible interrupt */
	knxjif_report_acceleration_data(knx);

	err = i2c_smbus_read_byte_data(knx->client, INT_REL);
	if (err < 0)
		dev_err(&knx->client->dev,
			"error clearing interrupt status: %d\n", err);

	return IRQ_HANDLED;
}

static int knxjif_device_power_on(struct knxjif_data *knx)
{
	if (knx->pdata.power_on)
		return knx->pdata.power_on();

	return 0;
}

static void knxjif_device_power_off(struct knxjif_data *knx)
{
	int err;

	knx->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(knx->client, CTRL_REG1, knx->ctrl_reg1);
	if (err < 0)
		dev_err(&knx->client->dev, "soft power off failed\n");

	if (knx->pdata.power_off)
		knx->pdata.power_off();
}

static int knxjif_enable(struct knxjif_data *knx)
{
	int err;

	err = knxjif_device_power_on(knx);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(knx->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(knx->client, INT_CTRL1, knx->int_ctrl);
	if (err < 0)
		return err;

	err = knxjif_update_g_range(knx, knx->pdata.g_range);
	if (err < 0)
		return err;

	disable_irq(knx->client->irq);
	/* turn on outputs */
	knx->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(knx->client, CTRL_REG1, knx->ctrl_reg1);
	if (err < 0)
		return err;
	mdelay(81);
	enable_irq(knx->client->irq);

	knx->sleep_flag = 0;

	err = knxjif_update_odr(knx, knx->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt */
	err = i2c_smbus_read_byte_data(knx->client, INT_REL);
	if (err < 0) {
		dev_err(&knx->client->dev,
			"error clearing interrupt: %d\n", err);
		goto fail;
	}

	return 0;

fail:
	knxjif_device_power_off(knx);
	return err;
}

static void knxjif_disable(struct knxjif_data *knx)
{
	knxjif_device_power_off(knx);
}

static int knxjif_input_open(struct input_dev *input)
{
	struct knxjif_data *knx = input_get_drvdata(input);

	return knxjif_enable(knx);
}

static void knxjif_input_close(struct input_dev *dev)
{
	struct knxjif_data *knx = input_get_drvdata(dev);

	knxjif_disable(knx);
}

static void __devinit knxjif_init_input_device(struct knxjif_data *knx,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "knxjif_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &knx->client->dev;
}

static int __devinit knxjif_setup_input_device(struct knxjif_data *knx)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&knx->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	knx->input_dev = input_dev;

	input_dev->open = knxjif_input_open;
	input_dev->close = knxjif_input_close;
	input_set_drvdata(input_dev, knx);

	knxjif_init_input_device(knx, input_dev);

	err = input_register_device(knx->input_dev);
	if (err) {
		dev_err(&knx->client->dev,
			"unable to register input polled device %s: %d\n",
			knx->input_dev->name, err);
		input_free_device(knx->input_dev);
		return err;
	}

	return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t knxjif_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", knx->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t knxjif_set_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);
	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	knx->last_poll_interval = max(interval, knx->pdata.min_interval);
	knxjif_update_odr(knx, knx->last_poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, knxjif_get_delay, knxjif_set_delay);

/* Allow users to enable and disable */
static ssize_t knxjif_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error < 0)
		return error;

	if (val)
		knxjif_enable(knx);
	else
		knxjif_disable(knx);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, knxjif_set_enable);

/* DEBUG - REMOVE */
/* Allow users to select a new g range */
static ssize_t knxjif_set_grange(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;
	unsigned int new_range;
	int error;

	error = kstrtouint(buf, 10, &new_range);
	if (error < 0)
		return error;

	switch(new_range) {
	case 2:
		new_range = KNXJIF_G_2G;
		break;
	case 4:
		new_range = KNXJIF_G_4G;
		break;
	case 8:
		new_range = KNXJIF_G_8G;
		break;
	default:
		dev_err(&client->dev, "invalid g-range requested\n");
		return -EINVAL;
		break;
	}
		
	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);

	knxjif_update_g_range(knx, new_range);
	i2c_smbus_write_byte_data(knx->client, CTRL_REG1, 0);
	i2c_smbus_write_byte_data(knx->client, CTRL_REG1, knx->ctrl_reg1);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	error = i2c_smbus_read_byte_data(knx->client, CTRL_REG1);
	dev_info(&client->dev, "new g-range selected, CTRL_REG1 = 0x%02x\n", error);

	return count;
}
static DEVICE_ATTR(grange, S_IWUSR, NULL, knxjif_set_grange);

/* Allow users to select a new output resolution (8-bit or 12-bit) */
static ssize_t knxjif_set_res(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;
	unsigned int new_res;
	int error;

	error = kstrtouint(buf, 10, &new_res);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if (new_res == 12) {
		knx->ctrl_reg1 |= RES_12BIT;
	} else if (new_res == 8) {
		knx->ctrl_reg1 &= ~RES_12BIT;
	} else {
		mutex_unlock(&input_dev->mutex);
		dev_err(&client->dev, "invalid resolution requested\n");
		return -EINVAL;
	}

	disable_irq(client->irq);

	i2c_smbus_write_byte_data(knx->client, CTRL_REG1, 0);
	i2c_smbus_write_byte_data(knx->client, CTRL_REG1, knx->ctrl_reg1);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	error = i2c_smbus_read_byte_data(knx->client, CTRL_REG1);
	dev_info(&client->dev, "new resolution selected, CTRL_REG1 = 0x%02x\n", error);

	return count;
}
static DEVICE_ATTR(res, S_IWUSR, NULL, knxjif_set_res);
/* <END> DEBUG - REMOVE */

static struct attribute *knxjif_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
/* DEBUG - REMOVE */
	&dev_attr_grange.attr,
	&dev_attr_res.attr,
/* <END> DEBUG - REMOVE */
	NULL
};

static struct attribute_group knxjif_attribute_group = {
	.attrs = knxjif_attributes
};

static int __devinit knxjif_verify(struct knxjif_data *knx)
{
	int retval;

	retval = knxjif_device_power_on(knx);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(knx->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&knx->client->dev, "read err int source\n");
		goto out;
	}

	if (retval == KXTF9_ID_VAL || retval == KXTI9_ID_VAL ||
			retval == KXTIK_ID_VAL || retval == KXTJ9_AA_ID_VAL ||
			retval == KXTJ9_ID_VAL || retval == KXTJ2_ID_VAL ||
			retval == KXCJ9_ID_VAL || retval == KXCJK_ID_VAL) {
		knx->device_id = retval;
		retval = 0;
	} else {
		retval = -EIO;
	}

out:
	knxjif_device_power_off(knx);
	return retval;
}

static int __devinit knxjif_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct knxjif_platform_data *pdata = client->dev.platform_data;
	struct knxjif_data *knx;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	if (!client->irq) {
		dev_err(&client->dev, "client->irq is NULL; exiting\n");
		return -EINVAL;
	}

	knx = kzalloc(sizeof(*knx), GFP_KERNEL);
	if (!knx) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	knx->client = client;
	knx->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}

	err = knxjif_verify(knx);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}

	i2c_set_clientdata(client, knx);

	knx->ctrl_reg1 = knx->pdata.res_12bit | knx->pdata.g_range | DRDYE;
	knx->sleep.thresh = SLEEP_THRESH_INIT;
	knx->wake.thresh = WAKE_THRESH_INIT;
	knx->wake.timer = WAKE_TIMER_INIT;
	knx->wake.reset_timer = WAKE_RESET_TIMER_INIT;
	knx->last_poll_interval = knx->pdata.init_interval;
	knx->int_ctrl |= KNXJIF_IEN | KNXJIF_IEA | KNXJIF_IEL;

	err = knxjif_setup_input_device(knx);
	if (err)
		goto err_pdata_exit;

	err = request_threaded_irq(client->irq, NULL, knxjif_isr,
			   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			   "knxjif-irq", knx);
	if (err) {
		dev_err(&client->dev, "request irq failed: %d\n", err);
		goto err_destroy_input;
	}

	err = sysfs_create_group(&client->dev.kobj, &knxjif_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(client->irq, knx);
err_destroy_input:
	input_unregister_device(knx->input_dev);
err_pdata_exit:
	if (knx->pdata.exit)
		knx->pdata.exit();
err_free_mem:
	kfree(knx);
	return err;
}

static int __devexit knxjif_remove(struct i2c_client *client)
{
	struct knxjif_data *knx = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &knxjif_attribute_group);
	free_irq(client->irq, knx);
	input_unregister_device(knx->input_dev);

	if (knx->pdata.exit)
		knx->pdata.exit();

	kfree(knx);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int knxjif_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		knxjif_disable(knx);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int knxjif_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxjif_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		knxjif_enable(knx);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(knxjif_pm_ops, knxjif_suspend, knxjif_resume);

static const struct i2c_device_id knxjif_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, knxjif_id);

static struct i2c_driver knxjif_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &knxjif_pm_ops,
	},
	.probe		= knxjif_probe,
	.remove		= __devexit_p(knxjif_remove),
	.id_table	= knxjif_id,
};

static int __init knxjif_init(void)
{
	int res = i2c_add_driver(&knxjif_driver);

#ifdef MODULE
	knxjif_init_i2c();
#endif

	return res;
}
module_init(knxjif_init);

static void __exit knxjif_exit(void)
{
#ifdef MODULE
	knxjif_cleanup_i2c();
#endif
	
	i2c_del_driver(&knxjif_driver);
}
module_exit(knxjif_exit);

MODULE_DESCRIPTION("KNXJIF accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
