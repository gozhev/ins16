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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input/knxd9.h>
#include <linux/workqueue.h>

#define NAME			"knxd9"
#define G_MAX			8000
#define KXSD9_ID_VAL		0xE1
#define KXUD9_ID_VAL		0xE3
/* OUTPUT REGISTERS */
#define XOUT_L			0x00
/* CONTROL REGISTERS */
#define CTRL_REGC		0x0C
#define CTRL_REGB		0x0D
/* CONTROL REGISTER C BITS */
#define G_MASK			0x03
#define LPF_MASK		0xE0
#define LPF1000			(0x04 << 5)
#define LPF500			(0x05 << 5)
#define LPF100			(0x06 << 5)
#define LPF50			(0x07 << 5)
/* CONTROL REGISTER B BITS */
#define CLKHLD			(1 << 7)
#define PC_ENABLE		(1 << 6)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3

/* KLP -- try to make everything be contained in the module */
#ifdef MODULE
static int i2c_bus =    2;   /* panda=4, odroid=2, zoom=2 */

module_param(i2c_bus, int, S_IRUGO);

struct i2c_client *knxd9_i2c_client;

struct knxd9_platform_data knxd9_data = {
	.min_interval	= 1,
	.init_interval	= 200,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.g_range	= KNXD9_G_2G,
};

static struct i2c_board_info knxd9_board_info = {
	I2C_BOARD_INFO("knxd9", 0),
	.platform_data = &knxd9_data,
};

/* list of all the supported accels */
static const unsigned short knxd9_addr_list[] = {0x18, I2C_CLIENT_END};

/* try to read the CTRL_REGC register */
static int __knxd9_probe_i2c(struct i2c_adapter *adapter, unsigned short addr)
{
	struct i2c_msg msgs[2];
	u8 reg_addr = 0x0c, value;
	int ret;

	msgs[0].addr = msgs[1].addr = addr;
	msgs[0].len = msgs[1].len = 1;
	msgs[0].flags = msgs[1].flags = 0;
	msgs[1].flags |= I2C_M_RD;
	msgs[0].buf = &reg_addr;
	msgs[1].buf = &value;

	ret = i2c_transfer(adapter, msgs, 2);

	pr_warn("knxd9 probing %d %d %d\n", (int)addr, (int)ret, (int)value);

	/* The reset value for Control Register C (0x0C) is 0xE1 or 0xE3 */
	return (ret == 2 && (value == KXSD9_ID_VAL || value == KXUD9_ID_VAL));
}

int __init knxd9_init_i2c(void)
{
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(i2c_bus);

	if (!adapter) {
		pr_warn("knxd9: i2c_get_adapter(%d) failed\n", i2c_bus);
		goto fail;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
	knxd9_i2c_client = i2c_new_probed_device(adapter,
			&knxd9_board_info,
			knxd9_addr_list,
			__knxd9_probe_i2c);
#else
	knxd9_i2c_client = i2c_new_probed_device(adapter,
			&knxd9_board_info,
			knxd9_addr_list);
#endif

	if (!knxd9_i2c_client) {
		pr_warn("knxd9: i2c_new_probed_device() failed\n");
		goto fail1;
	}

	i2c_put_adapter(adapter);

	return 0;

fail1:
	i2c_put_adapter(adapter);
fail:
	return -1;
}

void knxd9_cleanup_i2c(void)
{
	if (knxd9_i2c_client) {
		i2c_unregister_device(knxd9_i2c_client);
		knxd9_i2c_client = NULL;
	}
}
#endif /* MODULE */
/* KLP end */

/*
 * The following table lists the maximum appropriate poll interval for each
 * available LPF bandwidth.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} knxd9_lpf_table[] = {
	{ 4,	LPF1000 },
	{ 20,	LPF500 },
	{ 40,	LPF100 },
	{ 0,	LPF50  },
};

struct knxd9_data {
	struct i2c_client *client;
	struct knxd9_platform_data pdata;
	struct input_dev *input_dev;
	unsigned int last_poll_interval;
	struct delayed_work input_work;
	struct mutex lock;
	u8 g_range;
	u8 odr_mask;
	u8 ctrl_regc;
	u8 ctrl_regb;
};

static int knxd9_i2c_read(struct knxd9_data *knx, u8 addr, u8 *data, int len)
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

static void knxd9_report_acceleration_data(struct knxd9_data *knx)
{
	u8 acc_data[6]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 xyz[3];
	int err;

	err = knxd9_i2c_read(knx, XOUT_L, acc_data, 6);
	if (err < 0)
		dev_err(&knx->client->dev, "accelerometer data read failed\n");

	xyz[0] = ((acc_data[0] << 8 | acc_data[1]) >> 4) - 2048;
	xyz[1] = ((acc_data[2] << 8 | acc_data[3]) >> 4) - 2048;
	xyz[2] = ((acc_data[4] << 8 | acc_data[5]) >> 4) - 2048;

	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&knx->client->dev, "x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	input_report_abs(knx->input_dev, ABS_X, knx->pdata.negate_x ?
		-xyz[knx->pdata.axis_map_x] : xyz[knx->pdata.axis_map_x]);
	input_report_abs(knx->input_dev, ABS_Y, knx->pdata.negate_y ?
		-xyz[knx->pdata.axis_map_y] : xyz[knx->pdata.axis_map_y]);
	input_report_abs(knx->input_dev, ABS_Z, knx->pdata.negate_z ?
		-xyz[knx->pdata.axis_map_z] : xyz[knx->pdata.axis_map_z]);
	input_sync(knx->input_dev);
}

static int knxd9_update_g_range(struct knxd9_data *knx, u8 new_g_range)
{
	switch (new_g_range) {
	case KNXD9_G_2G:
		knx->g_range = 2;
		break;
	case KNXD9_G_4G:
		knx->g_range = 4;
		break;
	case KNXD9_G_6G:
		knx->g_range = 6;
		break;
	case KNXD9_G_8G:
		knx->g_range = 8;
		break;
	default:
		return -EINVAL;
	}

	knx->ctrl_regc &= ~G_MASK;
	knx->ctrl_regc |= new_g_range;

	return 0;
}

static int knxd9_update_odr(struct knxd9_data *knx, unsigned int poll_interval)
{
	int err;
	int i;

	if (poll_interval < knx->pdata.min_interval)
		return -EINVAL;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(knxd9_lpf_table); i++) {
		knx->odr_mask = knxd9_lpf_table[i].mask;
		if (poll_interval < knxd9_lpf_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGB, 0);
	if (err < 0)
		return err;

	knx->ctrl_regc &= ~LPF_MASK;

	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGC,
					knx->ctrl_regc | knx->odr_mask);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGB, knx->ctrl_regb);
	if (err < 0)
		return err;

	mutex_lock(&knx->lock);

	knx->last_poll_interval = poll_interval;
	if (knx->input_dev->users) {
		cancel_delayed_work_sync(&knx->input_work);
		schedule_delayed_work(&knx->input_work,
				msecs_to_jiffies(knx->last_poll_interval));
	}

	mutex_unlock(&knx->lock);

	return 0;
}

static int knxd9_device_power_on(struct knxd9_data *knx)
{
	if (knx->pdata.power_on)
		return knx->pdata.power_on();

	mdelay(25);

	return 0;
}

static void knxd9_device_power_off(struct knxd9_data *knx)
{
	int err;

	knx->ctrl_regb &= ~PC_ENABLE;
	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGB, knx->ctrl_regb);
	if (err < 0)
		dev_err(&knx->client->dev, "soft power off failed\n");

	cancel_delayed_work_sync(&knx->input_work);

	if (knx->pdata.power_off)
		knx->pdata.power_off();
}

static int knxd9_enable(struct knxd9_data *knx)
{
	int err;

	err = knxd9_device_power_on(knx);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGC, knx->ctrl_regc);
	if (err < 0)
		return err;

	/* turn on outputs */
	knx->ctrl_regb |= PC_ENABLE;
	err = i2c_smbus_write_byte_data(knx->client, CTRL_REGB, knx->ctrl_regb);
	if (err < 0)
		return err;

	schedule_delayed_work(&knx->input_work,
			msecs_to_jiffies(knx->last_poll_interval));

	err = knxd9_update_odr(knx, knx->last_poll_interval);
	if (err < 0)
		return err;

	return 0;
}

static void knxd9_disable(struct knxd9_data *knx)
{
	knxd9_device_power_off(knx);
}

static void knxd9_input_work_func(struct work_struct *work)
{
	struct knxd9_data *knx = container_of((struct delayed_work *)work,
						struct knxd9_data, input_work);
	knxd9_report_acceleration_data(knx);
	schedule_delayed_work(&knx->input_work,
			      msecs_to_jiffies(knx->last_poll_interval));
}

int knxd9_input_open(struct input_dev *dev)
{
	struct knxd9_data *knx = input_get_drvdata(dev);

	return knxd9_enable(knx);
}

void knxd9_input_close(struct input_dev *dev)
{
	struct knxd9_data *knx = input_get_drvdata(dev);

	knxd9_disable(knx);
}

static int __devinit knxd9_input_init(struct knxd9_data *knx)
{
	int err;

	INIT_DELAYED_WORK(&knx->input_work, knxd9_input_work_func);
	knx->input_dev = input_allocate_device();
	if (!knx->input_dev) {
		dev_err(&knx->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	knx->input_dev->open = knxd9_input_open;
	knx->input_dev->close = knxd9_input_close;
	input_set_drvdata(knx->input_dev, knx);

	__set_bit(EV_ABS, knx->input_dev->evbit);
	input_set_abs_params(knx->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(knx->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(knx->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	knx->input_dev->name = "knxd9_accel";
	knx->input_dev->id.bustype = BUS_I2C;
	knx->input_dev->dev.parent = &knx->client->dev;

	err = input_register_device(knx->input_dev);
	if (err) {
		dev_err(&knx->client->dev,
			"unable to register input device %s\n",
			knx->input_dev->name);
		return err;
	}

	return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t knxd9_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", knx->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t knxd9_set_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	knxd9_update_odr(knx, interval);

	return count;
}
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, knxd9_get_delay, knxd9_set_delay);

/* Allow users to enable and disable */
static ssize_t knxd9_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error < 0)
		return error;

	if (val)
		knxd9_enable(knx);
	else
		knxd9_disable(knx);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, knxd9_set_enable);

/* DEBUG - REMOVE */
/* Allow users to select a new g range */
static ssize_t knxd9_set_grange(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;
	unsigned int new_range;
	int error;

	error = kstrtouint(buf, 10, &new_range);
	if (error < 0)
		return error;

	switch(new_range) {
	case 2:
		new_range = KNXD9_G_2G;
		break;
	case 4:
		new_range = KNXD9_G_4G;
		break;
	case 6:
		new_range = KNXD9_G_6G;
		break;
	case 8:
		new_range = KNXD9_G_8G;
		break;
	default:
		dev_err(&client->dev, "invalid g-range requested\n");
		return -EINVAL;
		break;
	}
		
	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	knxd9_update_g_range(knx, new_range);
	i2c_smbus_write_byte_data(knx->client, CTRL_REGB, 0);
	i2c_smbus_write_byte_data(knx->client, CTRL_REGC, knx->ctrl_regc);
	i2c_smbus_write_byte_data(knx->client, CTRL_REGB, knx->ctrl_regb);

	mutex_unlock(&input_dev->mutex);

	error = i2c_smbus_read_byte_data(knx->client, CTRL_REGC);
	dev_info(&client->dev, "new g-range selected, CTRL_REGC = 0x%02x\n", error);

	return count;
}
static DEVICE_ATTR(grange, S_IWUSR, NULL, knxd9_set_grange);
/* <END> DEBUG - REMOVE */

static struct attribute *knxd9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
/* DEBUG - REMOVE */
	&dev_attr_grange.attr,
/* <END> DEBUG - REMOVE */
	NULL
};

static struct attribute_group knxd9_attribute_group = {
	.attrs = knxd9_attributes
};

static int __devinit knxd9_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct knxd9_platform_data *pdata = client->dev.platform_data;
	struct knxd9_data *knx;
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

	mutex_init(&knx->lock);

	err = knxd9_device_power_on(knx);
	if (err < 0)
		goto err_free_mem;

	i2c_set_clientdata(client, knx);

	knx->ctrl_regc = knx->pdata.g_range;
	knx->ctrl_regb = CLKHLD;
	knx->last_poll_interval = knx->pdata.init_interval;

	err = knxd9_input_init(knx);
	if (err < 0) {
		dev_err(&client->dev, "input init failed: %d\n", err);
		goto err_pdata_exit;
	}

	err = sysfs_create_group(&client->dev.kobj, &knxd9_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_destroy_input;
	}

	return 0;

err_destroy_input:
	input_unregister_device(knx->input_dev);
err_pdata_exit:
	if (knx->pdata.exit)
		knx->pdata.exit();
err_free_mem:
	kfree(knx);
	return err;
}

static int __devexit knxd9_remove(struct i2c_client *client)
{
	struct knxd9_data *knx = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &knxd9_attribute_group);
	input_unregister_device(knx->input_dev);

	if (knx->pdata.exit)
		knx->pdata.exit();

	kfree(knx);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int knxd9_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);
	struct input_dev *input_dev = knx->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		knxd9_disable(knx);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int knxd9_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct knxd9_data *knx = i2c_get_clientdata(client);
	int retval = 0;

	mutex_lock(&knx->input_dev->mutex);

	if (knx->input_dev->users)
		knxd9_enable(knx);

	mutex_unlock(&knx->input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(knxd9_pm_ops, knxd9_suspend, knxd9_resume);

static const struct i2c_device_id knxd9_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, knxd9_id);

static struct i2c_driver knxd9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &knxd9_pm_ops,
	},
	.probe		= knxd9_probe,
	.remove		= __devexit_p(knxd9_remove),
	.id_table	= knxd9_id,
};

static int __init knxd9_init(void)
{
	int res = i2c_add_driver(&knxd9_driver);

#ifdef MODULE
	knxd9_init_i2c();
#endif

	return res;
}
module_init(knxd9_init);

static void __exit knxd9_exit(void)
{
#ifdef MODULE
	knxd9_cleanup_i2c();
#endif
	
	i2c_del_driver(&knxd9_driver);
}
module_exit(knxd9_exit);

MODULE_DESCRIPTION("KNXD9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
