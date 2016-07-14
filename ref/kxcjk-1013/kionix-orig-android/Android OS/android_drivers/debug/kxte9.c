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
#include <linux/input/kxte9.h>
#include <linux/workqueue.h>

#define NAME			"kxte9"
#define G_MAX			2048
/* KXTE9 ID VALUE */
#define KXTE9_ID_VAL		0x00
/* OUTPUT REGISTERS */
#define XOUT			0x12
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define CTRL_REG1		0x1B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
#define ODR1E			0
#define ODR3E			(1 << 3)
#define ODR10E			(1 << 4)
#define ODR40E			(3 << 3)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3

/* KLP -- try to make everything be contained in the module */
static int i2c_bus =    2;   /* panda=4, odroid=2, zoom=2 */

module_param(i2c_bus, int, S_IRUGO);

struct i2c_client *kxte9_i2c_client;

struct kxte9_platform_data kxte9_data = {
	.min_interval	= 1,
	.init_interval	= 200,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static struct i2c_board_info kxte9_board_info = {
	I2C_BOARD_INFO("kxte9", 0),
	.platform_data = &kxte9_data,
};

/* list of all the supported accels */
static const unsigned short kxte9_addr_list[] = {0x0f, I2C_CLIENT_END};

/* try to read the WHO_AM_I register; the default value is 0 */
static int __kxte9_probe_i2c(struct i2c_adapter *adapter, unsigned short addr)
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

	pr_warn("kxte9 probing %d %d %d\n", (int)addr, (int)ret, (int)value);

	return (ret == 2 && value == 0);
}

int __init kxte9_init_i2c(void)
{
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(i2c_bus);

	if (!adapter) {
		pr_warn("kxte9: i2c_get_adapter(%d) failed\n", i2c_bus);
		goto fail;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
	kxte9_i2c_client = i2c_new_probed_device(adapter,
			&kxte9_board_info,
			kxte9_addr_list,
			__kxte9_probe_i2c);
#else
	kxte9_i2c_client = i2c_new_probed_device(adapter,
			&kxte9_board_info,
			kxte9_addr_list);
#endif

	if (!kxte9_i2c_client) {
		pr_warn("kxte9: i2c_new_probed_device() failed\n");
		goto fail1;
	}

	i2c_put_adapter(adapter);

	return 0;

fail1:
	i2c_put_adapter(adapter);
fail:
	return -1;
}

void kxte9_cleanup_i2c(void)
{
	if (kxte9_i2c_client) {
		i2c_unregister_device(kxte9_i2c_client);
		kxte9_i2c_client = NULL;
	}
}
/* KLP end */

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxte9_odr_table[] = {
	{ 100,	ODR40E },
	{ 334,	ODR10E },
	{ 1000,	ODR3E },
	{ 0,	ODR1E },
};

struct kxte9_data {
	struct i2c_client *client;
	struct kxte9_platform_data pdata;
	struct input_dev *input_dev;
	unsigned int last_poll_interval;
	struct delayed_work input_work;
	struct mutex lock;
	u8 shift;
	u8 ctrl_reg1;
	u8 odr_mask;
};

static int kxte9_i2c_read(struct kxte9_data *te9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = te9->client->addr,
			.flags = te9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = te9->client->addr,
			.flags = te9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(te9->client->adapter, msgs, 2);
}

static void kxte9_report_acceleration_data(struct kxte9_data *te9)
{
	u8 acc_data[3]; /* Data bytes from hardware x, y, z */
	int x, y, z;
	int err;

	err = kxte9_i2c_read(te9, XOUT, acc_data, 3);
	if (err < 0)
		dev_err(&te9->client->dev, "accelerometer data read failed\n");

	x = ((int)(acc_data[te9->pdata.axis_map_x] >> 2) - 32) << 6;
	y = ((int)(acc_data[te9->pdata.axis_map_y] >> 2) - 32) << 6;
	z = ((int)(acc_data[te9->pdata.axis_map_z] >> 2) - 32) << 6;

	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&te9->client->dev, "x:%d y:%d z:%d\n", x, y, z);
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	input_report_abs(te9->input_dev, ABS_X, te9->pdata.negate_x ? -x : x);
	input_report_abs(te9->input_dev, ABS_Y, te9->pdata.negate_y ? -y : y);
	input_report_abs(te9->input_dev, ABS_Z, te9->pdata.negate_z ? -z : z);
	input_sync(te9->input_dev);
}

static int kxte9_update_odr(struct kxte9_data *te9, unsigned int poll_interval)
{
	int err;
	int i;

	if (poll_interval < te9->pdata.min_interval)
		return -EINVAL;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxte9_odr_table); i++) {
		te9->odr_mask = kxte9_odr_table[i].mask;
		if (poll_interval < kxte9_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(te9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	te9->ctrl_reg1 &= ~ODR40E;

	err = i2c_smbus_write_byte_data(te9->client, CTRL_REG1,
					te9->ctrl_reg1 | te9->odr_mask);
	if (err < 0)
		return err;

	mutex_lock(&te9->lock);

	te9->last_poll_interval = poll_interval;
	if (te9->input_dev->users) {
		cancel_delayed_work_sync(&te9->input_work);
		schedule_delayed_work(&te9->input_work,
				msecs_to_jiffies(te9->last_poll_interval));
	}

	mutex_unlock(&te9->lock);

	return 0;
}

static int kxte9_device_power_on(struct kxte9_data *te9)
{
	if (te9->pdata.power_on)
		return te9->pdata.power_on();

	return 0;
}

static void kxte9_device_power_off(struct kxte9_data *te9)
{
	int err;

	te9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(te9->client, CTRL_REG1, te9->ctrl_reg1);
	if (err < 0)
		dev_err(&te9->client->dev, "soft power off failed\n");

	cancel_delayed_work_sync(&te9->input_work);

	if (te9->pdata.power_off)
		te9->pdata.power_off();
}

static int kxte9_enable(struct kxte9_data *te9)
{
	int err;

	err = kxte9_device_power_on(te9);
	if (err < 0)
		return err;

	/* turn on outputs */
	te9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(te9->client, CTRL_REG1, te9->ctrl_reg1);
	if (err < 0)
		return err;

	schedule_delayed_work(&te9->input_work,
			msecs_to_jiffies(te9->last_poll_interval));

	err = kxte9_update_odr(te9, te9->last_poll_interval);
	if (err < 0)
		return err;

	return 0;
}

static void kxte9_disable(struct kxte9_data *te9)
{
	kxte9_device_power_off(te9);
}

static void kxte9_input_work_func(struct work_struct *work)
{
	struct kxte9_data *te9 = container_of((struct delayed_work *)work,
						struct kxte9_data, input_work);
	kxte9_report_acceleration_data(te9);
	schedule_delayed_work(&te9->input_work,
			      msecs_to_jiffies(te9->last_poll_interval));
}

int kxte9_input_open(struct input_dev *dev)
{
	struct kxte9_data *te9 = input_get_drvdata(dev);

	return kxte9_enable(te9);
}

void kxte9_input_close(struct input_dev *dev)
{
	struct kxte9_data *te9 = input_get_drvdata(dev);

	kxte9_disable(te9);
}

static int __devinit kxte9_input_init(struct kxte9_data *te9)
{
	int err;

	INIT_DELAYED_WORK(&te9->input_work, kxte9_input_work_func);
	te9->input_dev = input_allocate_device();
	if (!te9->input_dev) {
		dev_err(&te9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	te9->input_dev->open = kxte9_input_open;
	te9->input_dev->close = kxte9_input_close;
	input_set_drvdata(te9->input_dev, te9);

	__set_bit(EV_ABS, te9->input_dev->evbit);
	input_set_abs_params(te9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(te9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(te9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	te9->input_dev->name = "kxte9_accel";
	te9->input_dev->id.bustype = BUS_I2C;
	te9->input_dev->dev.parent = &te9->client->dev;

	err = input_register_device(te9->input_dev);
	if (err) {
		dev_err(&te9->client->dev,
			"unable to register input device %s\n",
			te9->input_dev->name);
		return err;
	}

	return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kxte9_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxte9_data *te9 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", te9->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxte9_set_delay(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxte9_data *te9 = i2c_get_clientdata(client);
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	kxte9_update_odr(te9, interval);

	return count;
}
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kxte9_get_delay, kxte9_set_delay);

/* Allow users to enable and disable */
static ssize_t kxte9_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxte9_data *te9 = i2c_get_clientdata(client);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error < 0)
		return error;

	if (val)
		kxte9_enable(te9);
	else
		kxte9_disable(te9);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, kxte9_set_enable);

static struct attribute *kxte9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group kxte9_attribute_group = {
	.attrs = kxte9_attributes
};

static int __devinit kxte9_verify(struct kxte9_data *te9)
{
	int retval;

	retval = kxte9_device_power_on(te9);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(te9->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&te9->client->dev, "read err int source\n");
		goto out;
	}

	retval = (retval != KXTE9_ID_VAL) ? -EIO : 0;

out:
	kxte9_device_power_off(te9);
	return retval;
}

static int __devinit kxte9_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct kxte9_platform_data *pdata = client->dev.platform_data;
	struct kxte9_data *te9;
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

	te9 = kzalloc(sizeof(*te9), GFP_KERNEL);
	if (!te9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	te9->client = client;
	te9->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}

	mutex_init(&te9->lock);

	err = kxte9_verify(te9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}

	i2c_set_clientdata(client, te9);

	te9->ctrl_reg1 = 0;
	te9->last_poll_interval = te9->pdata.init_interval;

	err = kxte9_input_init(te9);
	if (err < 0) {
		dev_err(&client->dev, "input init failed: %d\n", err);
		goto err_pdata_exit;
	}

	err = sysfs_create_group(&client->dev.kobj, &kxte9_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_destroy_input;
	}

	return 0;

err_destroy_input:
	input_unregister_device(te9->input_dev);
err_pdata_exit:
	if (te9->pdata.exit)
		te9->pdata.exit();
err_free_mem:
	kfree(te9);
	return err;
}

static int __devexit kxte9_remove(struct i2c_client *client)
{
	struct kxte9_data *te9 = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &kxte9_attribute_group);
	input_unregister_device(te9->input_dev);

	if (te9->pdata.exit)
		te9->pdata.exit();

	kfree(te9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxte9_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxte9_data *te9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = te9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kxte9_disable(te9);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int kxte9_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxte9_data *te9 = i2c_get_clientdata(client);
	int retval = 0;

	mutex_lock(&te9->input_dev->mutex);

	if (te9->input_dev->users)
		kxte9_enable(te9);

	mutex_unlock(&te9->input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kxte9_pm_ops, kxte9_suspend, kxte9_resume);

static const struct i2c_device_id kxte9_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxte9_id);

static struct i2c_driver kxte9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &kxte9_pm_ops,
	},
	.probe		= kxte9_probe,
	.remove		= __devexit_p(kxte9_remove),
	.id_table	= kxte9_id,
};

static int __init kxte9_init(void)
{
	int res = i2c_add_driver(&kxte9_driver);
	if (MODULE)
		kxte9_init_i2c();

	return res;
}
module_init(kxte9_init);

static void __exit kxte9_exit(void)
{
	if (MODULE)
		kxte9_cleanup_i2c();
	
	i2c_del_driver(&kxte9_driver);
}
module_exit(kxte9_exit);

MODULE_DESCRIPTION("KXTE9 Android accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
