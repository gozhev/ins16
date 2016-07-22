#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

#include "mpu6050.h"

#define DRIVER_NAME "fast-mpu6050"
#define DRIVER_VERSION "0"
#define DEVICE_NAME "mpu6050"

static dev_t chrdev_id;
static struct cdev chrdev;

DEFINE_MUTEX (lock);
DECLARE_WAIT_QUEUE_HEAD (wq);
ktime_t tstmp;
uint8_t buffer[22];
int fresh;
struct i2c_client *i2c_client;

static int chrdev_release (struct inode *inode, struct file *filp)
{
    return 0;
}

static int chrdev_open (struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t chrdev_read (struct file *filp, char __user *buff, size_t count, 
        loff_t *offp)
{
    int rc;

    if (!fresh)
        wait_event_interruptible (wq, fresh);

    mutex_lock (&lock); 
    rc = copy_to_user (buff, buffer, 22);
    fresh = 0;
    mutex_unlock (&lock);

    if (rc)
        return -EFAULT;
    
    return 22;
}

static const struct file_operations chrdev_fops = {
	.owner = THIS_MODULE,
	/*.mmap = _mmap,*/
    .read = chrdev_read,
    .open = chrdev_open,
    .release = chrdev_release
};

static const struct i2c_device_id i2c_id_table[] = {
	{DEVICE_NAME, 0}, 
    { }
};

MODULE_DEVICE_TABLE (i2c, i2c_id_table);

static int reset_chip (const struct i2c_client *client)
{
	int rc;
    uint8_t reg, byte;

	/* reset to make sure previous state are not there */
    reg = MPU6050_RA_PWR_MGMT_1;
    byte = (1 << MPU6050_PWR1_DEVICE_RESET_BIT);
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;

	msleep (MPU6050_POWER_UP_TIME);

	/* toggle power state. After reset, the sleep bit could be on
		or off depending on the OTP settings. Toggling power would
		make it in a definite state as well as making the hardware
		state align with the software state */
    byte = (1 << MPU6050_PWR1_SLEEP_BIT);
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;

    byte = 0;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;
   
    msleep (MPU6050_TEMP_UP_TIME);

    /* switch clock needs to be careful. Only when gyro is on, can
	    clock source be switched to gyro. Otherwise, it must be set to
	    internal clock */
    byte = 0 | MPU6050_CLOCK_PLL_XGYRO;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;

	return 0;
}

static int configure_chip (const struct i2c_client *client)
{
	int rc;
    uint8_t reg, byte;

	/* set low-pass filter */
    reg = MPU6050_RA_CONFIG;
    byte = 0 | MPU6050_DLPF_BW_98;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;
	
    /* Set the full scale range for the gyroscope. */
    reg = MPU6050_RA_GYRO_CONFIG;
    byte = 0 | MPU6050_GYRO_FS_500 << MPU6050_GCONFIG_FS_SEL_BIT;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;

    /* Set the full scale range for the accelerometer. */
    reg = MPU6050_RA_ACCEL_CONFIG;
    byte = 0 | MPU6050_REV_C_ACCEL_FS_4 << MPU6050_ACONFIG_AFS_SEL_BIT;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;
    
    /* set prescaler */
    reg = MPU6050_RA_SMPLRT_DIV;
    byte = 0;
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;

    return 0;
}

static int power_down_chip (const struct i2c_client *client)
{
	int rc;
    uint8_t reg, byte;
    
    /* Put chip into sleep mode and set clock source to internal oscillator. */
    reg = MPU6050_RA_PWR_MGMT_1;
    byte = 0 | (1 << MPU6050_PWR1_SLEEP_BIT);
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;
    
    return 0;
}

static int enable_interrupts_on_chip (const struct i2c_client *client)
{
    int rc;
    uint8_t reg, byte;

    reg = MPU6050_RA_INT_ENABLE;
    byte = 0 | (1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
    rc = i2c_smbus_write_i2c_block_data (client, reg, 1, &byte);
	if (rc) return rc;
    
    return 0;
}

irqreturn_t irq_handler (int irq, void *p)
{
    tstmp = ktime_get ();
    return IRQ_WAKE_THREAD;
}

irqreturn_t irq_thread_fn (int irq, void *p)
{
    mutex_lock (&lock);
    
    i2c_smbus_read_i2c_block_data (i2c_client, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *(uint64_t *)(buffer+14) = ktime_to_ns (tstmp);
    fresh = 1;

    mutex_unlock (&lock);
    
    wake_up_interruptible (&wq);
    return IRQ_HANDLED;
}

static int i2c_probe (struct i2c_client *client, 
        const struct i2c_device_id *id)
{
	int rc;

	rc = i2c_check_functionality (client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK);

    if (!rc)
		return -ENOSYS;

	i2c_client = client;
    
    rc = reset_chip (client);
    /* if (rc) return rc; */

    rc = request_threaded_irq (i2c_client->irq, irq_handler, irq_thread_fn, 
            IRQF_TRIGGER_RISING, DRIVER_NAME, NULL);
	if (rc) {
        printk (KERN_ERR "%s: Failed to register irq.\n", __func__);
		return rc;
    }

    /* Register character device. */
    
    rc = alloc_chrdev_region (&chrdev_id, 0, 1, DEVICE_NAME);
    
    if (rc) {
        printk (KERN_ERR "%s: Failed to allocate cdev region.\n", __func__);
        free_irq (client->irq, NULL); 
        return -ENODEV;
	}

    cdev_init (&chrdev, &chrdev_fops);
    
    fresh = 0;
	
	rc = cdev_add (&chrdev, chrdev_id, 1);
    
    if (rc) {
        printk (KERN_ERR "%s: Failed to add cdev.\n", __func__);
        unregister_chrdev_region (chrdev_id, 1);
        free_irq (client->irq, NULL); 
		return -ENODEV;
	}
    
    rc = configure_chip (client);
    /* if (rc) return rc; */
    
    printk (KERN_INFO "%s: Attached new device: %s @ 0x%hx.\n", __func__, 
            client->name, client->addr);

    enable_interrupts_on_chip (client);
    return 0;
}

static int i2c_remove (struct i2c_client *client)
{
	cdev_del (&chrdev);
	unregister_chrdev_region (chrdev_id, 1);
    free_irq (client->irq, NULL); 

    power_down_chip (client);

    printk (KERN_INFO "%s: Device detached: %s @ 0x%hx.\n", __func__,
            client->name, client->addr);
	return 0;
}

static struct i2c_driver i2c_driver = {
	.id_table = i2c_id_table,
	.driver = {
        .name = DRIVER_NAME
    },
	.probe = i2c_probe,
	.remove = i2c_remove,
};

/* This macro replaces module_init & module_exit functions. */
module_i2c_driver (i2c_driver); 

MODULE_AUTHOR ("student");
MODULE_DESCRIPTION ("Fast MPU-6050 sensor driver.");
MODULE_LICENSE ("GPL");
