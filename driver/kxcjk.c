#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/input.h>

MODULE_AUTHOR("student");
MODULE_DESCRIPTION("kxcjk accelerometer driver");
MODULE_LICENSE("GPL");

#define DRIVER_VERSION "0"
#define DEVICE_NAME "kxcjk"

#define DEBUG 

#ifdef DEBUG
#define DBG(fmt, args...) printk("%s(%d): " fmt, __func__,__LINE__, ##args)
#else
#define DBG(fmt, args...)
#endif





// i2c command set for ilitek touch screen, see [1]
//
#define ILITEK_TP_CMD_READ_TOUCH_REPORTED_NUMBER 0x10
#define ILITEK_TP_CMD_READ_TOUCH_INFORMATION_REPORT 0x11
#define ILITEK_TP_CMD_READ_PANEL_INFORMATION 0x20
#define ILITEK_TP_SET_SLEEP 0x30
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION 0x40
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION 0x42
#define	ILITEK_TP_CMD_CALIBRATION 0xCC
#define	ILITEK_TP_CMD_CALIBRATION_STATUS 0xCD
#define ILITEK_TP_CMD_ERASE_BACKGROUND 0xCE

#define ILITEK_TP_TOUCH_STATUS_MASK 0x80
#define ILITEK_TP_TOUCH_KEY_MASK 0x40
#define ILITEK_TP_REPORTED_ID_MASK 0x3F
#define ILITEK_TP_TOUCH_STATUS_OFFSET 7
#define ILITEK_TP_TOUCH_KEY_OFFSET 6
#define ILITEK_TP_REPORTED_ID_OFFSET 0

#define __APPLY_MASK_AND_SHIFT_RIGHT(prefix,b) (((b) & (prefix##_MASK)) >> (prefix##_OFFSET))

#define ILITEK_TP_TOUCH_STATUS(b) __APPLY_MASK_AND_SHIFT_RIGHT(ILITEK_TP_TOUCH_STATUS,b)
#define ILITEK_TP_TOUCH_KEY(b) __APPLY_MASK_AND_SHIFT_RIGHT(ILITEK_TP_TOUCH_KEY,b)
#define ILITEK_TP_REPORTED_ID(b) __APPLY_MASK_AND_SHIFT_RIGHT(ILITEK_TP_REPORTED_ID,b)

#define ILITEK_TP_SCREEN_X(t) (((t)*(SCREEN_WIDTH-1))/tsdata.touch_width)
#define ILITEK_TP_SCREEN_Y(t) (((t)*(SCREEN_HEIGHT-1))/tsdata.touch_height)


static int ilitek_init(void);
static void ilitek_exit(void);
static void ilitek_set_input_param(void);
static int ilitek_i2c_register_device(void);
static int ilitek_i2c_probe(struct i2c_client*, const struct i2c_device_id*);
static int ilitek_i2c_polling_thread(void*);
static int ilitek_i2c_process_and_report(void);
static int ilitek_i2c_read_tp_info(void);
static int ilitek_i2c_read(struct i2c_client*, uint8_t, uint8_t*, int);
static int ilitek_i2c_remove(struct i2c_client*);


// general i2c ts driver variables combined into single global structure 
//
struct i2c_data {
    // input device
    struct input_dev *input;
    // i2c client
    struct i2c_client *client;
    // polling thread
    struct task_struct *thread;
    // max coordinates
    int touch_width, touch_height;
    // max touch points
    int max_tp;
    // check whether i2c driver is registered successfully
    int i2c_registered;
    // check whether input driver is registered successfully
    int input_registered;
};

static struct i2c_data tsdata = {0};


// i2c id table
//
static const struct i2c_device_id ilitek_i2c_id[] ={
	{ILITEK_I2C_DEVICE_NAME, 0}, 
    {}
};


MODULE_DEVICE_TABLE(tsdata, ilitek_i2c_id);


// declare i2c function table
//
static struct i2c_driver ilitek_i2c_driver = {
	.id_table = ilitek_i2c_id,
	.driver = {.name = ILITEK_I2C_DEVICE_NAME},
	.probe = ilitek_i2c_probe,
	.remove = ilitek_i2c_remove,
};


// set input device parameters
// 
static void ilitek_set_input_param(void)
{
	tsdata.input->evbit[0] = BIT_MASK(EV_ABS);
    input_set_abs_params(tsdata.input, ABS_MT_TRACKING_ID, 0, tsdata.max_tp-1, 0, 0);
    input_set_abs_params(tsdata.input, ABS_MT_POSITION_X, 0, SCREEN_WIDTH-1, 0, 0); // touch_width+2
    input_set_abs_params(tsdata.input, ABS_MT_POSITION_Y, 0, SCREEN_HEIGHT-1, 0, 0); // touch_height+2

    tsdata.input->name = ILITEK_I2C_DEVICE_NAME;
    tsdata.input->id.bustype = BUS_I2C;
    tsdata.input->dev.parent = &(tsdata.client)->dev;
	tsdata.input->id.vendor = 0xDEAD;
	tsdata.input->id.product = 0xBEEF;
	tsdata.input->id.version = 0x0001;
}


// process i2c data and then report to kernel
// 
static int ilitek_i2c_process_and_report(void)
{
	int i, n, x, y, id, st, ret;
    unsigned char buf[5]={0};

    static struct { int st, x, y; }
        saved[10] = {{0}};
    
    // read data from device, see [1]
    ret = ilitek_i2c_read(tsdata.client, ILITEK_TP_CMD_READ_TOUCH_REPORTED_NUMBER, buf, 1);
	if (ret < 0) return ret;

    n = buf[0];
    
    for (i = 0; i < n; i++) {
        ret = ilitek_i2c_read(tsdata.client, ILITEK_TP_CMD_READ_TOUCH_INFORMATION_REPORT, buf, 5);
        if (ret < 0) continue;
        
        id = ILITEK_TP_REPORTED_ID(buf[0]) - 1;
        st = ILITEK_TP_TOUCH_STATUS(buf[0]); 
        x = ILITEK_TP_SCREEN_X(((int)buf[1] << 8) + buf[2]);
        y = ILITEK_TP_SCREEN_Y(((int)buf[3] << 8) + buf[4]);
        
        if (st == saved[id].st) {
            if (st == 0) continue;
            if (x == saved[id].x && y == saved[id].y) continue;
        }
        
        saved[id].st = st;
        saved[id].x = x;
        saved[id].y = y;

        DBG("Point ID=%02X %s X=%04d  Y=%04d\n", id, st ? "TOUCH":"RELEASE", x, y);	
        
        input_event(tsdata.input, EV_ABS, ABS_MT_TRACKING_ID, id);
        if (st == 1) {
            input_event(tsdata.input, EV_ABS, ABS_MT_POSITION_X, x);
            input_event(tsdata.input, EV_ABS, ABS_MT_POSITION_Y, y);
        }
        input_mt_sync(tsdata.input);
    }
    input_sync(tsdata.input);

    return 0;
}


// read data from i2c device
//
static int ilitek_i2c_read(struct i2c_client *client, uint8_t cmd, 
        uint8_t *data, int length)
{
	int ret;
    struct i2c_msg msgs_cmd[] = {
		{.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,}};
    struct i2c_msg msgs_data[]={
		{.addr = client->addr, .flags = I2C_M_RD, .len=length, .buf=data,}};
    
    ret = i2c_transfer(client->adapter, msgs_cmd, 1);
	if (ret < 0) {
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
        return ret;
    }
    
    ret = i2c_transfer(client->adapter, msgs_data,1);
	if(ret < 0)
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
	
    return ret;
}


// i2c polling thread
//
static int ilitek_i2c_polling_thread(void *arg)
{
	printk(ILITEK_DEBUG_LEVEL "%s, enter\n", __func__);

	while (1) {
		// check whether we should exit or not
		if (kthread_should_stop()) {
			printk(ILITEK_DEBUG_LEVEL "%s, stop\n", __func__);
			break;
		}

		// this delay will influence the CPU usage
        // and response latency
		msleep(5);

		// read i2c data
		if (ilitek_i2c_process_and_report() < 0) {
			msleep(3000);
			printk(ILITEK_ERROR_LEVEL "%s, process error\n", __func__);
		}
	}
	
	printk(ILITEK_DEBUG_LEVEL "%s, exit\n", __func__);
	return 0;
}


// when adapter detects the i2c device, 
// this function will be invoked.
//
static int ilitek_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    // check whether i2c client adapter supports 
    // pure i2c-level commands or not; see [2]
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(ILITEK_ERROR_LEVEL "%s, I2C_FUNC_I2C not support\n", __func__);
        return -1;
    }
	
    tsdata.client = client;
    
    printk(ILITEK_DEBUG_LEVEL "%s, i2c new style format\n", __func__);
	printk("%s, IRQ: 0x%X\n", __func__, client->irq);
    return 0;
}


// when the i2c device want to detach from adapter, 
// this function will be invoked.
//
static int ilitek_i2c_remove(struct i2c_client *client)
{
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
	return 0;
}


//   read data from i2c device with delay between cmd & return data
//
static int ilitek_i2c_read_info(struct i2c_client *client,
        uint8_t cmd, uint8_t *data, int length)
{
	int ret;
	struct i2c_msg msgs_cmd[] = {
	    {.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,}};
	
	struct i2c_msg msgs_ret[] = {
	    {.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}};

	ret = i2c_transfer(client->adapter, msgs_cmd, 1);
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret); }
	
	msleep(10);

	ret = i2c_transfer(client->adapter, msgs_ret, 1);
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret); }
	return ret;
}


// read touch information
//
static int ilitek_i2c_read_tp_info(void)
{
	int ret;
    unsigned char buf[10]={0};

	// read firmware version
    ret = ilitek_i2c_read_info(tsdata.client, ILITEK_TP_CMD_GET_FIRMWARE_VERSION, buf, 3);
    if (ret < 0) return ret; 
	printk(ILITEK_DEBUG_LEVEL "%s, firmware version %d.%d.%d\n", __func__, buf[0], buf[1], buf[2]);

	// read protocol version
    ret = ilitek_i2c_read_info(tsdata.client, ILITEK_TP_CMD_GET_PROTOCOL_VERSION, buf, 2);
    if (ret < 0) return ret; 
    printk(ILITEK_DEBUG_LEVEL "%s, protocol version: %d.%d\n", __func__, buf[0], buf[1]);

    // read panel information
    ret = ilitek_i2c_read_info(tsdata.client, ILITEK_TP_CMD_READ_PANEL_INFORMATION, buf, 10);
    if (ret < 0) return ret; 
    
    // maximum touch point
    tsdata.max_tp = buf[6];
	
    // resolution for x and y direction
    tsdata.touch_width = ((int)buf[1] << 8) + buf[0];
    tsdata.touch_height = ((int)buf[3] << 8) + buf[2];

    printk(ILITEK_DEBUG_LEVEL "%s, touch_width: %d, touch_height: %d, max_tp: %d\n", 
            __func__, tsdata.touch_width, tsdata.touch_height, tsdata.max_tp);
    return 0;
}


// register i2c device and its input device
//
static int ilitek_i2c_register_device(void)
{
    int ret;
    
    // register i2c driver
    ret = i2c_add_driver(&ilitek_i2c_driver);
    if (ret) {
        printk(ILITEK_ERROR_LEVEL "%s, add i2c device, error\n", __func__);
        return ret;
    }
    printk(ILITEK_DEBUG_LEVEL "%s, add i2c device, success\n", __func__);
    tsdata.i2c_registered = 1;
    
    // check i2c client connection
    if (tsdata.client == NULL) {
        printk(ILITEK_ERROR_LEVEL "%s, no i2c board information\n", __func__);
        return -1;
    }
    printk(ILITEK_DEBUG_LEVEL "%s, client.addr: 0x%X\n", __func__, (unsigned int)tsdata.client->addr);
    printk(ILITEK_DEBUG_LEVEL "%s, client.adapter: 0x%X\n", __func__, (unsigned int)tsdata.client->adapter);
    printk(ILITEK_DEBUG_LEVEL "%s, client.driver: 0x%X\n", __func__, (unsigned int)tsdata.client->driver);
    if ((tsdata.client->addr == 0) || (tsdata.client->adapter == 0) || (tsdata.client->driver == 0)){
        printk(ILITEK_ERROR_LEVEL "%s, invalid client parameter\n", __func__);
        return -1;
    }
    
    // fill tsdata with valid touch screen parameters
    ilitek_i2c_read_tp_info();

    // init input device
    tsdata.input = input_allocate_device();
    if (tsdata.input == NULL) {
        printk(ILITEK_ERROR_LEVEL "%s, allocate input device, error\n", __func__);
        return -1;
    }
    ilitek_set_input_param();
    ret = input_register_device(tsdata.input);
    if (ret) {
        printk(ILITEK_ERROR_LEVEL "%s, register input device, error\n", __func__);
        return ret;
    }
    printk(ILITEK_ERROR_LEVEL "%s, register input device, success\n", __func__);
    tsdata.input_registered = 1;

    // run thread
    tsdata.thread = kthread_create(ilitek_i2c_polling_thread, NULL, "ilitek_i2c_thread");
    if (tsdata.thread == (struct task_struct*)ERR_PTR) {
        tsdata.thread = NULL;
        printk(ILITEK_ERROR_LEVEL "%s, kthread create, error\n", __func__);
        return -1;
    }
    wake_up_process(tsdata.thread);
    return 0;
}


// init function for driver
//
static int __init ilitek_init(void)
{
	int ret;

	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
    DBG("compiled with DEBUG enabled\n");

	// register i2c device
	ret = ilitek_i2c_register_device();
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, register i2c device, error\n", __func__);
		return ret;
	}
	return 0;
}


// driver exit function
//
static void __exit ilitek_exit(void)
{
    if (tsdata.thread) {
        printk(ILITEK_DEBUG_LEVEL "%s, stop i2c polling thread\n", __func__);
        kthread_stop(tsdata.thread);
    }

    if (tsdata.i2c_registered) {
        printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
        i2c_del_driver(&ilitek_i2c_driver);
    }

    if (tsdata.input_registered) {
        printk(ILITEK_DEBUG_LEVEL "%s, unregister input device\n", __func__);
        input_unregister_device(tsdata.input);
    }
    printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
}


// set init and exit function for this module
//
module_init(ilitek_init);
module_exit(ilitek_exit);

