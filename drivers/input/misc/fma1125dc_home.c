/*
 * AD714X CapTouch Programmable Controller driver (I2C bus)
 *
 * Copyright 2009-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/input.h>	/* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>

/*
 * 	Global variables TODO:(some should be wrapped with structure)
 */

#define DEV_NAME "fma1125dc_home"

static struct device *fma_dev;
static unsigned char _buttons_status;
static dev_t fma1125_devt;
static struct cdev cdev;
static DECLARE_WAIT_QUEUE_HEAD(wq);
static int flag = 0;

/*
 * TODO: some  should come with platform data
 */

#define FMA1125_REG_CHK 	0x28
#define FMA1125_WARM_RST 	0xFF
#define FMA1125_TCH_BYTE 	0x68
#define FMA1125_PA_CONF		0x1C
#define FMA1125_ALARM_IRQ	gpio_to_irq(4)


#if 0
/*
 *	SYSFS representation
 */
static ssize_t fma1125dc_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%0X\n", _buttons_status);
}

static DEVICE_ATTR(buttons, 0664, fma1125dc_show, NULL);

#endif


/*
 *	I2C IO wrappers TODO:check results
 */ 


static int fma1125dc_i2c_write(struct device *dev, unsigned char reg,
				unsigned char data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, data);


	return ret;
}

static int fma1125dc_i2c_read(struct device *dev, unsigned char reg,
				unsigned char *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;	
	ret = *data = i2c_smbus_read_byte_data(client, reg);

	return ret;
}

/*
 * 		Interrupt handler
 */

static irqreturn_t fma1125dc_interrupt_thread(int irq, void *data)
{

	/* 
	 * 	TODO: Maybe we need mutex to prevent races  
	 */

	
	//printk("Buttons  %X are touched\n", _buttons_status);
	// FIXME: wake up the queue
	flag = 1;
	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}


/*
 * 		Char driver ops 
 */

static ssize_t fma1125dc_dev_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	printk("Read \n");
	wait_event_interruptible(wq, flag != 0);
	flag = 0;
	fma1125dc_i2c_read(fma_dev,FMA1125_TCH_BYTE, &_buttons_status);
	ret = copy_to_user(buf, &_buttons_status, sizeof(_buttons_status));

	return sizeof(_buttons_status);
}

static int fma1125dc_dev_open(struct inode *inode, struct file *file)
{
	/*	Init workqueue */
	return 0;
}

static int fma1125dc_dev_release(struct inode *inode, struct file *file)
{
	/*	Close workqueue */
	return 0;
}
static const struct file_operations fma1125dc_dev_fops = {
	.owner		= THIS_MODULE,
	.read		= fma1125dc_dev_read,
	.open		= fma1125dc_dev_open,
	.release	= fma1125dc_dev_release,

};



/*
 * 		I2C driver ops implementation
 */


static int __devinit fma1125dc_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	
	
	int error;
	u8 reg;
	fma_dev = &client->dev;

	/*
	 *	TODO:input class NOW: via char device interface
	 */


	// TODO: what to do if the returns error
	error = alloc_chrdev_region(&fma1125_devt, 0, 1, DEV_NAME);
	if (error < 0)
	{
		printk(KERN_ERR "%s: failed to allocate char dev region\n",
			         __FILE__);
	}
	cdev_init(&cdev, &fma1125dc_dev_fops);
	cdev_add(&cdev, fma1125_devt, 1);


	/*
	 *	Chip startup sequence
	 */	

	msleep(300);

	// ff -> 28

	fma1125dc_i2c_write(fma_dev,FMA1125_REG_CHK,0xFF);

	/*
	 *	Here we should config other registers
	 */

	// ee -> 1C
	fma1125dc_i2c_write(fma_dev, 0x1C, 0xEE);
	// 7f -> 08	
	fma1125dc_i2c_write(fma_dev, 0x08, 0x7F);
	// a9 -> 29
	fma1125dc_i2c_write(fma_dev, 0x29, 0xA9);


	// XX -> FF register (warm reset)

	fma1125dc_i2c_write(fma_dev,FMA1125_WARM_RST,0xFF);

	// wait 10 ms

	msleep(10);

	// 00 -> 0x1c

	fma1125dc_i2c_write(fma_dev,FMA1125_PA_CONF,0x00);

	printk("FMA hbutton is configured\n");


	/*
	 *	sysfs attr
	 */

	//device_create_file(fma_dev,&dev_attr_buttons);
	

	/*
	 *	IRQ handler preparation
	 */
	error = request_threaded_irq(FMA1125_ALARM_IRQ, NULL, fma1125dc_interrupt_thread, 					     IRQF_TRIGGER_RISING ,  DEV_NAME, NULL);
	if (error) {
		//dev_err(dev, "can't allocate irq %d\n", gpio_to_irq(5));
		printk( "can't allocate irq %d\n", FMA1125_ALARM_IRQ);		
		//goto err_unreg_dev;
	}

	return 0;
}

static int __devexit fma1125dc_i2c_remove(struct i2c_client *client)
{
	printk("FMA1125_ALARM driver is removed");	
	//device_remove_file(fma_dev,&dev_attr_buttons);
	free_irq(FMA1125_ALARM_IRQ, NULL);
	cdev_del(&cdev);

	unregister_chrdev_region(fma1125_devt, 1);
	

	return 0;
}


/*
 * 		TODO: Power management 
 */

#ifdef CONFIG_PM
static int fma1125dc_i2c_suspend(struct device *dev)
{
	return 0;
}

static int fma1125dc_i2c_resume(struct device *dev)
{
	return 0;
}
#else
# define fma1125dc_i2c_suspend NULL
# define fma1125dc_i2c_resume  NULL
#endif

static const struct i2c_device_id fma1125dc_id[] = {
	{ "fma_home_btn", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fma1125dc_id);

static struct i2c_driver fma1125dc_i2c_driver = {
	.driver = {
		.name = "fma1125dc_home",
	},
	.probe    = fma1125dc_i2c_probe,
	.remove   = __devexit_p(fma1125dc_i2c_remove),
	.suspend  = fma1125dc_i2c_suspend,
	.resume   = fma1125dc_i2c_resume,
	.id_table = fma1125dc_id,
};

static int __init fma1125dc_i2c_init(void)
{
	return i2c_add_driver(&fma1125dc_i2c_driver);
}


static void __exit fma1125dc_i2c_exit(void)
{
	i2c_del_driver(&fma1125dc_i2c_driver);
}

module_init(fma1125dc_i2c_init);
module_exit(fma1125dc_i2c_exit);

MODULE_DESCRIPTION("Touch Sensor controller FMA1125DC I2C Bus Driver");
MODULE_AUTHOR("Roman Alyautdin ralyautdin@dev.rtsoft.ru");
MODULE_LICENSE("GPL");
