/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
 *
 * pixcir_i2c_ts.c V3.0	from v3.0 support TangoC solution and remove the previous soltutions
 *
 * pixcir_i2c_ts.c V3.1	Add bootloader function	7
 *			Add RESET_TP		9
 * 			Add ENABLE_IRQ		10
 *			Add DISABLE_IRQ		11
 * 			Add BOOTLOADER_STU	12
 *			Add ATTB_VALUE		13
 *			Add Write/Read Interface for APP software
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include "pixcir_i2c_ts.h"
#include <linux/gpio.h>

/*********************************Bee-0928-TOP****************************************/

#define PIXCIR_DEBUG		1


/*
#ifndef I2C_MAJOR
#define I2C_MAJOR 		125
#endif

#define I2C_MINORS 		256

#define	CALIBRATION_FLAG	1
#define	BOOTLOADER		7
#define RESET_TP		9

#define	ENABLE_IRQ		10
#define	DISABLE_IRQ		11
#define	BOOTLOADER_STU		12
#define ATTB_VALUE		13

#define X_MAX 1024
#define Y_MAX 768
*/
static unsigned char status_reg = 0;


struct i2c_dev
{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

/*static struct i2c_driver pixcir_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);
*/


struct pixcir_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	const struct pixcir_ts_platform_data *chip;
	bool exiting;
};





struct point_data{
	unsigned char	brn;  //broken line number
	unsigned char	brn_pre;
	unsigned char	id;    //finger ID
	unsigned int	posx;
	unsigned int	posy;
	unsigned int	strg;
	
};

static struct point_data point[5];





static void pixcir_ts_poscheck(struct pixcir_i2c_ts_data *data)
{
	struct pixcir_i2c_ts_data *tsdata = data;
	
	unsigned char *p;
	unsigned char touch, button;
	unsigned char rdbuf[32], wrbuf[1] = { 0 };
	int ret, i, j;


	ret = i2c_master_send(tsdata->client, wrbuf, sizeof(wrbuf));
	if (ret != sizeof(wrbuf)) {
		dev_err(&tsdata->client->dev,
			"%s: i2c_master_send failed(), ret=%d\n",
			__func__, ret);
		return;
	}

	ret = i2c_master_recv(tsdata->client, rdbuf, sizeof(rdbuf));
	if (ret != sizeof(rdbuf)) {
		dev_err(&tsdata->client->dev,
			"%s: i2c_master_recv() failed, ret=%d\n",
			__func__, ret);
		return;
	}

	touch = rdbuf[0] & 0x07;
	button = rdbuf[1]; /* not implemented in hw */
	p= &rdbuf[2];
	for (i=0; i<touch; i++)	{
		point[i].id = (*(p + 4)) & 0x07;	//finger_id[i] = (*(p+4));
		point[i].posx = (((*(p + 1) << 8)) + (*(p)));	//posx[i] = (*(p+1)<<8)+(*(p));
		point[i].posy = ((*(p + 3) << 8) + (*(p + 2))); //posy[i] = (*(p+3)<<8)+(*(p+2));
		point[i].strg = *(rdbuf + 27 + i);
		p += 5;
	}

	if(touch){


		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_X, point[0].posx);
		input_report_abs(tsdata->input, ABS_Y, point[0].posy);
		for(j = 0; j < touch; j++)
		{
			input_report_abs(tsdata->input, ABS_MT_POSITION_X, point[j].posx);
			input_report_abs(tsdata->input, ABS_MT_POSITION_Y, point[j].posy);
			input_mt_sync(tsdata->input);
		}
	} else {

		input_report_key(tsdata->input, BTN_TOUCH, 0);
	}

	input_sync(tsdata->input);


}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata = dev_id;

	while (!tsdata->exiting) {
		pixcir_ts_poscheck(tsdata);

		if (tsdata->chip->attb_read_val()) {
			break;
	        }
		msleep(20);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int pixcir_i2c_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pixcir_dev_pm_ops,
			 pixcir_i2c_ts_suspend, pixcir_i2c_ts_resume);

static int __devinit pixcir_i2c_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct pixcir_ts_platform_data *pdata = client->dev.platform_data;
	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	int error;


	if (!pdata) {
		dev_err(&client->dev, "platform data not defined\n");
		return -EINVAL;
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!tsdata || !input) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	tsdata->client = client;
	tsdata->input = input;
	tsdata->chip = pdata;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* For single touch */

	input_set_abs_params(input, ABS_X, 0, pdata->ts_x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, pdata->ts_y_max, 0, 0);


	/* For multi-touch */

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->ts_x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->ts_y_max, 0, 0);



	//TODO: smth with pressure and major should be added
	input_set_drvdata(input, tsdata);

	error = request_threaded_irq(client->irq, NULL, pixcir_ts_isr,
				     IRQF_TRIGGER_FALLING,
				     client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}
	disable_irq_nosync(client->irq);

	error = input_register_device(input);
	if (error)
		goto err_free_irq;

	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);
	pdata->tango_chip_reset();
	enable_irq(client->irq);
	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);
	return error;
}

static int __devexit pixcir_i2c_ts_remove(struct i2c_client *client)
{

	struct pixcir_i2c_ts_data *tsdata = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();
	free_irq(client->irq, tsdata);

	input_unregister_device(tsdata->input);
	kfree(tsdata);

	return 0;
}


static const struct i2c_device_id pixcir_i2c_ts_id[] = {
	{ "pixcir_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "pixcir_i2c_ts",
		.pm	= &pixcir_dev_pm_ops,
	},
	.probe		= pixcir_i2c_ts_probe,
	.remove		= __devexit_p(pixcir_i2c_ts_remove),
	.id_table	= pixcir_i2c_ts_id,
};

static int __init pixcir_i2c_ts_init(void)
{

	return i2c_add_driver(&pixcir_i2c_ts_driver);
}


static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
}


module_init(pixcir_i2c_ts_init);
module_exit(pixcir_i2c_ts_exit);


MODULE_LICENSE("GPL");

