/*
 *  linux/drivers/char/watchdog/max16058_wdt.h //FIXME
 *
 *  BRIEF MODULE DESCRIPTION
 *      OMAP Watchdog timer register definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */





#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <mach/hardware.h>
#include <plat/prcm.h>



#include "max16058_wdt.h"

struct max_wgt_platform_data *wgt_pdata;


static int __devinit max16058_wdt_probe(struct platform_device *pdev)
{

	if (wgt_pdata)
	{
		return -EBUSY;
	}	

	wgt_pdata = pdev->dev.platform_data;
	printk("WATCHDOG probe method!");

	return 0;
}


static int __devexit max16058_wdt_remove(struct platform_device *pdev)
{


	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int max16058_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{



	return 0;
}

static int max16058_wdt_resume(struct platform_device *pdev)
{

	return 0;
}

#else
#define	max16058_wdt_suspend		NULL
#define	max16058_wdt_resume		NULL
#endif



static struct platform_driver max16058_wdt_driver = {
	.probe		= max16058_wdt_probe,
	.remove		= __devexit_p(max16058_wdt_remove),
	.suspend	= max16058_wdt_suspend,
	.resume		= max16058_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "max16058_wdt",
	},
};

static int __init max16058_wdt_init(void)
{
	return platform_driver_register(&max16058_wdt_driver);
}

static void __exit max16058_wdt_exit(void)
{
	platform_driver_unregister(&max16058_wdt_driver);
}

module_init(max16058_wdt_init);
module_exit(max16058_wdt_exit);

MODULE_AUTHOR("Alyatdin Roman");
MODULE_LICENSE("GPL");
//MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);//FIXME
