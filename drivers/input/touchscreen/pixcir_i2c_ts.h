#ifndef	       _PIXCIR_I2C_TS_H
#define	       _PIXCIR_I2C_TS_H

/*TODO: check the actual wait time after reset*/

	#include <mach/gpio.h>

#define	       TRUE	       1
#define	       ATTB_PIN_LOW    0

struct pixcir_i2c_ts_platform {
       int (*attb_read_val) (void);
       int ts_x_max;
       int ts_y_max;
};



static int attb_read_val(void);
static void tangoC_init(void);
static void tangoC_reset(void);


#define ATTB		1
#define get_attb_value	gpio_get_value
#define	RESETPIN_SET0 	gpio_direction_output(2,0)
#define	RESETPIN_SET1	gpio_direction_output(2,1)

static int attb_read_val(void)
{
	return get_attb_value(ATTB);
}

static void tangoC_init(void)
{
	RESETPIN_SET1;
	mdelay(10);
	RESETPIN_SET0;
}

static void tangoC_reset(void)
{
	RESETPIN_SET1;
	mdelay(10);
	RESETPIN_SET0;
}





#endif

