#ifndef	       _PIXCIR_I2C_TS_H
#define	       _PIXCIR_I2C_TS_H



struct pixcir_ts_platform_data {
        int ts_x_max;
        int ts_y_max;
 	int (*tango_chip_reset)(void);
 	int (*attb_read_val)(void);
};


#endif

