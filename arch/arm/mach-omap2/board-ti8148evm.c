/*
 * Code for TI8148 EVM.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/qt602240_ts.h>
#include <linux/i2c/pcf857x.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps65910.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/asp.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/hdmi_lib.h>
#include <mach/board-ti814x.h>

#include "board-flash.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux     NULL
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL, /* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

//FIXME eeprom 8kbit
static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
};

static struct regulator_consumer_supply ti8148evm_mpu_supply =
	REGULATOR_SUPPLY("mpu", "mpu.0");

/*
 * DM814x/AM387x (TI814x) devices have restriction that none of the supply to
 * the device should be turned of.
 *
 * NOTE: To prevent turning off regulators not explicitly consumed by drivers
 * depending on it, ensure following:
 *	1) Set always_on = 1 for them OR
 *	2) Avoid calling regulator_has_full_constraints()
 *
 * With just (2), there will be a warning about incomplete constraints.
 * E.g., "regulator_init_complete: incomplete constraints, leaving LDO8 on"
 *
 * In either cases, the supply won't be disabled.
 *
 * We are taking approach (1).
 */
static struct regulator_init_data tps65911_reg_data[] = {
	/* VRTC */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VIO -VDDA 1.8V */
	{
		.constraints = {
			.min_uV = 1500000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VDD1 - MPU */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
		.num_consumer_supplies	= 1,
		.consumer_supplies	= &ti8148evm_mpu_supply,
	},

	/* VDD2 - DSP */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VDDCtrl - CORE */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1400000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO1 - VDAC */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO2 - HDMI */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO3 - GPIO 3.3V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO4 - PLL 1.8V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO5 - SPARE */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO6 - CDC */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO7 - SPARE */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO8 - USB 1.8V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},
};

static struct tps65910_board __refdata tps65911_pdata = {
	.irq				= 0,	/* No support currently */
	.gpio_base			= 0,	/* No support currently */
	.tps65910_pmic_init_data	= tps65911_reg_data,
};

static struct i2c_board_info __initdata ti814x_i2c_boardinfo1[] = {
	{	/* audio codec */
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{	/* PMIC */
		I2C_BOARD_INFO("tps65911", 0x2D),
		.platform_data = &tps65911_pdata,
	},
	{	/* DA converter alarmboard */
		I2C_BOARD_INFO("DAalarm", 0x30),
	},
	{	/* DA converter backlight brightness X3 board */
		I2C_BOARD_INFO("DAbacklight", 0x31),
	},
	{	/* Step down converter */
		I2C_BOARD_INFO("tps62353", 0x48),
	},
	{	/* current monitor */
		I2C_BOARD_INFO("currentmon", 0x49),
	},
	{	/* temperature sensor */
		I2C_BOARD_INFO("lm75", 0x4a),
	},
	{	/* aliases seen on 0x51 0x52 0x53 */
		I2C_BOARD_INFO("eeprom", 0x50),
		.platform_data	= &eeprom_info,
	},
};

static struct i2c_board_info __initdata ti814x_i2c_boardinfo2[] = {
        {       /* VGA DAC */
                I2C_BOARD_INFO("vga_dac", 0x20),
        },
        {       // DDC of VGA monitor 
                I2C_BOARD_INFO("DDC", 0x50),
        },
};

static struct i2c_board_info __initdata ti814x_i2c_boardinfo3[] = {
        {	/* ambiant light sensor */
                I2C_BOARD_INFO("als", 0x44),
        },
        {	/* touch button controller HOME */
                I2C_BOARD_INFO("fma_home_btn", 0x68),
        },
        {	/* touch button controller ALARM */
                I2C_BOARD_INFO("fma_alarm_btn", 0x69),
        },

};

/* Touchscreen platform data */

#include <../../../drivers/input/touchscreen/pixcir_i2c_ts.h>

#define GPIO_TSC_ATT	        1
#define GPIO_TSC_RST	        2
#define GPIO_TSC_INT	        1

static struct pixcir_i2c_ts_platform pixcir_platform_data = {
	.ts_x_max	  = 1024,
	.ts_y_max	  = 768,
};

static struct i2c_board_info __initdata ti814x_i2c_boardinfo4[] = {


	{
		 I2C_BOARD_INFO("pixcir_ts", 0x5c),
		 .platform_data = &pixcir_platform_data,
	},


};

#define GP_OUT		1
#define GP_IN		0
#define GP_USER		1
#define GP_KERNEL	0
#define GP_HIGH		1
#define GP_LOW		0

static void __init gpio_init(int nr, int direction, int level, int user, const char *name)
{
        /* if input level is not used (irrelevant) */
	int error;

        error = gpio_request(nr, name);
        if (error < 0) {
                printk(KERN_ERR "%s: failed to request GPIO"
                        ": %d\n", __func__, error);
        } else {
                if (direction == GP_OUT)
        		gpio_direction_output(nr, level);
                if (direction == GP_IN)
        		gpio_direction_input(nr);
                if (user == GP_USER)
        		gpio_export(nr, true);
                if (user == GP_KERNEL)
        		gpio_export(nr, false);
	}

}

static void __init gpios_mq(void)
{
        /* names are netnames from schematic */
	gpio_init( 1, GP_IN,  GP_IN,   GP_USER, "TC_ATTn");
	gpio_init( 2, GP_OUT, GP_LOW,  GP_USER, "RESET_TC");	//out of reset
	gpio_init( 3, GP_OUT, GP_HIGH, GP_USER, "RESET_TBC1n"); //out of reset
	gpio_init( 4, GP_IN,  GP_IN,   GP_USER, "TOUCHBT_TINT");
	gpio_init( 5, GP_IN,  GP_IN,   GP_USER, "TBC_INT");
	gpio_init( 6, GP_OUT, GP_HIGH, GP_USER, "RESET_TBC2n"); //out of reset
	gpio_init( 8, GP_IN,  GP_IN,   GP_USER, "TOUCHBT_GINT");
	gpio_init( 9, GP_IN,  GP_IN,   GP_USER, "TBC_INTG");
	gpio_init(14, GP_IN,  GP_IN,   GP_USER, "ALED_FAULTn");
	gpio_init(15, GP_OUT, GP_LOW,  GP_USER, "ALED_EN");
	gpio_init(16, GP_OUT, GP_HIGH, GP_USER, "SW_Y_ONn");
	gpio_init(17, GP_OUT, GP_HIGH, GP_USER, "SW_RD_ONn");
	gpio_init(18, GP_OUT, GP_HIGH, GP_USER, "SW_BL_ONn");
	gpio_init(19, GP_IN,  GP_IN,   GP_USER, "OVERTEMPn");
	gpio_init(20, GP_IN,  GP_IN,   GP_USER, "mSATA_PRESENTn");
	gpio_init(21, GP_IN,  GP_IN,   GP_USER, "LAN_IRQn");
	gpio_init(22, GP_IN,  GP_IN,   GP_USER, "SW_WP");
	gpio_init(23, GP_IN,  GP_IN,   GP_USER, "IRQ_ALS");
	gpio_init(24, GP_OUT, GP_HIGH, GP_USER, "LVDS_VDDENA"); //enable LVDS
	gpio_init(25, GP_OUT, GP_HIGH, GP_USER, "SHTDN_LVDSn"); //enable LVDS
	gpio_init(26, GP_OUT, GP_HIGH, GP_USER, "WDOG_TRIG");
	gpio_init(27, GP_OUT, GP_LOW,  GP_USER, "PWR_EN_TCn");
	gpio_init(28, GP_OUT, GP_LOW,  GP_USER, "WDOG_DISABLE");
	gpio_init(29, GP_OUT, GP_HIGH, GP_USER, "AUD_AMP_SHDNn");
	gpio_init(30, GP_IN,  GP_IN,   GP_USER, "BOOT_EN_FBn");
	gpio_init(31, GP_OUT, GP_LOW,  GP_USER, "BOOTEN_INT");
	gpio_init(51, GP_OUT, GP_HIGH, GP_USER, "USB[0]_EN");
	gpio_init(52, GP_OUT, GP_HIGH, GP_USER, "USB[1]_EN");
	gpio_init(53, GP_OUT, GP_HIGH, GP_USER, "LAN_RST_EN");
}

#if 0
static void __init pixcir_tsc_init(void)
{
	int error;
	/*
	*	Tune clock and data
	*/
	omap_mux_init_signal("dcan0_rx.i2c3_scl_mux1", TI814X_PULL_UP);
	omap_mux_init_signal("dcan0_tx.i2c3_sda_mux1", TI814X_PULL_UP);
	/*
	*	INT
	*/
	omap_mux_init_signal("mmc0_clk.gpio0_1", TI814X_PULL_UP);
	error = gpio_request(GPIO_TSC_ATT, "tsc_int");
	if (error < 0) {
		printk(KERN_ERR "%s: failed to request GPIO for TSC INT"
			": %d\n", __func__, error);
		return;
	}
        gpio_direction_input(GPIO_TSC_INT);
        ti814x_i2c_boardinfo4[0].irq = gpio_to_irq(GPIO_TSC_ATT);
	gpio_export(GPIO_TSC_INT, true);

	/*
	*	RST
	*/
	omap_mux_init_signal("mmc0_cmd.gpio0_2", TI814X_PULL_UP);
	error = gpio_request(GPIO_TSC_RST, "tsc_rst");
	if (error < 0) {
		printk(KERN_ERR "%s: failed to request GPIO for TSC RST"
			": %d\n", __func__, error);
		return;
	}
        gpio_direction_output(GPIO_TSC_RST, 0);
	gpio_export(GPIO_TSC_RST, true);
	
}
#endif

static void __init pixcir_tsc_init(void)
{
        ti814x_i2c_boardinfo4[0].irq = gpio_to_irq(GPIO_TSC_ATT);
}

static void __init ti814x_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, ti814x_i2c_boardinfo1,
				ARRAY_SIZE(ti814x_i2c_boardinfo1));
        /* // this bus is used by cortex M3 for hdmi; do not use
	omap_register_i2c_bus(2, 100, ti814x_i2c_boardinfo2,
				ARRAY_SIZE(ti814x_i2c_boardinfo2));
        */
	omap_register_i2c_bus(3, 100, ti814x_i2c_boardinfo3,
				ARRAY_SIZE(ti814x_i2c_boardinfo3));
	omap_register_i2c_bus(4, 100, ti814x_i2c_boardinfo4,
				ARRAY_SIZE(ti814x_i2c_boardinfo4));
}

static u8 ti8148_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data ti8148_evm_snd_data = {
	.tx_dma_offset	= 0x46800000,
	.rx_dma_offset	= 0x46800000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(ti8148_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= ti8148_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

/* NOR Flash partitions */
static struct mtd_partition ti814x_evm_norflash_partitions[] = {
	/* bootloader (U-Boot, etc) in first 5 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 2 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name		= "env",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 25 * SZ_2M,
		.mask_flags	= 0
	},
	/* reserved */
	{
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

/* NAND flash information */
static struct mtd_partition ti814x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "U-Boot-min",
		.offset         = 0,    /* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,/* Offset = 0x0 + 128K */
		.size           = 18 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 34 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x6C0000 */
		.size           = 1601 * SZ_128K,
	},
	{
		.name           = "Reserved",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0xCEE0000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* SPI fLash information */
struct mtd_partition ti8148_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name		= "U-Boot-min",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 32 * SZ_4K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND, /* 0x0 + (32*4K) */
		.size		= 64 * SZ_4K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND, /* 0x40000 + (32*4K) */
		.size		= 2 * SZ_4K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND, /* 0x42000 + (32*4K) */
		.size		= 640 * SZ_4K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND, /* 0x2C2000 + (32*4K) */
		.size		= MTDPART_SIZ_FULL, /* size ~= 1.1 MiB */
	}
};

const struct flash_platform_data ti8148_spi_flash = {
	.type		= "w25x32",
	.name		= "spi_flash",
	.parts		= ti8148_spi_partitions,
	.nr_parts	= ARRAY_SIZE(ti8148_spi_partitions),
};

struct spi_board_info __initdata ti8148_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &ti8148_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 75000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
};

void __init ti8148_spi_init(void)
{
	spi_register_board_info(ti8148_spi_slave_info,
				ARRAY_SIZE(ti8148_spi_slave_info));
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power		= 500,
	.instances	= 1,
};

static struct platform_device vpss_device = {
	.name = "vpss",
	.id = -1,
	.dev = {
		.platform_data = NULL,
	},
};
static void __init ti8148_evm_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static void __init ti814x_vpss_init(void)
{
	/*FIXME add platform data here*/

	if (platform_device_register(&vpss_device))
		printk(KERN_ERR "failed to register ti814x_vpss device\n");
	else
		printk(KERN_INFO "registered ti814x_vpss device\n");
}

static struct platform_device ti814x_hdmi_plat_device = {
	.name = "TI81XX_HDMI",
	.id = -1,
	.num_resources = 0,
	.dev = {
		/*.release = ti81xx_hdmi_platform_release,*/
		.platform_data = NULL,
	}
};

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_hdmi_platform_data ti8148_snd_hdmi_pdata = {
	.dma_addr = TI81xx_HDMI_WP + HDMI_WP_AUDIO_DATA,
	.channel = 53,
	.data_type = 4,
	.acnt = 4,
	.fifo_level = 0x20,
};

static struct platform_device ti8148_hdmi_audio_device = {
	.name   = "hdmi-dai",
	.id     = -1,
	.dev = {
		.platform_data = &ti8148_snd_hdmi_pdata,
	}
};

static struct platform_device ti8148_hdmi_codec_device = {
	.name   = "hdmi-dummy-codec",
	.id     = -1,
};

static struct platform_device *ti8148_devices[] __initdata = {
	&ti8148_hdmi_audio_device,
	&ti8148_hdmi_codec_device,
};
#endif

static void __init ti814x_hdmi_init(void)
{

	if (platform_device_register(&ti814x_hdmi_plat_device))
		printk(KERN_ERR "Could not register TI814x onchip-HDMI device\n");
	else
		printk(KERN_INFO "registered TI814x on-chip HDMI device\n");
	/*FIXME add platform data here*/
}

static void __init ti8148_evm_init(void)
{
	int bw; /* bus-width */

	//ti814x_mux_init(board_mux);
	omap_serial_init();
	//ti814x_tsc_init();
	gpios_mq();
	pixcir_tsc_init();
	ti814x_evm_i2c_init();
	ti81xx_register_mcasp(0, &ti8148_evm_snd_data);

	omap2_hsmmc_init(mmc);

	/* nand initialisation */
	if (cpu_is_ti814x()) {
		u32 *control_status = TI81XX_CTRL_REGADDR(0x40);
		if (*control_status & (1<<16))
			bw = 0; /*8-bit nand if BTMODE BW pin on board is ON*/
		else
			bw = 2; /*16-bit nand if BTMODE BW pin on board is OFF*/

		board_nand_init(ti814x_nand_partitions,
			ARRAY_SIZE(ti814x_nand_partitions), 0, bw);
	} else
		board_nand_init(ti814x_nand_partitions,
		ARRAY_SIZE(ti814x_nand_partitions), 0, NAND_BUSWIDTH_16);

	/* initialize usb */
	usb_musb_init(&musb_board_data);

	ti8148_spi_init();
	ti814x_vpss_init();
	ti814x_hdmi_init();
#ifdef CONFIG_SND_SOC_TI81XX_HDMI
	platform_add_devices(ti8148_devices, ARRAY_SIZE(ti8148_devices));
#endif
	regulator_use_dummy_regulator();
	board_nor_init(ti814x_evm_norflash_partitions,
		ARRAY_SIZE(ti814x_evm_norflash_partitions), 0);
}

static void __init ti8148_evm_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

MACHINE_START(TI8148EVM, "ti8148evm")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= ti8148_evm_map_io,
	.reserve         = ti81xx_reserve,
	.init_irq	= ti8148_evm_init_irq,
	.init_machine	= ti8148_evm_init,
	.timer		= &omap_timer,
MACHINE_END
