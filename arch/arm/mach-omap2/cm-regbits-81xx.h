/*
 * TI81XX CM register bits.
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

#ifndef __ARCH_ARM_MACH_OMAP2_CM_REGBITS_81XX_H
#define __ARCH_ARM_MACH_OMAP2_CM_REGBITS_81XX_H

#include "cm.h"

#define TI81XX_CLKSEL_0_0_SHIFT				0
#define TI81XX_CLKSEL_0_0_MASK				(1 << 0)

#define TI81XX_CLKSEL_0_1_SHIFT				0
#define TI81XX_CLKSEL_0_1_MASK				(3 << 0)

#define TI81XX_CLKSEL_0_2_SHIFT				0
#define TI81XX_CLKSEL_0_2_MASK				(7 << 0)

/* Modulemode bit */
#define TI81XX_MODULEMODE_SWCTRL			1

/* IDLEST bit */
#define TI81XX_IDLEST_SHIFT				16
#define TI81XX_IDLEST_MASK				(3 << 16)
#define TI81XX_IDLEST_VAL				3

/* Used for clockdomain control */
#define TI81XX_CLKTRCTRL_SHIFT				0
#define TI81XX_CLKTRCTRL_MASK				(3 << 0)

/* Select CLKOUT_MUX source */
#define TI814X_CMDEVICE_SYSCLKOUT_SRC_SHIFT		0
#define TI814X_CMDEVICE_SYSCLKOUT_SRC_MASK		(3 << 0)

#define TI814X_CMDEVICE_SYSCLKOUT_DIV_SHIFT		3
#define TI814X_CMDEVICE_SYSCLKOUT_DIV_MASK		(7 << 3)

#define TI814X_CMDEVICE_SYSCLKOUT_EN_SHIFT		7
#define TI814X_CMDEVICE_SYSCLKOUT_EN_MASK		(1 << 7)

/* Select OSC source */
#define TI814X_OSC_L3_PLL_CLKS_SHIFT			0
#define TI814X_OSC_L3_PLL_CLKS_MASK			(1 << 0)

#define TI814X_OSC_VIDEO0_PLL_CLKS_SHIFT		16
#define TI814X_OSC_VIDEO0_PLL_CLKS_MASK			(1 << 16)

#define TI814X_OSC_VIDEO1_PLL_CLKS_SHIFT		17
#define TI814X_OSC_VIDEO1_PLL_CLKS_MASK			(1 << 17)

#define TI814X_OSC_HDMI_PLL_CLKS_SHIFT			18
#define TI814X_OSC_HDMI_PLL_CLKS_MASK			(1 << 18)

#define TI814X_OSC_AUDIO_PLL_CLKS_SHIFT			24
#define TI814X_OSC_AUDIO_PLL_CLKS_MASK			(1 << 24)

#define TI814X_OSC_DDR_PLL_CLKS_SHIFT			25
#define TI814X_OSC_DDR_PLL_CLKS_MASK			(1 << 25)

#define TI814X_OSC_USB_PLL_CLKS_SHIFT			26
#define TI814X_OSC_USB_PLL_CLKS_MASK			(1 << 26)

#define TI814X_OSC_DSS_PLL_CLKS_SHIFT			27
#define TI814X_OSC_DSS_PLL_CLKS_MASK			(1 << 27)

#define TI814X_OSC_ISS_PLL_CLKS_SHIFT			28
#define TI814X_OSC_ISS_PLL_CLKS_MASK			(1 << 28)

#define TI814X_OSC_IVA_PLL_CLKS_SHIFT			29
#define TI814X_OSC_IVA_PLL_CLKS_MASK			(1 << 29)

#define TI814X_OSC_SGX_PLL_CLKS_SHIFT			30
#define TI814X_OSC_SGX_PLL_CLKS_MASK			(1 << 30)

#define TI814X_OSC_GEM_PLL_CLKS_SHIFT			31
#define TI814X_OSC_GEM_PLL_CLKS_MASK			(1 << 31)

/* Select ARM_CLKSRC */
#define TI814X_ARM_CLKS_SHIFT				0
#define TI814X_ARM_CLKS_MASK				(1 << 0)

/* Select VIDEO_PLL_CLKSRC */
#define TI814X_VIDEO_PLL_CLK2S_SHIFT			0
#define TI814X_VIDEO_PLL_CLK2S_MASK			(1 << 0)

#define TI814X_VIDEO_PLL_OUTMUX_CLKS_SHIFT		8
#define TI814X_VIDEO_PLL_OUTMUX_CLKS_MASK		(3 << 8)

#define TI814X_VIDEO_PLL_TPPSSSTCO_SHIFT		16
#define TI814X_VIDEO_PLL_TPPSSSTCO_MASK			(3 << 16)

#define TI814X_VIDEO_PLL_TPPSSSTSO_MUX_SHIFT		18
#define TI814X_VIDEO_PLL_TPPSSSTSO_MUX_MASK		(3 << 18)

#define TI814X_VIDEO_PLL_HD_VENC_G_SHIFT		24
#define TI814X_VIDEO_PLL_HD_VENC_G_MASK			(1 << 24)

/* Select MLB_ATL_CLKSRC */
#define TI814X_MLB_ATL_MLB_SHIFT			0
#define TI814X_MLB_ATL_MLB_MASK				(1 << 0)

#define TI814X_MLB_ATL_ATL_SHIFT			16
#define TI814X_MLB_ATL_ATL_MASK				(3 << 16)

/* Select McASP345_AUX_CLKSRC */
#define TI814X_MCASP3_AUX_SHIFT				0
#define TI814X_MCASP3_AUX_MASK				(7 << 0)

#define TI814X_MCASP4_AUX_SHIFT				8
#define TI814X_MCASP4_AUX_MASK				(7 << 8)

#define TI814X_MCASP5_AUX_SHIFT				16
#define TI814X_MCASP5_AUX_MASK				(7 << 16)

/* Select MCASP_AHCLK_CLKSRC */
#define TI814X_MCASP0_AHCLKX_SHIFT			0
#define TI814X_MCASP0_AHCLKX_MASK			(7 << 0)

#define TI814X_MCASP0_AHCLKR_SHIFT			3
#define TI814X_MCASP0_AHCLKR_MASK			(7 << 3)

#define TI814X_MCASP1_AHCLKX_SHIFT			6
#define TI814X_MCASP1_AHCLKX_MASK			(7 << 6)

#define TI814X_MCASP1_AHCLKR_SHIFT			9
#define TI814X_MCASP1_AHCLKR_MASK			(7 << 9)

#define TI814X_MCASP2_AHCLKX_SHIFT			16
#define TI814X_MCASP2_AHCLKX_MASK			(7 << 16)

#define TI814X_MCASP3_AHCLKX_SHIFT			19
#define TI814X_MCASP3_AHCLKX_MASK			(7 << 19)

#define TI814X_MCASP4_AHCLKX_SHIFT			22
#define TI814X_MCASP4_AHCLKX_MASK			(7 << 22)

#define TI814X_MCASP5_AHCLKX_SHIFT			25
#define TI814X_MCASP5_AHCLKX_MASK			(7 << 25)

/* Select McBSP_UART_CLKSRC */
#define TI814X_MCBSP_SHIFT				0
#define TI814X_MCBSP_MASK				(7 << 0)

#define TI814X_UART3_SHIFT				3
#define TI814X_UART3_MASK				(3 << 3)

#define TI814X_UART4_SHIFT				5
#define TI814X_UART4_MASK				(3 << 5)

#define TI814X_UART5_SHIFT				7
#define TI814X_UART5_MASK				(3 << 7)

/* Select HDMI_I2S_CLKSRC */
#define TI814X_HDMI_I2S_CLKSRC_SHIFT			0
#define TI814X_HDMI_I2S_CLKSRC_MASK			(7 << 0)

/* GPIO DB CLK EN Bits */
#define TI816X_GPIO_0_DBCLK_SHIFT			8
#define TI816X_GPIO_1_DBCLK_SHIFT			8

/* Select DMTIMER_CLKSRC */
#define TI814X_DMTIMER1_CLKS_SHIFT			3
#define TI814X_DMTIMER1_CLKS_MASK			(7 << 3)

#define TI814X_DMTIMER2_CLKS_SHIFT			6
#define TI814X_DMTIMER2_CLKS_MASK			(7 << 6)

#define TI814X_DMTIMER3_CLKS_SHIFT			9
#define TI814X_DMTIMER3_CLKS_MASK			(7 << 9)

#define TI814X_DMTIMER4_CLKS_SHIFT			16
#define TI814X_DMTIMER4_CLKS_MASK			(7 << 16)

#define TI814X_DMTIMER5_CLKS_SHIFT			19
#define TI814X_DMTIMER5_CLKS_MASK			(7 << 19)

#define TI814X_DMTIMER6_CLKS_SHIFT			22
#define TI814X_DMTIMER6_CLKS_MASK			(7 << 22)

#define TI814X_DMTIMER7_CLKS_SHIFT			25
#define TI814X_DMTIMER7_CLKS_MASK			(7 << 25)

#define TI814X_DMTIMER8_CLKS_SHIFT			0
#define TI814X_DMTIMER8_CLKS_MASK			(7 << 0)

/* Select CLKOUT_MUX source */
#define TI814X_CLKOUT0_MUX_SHIFT			0
#define TI814X_CLKOUT0_MUX_MASK				(0xF << 0)

#define TI814X_CLKOUT1_MUX_SHIFT			16
#define TI814X_CLKOUT1_MUX_MASK				(0xF << 16)

/* Select RMII_REFCLK_SRC */
#define TI814X_RMII_REFCLK_SHIFT			0
#define TI814X_RMII_REFCLK_MASK				(1 << 0)

#define TI814X_CPTS_RFT_SHIFT				1
#define TI814X_CPTS_RFT_MASK				(7 << 1)

/* Select SECSS_CLK_SRC */
#define TI814X_SECSS_CLKS_SHIFT				0
#define TI814X_SECSS_CLKS_MASK				(1 << 0)

/* Select SYSCLK18_CLKSRC */
#define TI814X_SYSCLK18_SHIFT				0
#define TI814X_SYSCLK18_MASK				(1 << 0)

/* Select WDT0_CLKSRC */
#define TI814X_WDT0_SHIFT				0
#define TI814X_WDT0_MASK				(1 << 0)

/* Select EMIF_CLK_GATE */
#define TI814X_EMIFSS_CLK_GATE_SHIFT			0
#define TI814X_EMIFSS_CLK_GATE_ENABLE			(0 << 1)
#define TI814X_EMIFSS_CLK_GATE_DISABLE			(1 << 1)

#define TI814X_DDRPHY1_CLK_GATE_SHIFT			1
#define TI814X_DDRPHY1_CLK_GATE_ENABLE			(0 << 1)
#define TI814X_DDRPHY1_CLK_GATE_DISABLE			(1 << 1)

/* Select DMTIMER_CLK_CHANGE */
#define TI814X_DMTIMER8_IDLEREQ_SHIFT			0
#define TI814X_DMTIMER8_IDLEREQ_MASK			(1 << 0)

#define TI814X_DMTIMER1_IDLEREQ_SHIFT			1
#define TI814X_DMTIMER1_IDLEREQ_MASK			(1 << 1)

#define TI814X_DMTIMER2_IDLEREQ_SHIFT			2
#define TI814X_DMTIMER2_IDLEREQ_MASK			(1 << 2)

#define TI814X_DMTIMER3_IDLEREQ_SHIFT			3
#define TI814X_DMTIMER3_IDLEREQ_MASK			(1 << 3)

#define TI814X_DMTIMER4_IDLEREQ_SHIFT			4
#define TI814X_DMTIMER4_IDLEREQ_MASK			(1 << 4)

#define TI814X_DMTIMER5_IDLEREQ_SHIFT			5
#define TI814X_DMTIMER5_IDLEREQ_MASK			(1 << 5)

#define TI814X_DMTIMER6_IDLEREQ_SHIFT			6
#define TI814X_DMTIMER6_IDLEREQ_MASK			(1 << 6)

#define TI814X_DMTIMER7_IDLEREQ_SHIFT			7
#define TI814X_DMTIMER7_IDLEREQ_MASK			(1 << 7)

#define TI814X_DMTIMER8_IDLESTATUS_SHIFT		8
#define TI814X_DMTIMER8_IDLESTATUS_MASK			(1 << 8)

#define TI814X_DMTIMER1_IDLESTATUS_SHIFT		9
#define TI814X_DMTIMER1_IDLESTATUS_MASK			(1 << 9)

#define TI814X_DMTIMER2_IDLESTATUS_SHIFT		10
#define TI814X_DMTIMER2_IDLESTATUS_MASK			(1 << 10)

#define TI814X_DMTIMER3_IDLESTATUS_SHIFT		11
#define TI814X_DMTIMER3_IDLESTATUS_MASK			(1 << 11)

#define TI814X_DMTIMER4_IDLESTATUS_SHIFT		12
#define TI814X_DMTIMER4_IDLESTATUS_MASK			(1 << 12)

#define TI814X_DMTIMER5_IDLESTATUS_SHIFT		13
#define TI814X_DMTIMER5_IDLESTATUS_MASK			(1 << 13)

#define TI814X_DMTIMER6_IDLESTATUS_SHIFT		14
#define TI814X_DMTIMER6_IDLESTATUS_MASK			(1 << 14)

#define TI814X_DMTIMER7_IDLESTATUS_SHIFT		15
#define TI814X_DMTIMER7_IDLESTATUS_MASK			(1 << 15)

/* Select DEEPSLEEP_CTRL */
#define TI814X_DEEPSLEEP_CTRL_DSCOUNT_SHIFT		0
#define TI814X_DEEPSLEEP_CTRL_DSCOUNT_MASK		(0xFFFF << 0)

#define TI814X_DEEPSLEEP_CTRL_DSPOLARITY_SHIFT		16
#define TI814X_DEEPSLEEP_CTRL_DSPOLARITY_MASK		(1 << 16)

#define TI814X_DEEPSLEEP_CTRL_DSENABLE_SHIFT		17
#define TI814X_DEEPSLEEP_CTRL_DSENABLE_MASK		(1 << 17)

/* Select DEEPSLEEP_STATUS */
#define TI814X_DEEPSLEEP_STATUS_DSCOMPLETE_SHIFT	0
#define TI814X_DEEPSLEEP_STATUS_DSCOMPLETE_MASK		(1 << 0)

/* PLL register field bits */
#define TI816X_PLL_NVAL_SHIFT				16
#define TI816X_PLL_NVAL_MASK				(0xFFFF << 16)

#define TI816X_PLL_PVAL_SHIFT				8
#define TI816X_PLL_PVAL_MASK				(0xFF << 8)

#define TI816X_PLL_LOCK_STS_SHIFT			7
#define TI816X_PLL_LOCK_STS_MASK			(1 << 7)

#define TI816X_PLL_ENABLE_SHIFT				3
#define TI816X_PLL_ENABLE_MASK				(1 << 3)

#define TI816X_PLL_BYPASS_SHIFT				2
#define TI816X_PLL_BYPASS_MASK				(1 << 2)

#define TI816X_PLL_LOCK_OUT_SEL_SHIFT			0
#define TI816X_PLL_LOCK_OUT_SEL_MASK			(1 << 0)

#define TI816X_PLL_FRACFREQ_SHIFT			0
#define TI816X_PLL_FRACFREQ_MASK			(0xFFFFFF << 0)

#define TI816X_PLL_INTFREQ_SHIFT			24
#define TI816X_PLL_INTFREQ_MASK				(0xF << 24)

#define TI816X_PLL_TRUNC_SHIFT				28
#define TI816X_PLL_TRUNC_MASK				(1 << 28)

#define TI816X_PLL_LDFREQ_SHIFT				31
#define TI816X_PLL_LDFREQ_MASK				(1 << 31)

#define TI816X_PLL_MDIV_SHIFT				0
#define TI816X_PLL_MDIV_MASK				(0xFF << 0)

#define TI816X_PLL_LDMDIV_SHIFT				8
#define TI816X_PLL_LDMDIV_MASK				(1 << 8)

#define TI816X_FAPLL_PWD_SHIFT				0x1

#define TI816X_PLL_MAX_MULT				0x1FF
#define TI816X_PLL_MAX_DIV				0xFF
#define TI816X_PLL_MIN_DIV				1
#define TI816X_PLL_RATE_TOLARANCE			10000000

/* Flag to specify FREQ, M values are contributing for
 * synthesizer frequency change
 */
#define TI816X_SYN_FREQ_VALUE_PRESENT			0x1

#define TI816X_SYNTHESIZER_ID1				1
#define TI816X_SYNTHESIZER_ID2				2
#define TI816X_SYNTHESIZER_ID3				3
#define TI816X_SYNTHESIZER_ID4				4
#define TI816X_SYNTHESIZER_ID5				5
#define TI816X_SYNTHESIZER_ID6				6
#define TI816X_SYNTHESIZER_ID7				7

/* Main PLL */
#define TI816X_MAINPLL_ID				1
#define TI816X_MAINPLL_NUM_SYN				7
#define TI816X_MAINPLL_BYPASS_EN			1
#define TI816X_MAINPLL_N				64
#define TI816X_MAINPLL_P				0x1
#define TI816X_MAINPLL_INTFREQ1				0x8
#define TI816X_MAINPLL_FRACFREQ1			0x800000
#define TI816X_MAINPLL_MDIV1				0x2
#define TI816X_MAINPLL_INTFREQ2				0xE
#define TI816X_MAINPLL_FRACFREQ2			0x0
#define TI816X_MAINPLL_MDIV2				0x1
#define TI816X_MAINPLL_INTFREQ3				0x8
#define TI816X_MAINPLL_FRACFREQ3			0xAAAAB0
#define TI816X_MAINPLL_MDIV3				0x3
#define TI816X_MAINPLL_INTFREQ4				0x9
#define TI816X_MAINPLL_FRACFREQ4			0x55554F
#define TI816X_MAINPLL_MDIV4				0x3
#define TI816X_MAINPLL_INTFREQ5				0x9
#define TI816X_MAINPLL_FRACFREQ5			0x374BC6
#define TI816X_MAINPLL_MDIV5				0xC
#define TI816X_MAINPLL_INTFREQ6				0
#define TI816X_MAINPLL_FRACFREQ6			0
#define TI816X_MAINPLL_MDIV6				0x48
#define TI816X_MAINPLL_INTFREQ7				0
#define TI816X_MAINPLL_FRACFREQ7			0
#define TI816X_MAINPLL_MDIV7				0x4

#define TI816X_MAINPLL_PWD_CLK1_SHIFT			1
#define TI816X_MAINPLL_PWD_CLK1_MASK			(1 << 1)

#define TI816X_MAINPLL_PWD_CLK2_SHIFT			2
#define TI816X_MAINPLL_PWD_CLK2_MASK			(1 << 2)

#define TI816X_MAINPLL_PWD_CLK3_SHIFT			3
#define TI816X_MAINPLL_PWD_CLK3_MASK			(1 << 3)

#define TI816X_MAINPLL_PWD_CLK4_SHIFT			4
#define TI816X_MAINPLL_PWD_CLK4_MASK			(1 << 4)

#define TI816X_MAINPLL_PWD_CLK5_SHIFT			5
#define TI816X_MAINPLL_PWD_CLK5_MASK			(1 << 5)

#define TI816X_MAINPLL_PWD_CLK6_SHIFT			6
#define TI816X_MAINPLL_PWD_CLK6_MASK			(1 << 6)

#define TI816X_MAINPLL_PWD_CLK7_SHIFT			7
#define TI816X_MAINPLL_PWD_CLK7_MASK			(1 << 7)

/* DDR PLL */
#define TI816X_DDRPLL_ID				2
#define TI816X_DDRPLL_BYPASS_EN				0
/* For 400 MHz */
#define TI816X_DDRPLL_400_N				59
#define TI816X_DDRPLL_400_P				0x1
#define TI816X_DDRPLL_400_INTFREQ1			0
#define TI816X_DDRPLL_400_FRACFREQ1			0
#define TI816X_DDRPLL_400_MDIV1				0x4
#define TI816X_DDRPLL_400_INTFREQ2			0x8
#define TI816X_DDRPLL_400_FRACFREQ2			0xD99999
#define TI816X_DDRPLL_400_MDIV2				0x1E
#define TI816X_DDRPLL_400_INTFREQ3			0x8
#define TI816X_DDRPLL_400_FRACFREQ3			0x0
#define TI816X_DDRPLL_400_MDIV3				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_400_INTFREQ4			0xE
#define TI816X_DDRPLL_400_FRACFREQ4			0x0
#define TI816X_DDRPLL_400_MDIV4				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_400_INTFREQ5			0xE
#define TI816X_DDRPLL_400_FRACFREQ5			0x0
#define TI816X_DDRPLL_400_MDIV5				0x4

/* For 531 MHz */
#define TI816X_DDRPLL_531_N				59
#define TI816X_DDRPLL_531_P				0x1
#define TI816X_DDRPLL_531_INTFREQ1			0
#define TI816X_DDRPLL_531_FRACFREQ1			0
#define TI816X_DDRPLL_531_MDIV1				0x3
#define TI816X_DDRPLL_531_INTFREQ2			0x8
#define TI816X_DDRPLL_531_FRACFREQ2			0xD99999
#define TI816X_DDRPLL_531_MDIV2				0x1E
#define TI816X_DDRPLL_531_INTFREQ3			0x8
#define TI816X_DDRPLL_531_FRACFREQ3			0x0
#define TI816X_DDRPLL_531_MDIV3				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_531_INTFREQ4			0xE
#define TI816X_DDRPLL_531_FRACFREQ4			0x0
#define TI816X_DDRPLL_531_MDIV4				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_531_INTFREQ5			0xE
#define TI816X_DDRPLL_531_FRACFREQ5			0x0
#define TI816X_DDRPLL_531_MDIV5				0x4

/* For 675 MHz */
#define TI816X_DDRPLL_675_N				50
#define TI816X_DDRPLL_675_P				0x1
#define TI816X_DDRPLL_675_INTFREQ1			0
#define TI816X_DDRPLL_675_FRACFREQ1			0
#define TI816X_DDRPLL_675_MDIV1				0x2
#define TI816X_DDRPLL_675_INTFREQ2			0x7
#define TI816X_DDRPLL_675_FRACFREQ2			0x800000
#define TI816X_DDRPLL_675_MDIV2				0x1E
#define TI816X_DDRPLL_675_INTFREQ3			0x6
#define TI816X_DDRPLL_675_FRACFREQ3			0xC00000
#define TI816X_DDRPLL_675_MDIV3				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_675_INTFREQ4			0xE
#define TI816X_DDRPLL_675_FRACFREQ4			0x0
#define TI816X_DDRPLL_675_MDIV4				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_675_INTFREQ5			0xE
#define TI816X_DDRPLL_675_FRACFREQ5			0x0
#define TI816X_DDRPLL_675_MDIV5				0x4

/* For 796 MHz */
#define TI816X_DDRPLL_796_N				59
#define TI816X_DDRPLL_796_P				0x1
#define TI816X_DDRPLL_796_INTFREQ1			0
#define TI816X_DDRPLL_796_FRACFREQ1			0
#define TI816X_DDRPLL_796_MDIV1				0x2
#define TI816X_DDRPLL_796_INTFREQ2			0x8
#define TI816X_DDRPLL_796_FRACFREQ2			0xD99999
#define TI816X_DDRPLL_796_MDIV2				0x1E
#define TI816X_DDRPLL_796_INTFREQ3			0x8
#define TI816X_DDRPLL_796_FRACFREQ3			0x0
#define TI816X_DDRPLL_796_MDIV3				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_796_INTFREQ4			0xE
#define TI816X_DDRPLL_796_FRACFREQ4			0x0
#define TI816X_DDRPLL_796_MDIV4				0x4
/* Expansion DDR clk */
#define TI816X_DDRPLL_796_INTFREQ5			0xE
#define TI816X_DDRPLL_796_FRACFREQ5			0x0
#define TI816X_DDRPLL_796_MDIV5				0x4

#define TI816X_DDRPLL_PWD_CLK1_SHIFT			1
#define TI816X_DDRPLL_PWD_CLK1_MASK			(1 << 1)

#define TI816X_DDRPLL_PWD_CLK2_SHIFT			2
#define TI816X_DDRPLL_PWD_CLK2_MASK			(1 << 2)

#define TI816X_DDRPLL_PWD_CLK3_SHIFT			3
#define TI816X_DDRPLL_PWD_CLK3_MASK			(1 << 3)

#define TI816X_DDRPLL_PWD_CLK4_SHIFT			4
#define TI816X_DDRPLL_PWD_CLK4_MASK			(1 << 4)

#define TI816X_DDRPLL_PWD_CLK5_SHIFT			5
#define TI816X_DDRPLL_PWD_CLK5_MASK			(1 << 5)

/* Video PLL */
#define TI816X_VIDEOPLL_ID				3
#define TI816X_VIDEOPLL_BYPASS_EN			1
#define TI816X_VIDEOPLL_N				110
#define TI816X_VIDEOPLL_P				0x2
#define TI816X_VIDEOPLL_INTFREQ1			0xB
#define TI816X_VIDEOPLL_FRACFREQ1			0x0
#define TI816X_VIDEOPLL_MDIV1				0x5
#define TI816X_VIDEOPLL_INTFREQ2			0xA
#define TI816X_VIDEOPLL_FRACFREQ2			0x0
#define TI816X_VIDEOPLL_MDIV2				0x2
#define TI816X_VIDEOPLL_INTFREQ3			0xA
#define TI816X_VIDEOPLL_FRACFREQ3			0x0
#define TI816X_VIDEOPLL_MDIV3				0x2

#define TI816X_VIDEOPLL_PWD_CLK1_SHIFT			1
#define TI816X_VIDEOPLL_PWD_CLK1_MASK			(1 << 1)

#define TI816X_VIDEOPLL_PWD_CLK2_SHIFT			2
#define TI816X_VIDEOPLL_PWD_CLK2_MASK			(1 << 2)

#define TI816X_VIDEOPLL_PWD_CLK3_SHIFT			3
#define TI816X_VIDEOPLL_PWD_CLK3_MASK			(1 << 3)

/* Audio PLL */
#define TI816X_AUDIOPLL_ID				4
#define TI816X_AUDIOPLL_BYPASS_EN			1
#define TI816X_AUDIOPLL_N				64
#define TI816X_AUDIOPLL_P				0x19
#define TI816X_AUDIOPLL_INTFREQ2			0xE
#define TI816X_AUDIOPLL_FRACFREQ2			0x0
#define TI816X_AUDIOPLL_MDIV2				0x4
#define TI816X_AUDIOPLL_INTFREQ3			0x9
#define TI816X_AUDIOPLL_FRACFREQ3			0x0
#define TI816X_AUDIOPLL_MDIV3				0x5
#define TI816X_AUDIOPLL_INTFREQ4			0x9
#define TI816X_AUDIOPLL_FRACFREQ4			0xCBC148
#define TI816X_AUDIOPLL_MDIV4				0x14
#define TI816X_AUDIOPLL_INTFREQ5			0xD
#define TI816X_AUDIOPLL_FRACFREQ5			0x800000
#define TI816X_AUDIOPLL_MDIV5				0x14

#define TI816X_AUDIOPLL_PWD_CLK2_SHIFT			2
#define TI816X_AUDIOPLL_PWD_CLK2_MASK			(1 << 2)

#define TI816X_AUDIOPLL_PWD_CLK3_SHIFT			3
#define TI816X_AUDIOPLL_PWD_CLK3_MASK			(1 << 3)

#define TI816X_AUDIOPLL_PWD_CLK4_SHIFT			4
#define TI816X_AUDIOPLL_PWD_CLK4_MASK			(1 << 4)

#define TI816X_AUDIOPLL_PWD_CLK5_SHIFT			5
#define TI816X_AUDIOPLL_PWD_CLK5_MASK			(1 << 5)

/* ADPLLL */

/* <ADPLL>_MN2DIV */
#define TI814X_ADPLL_M_MULT_SHIFT			0
#define TI814X_ADPLL_M_MULT_MASK			(0xfff << 0)
#define TI814X_ADPLL_N2_DIV_SHIFT			16
#define TI814X_ADPLL_N2_DIV_MASK			(0xf << 16)

/* <ADPLL>_M2NDIV */
#define TI814X_ADPLL_N_DIV_SHIFT			0
#define TI814X_ADPLL_N_DIV_MASK				(0x7f << 0)
#define TI814X_ADPLL_M2_DIV_SHIFT			16
#define TI814X_ADPLL_M2_DIV_MASK			(0x1f << 16)

/* <ADPLL>_FRACDIV */
#define TI814X_ADPLL_FRACT_MULT_SHIFT			0
#define TI814X_ADPLL_FRACT_MULT_MASK			(0x3ffff << 0)
#define TI814X_ADPLLJ_SDDIV_SHIFT			24
#define TI814X_ADPLLJ_SDDIV_MASK			(0xff << 24)

/* <ADPLL>_CLKCTRL */
#define TI814X_ADPLL_TINITZ_SHIFT			0
#define TI814X_ADPLL_BYPASS_SEL_SHIFT			18
#define TI814X_ADPLL_BYPASS_SEL_MASK			(0x1 << 18)
#define TI814X_EN_ADPLL_CLKOUT_SHIFT			20
#define TI814X_EN_ADPLL_CLKOUT_MASK			(0x1 << 20)
#define TI814X_EN_ADPLL_STBYRET_SHIFT			21
#define TI814X_EN_ADPLL_STBYRET_MASK			(0x1 << 21)
#define TI814X_EN_ADPLL_BYPASS_SHIFT			23
#define TI814X_EN_ADPLL_BYPASS_MASK			(0x1 << 23)
#define TI814X_ADPLL_CLKOUTLDOEN_SHIFT			19
#define TI814X_ADPLL_CLKDCOLDOEN_SHIFT			29

/* <ADPLL>_STATUS */
#define TI814X_ST_PHASELOCK_SHIFT			10
#define TI814X_ST_PHASELOCK_MASK			(0x1 << 10)
#define TI814X_ST_FREQLOCK_SHIFT			9
#define TI814X_ST_FREQLOCK_MASK				(0x1 << 9)
#define TI814X_ST_BYPASS_SHIFT				0
#define TI814X_ST_BYPASS_MASK				(0x1 << 0)
#define TI814X_ST_ADPLL_MASK			(TI814X_ST_PHASELOCK_MASK |\
						TI814X_ST_FREQLOCK_MASK |\
						TI814X_ST_BYPASS_MASK)
#define TI814X_ADPLL_STBYRET_ACK_SHIFT			7
#define TI814X_ADPLL_STBYRET_ACK_MASK			(0x1 << 7)

/* MODENAPLL_CLKCTRL */
#define TI814X_EN_MODENA_ADPLL_DRIFTGUARD_SHIFT		11
#define TI814X_EN_MODENA_ADPLL_DRIFTGUARD_MASK		(0x1 << 11)
#define TI814X_EN_MODENA_ADPLL_STOPMODE_SHIFT		14
#define TI814X_EN_MODENA_ADPLL_STOPMODE_MASK		(0x1 << 14)

/* ADPLLLJ_CLKCTRL */
#define TI814X_ADPLLJ_FREQ_SEL_SHIFT			10
#define TI814X_ADPLLJ_FREQ_SEL_MASK			(0x7 << 10)

/* TENABLE and TENABLEDIV bitfields */
#define TI814X_ADPLL_LOAD_MN_SHIFT			0
#define TI814X_ADPLL_LOAD_M2N2_SHIFT			0

#endif
