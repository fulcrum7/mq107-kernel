/*
 * TI81XX CM register access macros. Also contains CM module offsets.
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

#ifndef __ARCH_ARM_MACH_OMAP2_CM81XX_H
#define __ARCH_ARM_MACH_OMAP2_CM81XX_H

#include "prcm-common.h"

#define TI81XX_CM_REGADDR(module, reg)					\
	TI81XX_L4_SLOW_IO_ADDRESS(TI81XX_PRCM_BASE + (module) + (reg))
#define TI814X_PLL_REGADDR(reg)						\
	TI81XX_L4_SLOW_IO_ADDRESS(TI814X_PLL_BASE + (reg))
#define TI814X_ADPLL_REGADDR(adpll_base, reg)				\
	TI81XX_L4_SLOW_IO_ADDRESS(TI814X_PLL_BASE + (adpll_base) + (reg))

#if defined(CONFIG_ARCH_TI814X)
/* ARM frequency */
#define ARM_FREQ_OPP_100	600000000 /* Hz */
#define ARM_FREQ_OPP_50		300000000 /* Hz */
#endif

/*
 * TI81XX common CM module offsets
 */

#define TI81XX_CM_DEVICE_MOD			0x0100	/* 256B */
#define TI81XX_CM_DPLL_MOD			0x0300	/* 256B */
#define TI81XX_CM_ALWON_MOD			0x1400	/* 1KB */

/*
 * TI816X CM module offsets
 */

#define TI816X_CM_ACTIVE_MOD			0x0400	/* 256B */
#define TI816X_CM_DEFAULT_MOD			0x0500	/* 256B */
#define TI816X_CM_IVAHD0_MOD			0x0600	/* 256B */
#define TI816X_CM_IVAHD1_MOD			0x0700	/* 256B */
#define TI816X_CM_IVAHD2_MOD			0x0800	/* 256B */
#define TI816X_CM_SGX_MOD			0x0900	/* 256B */

/*
 * TI814X CM module offsets
 */

#define TI814X_CM_DSP_MOD			0x0400  /* 256B */
#define TI814X_CM_ALWON2_MOD			0x0500  /* 256B */
#define TI814X_CM_HDVICP_MOD			0x0600  /* 256B */
#define TI814X_CM_ISP_MOD			0x0700  /* 256B */
#define TI814X_CM_HDVPSS_MOD			0x0800  /* 256B */
#define TI814X_CM_GFX_MOD			0x0900  /* 256B */

/*
 * Clock domain register offsets - these are generally CLKSTCTRL registers for
 * respective modules.
 */

/* ALWON */
#define TI81XX_CM_ALWON_MPU_CLKDM		0x001C
#define TI81XX_CM_ALWON_L3_SLOW_CLKDM		0x0000
#define TI81XX_CM_ETHERNET_CLKDM		0x0004
#define TI81XX_CM_MMU_CLKDM			0x000C
#define TI81XX_CM_MMUCFG_CLKDM			0x0010

/* ACTIVE */
#define TI816X_CM_ACTIVE_GEM_CLKDM		0x0000

/* IVAHD0 */
#define TI816X_CM_IVAHD0_CLKDM			0x0000

/* IVAHD1 */
#define TI816X_CM_IVAHD1_CLKDM			0x0000

/* IVAHD2 */
#define TI816X_CM_IVAHD2_CLKDM			0x0000

/* SGX */
#define TI816X_CM_SGX_CLKDM			0x0000

/* DEFAULT */
#define TI816X_CM_DEFAULT_L3_MED_CLKDM		0x0004
#define TI816X_CM_DEFAULT_DUCATI_CLKDM		0x0018
#define TI816X_CM_DEFAULT_PCI_CLKDM		0x0010
#define TI816X_CM_DEFAULT_L3_SLOW_CLKDM		0x0014

/* DSP */
#define TI814X_CM_DSP_CLKDM			0x0000

/* HDVICP */
#define TI814X_CM_HDVICP_CLKDM			0x0000

/* ALWYON2 */
#define TI814X_CM_ALWON2_MC_CLKDM		0x0018
#define TI814X_CM_ALWON2_L3_MED_CLKDM		0x0004
#define TI814X_CM_ALWON2_PCI_CLKDM		0x0010
#define TI814X_CM_ALWON2_L3_SLOW_CLKDM		0x0014

/* GFX */
#define TI814X_CM_GFX_CLKDM			0x0000

/* HDVPSS */
#define TI814X_CM_HDVPSS_CLKDM			0x0000

/* ADPLLJ control and config register offsets */
#define ADPLLJ_CLKCTRL				0x4
#define ADPLLJ_TENABLE				0x8
#define ADPLLJ_TENDIV				0xC
#define ADPLLJ_M2NDIV				0x10
#define ADPLLJ_MN2DIV				0x14
#define ADPLLJ_FRACDIV				0x18
#define ADPLLJ_STATUS				0x24
/* PLL subsystem module offsets */
#define MODENA_PLL_BASE			(0x048)
#define DSP_PLL_BASE				(0x080)
#define SGX_PLL_BASE				(0x0B0)
#define IVA_PLL_BASE				(0x0E0)
#define L3_PLL_BASE				(0x110)
#define ISS_PLL_BASE				(0x140)
#define DSS_PLL_BASE				(0x170)
#define VIDEO0_PLL_BASE			(0x1A0)
#define VIDEO1_PLL_BASE			(0x1D0)
#define HDMI_PLL_BASE				(0x200)
#define AUDIO_PLL_BASE				(0x230)
#define USB_PLL_BASE				(0x260)
#define DDR_PLL_BASE				(0x290)

/*
 * CM register addresses
 */

/* CM_DPLL */
/* TI81XX specific definitions */
#define TI81XX_CM_DPLL_SYSCLK3_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0008)
#define TI81XX_CM_DPLL_SYSCLK6_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0014)
#define TI81XX_CM_DPLL_SYSCLK10_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0024)
#define TI81XX_CM_DPLL_SYSCLK19_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x004C)
#define TI81XX_CM_DPLL_SYSCLK20_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0050)
#define TI81XX_CM_DPLL_SYSCLK21_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0054)
#define TI81XX_CM_DPLL_SYSCLK22_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0058)
#define TI81XX_CM_DPLL_SYSCLK14_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0070)
#define TI81XX_CM_DPLL_SYSCLK16_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0074)
#define TI81XX_CM_DPLL_SYSCLK18_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0078)
#define TI81XX_CM_DPLL_AUDIOCLK_MCASP0_CLKSEL	TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x007C)
#define TI81XX_CM_DPLL_AUDIOCLK_MCASP1_CLKSEL	TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0080)
#define TI81XX_CM_DPLL_AUDIOCLK_MCASP2_CLKSEL	TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0084)
#define TI81XX_CM_DPLL_AUDIOCLK_MCBSP_CLKSEL	TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0088)
#define TI81XX_CM_DPLL_HDMI_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00AC)
#define TI81XX_CM_DPLL_SYSCLK23_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00B0)

/* TI814X specific definitions */
#define TI814X_CM_DPLL_PV2B3_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0040)
#define TI814X_CM_DPLL_PV1C1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0044)
#define TI814X_CM_DPLL_PV0D1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0048)
#define TI814X_CM_DPLL_RTCDIVA_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x005C)

/* TI816X specific definitions */
#define TI816X_CM_DPLL_SYSCLK1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0000)
#define TI816X_CM_DPLL_SYSCLK2_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0004)
#define TI816X_CM_DPLL_SYSCLK4_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x000C)
#define TI816X_CM_DPLL_SYSCLK5_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0010)
#define TI816X_CM_DPLL_SYSCLK7_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0018)
#define TI816X_CM_DPLL_SYSCLK11_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x002C)
#define TI816X_CM_DPLL_SYSCLK12_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0030)
#define TI816X_CM_DPLL_SYSCLK13_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0034)
#define TI816X_CM_DPLL_SYSCLK15_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0038)
#define TI816X_CM_DPLL_VPB3_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0040)
#define TI816X_CM_DPLL_VPC1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0044)
#define TI816X_CM_DPLL_VPD1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0048)
#define TI816X_CM_DPLL_SYSCLK22_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0058)
#define TI816X_CM_DPLL_APA_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x005C)
#define TI816X_CM_DPLL_TIMER1_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0090)
#define TI816X_CM_DPLL_TIMER2_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0094)
#define TI816X_CM_DPLL_TIMER3_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x0098)
#define TI816X_CM_DPLL_TIMER4_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x009C)
#define TI816X_CM_DPLL_TIMER5_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00A0)
#define TI816X_CM_DPLL_TIMER6_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00A4)
#define TI816X_CM_DPLL_TIMER7_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00A8)
#define TI816X_CM_DPLL_SYSCLK24_CLKSEL		TI81XX_CM_REGADDR(TI81XX_CM_DPLL_MOD, 0x00B4)

/* CM_DEVICE */
#define TI814X_CM_CLKOUT_CTRL			TI81XX_CM_REGADDR(TI81XX_CM_DEVICE_MOD, 0x0000)

/* CM_DSP */
#define TI814X_CM_DSP_CLKSTCTRL			TI81XX_CM_REGADDR(TI814X_CM_DSP_MOD, 0x0000)
#define TI814X_CM_DSP_CLKCTRL			TI81XX_CM_REGADDR(TI814X_CM_DSP_MOD, 0x0020)

/* CM_ACTIVE */
#define TI816X_CM_ACTIVE_GEM_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0000)
#define TI816X_CM_ACTIVE_HDDSS_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0004)
#define TI816X_CM_ACTIVE_HDMI_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0008)
#define TI816X_CM_ACTIVE_GEM_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0020)
#define TI816X_CM_ACTIVE_HDDSS_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0024)
#define TI816X_CM_ACTIVE_HDMI_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_ACTIVE_MOD, 0x0028)

/* CM_ALWON2 */
#define TI814X_CM_ALWON2_L3_MED_CLKSTCTRL	TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0004)
#define TI814X_CM_ALWON2_TPPSS_CLKSTCTRL	TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x000C)
#define TI814X_CM_ALWON2_PCI_CLKSTCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0010)
#define TI814X_CM_ALWON2_L3_SLOW_CLKSTCTRL	TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0014)
#define TI814X_CM_ALWON2_MC_CLKSTCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0018)
#define TI814X_CM_ALWON2_DMM_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0028)
#define TI814X_CM_ALWON2_FW_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x002C)
#define TI814X_CM_ALWON2_TPPSS_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0054)
#define TI814X_CM_ALWON2_USB_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0058)
#define TI814X_CM_ALWON2_SATA_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0060)
#define TI814X_CM_ALWON2_MC_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0074)
#define TI814X_CM_ALWON2_PCI_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ALWON2_MOD, 0x0078)

/* CM_DEFAULT */
#define TI816X_CM_DEFAULT_L3_MED_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0004)
#define TI816X_CM_DEFAULT_L3_FAST_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0008)
#define TI816X_CM_DEFAULT_TPPSS_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x000C)
#define TI816X_CM_DEFAULT_PCI_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0010)
#define TI816X_CM_DEFAULT_L3_SLOW_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0014)
#define TI816X_CM_DEFAULT_DUCATI_CLKSTCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0018)
#define TI816X_CM_DEFAULT_EMIF_0_CLKCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0020)
#define TI816X_CM_DEFAULT_EMIF_1_CLKCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0024)
#define TI816X_CM_DEFAULT_DMM_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0028)
#define TI816X_CM_DEFAULT_FW_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x002C)
#define TI816X_CM_DEFAULT_TPPSS_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0054)
#define TI816X_CM_DEFAULT_USB_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0058)
#define TI816X_CM_DEFAULT_SATA_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0060)
#define TI816X_CM_DEFAULT_DUCATI_CLKCTRL	TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0074)
#define TI816X_CM_DEFAULT_PCI_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_DEFAULT_MOD, 0x0078)

/* CM_HDVICP */
#define TI814X_CM_HDVICP_CLKSTCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVICP_MOD, 0x0000)
#define TI814X_CM_HDVICP_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVICP_MOD, 0x0020)
#define TI814X_CM_HDVICP_SL2_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVICP_MOD, 0x0024)

/* CM_ISP */
#define TI814X_CM_ISP_CLKSTCTRL			TI81XX_CM_REGADDR(TI814X_CM_ISP_MOD, 0x0000)
#define TI814X_CM_ISP_ISP_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ISP_MOD, 0x0020)
#define TI814X_CM_ISP_FDIF_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_ISP_MOD, 0x0024)

/* CM_HDVPSS */
#define TI814X_CM_HDVPSS_CLKSTCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVPSS_MOD, 0x0000)
#define TI814X_CM_HDVPSS_HDVPSS_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVPSS_MOD, 0x0020)
#define TI814X_CM_HDVPSS_HDMI_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_HDVPSS_MOD, 0x0024)

/* CM_GFX */
#define TI814X_CM_GFX_CLKSTCTRL			TI81XX_CM_REGADDR(TI814X_CM_GFX_MOD, 0x0000)
#define TI814X_CM_GFX_GFX_CLKCTRL		TI81XX_CM_REGADDR(TI814X_CM_GFX_MOD, 0x0020)

/* CM_IVAHD0 */
#define TI816X_CM_IVAHD0_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD0_MOD, 0x0000)
#define TI816X_CM_IVAHD0_IVAHD_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD0_MOD, 0x0020)
#define TI816X_CM_IVAHD0_SL2_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD0_MOD, 0x0024)

/* CM_IVAHD1 */
#define TI816X_CM_IVAHD1_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD1_MOD, 0x0000)
#define TI816X_CM_IVAHD1_IVAHD_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD1_MOD, 0x0020)
#define TI816X_CM_IVAHD1_SL2_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD1_MOD, 0x0024)

/* CM_IVAHD2 */
#define TI816X_CM_IVAHD2_CLKSTCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD2_MOD, 0x0000)
#define TI816X_CM_IVAHD2_IVAHD_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD2_MOD, 0x0020)
#define TI816X_CM_IVAHD2_SL2_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_IVAHD2_MOD, 0x0024)

/* CM_SGX */
#define TI816X_CM_SGX_CLKSTCTRL			TI81XX_CM_REGADDR(TI816X_CM_SGX_MOD, 0x0000)
#define TI816X_CM_SGX_SGX_CLKCTRL		TI81XX_CM_REGADDR(TI816X_CM_SGX_MOD, 0x0020)

/* CM_ALWON */
#define TI81XX_CM_ALWON_L3_SLOW_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0000)
#define TI81XX_CM_ETHERNET_CLKSTCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0004)
#define TI81XX_CM_ALWON_L3_MED_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0008)
#define TI81XX_CM_MMU_CLKSTCTRL			TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x000C)
#define TI81XX_CM_MMUCFG_CLKSTCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0010)
#define TI81XX_CM_ALWON_OCMC_0_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0014)
#define TI81XX_CM_ALWON_MPU_CLKSTCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x001C)
#define TI81XX_CM_ALWON_SYSCLK4_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0020)
#define TI81XX_CM_ALWON_SYSCLK6_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0028)
#define TI81XX_CM_ALWON_RTC_CLKSTCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x002C)
#define TI81XX_CM_ALWON_L3_FAST_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0030)
#define TI81XX_CM_ALWON_MCASP0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0140)
#define TI81XX_CM_ALWON_MCASP1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0144)
#define TI81XX_CM_ALWON_MCASP2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0148)
#define TI81XX_CM_ALWON_MCBSP_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x014C)
#define TI81XX_CM_ALWON_UART_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0150)
#define TI81XX_CM_ALWON_UART_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0154)
#define TI81XX_CM_ALWON_UART_2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0158)
#define TI81XX_CM_ALWON_GPIO_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x015C)
#define TI81XX_CM_ALWON_GPIO_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0160)
#define TI81XX_CM_ALWON_WDTIMER_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x018C)
#define TI81XX_CM_ALWON_SPI_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0190)
#define TI81XX_CM_ALWON_MAILBOX_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0194)
#define TI81XX_CM_ALWON_SPINBOX_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0198)
#define TI81XX_CM_ALWON_MMUDATA_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x019C)
#define TI81XX_CM_ALWON_MMUCFG_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01A8)
#define TI81XX_CM_ALWON_SDIO_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01B0)
#define TI81XX_CM_ALWON_OCMC_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01B4)
#define TI81XX_CM_ALWON_SMARTCARD_0_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01BC)
#define TI81XX_CM_ALWON_SMARTCARD_1_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01C0)
#define TI81XX_CM_ALWON_CONTROL_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01C4)
#define TI81XX_CM_ALWON_SECSS_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01C8)
#define TI81XX_CM_ALWON_GPMC_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01D0)
#define TI81XX_CM_ALWON_ETHERNET_0_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01D4)
#define TI81XX_CM_ALWON_ETHERNET_1_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01D8)
#define TI81XX_CM_ALWON_MPU_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01DC)
#define TI81XX_CM_ALWON_DEBUGSS_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01E0)
#define TI81XX_CM_ALWON_L3_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01E4)
#define TI81XX_CM_ALWON_L4HS_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01E8)
#define TI81XX_CM_ALWON_L4LS_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01EC)
#define TI81XX_CM_ALWON_RTC_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01F0)
#define TI81XX_CM_ALWON_TPCC_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01F4)
#define TI81XX_CM_ALWON_TPTC0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01F8)
#define TI81XX_CM_ALWON_TPTC1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01FC)
#define TI81XX_CM_ALWON_TPTC2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0200)
#define TI81XX_CM_ALWON_TPTC3_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0204)
#define TI81XX_CM_ALWON_SR_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0208)
#define TI81XX_CM_ALWON_SR_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x020C)
#define TI81XX_CM_ALWON_SR_2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0210)
#define TI81XX_CM_ALWON_SR_3_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0214)

/* For TI814X */
#define TI814X_CM_ALWON_VCP_CLKSTCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0018)
#define TI814X_CM_ALWON_I2C_02_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0164)
#define TI814X_CM_ALWON_I2C_13_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0168)
#define TI814X_CM_ALWON_MCASP_3_4_5_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x016C)
#define TI814X_CM_ALWON_ATL_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0170)
#define TI814X_CM_ALWON_MLB_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0174)
#define TI814X_CM_ALWON_PATA_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0178)
#define TI814X_CM_ALWON_UART_3_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0180)
#define TI814X_CM_ALWON_UART_4_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0184)
#define TI814X_CM_ALWON_UART_5_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0188)
#define TI814X_CM_ALWON_VCP_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01B8)
#define TI814X_CM_ALWON_DCAN_0_1_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0218)
#define TI814X_CM_ALWON_MMCHS_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x021C)
#define TI814X_CM_ALWON_MMCHS_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0220)
#define TI814X_CM_ALWON_MMCHS_2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0224)

/* For TI816X */
#define TI816X_CM_ALWON_OCMC_1_CLKSTCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0018)
#define TI816X_CM_ALWON_I2C_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0164)
#define TI816X_CM_ALWON_I2C_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0168)
#define TI816X_CM_ALWON_TIMER_0_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x016C)
#define TI816X_CM_ALWON_TIMER_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0170)
#define TI816X_CM_ALWON_TIMER_2_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0174)
#define TI816X_CM_ALWON_TIMER_3_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0178)
#define TI816X_CM_ALWON_TIMER_4_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x017C)
#define TI816X_CM_ALWON_TIMER_5_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0180)
#define TI816X_CM_ALWON_TIMER_6_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0184)
#define TI816X_CM_ALWON_TIMER_7_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0188)
#define TI816X_CM_ALWON_VLYNQ_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01A0)
#define TI816X_CM_ALWON_OCMC_1_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x01B8)
#define TI816X_CM_ALWON_SR_4_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0218)
#define TI816X_CM_ALWON_SR_5_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x021C)
#define TI816X_CM_ALWON_SR_6_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0220)
#define TI816X_CM_ALWON_SR_7_CLKCTRL		TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0224)
#define TI816X_CM_ALWON_CUST_EFUSE_CLKCTRL	TI81XX_CM_REGADDR(TI81XX_CM_ALWON_MOD, 0x0228)

#define TI816X_CM_ALWON_MCBSP_FSX_EN		TI81XX_L4_SLOW_IO_ADDRESS(TI81XX_SCM_BASE + 0xA50)
#define TI816X_CM_ALWON_MCBSP_CLKS_EN		TI81XX_L4_SLOW_IO_ADDRESS(TI81XX_SCM_BASE + 0xA60)

/*
 * TI814X "PLL Clock Muxing and Gating Control" is located separately from CM
 * but for now we are keeping its register here.
 *
 * TODO: Move to appropriate location (new file?).
 */
#define TI814X_PLL_CMGC_OSC_SRC			TI814X_PLL_REGADDR(0x02C0)
#define TI814X_PLL_CMGC_ARM_CLKSRC		TI814X_PLL_REGADDR(0x02C4)
#define TI814X_PLL_CMGC_VIDEO_PLL_CLKSRC	TI814X_PLL_REGADDR(0x02C8)
#define TI814X_PLL_CMGC_MLB_ATL_CLKSRC		TI814X_PLL_REGADDR(0x02CC)
#define TI814X_PLL_CMGC_MCASP345_AUX_CLKSRC	TI814X_PLL_REGADDR(0x02D0)
#define TI814X_PLL_CMGC_MCASP_AHCLK_CLKSRC	TI814X_PLL_REGADDR(0x02D4)
#define TI814X_PLL_CMGC_MCBSP_UART_CLKSRC	TI814X_PLL_REGADDR(0x02D8)
#define TI814X_PLL_CMGC_HDMI_I2S_CLKSRC		TI814X_PLL_REGADDR(0x02DC)
#define TI814X_PLL_CMGC_DMTIMER_CLKSRC		TI814X_PLL_REGADDR(0x02E0)
#define TI814X_PLL_CMGC_CLKOUT_MUX		TI814X_PLL_REGADDR(0x02E4)
#define TI814X_PLL_CMGC_RMII_REF_CLKSRC		TI814X_PLL_REGADDR(0x02E8)
#define TI814X_PLL_CMGC_SECSS_CLKSRC		TI814X_PLL_REGADDR(0x02EC)
#define TI814X_PLL_CMGC_SYSCLK18_CLKSRC		TI814X_PLL_REGADDR(0x02F0)
#define TI814X_PLL_CMGC_WDT0_CLKSRC		TI814X_PLL_REGADDR(0x02F4)
#define TI814X_PLL_CMGC_EMIF_CLK_GATE		TI814X_PLL_REGADDR(0x030C)
#define TI814X_PLL_CMGC_TIMER_CLK_CHANGE	TI814X_PLL_REGADDR(0x0320)
#define TI814X_PLL_CMGC_DEEPSLEEP_CTRL		TI814X_PLL_REGADDR(0x0324)
#define TI814X_PLL_CMGC_DEEPSLEEP_STATUS	TI814X_PLL_REGADDR(0x0328)

#endif
