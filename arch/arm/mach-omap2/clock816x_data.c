/*
 * arch/arm/mach-omap2/clock816x_data.c
 *
 * Clock data for ti816x.
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
#include <linux/list.h>
#include <linux/clk.h>

#include <plat/clkdev_omap.h>

#include "control.h"
#include "clock.h"
#include "clock81xx.h"
#include "cm.h"
#include "cm81xx.h"
#include "cm-regbits-81xx.h"
#include "prm.h"

/*
 * Notes:
 *
 * - Various leaf modules clocks don't have separate interface and functional
 *   clock gating controls. Moreover, interface clock parents (SYSCLKs) do not
 *   have enable/disable controls. Thus, we can probably remove interface clocks
 *   and use dummy clocks instead. Only issue could be related to disabling
 *   SYSCLK parents (PLL). At present, the 'ick' are provided with clkops_null
 *   operations for enable/disable (since they are taken care by 'fck's).
 *
 * - Skipping PLL data and configuration for now. All the PLL (root) clocks are
 *   referred with their default rates.
 *
 * - These default rates are approximate rounded values which will be different
 *   from the actual output of parent clock from PLL (e.g, SYSCLK3 input is
 *   taken as 600MHz, while the actual Main PLL clk3 output would be around
 *   531.49 MHz). Of course, these rounded values are taken from the spec.
 *
 * - Use 'null' operations for few SYSCLKs (such as 4, 5, 6 etc.), which do not
 *   have software control for enable/disable (control registers are RO).
 *
 * - Numbering for modules such as UART, I2C etc., which have multiple
 *   instances, starts from 1 while register definitions are maintained starting
 *   from 0 as per spec. This is followed to avoid confusion with omaps, where
 *   the numbering start from 1. Some exceptions include Timers, IVAHDs.
 *
 * - The IDLEST bit and register for many clocks (e.g., mmchs1_fck) do not match
 *   with the default implementation as part of clkops_omap2_dflt_wait so for
 *   now, we use ti816x specific clock ops for idlest wait.
 *
 * - Note that the above is not applicable for some modules such as Ducati, GEM
 *   and IVAHD clocks as they do not actually get enabled even after clk_enable
 *   as "wait for enable" throws errors. For such modules, we retain
 *   clkops_omap2_dflt and rely on module owners to poll for enable status
 *   before accessing respective module.
 */

static struct clk secure_32k_ck = {
	.name		= "secure_32k_ck",
	.ops		= &clkops_null,
	.rate		= 32768,
	.flags		= RATE_IN_TI816X,
};

static struct clk sys_32k_ck = {
	.name		= "sys_32k_ck",
	.ops		= &clkops_null,
	.rate		= 32768,
	.flags		= RATE_IN_TI816X,
};

static struct clk tclkin_ck = {
	.name		= "tclkin_ck",
	.ops		= &clkops_null,
	.rate		= 32768,		/* FIXME */
	.flags		= RATE_IN_TI816X,
};

static struct clk sys_clkin_ck = {
	.name		= "sys_clkin_ck",
	.ops		= &clkops_null,
	.rate		= 27000000,
	.flags		= RATE_IN_TI816X,
};

/* MAIN FAPLL */
static struct fapll_data fapll_main_fd = {
	.fapll_id	= TI816X_MAINPLL_ID,
	.control_reg	= TI816X_MAINPLL_CTRL,
	.pwd_reg	= TI816X_MAINPLL_PWD,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.mult_mask	= TI816X_PLL_NVAL_MASK,
	.div_mask	= TI816X_PLL_PVAL_MASK,
	.bypass_mask	= TI816X_PLL_BYPASS_MASK,
	.enable_mask	= TI816X_PLL_ENABLE_MASK,
	.lock_mask	= TI816X_PLL_LOCK_OUT_SEL_MASK,
	.lock_sts_mask	= TI816X_PLL_LOCK_STS_MASK,
	.freq_frac_mask	= TI816X_PLL_FRACFREQ_MASK,
	.freq_int_mask	= TI816X_PLL_INTFREQ_MASK,
	.trunc_mask	= TI816X_PLL_TRUNC_MASK,
	.post_div_mask	= TI816X_PLL_MDIV_MASK,
	.ldfreq_mask	= TI816X_PLL_LDFREQ_MASK,
	.lddiv1_mask	= TI816X_PLL_LDMDIV_MASK,
	.bypass_en	= TI816X_MAINPLL_BYPASS_EN,
	.modes		= FAPLL_LOW_POWER_BYPASS,
	.rate_tolerance = TI816X_PLL_RATE_TOLARANCE,
	.max_multiplier	= TI816X_PLL_MAX_MULT,
	.min_divider	= TI816X_PLL_MIN_DIV,
	.max_divider	= TI816X_PLL_MAX_DIV,
	.mult_n		= TI816X_MAINPLL_N,
	.pre_div_p	= TI816X_MAINPLL_P,
};

/* Synthesizer 1 from Main PLL */
static struct clk main_pll_clk1_ck = {
	.name		= "main_pll_clk1_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID1,
	.freq_reg	= TI816X_MAINPLL_FREQ1,
	.post_div_reg	= TI816X_MAINPLL_DIV1,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK1_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 2 from Main PLL */
static struct clk main_pll_clk2_ck = {
	.name		= "main_pll_clk2_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID2,
	.freq_reg	= TI816X_MAINPLL_FREQ2,
	.post_div_reg	= TI816X_MAINPLL_DIV2,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK2_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 3 from Main PLL */
static struct clk main_pll_clk3_ck = {
	.name		= "main_pll_clk3_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID3,
	.freq_reg	= TI816X_MAINPLL_FREQ3,
	.post_div_reg	= TI816X_MAINPLL_DIV3,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK3_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 4 from Main PLL */
static struct clk main_pll_clk4_ck = {
	.name		= "main_pll_clk4_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID4,
	.freq_reg	= TI816X_MAINPLL_FREQ4,
	.post_div_reg	= TI816X_MAINPLL_DIV4,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK4_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 5 from Main PLL */
static struct clk main_pll_clk5_ck = {
	.name		= "main_pll_clk5_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID5,
	.freq_reg	= TI816X_MAINPLL_FREQ5,
	.post_div_reg	= TI816X_MAINPLL_DIV5,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK5_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 6 from Main PLL */
static struct clk main_pll_clk6_ck = {
	.name		= "main_pll_clk6_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID6,
	.post_div_reg	= TI816X_MAINPLL_DIV6,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK6_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 7 from Main PLL */
static struct clk main_pll_clk7_ck = {
	.name		= "main_pll_clk7_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_main_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID7,
	.post_div_reg	= TI816X_MAINPLL_DIV7,
	.pwd_mask	= TI816X_MAINPLL_PWD_CLK7_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

static const struct clksel_rate div8_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_TI816X },
	{ .div = 2, .val = 1, .flags = RATE_IN_TI816X },
	{ .div = 3, .val = 2, .flags = RATE_IN_TI816X },
	{ .div = 4, .val = 3, .flags = RATE_IN_TI816X },
	{ .div = 5, .val = 4, .flags = RATE_IN_TI816X },
	{ .div = 6, .val = 5, .flags = RATE_IN_TI816X },
	{ .div = 7, .val = 6, .flags = RATE_IN_TI816X },
	{ .div = 8, .val = 7, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk1_div[] = {
	{ .parent = &main_pll_clk1_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk1_ck = {
	.name		= "sysclk1_ck",
	.parent		= &main_pll_clk1_ck,
	.clksel		= sysclk1_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK1_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk gem_ick = {
	.name		= "gem_ick",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_ACTIVE_GEM_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "active_gem_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static const struct clksel sysclk2_div[] = {
	{ .parent = &main_pll_clk2_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk2_ck = {
	.name		= "sysclk2_ck",
	.parent		= &main_pll_clk2_ck,
	.clksel		= sysclk2_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK2_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mpu_ck = {
	.name		= "mpu_ck",
	.parent		= &sysclk2_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MPU_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_mpu_clkdm",
	.recalc		= &followparent_recalc,
};

/* SGX maximum frequency is 333MHz*/
static const struct clksel_rate sgx_div6_rates[] = {
	{ .div = 3, .val = 2, .flags = RATE_IN_TI816X },
	{ .div = 4, .val = 3, .flags = RATE_IN_TI816X },
	{ .div = 5, .val = 4, .flags = RATE_IN_TI816X },
	{ .div = 6, .val = 5, .flags = RATE_IN_TI816X },
	{ .div = 7, .val = 6, .flags = RATE_IN_TI816X },
	{ .div = 8, .val = 7, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk23_div[] = {
	{ .parent = &main_pll_clk2_ck, .rates = sgx_div6_rates },
	{ .parent = NULL },
};

static struct clk sysclk23_ck = {
	.name		= "sysclk23_ck",
	.parent		= &main_pll_clk2_ck,
	.clksel		= sysclk23_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK23_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk sgx_ck = {
	.name		= "sgx_ck",
	.parent		= &sysclk23_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_SGX_SGX_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "sgx_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static const struct clksel sysclk3_div[] = {
	{ .parent = &main_pll_clk3_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk3_ck = {
	.name		= "sysclk3_ck",
	.parent		= &main_pll_clk3_ck,
	.clksel		= sysclk3_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK3_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk ivahd0_ck = {
	.name		= "ivahd0_ck",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD0_IVAHD_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd0_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk ivahd0_sl2_ick = {
	.name		= "ivahd0_sl2_ick",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD0_SL2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd0_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk ivahd1_ck = {
	.name		= "ivahd1_ck",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD1_IVAHD_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd1_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk ivahd1_sl2_ick = {
	.name		= "ivahd1_sl2_ick",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD1_SL2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd1_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk ivahd2_ck = {
	.name		= "ivahd2_ck",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD2_IVAHD_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd2_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk ivahd2_sl2_ick = {
	.name		= "ivahd2_sl2_ick",
	.parent		= &sysclk3_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_IVAHD2_SL2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "ivahd2_clkdm",
	.recalc		= &followparent_recalc,
	.set_rate	= &ti816x_clksel_set_rate,
};

static const struct clksel_rate div2_sysclk4_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_TI816X },
	{ .div = 2, .val = 1, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk4_div[] = {
	{ .parent = &main_pll_clk4_ck, .rates = div2_sysclk4_rates },
	{ .parent = NULL },
};

static struct clk sysclk4_ck = {
	.name		= "sysclk4_ck",
	.parent		= &main_pll_clk4_ck,
	.clksel		= sysclk4_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK4_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_0_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mmu_ick = {
	.name		= "mmu_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MMUDATA_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "mmu_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk ducati_ucache_ick = {
	.name		= "ducati_ucache_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "default_ducati_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk tpcc_ick = {
	.name		= "tpcc_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_TPCC_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk tptc0_ick = {
	.name		= "tptc0_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_TPTC0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk tptc1_ick = {
	.name		= "tptc1_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_TPTC1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk tptc2_ick = {
	.name		= "tptc2_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_TPTC2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk tptc3_ick = {
	.name		= "tptc3_ick",
	.parent		= &sysclk4_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_TPTC3_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static const struct clksel_rate div_4_1_rates[] = {
	{ .div = 4, .val = 1, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk6_div[] = {
	{ .parent = &sysclk4_ck, .rates = div_4_1_rates },
	{ .parent = NULL },
};

static struct clk sysclk6_ck = {
	.name		= "sysclk6_ck",
	.parent		= &sysclk4_ck,
	.clksel		= sysclk6_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK6_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_0_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mmu_cfg_ick = {
	.name		= "mmu_cfg_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MMUCFG_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "mmu_cfg_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mailbox_ick = {
	.name		= "mailbox_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MAILBOX_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk spinbox_ick = {
	.name		= "spinbox_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_SPINBOX_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gem_vbusp_fck = {
	.name		= "gem_vbusp_fck",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "active_gem_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk uart1_ick = {
	.name		= "uart1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk uart2_ick = {
	.name		= "uart2_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk uart3_ick = {
	.name		= "uart3_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk wdt2_ick = {
	.name		= "wdt2_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mcspi1_ick = {
	.name		= "mcspi1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk usbotg_ick = {
	.name		= "usb_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_usb,
	.enable_reg	= TI816X_CM_DEFAULT_USB_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "default_usb_clkdm",
	.recalc		= &followparent_recalc,
};

/* Note: We are referring dmtimers as gptimers to match omap convention */

static struct clk gpt1_ick = {
	.name		= "gpt1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt2_ick = {
	.name		= "gpt2_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt3_ick = {
	.name		= "gpt3_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt4_ick = {
	.name		= "gpt4_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt5_ick = {
	.name		= "gpt5_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt6_ick = {
	.name		= "gpt6_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpt7_ick = {
	.name		= "gpt7_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_ick = {
	.name		= "gpio1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_GPIO_0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_ick = {
	.name		= "gpio2_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_GPIO_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpmc_fck = {
	.name		= "gpmc_fck",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_GPMC_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_ick = {
	.name		= "i2c1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk i2c2_ick = {
	.name		= "i2c2_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

/* ELM Clock(final) */
static struct clk elm_ick = {
	.name       = "elm_ick",
	.parent     = &sysclk6_ck,
	.ops        = &clkops_null,
	.clkdm_name = "alwon_l3_slow_clkdm",
	.recalc     = &followparent_recalc,
};

static struct clk elm_fck = {
    .name       = "elm_fck",
    .parent     = &sysclk6_ck,
    .ops        = &clkops_null,
    .clkdm_name = "alwon_l3_slow_clkdm",
    .recalc     = &followparent_recalc,
};

static struct clk mmchs1_ick = {
	.name		= "mmchs1_ick",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

/* HVT smartReflex sensor clock */
static struct clk sr_hvt_fck = {
	.name		= "sr_hvt_fck",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI81XX_CM_ALWON_SR_0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

/* SVT smartReflex sensor clock */
static struct clk sr_svt_fck = {
	.name		= "sr_svt_fck",
	.parent		= &sysclk6_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI81XX_CM_ALWON_SR_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static const struct clksel_rate div_2_1_rates[] = {
	{ .div = 2, .val = 1, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk5_div[] = {
	{ .parent = &sysclk4_ck, .rates = div_2_1_rates },
	{ .parent = NULL },
};

static struct clk sysclk5_ck = {
	.name		= "sysclk5_ck",
	.parent		= &sysclk4_ck,
	.clksel		= sysclk5_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK5_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_0_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk pcie_ck = {
	.name		= "pcie_ck",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_ti81xx_pcie,
	.enable_reg	= TI816X_CM_DEFAULT_PCI_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "default_pcie_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk sata_ick = {
	.name		= "sata_ick",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_DEFAULT_SATA_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "default_l3_med_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk emac1_ick = {
	.name		= "emac1_ick",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_ETHERNET_0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_ethernet_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk emac2_ick = {
	.name		= "emac2_ick",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_ETHERNET_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_ethernet_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk ducati_ick = {
	.name		= "ducati_ick",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= TI816X_CM_DEFAULT_DUCATI_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "default_ducati_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gem_trc_fck = {
	.name		= "gem_trc_fck",
	.parent		= &sysclk5_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "active_gem_clkdm",
	.recalc		= &followparent_recalc,
};

/* DDR FAPLL */
static struct fapll_data fapll_ddr_fd = {
	.fapll_id	= TI816X_DDRPLL_ID,
	.control_reg	= TI816X_DDRPLL_CTRL,
	.pwd_reg	= TI816X_DDRPLL_PWD,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.mult_mask	= TI816X_PLL_NVAL_MASK,
	.div_mask	= TI816X_PLL_PVAL_MASK,
	.bypass_mask	= TI816X_PLL_BYPASS_MASK,
	.enable_mask	= TI816X_PLL_ENABLE_MASK,
	.lock_mask	= TI816X_PLL_LOCK_OUT_SEL_MASK,
	.lock_sts_mask	= TI816X_PLL_LOCK_STS_MASK,
	.freq_frac_mask	= TI816X_PLL_FRACFREQ_MASK,
	.freq_int_mask	= TI816X_PLL_INTFREQ_MASK,
	.trunc_mask	= TI816X_PLL_TRUNC_MASK,
	.post_div_mask	= TI816X_PLL_MDIV_MASK,
	.ldfreq_mask	= TI816X_PLL_LDFREQ_MASK,
	.lddiv1_mask	= TI816X_PLL_LDMDIV_MASK,
	.bypass_en	= TI816X_DDRPLL_BYPASS_EN,
	.modes		= FAPLL_LOW_POWER_BYPASS,
	.rate_tolerance = TI816X_PLL_RATE_TOLARANCE,
	.max_multiplier	= TI816X_PLL_MAX_MULT,
	.min_divider	= TI816X_PLL_MIN_DIV,
	.max_divider	= TI816X_PLL_MAX_DIV,
	.mult_n		= TI816X_DDRPLL_400_N,
	.pre_div_p	= TI816X_DDRPLL_400_P,
};

/* Synthesizer 1 from DDR PLL */
static struct clk ddr_pll_clk1_ck = {
	.name		= "ddr_pll_clk1_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_ddr_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID1,
	.post_div_reg	= TI816X_DDRPLL_DIV1,
	.pwd_mask	= TI816X_DDRPLL_PWD_CLK1_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 2 from DDR PLL */
static struct clk ddr_pll_clk2_ck = {
	.name		= "ddr_pll_clk2_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_ddr_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID2,
	.freq_reg	= TI816X_DDRPLL_FREQ2,
	.post_div_reg	= TI816X_DDRPLL_DIV2,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_DDRPLL_PWD_CLK2_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 3 from DDR PLL */
static struct clk ddr_pll_clk3_ck = {
	.name		= "ddr_pll_clk3_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_ddr_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID3,
	.freq_reg	= TI816X_DDRPLL_FREQ3,
	.post_div_reg	= TI816X_DDRPLL_DIV3,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_DDRPLL_PWD_CLK3_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 4 from DDR PLL */
static struct clk ddr_pll_clk4_ck = {
	.name		= "ddr_pll_clk4_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_ddr_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID4,
	.freq_reg	= TI816X_DDRPLL_FREQ4,
	.post_div_reg	= TI816X_DDRPLL_DIV4,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_DDRPLL_PWD_CLK4_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 5 from DDR PLL */
static struct clk ddr_pll_clk5_ck = {
	.name		= "ddr_pll_clk5_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_ddr_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID5,
	.freq_reg	= TI816X_DDRPLL_FREQ5,
	.post_div_reg	= TI816X_DDRPLL_DIV5,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_DDRPLL_PWD_CLK4_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_null,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

static const struct clksel sysclk10_div[] = {
	{ .parent = &ddr_pll_clk2_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk10_ck = {
	.name		= "sysclk10_ck",
	.parent		= &ddr_pll_clk2_ck,
	.clksel		= sysclk10_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK10_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk uart1_fck = {
	.name		= "uart1_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_UART_0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk uart2_fck = {
	.name		= "uart2_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_UART_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk uart3_fck = {
	.name		= "uart3_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_UART_2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mcspi1_fck = {
	.name		= "mcspi1_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_SPI_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_fck = {
	.name		= "i2c1_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_I2C_0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk i2c2_fck = {
	.name		= "i2c2_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_I2C_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mmchs1_fck = {
	.name		= "mmchs1_fck",
	.parent		= &sysclk10_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_SDIO_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static const struct clksel sysclk24_div[] = {
	{ .parent = &main_pll_clk5_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk24_ck = {
	.name		= "sysclk24_ck",
	.parent		= &main_pll_clk5_ck,
	.clksel		= sysclk24_div,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK24_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

/* VIDEO FAPLL */
static struct fapll_data fapll_video_fd = {
	.fapll_id	= TI816X_VIDEOPLL_ID,
	.control_reg	= TI816X_VIDEOPLL_CTRL,
	.pwd_reg	= TI816X_VIDEOPLL_PWD,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.mult_mask	= TI816X_PLL_NVAL_MASK,
	.div_mask	= TI816X_PLL_PVAL_MASK,
	.bypass_mask	= TI816X_PLL_BYPASS_MASK,
	.enable_mask	= TI816X_PLL_ENABLE_MASK,
	.lock_mask	= TI816X_PLL_LOCK_OUT_SEL_MASK,
	.lock_sts_mask	= TI816X_PLL_LOCK_STS_MASK,
	.freq_frac_mask	= TI816X_PLL_FRACFREQ_MASK,
	.freq_int_mask	= TI816X_PLL_INTFREQ_MASK,
	.trunc_mask	= TI816X_PLL_TRUNC_MASK,
	.post_div_mask	= TI816X_PLL_MDIV_MASK,
	.ldfreq_mask	= TI816X_PLL_LDFREQ_MASK,
	.lddiv1_mask	= TI816X_PLL_LDMDIV_MASK,
	.bypass_en	= TI816X_VIDEOPLL_BYPASS_EN,
	.modes		= FAPLL_LOW_POWER_BYPASS,
	.rate_tolerance = TI816X_PLL_RATE_TOLARANCE,
	.max_multiplier	= TI816X_PLL_MAX_MULT,
	.min_divider	= TI816X_PLL_MIN_DIV,
	.max_divider	= TI816X_PLL_MAX_DIV,
	.mult_n		= TI816X_VIDEOPLL_N,
	.pre_div_p	= TI816X_VIDEOPLL_P,
};

/* Synthesizer 1 from VIDEO PLL */
static struct clk video_pll_clk1_ck = {
	.name		= "video_pll_clk1_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_video_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID1,
	.freq_reg	= TI816X_VIDEOPLL_FREQ1,
	.post_div_reg	= TI816X_VIDEOPLL_DIV1,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_VIDEOPLL_PWD_CLK1_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 2 from VIDEO PLL */
static struct clk video_pll_clk2_ck = {
	.name		= "video_pll_clk2_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_video_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID2,
	.freq_reg	= TI816X_VIDEOPLL_FREQ2,
	.post_div_reg	= TI816X_VIDEOPLL_DIV2,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_VIDEOPLL_PWD_CLK2_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 3 from VIDEO PLL */
static struct clk video_pll_clk3_ck = {
	.name		= "video_pll_clk3_ck",
	.parent		= &sys_clkin_ck,
	.fapll_data	= &fapll_video_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID3,
	.freq_reg	= TI816X_VIDEOPLL_FREQ3,
	.post_div_reg	= TI816X_VIDEOPLL_DIV3,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_VIDEOPLL_PWD_CLK3_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

static struct clk audio_pll_clk1_ck = {
	.name		= "audio_pll_clk1_ck",
	.ops		= &clkops_null,
	.rate		= 32768,
	.flags		= RATE_IN_TI816X,
};

/* AUDIO FAPLL */
static struct fapll_data fapll_audio_fd = {
	.fapll_id	= TI816X_AUDIOPLL_ID,
	.control_reg	= TI816X_AUDIOPLL_CTRL,
	.pwd_reg	= TI816X_AUDIOPLL_PWD,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &main_pll_clk7_ck,
	.mult_mask	= TI816X_PLL_NVAL_MASK,
	.div_mask	= TI816X_PLL_PVAL_MASK,
	.bypass_mask	= TI816X_PLL_BYPASS_MASK,
	.enable_mask	= TI816X_PLL_ENABLE_MASK,
	.lock_mask	= TI816X_PLL_LOCK_OUT_SEL_MASK,
	.lock_sts_mask	= TI816X_PLL_LOCK_STS_MASK,
	.freq_frac_mask	= TI816X_PLL_FRACFREQ_MASK,
	.freq_int_mask	= TI816X_PLL_INTFREQ_MASK,
	.trunc_mask	= TI816X_PLL_TRUNC_MASK,
	.post_div_mask	= TI816X_PLL_MDIV_MASK,
	.ldfreq_mask	= TI816X_PLL_LDFREQ_MASK,
	.lddiv1_mask	= TI816X_PLL_LDMDIV_MASK,
	.bypass_en	= TI816X_AUDIOPLL_BYPASS_EN,
	.modes		= FAPLL_LOW_POWER_BYPASS,
	.rate_tolerance = TI816X_PLL_RATE_TOLARANCE,
	.max_multiplier	= TI816X_PLL_MAX_MULT,
	.min_divider	= TI816X_PLL_MIN_DIV,
	.max_divider	= TI816X_PLL_MAX_DIV,
	.mult_n		= TI816X_AUDIOPLL_N,
	.pre_div_p	= TI816X_AUDIOPLL_P,
};

/* Synthesizer 2 from AUDIO PLL */
static struct clk audio_pll_clk2_ck = {
	.name		= "audio_pll_clk2_ck",
	.parent		= &main_pll_clk7_ck,
	.fapll_data	= &fapll_audio_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID2,
	.freq_reg	= TI816X_AUDIOPLL_FREQ2,
	.post_div_reg	= TI816X_AUDIOPLL_DIV2,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_AUDIOPLL_PWD_CLK2_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 3 from AUDIO PLL */
static struct clk audio_pll_clk3_ck = {
	.name		= "audio_pll_clk3_ck",
	.parent		= &main_pll_clk7_ck,
	.fapll_data	= &fapll_audio_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID3,
	.freq_reg	= TI816X_AUDIOPLL_FREQ3,
	.post_div_reg	= TI816X_AUDIOPLL_DIV3,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_AUDIOPLL_PWD_CLK3_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 4 from AUDIO PLL */
static struct clk audio_pll_clk4_ck = {
	.name		= "audio_pll_clk4_ck",
	.parent		= &main_pll_clk7_ck,
	.fapll_data	= &fapll_audio_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID4,
	.freq_reg	= TI816X_AUDIOPLL_FREQ4,
	.post_div_reg	= TI816X_AUDIOPLL_DIV4,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_AUDIOPLL_PWD_CLK4_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

/* Synthesizer 5 from AUDIO PLL */
static struct clk audio_pll_clk5_ck = {
	.name		= "audio_pll_clk5_ck",
	.parent		= &main_pll_clk7_ck,
	.fapll_data	= &fapll_audio_fd,
	.synthesizer_id	= TI816X_SYNTHESIZER_ID5,
	.freq_reg	= TI816X_AUDIOPLL_FREQ5,
	.post_div_reg	= TI816X_AUDIOPLL_DIV5,
	.freq_flag	= TI816X_SYN_FREQ_VALUE_PRESENT,
	.pwd_mask	= TI816X_AUDIOPLL_PWD_CLK5_MASK,
	.init		= &ti816x_init_fapll_parent,
	.ops		= &clkops_ti816x_fapll_ops,
	.recalc		= &ti816x_fapll_recalc,
	.round_rate	= &ti816x_fapll_round_rate,
	.set_rate	= &ti816x_fapll_set_rate,
};

static const struct clksel audio_pll_a_div[] = {
	{ .parent = &audio_pll_clk1_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk audio_pll_a_ck = {
	.name		= "audio_pll_a_ck",
	.parent		= &audio_pll_clk1_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= audio_pll_a_div,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_APA_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate div_1_0_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel_rate div_1_1_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel_rate div_1_2_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel_rate div_1_3_rates[] = {
	{ .div = 1, .val = 3, .flags = RATE_IN_TI816X },
	{ .div = 0 },
};

static const struct clksel sysclk18_mux_sel[] = {
	{ .parent = &sys_32k_ck, .rates = div_1_0_rates },
	{ .parent = &audio_pll_a_ck, .rates = div_1_1_rates },
	{ .parent = NULL}
};

static struct clk sysclk18_ck = {
	.name		= "sysclk18_ck",
	.parent		= &audio_pll_a_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= sysclk18_mux_sel,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK18_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_0_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel sysclk20_div[] = {
	{ .parent = &audio_pll_clk3_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk20_ck = {
	.name		= "sysclk20_ck",
	.parent		= &audio_pll_clk3_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= sysclk20_div,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK20_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel sysclk21_div[] = {
	{ .parent = &audio_pll_clk4_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk21_ck = {
	.name		= "sysclk21_ck",
	.parent		= &audio_pll_clk4_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= sysclk21_div,
	.ops		= &clkops_null,
	.clksel_reg	= TI81XX_CM_DPLL_SYSCLK21_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel sysclk22_div[] = {
	{ .parent = &audio_pll_clk5_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk sysclk22_ck = {
	.name		= "sysclk22_ck",
	.parent		= &audio_pll_clk5_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= sysclk22_div,
	.ops		= &clkops_null,
	.clksel_reg	= TI816X_CM_DPLL_SYSCLK22_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_2_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel gpt_mux_sel[] = {
	{ .parent = &tclkin_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk18_ck, .rates = div_1_1_rates },
	{ .parent = &sys_clkin_ck, .rates = div_1_2_rates },
	{ .parent = NULL}
};

static const struct clksel mcasp0to2_mux_sel[] = {
	{ .parent = &sysclk20_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk21_ck, .rates = div_1_1_rates },
	{ .parent = &sysclk22_ck, .rates = div_1_1_rates },
	{ .parent = NULL}
};

static struct clk mcb_fsx_ck = {
	.name		= "mcb_fsx_ck",
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_MCBSP_FSX_EN,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.rate		= 0,
};

static struct clk mcb_clks_ck = {
	.name		= "mcb_clks_ck",
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_MCBSP_CLKS_EN,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.rate		= 0,
};

static struct clk pin_mux_out_ck = {
	.name		= "pin_mux_out_ck",
	.parent		= &mcb_fsx_ck,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel mcbsp_mux_sel[] = {
	{ .parent = &sysclk20_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk21_ck, .rates = div_1_1_rates },
	{ .parent = &sysclk22_ck, .rates = div_1_2_rates },
	{ .parent = &pin_mux_out_ck, .rates = div_1_3_rates}
};

static struct clk gpt1_fck = {
	.name		= "gpt1_fck",
	.parent		= &sysclk18_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= gpt_mux_sel,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER1_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt2_fck = {
	.name		= "gpt2_fck",
	.parent		= &sysclk18_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER2_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt3_fck = {
	.name		= "gpt3_fck",
	.parent		= &sys_clkin_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_3_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER3_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt4_fck = {
	.name		= "gpt4_fck",
	.parent		= &sysclk18_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_4_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER4_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt5_fck = {
	.name		= "gpt5_fck",
	.parent		= &sysclk18_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_5_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER5_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt6_fck = {
	.name		= "gpt6_fck",
	.parent		= &sysclk18_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_6_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER6_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt7_fck = {
	.name		= "gpt7_fck",
	.parent		= &sysclk18_ck,
	.clksel		= gpt_mux_sel,
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI816X_CM_ALWON_TIMER_7_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI816X_CM_DPLL_TIMER7_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk wdt2_fck = {
	.name		= "wdt2_fck",
	.parent		= &sysclk18_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_WDTIMER_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_dbck = {
	.name		= "gpio1_dbck",
	.parent		= &sysclk18_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_GPIO_0_CLKCTRL,
	.enable_bit	= TI816X_GPIO_0_DBCLK_SHIFT,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_dbck = {
	.name		= "gpio2_dbck",
	.parent		= &sysclk18_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_GPIO_1_CLKCTRL,
	.enable_bit	= TI816X_GPIO_1_DBCLK_SHIFT,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mmchsdb1_fck = {
	.name		= "mmchsdb1_fck",
	.parent		= &sysclk18_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mcasp0_fck = {
	.name		= "mcasp0_fck",
	.parent		= &sysclk20_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= mcasp0to2_mux_sel,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MCASP0_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI81XX_CM_DPLL_AUDIOCLK_MCASP0_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk mcasp1_fck = {
	.name		= "mcasp1_fck",
	.parent		= &sysclk20_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= mcasp0to2_mux_sel,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MCASP1_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI81XX_CM_DPLL_AUDIOCLK_MCASP1_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &ti816x_clksel_set_rate,
};

static struct clk mcasp2_fck = {
	.name		= "mcasp2_fck",
	.parent		= &sysclk20_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= mcasp0to2_mux_sel,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MCASP2_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI81XX_CM_DPLL_AUDIOCLK_MCASP2_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &ti816x_clksel_set_rate,
};

/* RTC Functional clock */
static struct clk rtc_c32k_fck = {
	.name		= "rtc_c32k_fck",
	.parent		= &sysclk18_ck,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_RTC_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp_fck = {
	.name		= "mcbsp_fck",
	.parent		= &sysclk20_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= mcbsp_mux_sel,
	.ops		= &clkops_ti81xx_dflt_wait,
	.enable_reg	= TI81XX_CM_ALWON_MCBSP_CLKCTRL,
	.enable_bit	= TI81XX_MODULEMODE_SWCTRL,
	.clksel_reg	= TI81XX_CM_DPLL_AUDIOCLK_MCBSP_CLKSEL,
	.clksel_mask	= TI81XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "alwon_l3_slow_clkdm",
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &ti816x_clksel_set_rate,
};

/*
 * clkdev
 *
 * FIXME: Some of the external clocks (e.g., tclk) are kept here for
 * completeness.
 */
static struct omap_clk ti816x_clks[] = {
	CLK(NULL,		"secure_32k_ck",	&secure_32k_ck,		CK_TI816X),
	CLK(NULL,		"sys_32k_ck",		&sys_32k_ck,		CK_TI816X),
	CLK(NULL,		"tclkin_ck",		&tclkin_ck,		CK_TI816X),
	CLK(NULL,		"sys_clkin_ck",		&sys_clkin_ck,		CK_TI816X),
	CLK(NULL,		"main_pll_clk1_ck",	&main_pll_clk1_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk2_ck",	&main_pll_clk2_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk3_ck",	&main_pll_clk3_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk4_ck",	&main_pll_clk4_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk5_ck",	&main_pll_clk5_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk6_ck",	&main_pll_clk6_ck,	CK_TI816X),
	CLK(NULL,		"main_pll_clk7_ck",	&main_pll_clk7_ck,	CK_TI816X),
	CLK(NULL,		"sysclk1_ck",		&sysclk1_ck,		CK_TI816X),
	CLK(NULL,		"gem_ick",		&gem_ick,		CK_TI816X),
	CLK(NULL,		"sysclk2_ck",		&sysclk2_ck,		CK_TI816X),
	CLK(NULL,		"mpu_ck",		&mpu_ck,		CK_TI816X),
	CLK(NULL,		"sysclk23_ck",		&sysclk23_ck,		CK_TI816X),
	CLK(NULL,		"sgx_ck",		&sgx_ck,		CK_TI816X),
	CLK(NULL,		"sysclk3_ck",		&sysclk3_ck,		CK_TI816X),
	CLK(NULL,		"ivahd0_ck",		&ivahd0_ck,		CK_TI816X),
	CLK(NULL,		"ivahd0_sl2_ick",	&ivahd0_sl2_ick,	CK_TI816X),
	CLK(NULL,		"ivahd1_ck",		&ivahd1_ck,		CK_TI816X),
	CLK(NULL,		"ivahd1_sl2_ick",	&ivahd1_sl2_ick,	CK_TI816X),
	CLK(NULL,		"ivahd2_ck",		&ivahd2_ck,		CK_TI816X),
	CLK(NULL,		"ivahd2_sl2_ick",	&ivahd2_sl2_ick,	CK_TI816X),
	CLK(NULL,		"sysclk4_ck",		&sysclk4_ck,		CK_TI816X),
	CLK(NULL,		"mmu_ick",		&mmu_ick,		CK_TI816X),
	CLK(NULL,		"ducati_ucache_ick",	&ducati_ucache_ick,	CK_TI816X),
	CLK(NULL,		"tpcc_ick",		&tpcc_ick,		CK_TI816X),
	CLK(NULL,		"tptc0_ick",		&tptc0_ick,		CK_TI816X),
	CLK(NULL,		"tptc1_ick",		&tptc1_ick,		CK_TI816X),
	CLK(NULL,		"tptc2_ick",		&tptc2_ick,		CK_TI816X),
	CLK(NULL,		"tptc3_ick",		&tptc3_ick,		CK_TI816X),
	CLK(NULL,		"sysclk6_ck",		&sysclk6_ck,		CK_TI816X),
	CLK(NULL,		"mmu_cfg_ick",		&mmu_cfg_ick,		CK_TI816X),
	CLK(NULL,		"mailbox_ick",		&mailbox_ick,		CK_TI816X),
	CLK(NULL,		"spinbox_ick",		&spinbox_ick,		CK_TI816X),
	CLK(NULL,		"gem_vbusp_fck",	&gem_vbusp_fck,		CK_TI816X),
	CLK(NULL,		"uart1_ick",		&uart1_ick,		CK_TI816X),
	CLK(NULL,		"uart2_ick",		&uart2_ick,		CK_TI816X),
	CLK(NULL,		"uart3_ick",		&uart3_ick,		CK_TI816X),
	CLK("omap2_mcspi.1",	"ick",			&mcspi1_ick,		CK_TI816X),
	CLK(NULL,		"wdt2_ick",		&wdt2_ick,		CK_TI816X),
	CLK(NULL,		"gpt1_ick",		&gpt1_ick,		CK_TI816X),
	CLK(NULL,		"gpt2_ick",		&gpt2_ick,		CK_TI816X),
	CLK(NULL,		"gpt3_ick",		&gpt3_ick,		CK_TI816X),
	CLK(NULL,		"gpt4_ick",		&gpt4_ick,		CK_TI816X),
	CLK(NULL,		"gpt5_ick",		&gpt5_ick,		CK_TI816X),
	CLK(NULL,		"gpt6_ick",		&gpt6_ick,		CK_TI816X),
	CLK(NULL,		"gpt7_ick",		&gpt7_ick,		CK_TI816X),
	CLK(NULL,		"gpio1_ick",		&gpio1_ick,		CK_TI816X),
	CLK(NULL,		"gpio2_ick",		&gpio2_ick,		CK_TI816X),
	CLK("omap_i2c.1",	"ick",			&i2c1_ick,		CK_TI816X),
	CLK("omap_i2c.2",	"ick",			&i2c2_ick,		CK_TI816X),
	CLK(NULL,		"elm_ick",		&elm_ick,		CK_TI816X),
	CLK(NULL,		"elm_fck",		&elm_fck,		CK_TI816X),
	CLK("mmci-omap-hs.0",	"ick",			&mmchs1_ick,		CK_TI816X),
	CLK(NULL,		"sr_hvt_fck",		&sr_hvt_fck,		CK_TI816X),
	CLK(NULL,		"sr_svt_fck",		&sr_svt_fck,		CK_TI816X),
	CLK(NULL,		"ddr_pll_clk1_ck",	&ddr_pll_clk1_ck,	CK_TI816X),
	CLK(NULL,		"ddr_pll_clk2_ck",	&ddr_pll_clk2_ck,	CK_TI816X),
	CLK(NULL,		"ddr_pll_clk3_ck",	&ddr_pll_clk3_ck,	CK_TI816X),
	CLK(NULL,		"ddr_pll_clk4_ck",	&ddr_pll_clk4_ck,	CK_TI816X),
	CLK(NULL,		"ddr_pll_clk5_ck",	&ddr_pll_clk5_ck,	CK_TI816X),
	CLK(NULL,		"sysclk10_ck",		&sysclk10_ck,		CK_TI816X),
	CLK(NULL,		"uart1_fck",		&uart1_fck,		CK_TI816X),
	CLK(NULL,		"uart2_fck",		&uart2_fck,		CK_TI816X),
	CLK(NULL,		"uart3_fck",		&uart3_fck,		CK_TI816X),
	CLK("ti81xx-usbss",	"usb_ick",		&usbotg_ick,		CK_TI816X),
	CLK(NULL,		"sysclk5_ck",		&sysclk5_ck,		CK_TI816X),
	CLK(NULL,		"pcie_ck",		&pcie_ck,		CK_TI816X),
	CLK("ahci.0",		NULL,			&sata_ick,		CK_TI816X),
	CLK("davinci_emac.0",	NULL,			&emac1_ick,		CK_TI816X),
	CLK("davinci_emac.1",	NULL,			&emac2_ick,		CK_TI816X),
	CLK(NULL,		"ducati_ick",		&ducati_ick,		CK_TI816X),
	CLK(NULL,		"gem_trc_fck",		&gem_trc_fck,		CK_TI816X),
	CLK("omap2_mcspi.1",	"fck",		&mcspi1_fck,	CK_TI816X),
	CLK(NULL,		"gpmc_fck",		&gpmc_fck,		CK_TI816X),
	CLK("omap_i2c.1",	"fck",			&i2c1_fck,		CK_TI816X),
	CLK("omap_i2c.2",	"fck",			&i2c2_fck,		CK_TI816X),
	CLK("mmci-omap-hs.0",	"fck",			&mmchs1_fck,		CK_TI816X),
	CLK(NULL,		"sysclk24_ck",		&sysclk24_ck,		CK_TI816X),
	CLK(NULL,		"video_pll_clk1_ck",	&video_pll_clk1_ck,	CK_TI816X),
	CLK(NULL,		"video_pll_clk2_ck",	&video_pll_clk2_ck,	CK_TI816X),
	CLK(NULL,		"video_pll_clk3_ck",	&video_pll_clk3_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_clk1_ck",	&audio_pll_clk1_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_clk2_ck",	&audio_pll_clk2_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_clk3_ck",	&audio_pll_clk3_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_clk4_ck",	&audio_pll_clk4_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_clk5_ck",	&audio_pll_clk5_ck,	CK_TI816X),
	CLK(NULL,		"audio_pll_a_ck",	&audio_pll_a_ck,	CK_TI816X),
	CLK(NULL,		"sysclk18_ck",		&sysclk18_ck,		CK_TI816X),
	CLK(NULL,		"gpt1_fck",		&gpt1_fck,		CK_TI816X),
	CLK(NULL,		"gpt2_fck",		&gpt2_fck,		CK_TI816X),
	CLK(NULL,		"gpt3_fck",		&gpt3_fck,		CK_TI816X),
	CLK(NULL,		"gpt4_fck",		&gpt4_fck,		CK_TI816X),
	CLK(NULL,		"gpt5_fck",		&gpt5_fck,		CK_TI816X),
	CLK(NULL,		"gpt6_fck",		&gpt6_fck,		CK_TI816X),
	CLK(NULL,		"gpt7_fck",		&gpt7_fck,		CK_TI816X),
	CLK(NULL,		"wdt2_fck",		&wdt2_fck,		CK_TI816X),
	CLK(NULL,		"gpio1_dbck",		&gpio1_dbck,		CK_TI816X),
	CLK(NULL,		"gpio2_dbck",		&gpio2_dbck,		CK_TI816X),
	CLK("mmci-omap-hs.0",	"mmchsdb_fck",		&mmchsdb1_fck,		CK_TI816X),
	CLK(NULL,		"sysclk20_ck",		&sysclk20_ck,		CK_TI816X),
	CLK(NULL,		"sysclk21_ck",		&sysclk21_ck,		CK_TI816X),
	CLK(NULL,		"sysclk22_ck",		&sysclk22_ck,		CK_TI816X),
	CLK("davinci-mcasp.0",	NULL,			&mcasp0_fck,		CK_TI816X),
	CLK("davinci-mcasp.1",	NULL,			&mcasp1_fck,		CK_TI816X),
	CLK("davinci-mcasp.2",	NULL,			&mcasp2_fck,		CK_TI816X),
	CLK(NULL,		"rtc_c32k_fck",		&rtc_c32k_fck,		CK_TI816X),
	CLK("hdmi-dai", 	NULL,			&mcbsp_fck,		CK_TI816X),
	CLK(NULL,		"mcb_fsx_ck",		&mcb_fsx_ck,		CK_TI816X),
	CLK(NULL,		"mcb_clks_ck",		&mcb_clks_ck,		CK_TI816X),
	CLK(NULL,		"pin_mux_out_ck",	&pin_mux_out_ck,	CK_TI816X),
};

int __init ti816x_clk_init(void)
{
	struct omap_clk *c;
	u32 cpu_clkflg;

	if (cpu_is_ti816x()) {
		cpu_mask = RATE_IN_TI816X;
		cpu_clkflg = CK_TI816X;
	}

	clk_init(&omap2_clk_functions);

	for (c = ti816x_clks; c < ti816x_clks + ARRAY_SIZE(ti816x_clks); c++)
		clk_preinit(c->lk.clk);

	for (c = ti816x_clks; c < ti816x_clks + ARRAY_SIZE(ti816x_clks); c++)
		if (c->cpu & cpu_clkflg) {
			clkdev_add(&c->lk);
			clk_register(c->lk.clk);
			omap2_init_clk_clkdm(c->lk.clk);
		}

	recalculate_root_clocks();

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable_init_clocks();

	return 0;
}
/*
 *  This function is used to select the input clock from
 *  PINCTRL149 or PINCTRL153 (MCA2_ASFX/MCB_SFX or MCA2_AHCLKR/MCB_CLKS)
 *  clk 1 - MCA2_ASFX
 *  clk 2 - MCA2_AHCLKR
 *  rate  - rate of the external clk
 */
void mcb_clk_sel_pins(int clk, int rate)
{
	switch (clk) {
	case 1:
		mcb_fsx_ck.rate = rate;
		clk_enable(&mcb_fsx_ck);
		clk_reparent(&pin_mux_out_ck, &mcb_fsx_ck);
		break;
	case 2:
		mcb_clks_ck.rate = rate;
		clk_enable(&mcb_clks_ck);
		clk_reparent(&pin_mux_out_ck, &mcb_clks_ck);
		break;
	default:
		break;
	}
}
