/*
 * OMAP clock: data structure definitions, function prototypes, shared macros
 *
 * Copyright (C) 2004-2005, 2008-2010 Nokia Corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_OMAP_CLOCK_H
#define __ARCH_ARM_OMAP_CLOCK_H

#include <linux/list.h>

struct module;
struct clk;
struct clockdomain;

/**
 * struct clkops - some clock function pointers
 * @enable: fn ptr that enables the current clock in hardware
 * @disable: fn ptr that enables the current clock in hardware
 * @find_idlest: function returning the IDLEST register for the clock's IP blk
 * @find_companion: function returning the "companion" clk reg for the clock
 *
 * A "companion" clk is an accompanying clock to the one being queried
 * that must be enabled for the IP module connected to the clock to
 * become accessible by the hardware.  Neither @find_idlest nor
 * @find_companion should be needed; that information is IP
 * block-specific; the hwmod code has been created to handle this, but
 * until hwmod data is ready and drivers have been converted to use PM
 * runtime calls in place of clk_enable()/clk_disable(), @find_idlest and
 * @find_companion must, unfortunately, remain.
 */
struct clkops {
	int			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	void			(*find_idlest)(struct clk *, void __iomem **,
					       u8 *, u8 *);
	void			(*find_companion)(struct clk *, void __iomem **,
						  u8 *);
};

#ifdef CONFIG_ARCH_OMAP2PLUS

/* struct clksel_rate.flags possibilities */
#define RATE_IN_242X		(1 << 0)
#define RATE_IN_243X		(1 << 1)
#define RATE_IN_3430ES1		(1 << 2)	/* 3430ES1 rates only */
#define RATE_IN_3430ES2PLUS	(1 << 3)	/* 3430 ES >= 2 rates only */
#define RATE_IN_36XX		(1 << 4)
#define RATE_IN_4430		(1 << 5)
#define RATE_IN_TI814X		(1 << 6)
#define RATE_IN_TI816X		(1 << 7)

#define RATE_IN_24XX		(RATE_IN_242X | RATE_IN_243X)
#define RATE_IN_34XX		(RATE_IN_3430ES1 | RATE_IN_3430ES2PLUS)
#define RATE_IN_3XXX		(RATE_IN_34XX | RATE_IN_36XX)

/* RATE_IN_3430ES2PLUS_36XX includes 34xx/35xx with ES >=2, and all 36xx/37xx */
#define RATE_IN_3430ES2PLUS_36XX	(RATE_IN_3430ES2PLUS | RATE_IN_36XX)


/**
 * struct clksel_rate - register bitfield values corresponding to clk divisors
 * @val: register bitfield value (shifted to bit 0)
 * @div: clock divisor corresponding to @val
 * @flags: (see "struct clksel_rate.flags possibilities" above)
 *
 * @val should match the value of a read from struct clk.clksel_reg
 * AND'ed with struct clk.clksel_mask, shifted right to bit 0.
 *
 * @div is the divisor that should be applied to the parent clock's rate
 * to produce the current clock's rate.
 *
 * XXX @flags probably should be replaced with an struct omap_chip.
 */
struct clksel_rate {
	u32			val;
	u8			div;
	u8			flags;
};

/**
 * struct clksel - available parent clocks, and a pointer to their divisors
 * @parent: struct clk * to a possible parent clock
 * @rates: available divisors for this parent clock
 *
 * A struct clksel is always associated with one or more struct clks
 * and one or more struct clksel_rates.
 */
struct clksel {
	struct clk		 *parent;
	const struct clksel_rate *rates;
};

/**
 * struct dpll_data - DPLL registers and integration data
 * @mult_div1_reg: register containing the DPLL M and N bitfields
 * @mult_mask: mask of the DPLL M bitfield in @mult_div1_reg
 * @div1_mask: mask of the DPLL N bitfield in @mult_div1_reg
 * @clk_bypass: struct clk pointer to the clock's bypass clock input
 * @clk_ref: struct clk pointer to the clock's reference clock input
 * @control_reg: register containing the DPLL mode bitfield
 * @enable_mask: mask of the DPLL mode bitfield in @control_reg
 * @rate_tolerance: maximum variance allowed from target rate (in Hz)
 * @last_rounded_rate: cache of the last rate result of omap2_dpll_round_rate()
 * @last_rounded_m: cache of the last M result of omap2_dpll_round_rate()
 * @max_multiplier: maximum valid non-bypass multiplier value (actual)
 * @last_rounded_n: cache of the last N result of omap2_dpll_round_rate()
 * @min_divider: minimum valid non-bypass divider value (actual)
 * @max_divider: maximum valid non-bypass divider value (actual)
 * @modes: possible values of @enable_mask
 * @autoidle_reg: register containing the DPLL autoidle mode bitfield
 * @idlest_reg: register containing the DPLL idle status bitfield
 * @autoidle_mask: mask of the DPLL autoidle mode bitfield in @autoidle_reg
 * @freqsel_mask: mask of the DPLL jitter correction bitfield in @control_reg
 * @idlest_mask: mask of the DPLL idle status bitfield in @idlest_reg
 * @auto_recal_bit: bitshift of the driftguard enable bit in @control_reg
 * @recal_en_bit: bitshift of the PRM_IRQENABLE_* bit for recalibration IRQs
 * @recal_st_bit: bitshift of the PRM_IRQSTATUS_* bit for recalibration IRQs
 * @flags: DPLL type/features (see below)
 * @dpll_id: Integer value used for uniquely identifying an ADPLL
 * @div_m2n_reg: register containing the DPLL M2 and N bitfields
 * @div_m2_mask: mask of the DPLL M2 bitfield in @div_m2n_reg
 * @div_n_mask: mask of the DPLL N bitfield in @div_m2n_reg
 * @post_div_m2: fixed m2 value for this DPLL
 * @frac_mult_reg: register containing the fractional part of multiplier(m)
 * @frac_mult_mask: mask of the fractional part bitfield in @fract_mult_reg
 * @last_rounded_frac_m: cache of the last Fract M result of _round_rate()
 * @bypass_bit: bitshift for entering bypass mode in @control_reg
 * @byp_clk_src_bit:bitshift for selecting source clk in bypass @control_reg
 * @stby_ret_bit: bitshift for entering stand-by-ret mode in @control_reg
 * @stop_mode_bit: bitshift for entering stop-mode in @control_reg
 * @soft_reset_bit: bitshift for resetting the DPLL logic (TINITZ)@control_reg
 * @load_mn_reg: register for loading M and N values (TENABLE)
 * @load_m2n2_reg: register for loading M2 and N2 values (TENABLEDIV)
 * @dco_freq_sel: DCO FREQ selection mode for ADPLLLJs
 *
 * Possible values for @flags:
 * DPLL_J_TYPE: "J-type DPLL" (only some 36xx, 4xxx DPLLs)
 *
 * @freqsel_mask is only used on the OMAP34xx family and AM35xx.
 *
 * XXX Some DPLLs have multiple bypass inputs, so it's not technically
 * correct to only have one @clk_bypass pointer.
 *
 * XXX @rate_tolerance should probably be deprecated - currently there
 * don't seem to be any usecases for DPLL rounding that is not exact.
 *
 * XXX The runtime-variable fields (@last_rounded_rate, @last_rounded_m,
 * @last_rounded_n) should be separated from the runtime-fixed fields
 * and placed into a differenct structure, so that the runtime-fixed data
 * can be placed into read-only space.
 */
struct dpll_data {
	void __iomem	*mult_div1_reg;
	u32		mult_mask;
	u32		div1_mask;
	struct clk	*clk_bypass;
	struct clk	*clk_ref;
	void __iomem	*control_reg;
	u32		enable_mask;
	unsigned int	rate_tolerance;
	unsigned long	last_rounded_rate;
	u16		last_rounded_m;
	u16		max_multiplier;
	u8		last_rounded_n;
	u8		min_divider;
	u8		max_divider;
	u8		modes;
#if (defined(CONFIG_ARCH_OMAP3) ||\
	defined(CONFIG_ARCH_OMAP4) ||\
	defined(CONFIG_ARCH_TI81XX))
	void __iomem	*autoidle_reg;
	void __iomem	*idlest_reg;
	u32		autoidle_mask;
	u32		freqsel_mask;
	u32		idlest_mask;
	u32		dco_mask;
	u32		sddiv_mask;
	u8		auto_recal_bit;
	u8		recal_en_bit;
	u8		recal_st_bit;
	u8		flags;
	/* data fields required by TI814X ADPLLs */
	u8		dpll_id;
	void __iomem	*div_m2n_reg;
	u32		div_m2_mask;
	u32		div_n_mask;
	u8		pre_div_n;
	u8		post_div_m2;
	void __iomem	*frac_mult_reg;
	u32		frac_mult_mask;
	u32		last_rounded_frac_m;
	u8		last_rounded_m2;
	u32		bypass_bit;
	u8		byp_clk_src_bit;
	u8		stby_ret_bit;
	u8		stop_mode_bit;
	void __iomem	*load_mn_reg;
	void __iomem	*load_m2n2_reg;
	u8		dco_freq_sel;
#  endif
};

/**
 * struct fapll_data - FAPLL registers and integration data
 * @fapll_id: Identification number for FAPLL's
 * @control_reg: register containing the FAPLL N and P bitfields
 * @mult_mask: mask of the FAPLL N bitfield in @control_reg
 * @div_mask: mask of the FAPLL P bitfield in @control_reg
 * @bypass_mask: bypass mask of the FAPLL mode bitfield in @control_reg
 * @enable_mask: enable mask of the FAPLL mode bitfield in @control_reg
 * @lock_mask: lock mask of the FAPLL mode bitfield in @control_reg
 * @lock_sts_mask: lock status mask of the FAPLL mode bitfield in @control_reg
 * @pwd_reg: register containing the power down bitfields
 * @clk_bypass: struct clk pointer to the clock's bypass clock input
 * @clk_ref: struct clk pointer to the clock's reference clock input
 * @freq_frac_mask: mask of the FAPLL freq fracional part bitfield
 * @freq_int_mask: mask of the FAPLL freq integer part bitfield
 * @post_div_mask: mask of the FAPLL post divider bitfield
 * @ldfreq_mask: mask of the load freq value to synthesizer
 * @lddiv1_mask: mask to load M value to synthesizer
 * @trunc_mask: mask for enabling the truncate correction @freq_reg
 * @bypass_en: Enable value for checking the bypass mode of FAPLL
 * @modes: possible values of @enable_mask
 * @rate_tolerance: maximum variance allowed from target rate (in Hz)
 * @last_rounded_rate: cache of the last rate result of omap2_dpll_round_rate()
 * @last_rounded_m: cache of the last M result of ti816x_fapll_round_rate()
 * @last_rounded_freq_int: cache of the last freq integer result
 * @last_rounded_freq_frac: cache of the last freq fractional result
 * @mult_n: multiplier value for FAPLL (N)
 * @pre_div_p: pre divider value for FAPLL (P)
 * @max_multiplier: maximum valid non-bypass multiplier value (actual)
 * @min_divider: minimum valid non-bypass divider value (actual)
 * @max_divider: maximum valid non-bypass divider value (actual)
 *
 * XXX @rate_tolerance should probably be deprecated - currently there
 * don't seem to be any usecases for FAPLL rounding that is not exact.
 *
 * XXX The runtime-variable fields (@last_rounded_rate, @last_rounded_m,
 * @last_rounded_n) should be separated from the runtime-fixed fields
 * and placed into a differenct structure, so that the runtime-fixed data
 * can be placed into read-only space.
 */
struct fapll_data {
	int			fapll_id;
	void __iomem		*control_reg;
	u32			mult_mask;
	u32			div_mask;
	u32			bypass_mask;
	u32			enable_mask;
	u32			lock_mask;
	u32			lock_sts_mask;
	void __iomem		*pwd_reg;
	struct clk		*clk_bypass;
	struct clk		*clk_ref;
	u32			freq_frac_mask;
	u32			freq_int_mask;
	u32			post_div_mask;
	u32			ldfreq_mask;
	u32			lddiv1_mask;
	u32			trunc_mask;
	u32			bypass_en;
	u8			modes;
	unsigned int		rate_tolerance;
	unsigned long		last_rounded_rate;
	u8			last_rounded_m;
	u8			last_rounded_freq_int;
	u32			last_rounded_freq_frac;
	u16			mult_n;
	u8			pre_div_p;
	u16			max_multiplier;
	u8			min_divider;
	u8			max_divider;
};

#endif

/* struct clk.flags possibilities */
#define ENABLE_REG_32BIT	(1 << 0)	/* Use 32-bit access */
#define CLOCK_IDLE_CONTROL	(1 << 1)
#define CLOCK_NO_IDLE_PARENT	(1 << 2)
#define ENABLE_ON_INIT		(1 << 3)	/* Enable upon framework init */
#define INVERT_ENABLE		(1 << 4)	/* 0 enables, 1 disables */

/**
 * struct clk - OMAP struct clk
 * @node: list_head connecting this clock into the full clock list
 * @ops: struct clkops * for this clock
 * @name: the name of the clock in the hardware (used in hwmod data and debug)
 * @parent: pointer to this clock's parent struct clk
 * @children: list_head connecting to the child clks' @sibling list_heads
 * @sibling: list_head connecting this clk to its parent clk's @children
 * @rate: current clock rate
 * @enable_reg: register to write to enable the clock (see @enable_bit)
 * @recalc: fn ptr that returns the clock's current rate
 * @set_rate: fn ptr that can change the clock's current rate
 * @round_rate: fn ptr that can round the clock's current rate
 * @init: fn ptr to do clock-specific initialization
 * @enable_bit: bitshift to write to enable/disable the clock (see @enable_reg)
 * @usecount: number of users that have requested this clock to be enabled
 * @fixed_div: when > 0, this clock's rate is its parent's rate / @fixed_div
 * @flags: see "struct clk.flags possibilities" above
 * @clksel_reg: for clksel clks, register va containing src/divisor select
 * @clksel_mask: bitmask in @clksel_reg for the src/divisor selector
 * @clksel: for clksel clks, pointer to struct clksel for this clock
 * @dpll_data: for DPLLs, pointer to struct dpll_data for this clock
 * @clkdm_name: clockdomain name that this clock is contained in
 * @clkdm: pointer to struct clockdomain, resolved from @clkdm_name at runtime
 * @rate_offset: bitshift for rate selection bitfield (OMAP1 only)
 * @src_offset: bitshift for source selection bitfield (OMAP1 only)
 * @synthesizer_id: synthesizer id in a particular FAPLL
 * @pwd_mask: mask of the FAPLL to put all syn's in power down
 * @freq_flag: Flag for setting the freq part is persent or not
 * @fapll_data: for FAPLLs, pointer to struct fapll_data for this clock
 * @freq_reg: for synthesizer clks, output rate is based on this value
 * @post_div_reg: register containing M value of FAPLL
 *
 * XXX @rate_offset, @src_offset should probably be removed and OMAP1
 * clock code converted to use clksel.
 *
 * XXX @usecount is poorly named.  It should be "enable_count" or
 * something similar.  "users" in the description refers to kernel
 * code (core code or drivers) that have called clk_enable() and not
 * yet called clk_disable(); the usecount of parent clocks is also
 * incremented by the clock code when clk_enable() is called on child
 * clocks and decremented by the clock code when clk_disable() is
 * called on child clocks.
 *
 * XXX @clkdm, @usecount, @children, @sibling should be marked for
 * internal use only.
 *
 * @children and @sibling are used to optimize parent-to-child clock
 * tree traversals.  (child-to-parent traversals use @parent.)
 *
 * XXX The notion of the clock's current rate probably needs to be
 * separated from the clock's target rate.
 */
struct clk {
	struct list_head	node;
	const struct clkops	*ops;
	const char		*name;
	struct clk		*parent;
	struct list_head	children;
	struct list_head	sibling;	/* node for children */
	unsigned long		rate;
	void __iomem		*enable_reg;
	unsigned long		(*recalc)(struct clk *);
	int			(*set_rate)(struct clk *, unsigned long);
	long			(*round_rate)(struct clk *, unsigned long);
	void			(*init)(struct clk *);
	u8			enable_bit;
	s8			usecount;
	u8			fixed_div;
	u8			flags;
#ifdef CONFIG_ARCH_OMAP2PLUS
	void __iomem		*clksel_reg;
	u32			clksel_mask;
	const struct clksel	*clksel;
	struct dpll_data	*dpll_data;
	const char		*clkdm_name;
	struct clockdomain	*clkdm;
	u8			rate_offset;
	u8			src_offset;
#ifdef CONFIG_ARCH_TI81XX
	u32			synthesizer_id;
	u32			pwd_mask;
	u32			freq_flag;
	struct fapll_data	*fapll_data;
	void __iomem		*freq_reg;
	void __iomem		*post_div_reg;
	u32			div_def_val;
#endif
#endif
#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
	struct dentry		*dent;	/* For visible tree hierarchy */
#endif
};

struct cpufreq_frequency_table;

struct clk_functions {
	int		(*clk_enable)(struct clk *clk);
	void		(*clk_disable)(struct clk *clk);
	long		(*clk_round_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_parent)(struct clk *clk, struct clk *parent);
	void		(*clk_allow_idle)(struct clk *clk);
	void		(*clk_deny_idle)(struct clk *clk);
	void		(*clk_disable_unused)(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
	void		(*clk_init_cpufreq_table)(struct cpufreq_frequency_table **);
	void		(*clk_exit_cpufreq_table)(struct cpufreq_frequency_table **);
#endif
};

extern int mpurate;

extern int clk_init(struct clk_functions *custom_clocks);
extern void clk_preinit(struct clk *clk);
extern int clk_register(struct clk *clk);
extern void clk_reparent(struct clk *child, struct clk *parent);
extern void clk_unregister(struct clk *clk);
extern void propagate_rate(struct clk *clk);
extern void recalculate_root_clocks(void);
extern unsigned long followparent_recalc(struct clk *clk);
extern void clk_enable_init_clocks(void);
unsigned long omap_fixed_divisor_recalc(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
extern void clk_init_cpufreq_table(struct cpufreq_frequency_table **table);
extern void clk_exit_cpufreq_table(struct cpufreq_frequency_table **table);
#endif
extern struct clk *omap_clk_get_by_name(const char *name);

extern const struct clkops clkops_null;

extern struct clk dummy_ck;

#endif
