/*
 * TI816X FAPLL clock functions
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/string.h>

#include <asm/div64.h>
#include <asm/io.h>
#include <plat/clock.h>

#include "clock.h"
#include "cm-regbits-81xx.h"

/* Macros */
/* Internal ref clock frequency range in (Hz) */
/* Modena PLL */
#define ADPLLS_FINT_BAND_MIN		32000
#define ADPLLS_FINT_BAND_MAX		52000000

/* ADPLLLJx */
#define ADPLLJ_FINT_BAND_MIN		500000
#define ADPLLJ_FINT_BAND_MAX		2500000

/* _dpll_test_fint() return codes */
#define ADPLL_FINT_UNDERFLOW		-1
#define ADPLL_FINT_INVALID		-2

/* Maximum multiplier & divider values for ADPLL */
/* Modena PLL */
#define TI814X_ADPLLS_MAX_MULT		2047
#define TI814X_ADPLLS_MAX_DIV		127

/* ADPLLLJx */
#define TI814X_ADPLLJ_MAX_MULT		4095
#define TI814X_ADPLLJ_MAX_DIV		255

#define TI814X_ADPLL_MIN_DIV		0
#define TI814X_ADPLL_MIN_MULT		2

/* Number of times to check the status register for a state change */
#define MAX_DPLL_WAIT_TRIES		100000

/* To prevent overflow during calculation */
#define ADPLL_ROUNDING_DIVIDER		10000

/* Private functions */

/* _ti814x_wait_dpll_status: wait for a DPLL to enter a specific state
 * @clk - Pointer to ADPLL clk structure
 * @state - State to enter
 */
static int _ti814x_wait_dpll_status(struct clk *clk, u32 state)
{
	const struct dpll_data *dd;
	int i = 0;
	int ret = -EINVAL;

	dd = clk->dpll_data;

	state <<= __ffs(dd->idlest_mask);

	while (((__raw_readl(dd->idlest_reg) & dd->idlest_mask) != state) &&
		i < MAX_DPLL_WAIT_TRIES) {
		i++;
		udelay(1);
	}
	if (i == MAX_DPLL_WAIT_TRIES) {
		pr_debug("clock: %s failed transition to '%s'\n",
			clk->name,
			(state == ST_ADPLL_BYPASSED) ? "bypassed" : "locked");
	} else {
		pr_debug("clock: %s transitioned to '%s'\n",
			clk->name,
			(state == ST_ADPLL_BYPASSED) ? "bypassed" : "locked");
		ret = 0;
	}
	return ret;
}

/*
 * _ti814x_dpll_bypass - instruct a DPLL to bypass and wait for readiness
 * @clk: pointer to an ADPLL struct clk
 *
 * Instructs a ADPLL to enter low-power bypass mode.
 * return -EINVAL.
 */
static int _ti814x_dpll_bypass(struct clk *clk)
{
	const struct dpll_data *dd = clk->dpll_data;
	u32 v;
	int r;

	if (!(dd->modes & (1 << ADPLL_LOW_POWER_BYPASS)))
		return -EINVAL;

	pr_debug("clock: configuring DPLL %s for low-power bypass\n",
			clk->name);

	/* Instruct ADPLL to enter Bypass mode 1 - bypass 0 - Active&Locked */
	v = __raw_readl(dd->control_reg);
	v |= (1 << dd->bypass_bit);
	__raw_writel(v, dd->control_reg);

	v = __raw_readl(dd->control_reg);
	v |= (1 << dd->soft_reset_bit);
	__raw_writel(v, dd->control_reg);

	r = _ti814x_wait_dpll_status(clk, ST_ADPLL_BYPASSED);

	return r;
}

/*
 * _ti814x_rel_dpll_bypass - instruct an ADPLL to exit from bypass
 * @clk: pointer to an ADPLL struct clk
 *
 * Instructs a ADPLL to enter low-power bypass mode.
 * return -EINVAL.
 */
static int _ti814x_rel_dpll_bypass(struct clk *clk)
{
	u32 v;
	const struct dpll_data *dd;
	dd = clk->dpll_data;

	/* Instruct ADPLL to exit Bypass mode 1 - bypass 0 - Active&Locked */
	v = __raw_readl(dd->control_reg);
	v &= ~(1 << dd->bypass_bit);
	__raw_writel(v, dd->control_reg);

	return 0;
}

/*
 * _ti814x_dpll_lock - instruct an ADPLL to lock and wait for readiness
 * @clk: pointer to an ADPLL struct clk
 *
 * Instructs an  ADPLL to lock.  Waits for the ADPLL to report
 * readiness before returning.
 */
static int _ti814x_dpll_lock(struct clk *clk)
{
	struct dpll_data *dd = clk->dpll_data;
	int r;

	pr_debug("clock: locking DPLL %s\n", clk->name);
	if (!(dd->modes & (1 << ADPLL_LOCKED)))
		return -EINVAL;

	_ti814x_rel_dpll_bypass(clk);

	r = _ti814x_wait_dpll_status(clk, ST_ADPLL_LOCKED);

	return r;
}

/* _ti814x_wait_dpll_ret_ack: wait for an ADPLL to acknowledge ret request
 * @clk - pointer to ADPLL struct clk
 *
 * Checks the SYSBYRETACK bit in ADPLL status register to be set
 * return -EINVAL
 */
static int _ti814x_wait_dpll_ret_ack(struct clk *clk)
{
	const struct dpll_data *dd = clk->dpll_data;
	int i = -1, v;
	int ret = -EINVAL;

	do {
		udelay(1);
		i++;
		v = (__raw_readl(dd->idlest_reg) &
			TI814X_ADPLL_STBYRET_ACK_MASK);
		v >>= __ffs(TI814X_ADPLL_STBYRET_ACK_MASK);
	} while ((v != 0x1) && i < MAX_DPLL_WAIT_TRIES);

	if (i == MAX_DPLL_WAIT_TRIES) {
		pr_debug("clock: %s failed to gate internal clocks\n",
			clk->name);
	} else {
		pr_debug("clock: %s internal clocks gated in %d loops\n",
			clk->name, i);
		ret = 0;
	}
	return ret;
}

/*
 * _ti814_dpll_stop - instruct a DPLL to stop
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs an ADPLL to enter low-power stop mode.
 */
static int _ti814_dpll_stop(struct clk *clk)
{
	struct dpll_data *dd = clk->dpll_data;
	u32 v;
	int r = 0;

	if (!(dd->modes & (1 << ADPLL_LOW_POWER_STOP)))
		return -EINVAL;

	pr_debug("clock: stopping DPLL %s\n", clk->name);

	/* DPLL must be in retention before entering stop mode */
	v = __raw_readl(dd->control_reg);
	v |= (1 << dd->stby_ret_bit);
	__raw_writel(v, dd->control_reg);


	/* Read and wait for retention ack from PLL_STATUS register */
	r = _ti814x_wait_dpll_ret_ack(clk);
	if (r) {
		pr_debug("clock: %s DPLL could not be placed in retention\n",
			clk->name);

		/* Return from retention as Ack not received */
		v = __raw_readl(dd->control_reg);
		v &= ~(1 << dd->stby_ret_bit);
		__raw_writel(v, dd->control_reg);
		return -EINVAL;
	}
	if (dd->flags & TI814X_ADPLL_LS_TYPE) {

		/* Enter DPLL stop mode */
		v = __raw_readl(dd->control_reg);
		v |= (1 << dd->stop_mode_bit);
		__raw_writel(v, dd->control_reg);
	}
	return 0;
}

/*
 * _adpll_compute_new_rate - compute and return the rate with given values
 * @ref_rate: Input frequency
 * @m,frac_m,n,m2 - Multiplier and divider values to use for rate calculation
 * @return - Computed rate
 */
static unsigned long _adpll_compute_new_rate(unsigned long ref_rate,
			unsigned int m, unsigned int frac_m,
			unsigned int n, unsigned int m2)
{
	unsigned long long num;
	unsigned long long den;

	num = (unsigned long long)((m << 18) + frac_m);
	num = (unsigned long long) (ref_rate * num);
	den = (unsigned long long)(((n+1) << 18)*m2);

	do_div(num, den);

	return num;
}

/*
 * _adpll_test_fint - test whether an Fint(REFCLK) value is valid
 * @clk: ADPLL struct clk to test
 * @n: divider value (N) to test
 *
 * Tests whether a particular divider @n will result in a valid ADPLL
 * internal clock frequency Fint(REFCLK).
 * @return - 0 if Fint valid else UNDERFLOW/INVALID
 */
static int _adpll_test_fint(struct clk *clk, u8 n)
{
	struct dpll_data *dd = clk->dpll_data;
	long fint;
	int ret = 0;

	/* DPLL divider must result in a valid jitter correction val */
	fint = clk->parent->rate / (n + 1);
	if (dd->flags & TI814X_ADPLL_LS_TYPE) {
		if (fint < ADPLLS_FINT_BAND_MIN) {

			pr_debug("rejecting n=%d due to Fint failure\n", n);
			ret = ADPLL_FINT_UNDERFLOW;
		} else if (fint > ADPLLS_FINT_BAND_MAX) {

			pr_debug("rejecting n=%d due to Fint failure\n", n);
			ret = ADPLL_FINT_INVALID;

		}
	} else {
		if (fint < ADPLLJ_FINT_BAND_MIN) {

			pr_debug("rejecting n=%d due to Fint failure\n", n);
			dd->max_divider = n;
			ret = ADPLL_FINT_UNDERFLOW;
		} else if (fint > ADPLLJ_FINT_BAND_MAX) {

			pr_debug("rejecting n=%d due to Fint failure\n", n);
			dd->min_divider = n;
			ret = ADPLL_FINT_INVALID;

		}

	}

	return ret;
}

/*
 *_dpll_get_rounded_vals - Compute rounded values for multiplier(M) and
 * fractional multiplier to get the targeted rate
 * @clk - pointer to ADPLL clk structure
 * @target_rate - Rate to be achieved
 *
 * Compute and return the values, return error if not possible to get the
 * rate with in the multiplier limits
 */
static int _dpll_get_rounded_vals(struct clk *clk, unsigned long target_rate)
{
	int i, m;
	int r = 0;
	struct dpll_data *dd;
	unsigned long inp_rate;
	unsigned int remainder, quotient;
	unsigned int frac_overflow;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	dd->last_rounded_n = dd->pre_div_n;
	inp_rate = (clk->parent->rate / (dd->post_div_m2));
	inp_rate = inp_rate/ADPLL_ROUNDING_DIVIDER;
	target_rate = target_rate/ADPLL_ROUNDING_DIVIDER;
	quotient = (target_rate*(dd->pre_div_n + 1)) / inp_rate;
	remainder = (target_rate*(dd->pre_div_n + 1)) % inp_rate;

	if ((remainder == 0) && (clk->flags & TI814X_ADPLL_LS_TYPE)) {
		for (i = TI814X_ADPLL_MIN_MULT; i <= dd->max_multiplier; i++) {
			if ((quotient % i) == 0) {
				dd->last_rounded_m = i;
				dd->last_rounded_frac_m = 0;
				break;
			}
		}
	} else {
		m = quotient;
		if ((m < TI814X_ADPLL_MIN_MULT) || (m > dd->max_multiplier))
			return -EINVAL;
		dd->last_rounded_m = m;
		dd->last_rounded_frac_m = (((remainder << 18))/inp_rate);
		frac_overflow = dd->last_rounded_frac_m >> 18;
		if (frac_overflow > 0) {
			dd->last_rounded_m = dd->last_rounded_m + frac_overflow;
			dd->last_rounded_frac_m = dd->last_rounded_frac_m &
						TI814X_ADPLL_FRACT_MULT_MASK;
		}
	}
	return r;
}

/* Public functions */

/*
 * ti814x_init_dpll_parent - Initialise an ADPLL's parent
 * @clk - pointer to an ADPLL clk structure
 *
 * Clock needs to be in bypass mode before changing the parent
 */
void ti814x_init_dpll_parent(struct clk *clk)
{
	u32 v;
	struct dpll_data *dd = clk->dpll_data;

	if (!dd)
		return;
	/* Check dpll status */
	v = __raw_readl(dd->control_reg);
	v &= (1 << dd->bypass_bit);
	v >>= dd->bypass_bit;

	/* Reparent in case the dpll is in bypass */
	if (v == 1)
		clk_reparent(clk, dd->clk_bypass);
	return;
}

/**
 * ti814x_get_dpll_rate - returns the current ADPLL CLKOUT rate
 * @clk: pointer to  an ADPLL clk structure
 *
 * ADPLLs can be locked or bypassed - basically, enabled or disabled.
 * Read the multiplier an ddivider values from registers to compute rate
 */
u32 ti814x_get_dpll_rate(struct clk *clk)
{
	unsigned long long dpll_rate;
	unsigned long long num, den;
	u32 m, frac_m, n, m2, v, ctrl;
	struct dpll_data *dd = clk->dpll_data;

	if (!dd)
		return 0;

	/* Return bypass rate if DPLL is bypassed or stbyret/stopmode */
	ctrl = __raw_readl(dd->control_reg);
	v = (ctrl & (1 << dd->bypass_bit)) | (ctrl & (1 << dd->stby_ret_bit));

	if (v)
		return dd->clk_bypass->rate;

	v = __raw_readl(dd->mult_div1_reg);
	m = v & dd->mult_mask;
	m >>= __ffs(dd->mult_mask);

	v = __raw_readl(dd->frac_mult_reg);
	frac_m = v & dd->frac_mult_mask;
	frac_m >>= __ffs(dd->frac_mult_mask);

	num = (unsigned long long)((m << 18) + frac_m);

	v = __raw_readl(dd->div_m2n_reg);
	n = v & dd->div_n_mask;
	n >>= __ffs(dd->div_n_mask);
	m2 = v & dd->div_m2_mask;
	m2 >>= __ffs(dd->div_m2_mask);

	den = (unsigned long long)(((n+1) << 18)*m2);
	dpll_rate = (unsigned long long)(dd->clk_ref->rate * num);
	do_div(dpll_rate, den);

	return dpll_rate;
}

/**
 * ti814x_dpll_recalc - recalculate ADPLL rate
 * @clk: ADPLL struct clk
 *
 * Recalculate and propagate the FAPLL rate.
 */
unsigned long ti814x_dpll_recalc(struct clk *clk)
{
	return ti814x_get_dpll_rate(clk);

}

/*
 * ti814x_dpll_enable - Enable an ADPLL
 * @clk - pointer to struct clk of ADPLL
 */
int ti814x_dpll_enable(struct clk *clk)
{
	int r;
	struct dpll_data *dd;
	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	if (clk->rate == dd->clk_bypass->rate) {

		WARN_ON(clk->parent != dd->clk_bypass);
		r = _ti814x_dpll_bypass(clk);
	} else {
		WARN_ON(clk->parent != dd->clk_ref);
		r = _ti814x_dpll_lock(clk);
	}
	if (!r)
		clk->rate = ti814x_get_dpll_rate(clk);
	return r;
}

/**
 * ti814x_dpll_disable - Disable an ADPLL( enter low-power stop)
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs an ADPLL to enter low-power stop.  This function is
 * intended for use in struct clkops.  No return value.
 */
void ti814x_dpll_disable(struct clk *clk)
{

	_ti814_dpll_stop(clk);
}

/*
 * ti814x_dpll_program - Program an ADPLL with given multiplier an div values
 * @clk - pointer to struct clk of ADPLL
 * @m - multiplier M
 * @frac_m - Fractional part of the multiplier
 * @n - pre divider N
 * @m2 - Post divider M2
 *
 * ADPLL must be in bypass while programming with new values
 */
static int ti814x_dpll_program(struct clk *clk, u16 m, u32 frac_m, u8 n, u8 m2)
{
	struct dpll_data *dd = clk->dpll_data;
	u32 v;

	if (!dd)
		return -EINVAL;

	_ti814x_dpll_bypass(clk);

	/* Set DPLL multiplier (m)*/
	v = __raw_readl(dd->mult_div1_reg);
	v &= ~(dd->mult_mask);
	v |= m << __ffs(dd->mult_mask);
	__raw_writel(v, dd->mult_div1_reg);

	/* Set DPLL Fractional multiplier (f) */
	v = __raw_readl(dd->frac_mult_reg);
	v &= ~(dd->frac_mult_mask);
	v |= frac_m << __ffs(dd->frac_mult_mask);
	__raw_writel(v, dd->frac_mult_reg);

	/* Set DPLL pre-divider (n) and post-divider (m2)*/
	v = __raw_readl(dd->div_m2n_reg);
	v &= ~(dd->div_m2_mask | dd->div_n_mask);
	v |= m2 << __ffs(dd->div_m2_mask);
	v |= n << __ffs(dd->div_n_mask);
	__raw_writel(v, dd->div_m2n_reg);

	/* Assert TENABLE to load M and N values */
	v = __raw_readl(dd->load_mn_reg);
	v |= (1 << TI814X_ADPLL_LOAD_MN_SHIFT);
	__raw_writel(v, dd->load_mn_reg);

	v = __raw_readl(dd->load_m2n2_reg);
	v |= (1 << TI814X_ADPLL_LOAD_M2N2_SHIFT);
	__raw_writel(v, dd->load_m2n2_reg);

	_ti814x_dpll_lock(clk);

	return 0;
}

/**
 * ti816x_dpll_round_rate - round a target rate for an ADPLL
 * @clk: struct clk * for a dpll
 * @target_rate: desired ADPLL clock rate
 *
 * Given a dpll, a desired target rate, and a rate tolerance, round
 * the target rate to a possible, programmable rate for this adpll.
 * Rate tolerance is assumed to be set by the caller before this
 * function is called. Attempts to select the minimum possible n
 * within the tolerance to reduce power consumption. Stores the
 * computed (m, n) in the adpll's dpll_data structure so set_rate()
 * will not need to call this (expensive) function again. Returns ~0
 * if the target rate cannot be rounded,if the rate is
 * too low  or the rounded rate upon success.
 */
long ti814x_dpll_round_rate(struct clk *clk, unsigned long target_rate)
{
	struct dpll_data *dd;
	int ret;

	if (!clk || !clk->dpll_data)
		return ~0;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	pr_debug("clock: starting dpll round_rate for clock %s, target rate "
					"%ld\n", clk->name, target_rate);
	dd->last_rounded_rate = 0;

	ret = _dpll_get_rounded_vals(clk, target_rate);
	if (ret) {
		pr_err("Failed to get the rounded values\n");
		return ret;
	}

	dd->last_rounded_rate = _adpll_compute_new_rate(dd->clk_ref->rate,
				dd->last_rounded_m,
				dd->last_rounded_frac_m,
				dd->last_rounded_n,
				dd->post_div_m2);

	return dd->last_rounded_rate;
}

/**
 * ti814x_dpll_set_rate - set an ADPLL rate
 * @clk: struct clk * of ADPLL to set
 * @rate: rounded target rate
 *
 * Set the ADPLL CLKOUT to the target rate.  If the ADPLL can enter
 * low-power bypass, and the target rate is the bypass source clock
 * rate, then configure the ADPLL for bypass.  Otherwise, round the
 * target rate if it hasn't been done already, then program and lock
 * the DPLL.  Returns -EINVAL upon error, or 0 upon success.
 */
int ti814x_dpll_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *new_parent = NULL;
	struct dpll_data *dd;
	int ret;

	if (!clk || !rate)
		return -EINVAL;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	if (rate == ti814x_get_dpll_rate(clk))
		return 0;
	ret = _adpll_test_fint(clk, dd->pre_div_n);
	if (ret)
		return -EINVAL;

	/*
	 * Ensure both the bypass and ref clocks are enabled prior to
	 * doing anything; we need the bypass clock running to reprogram
	 * the DPLL.
	 */
	omap2_clk_enable(dd->clk_bypass);
	omap2_clk_enable(dd->clk_ref);

	if ((dd->clk_bypass->rate == rate) &&
		(clk->dpll_data->modes & (1 << ADPLL_LOW_POWER_BYPASS))) {
		pr_debug("clock: %s: set rate: entering bypass.\n", clk->name);

		ret = _ti814x_dpll_bypass(clk);
		if (!ret)
			new_parent = dd->clk_bypass;
	} else {
		if (dd->last_rounded_rate != rate)
			ti814x_dpll_round_rate(clk, rate);

		if (dd->last_rounded_rate == 0)
			return -EINVAL;

		pr_debug("clock: %s: set rate: locking rate to %lu.\n",
			clk->name, dd->last_rounded_rate);

		ret = ti814x_dpll_program(clk,
					dd->last_rounded_m,
					dd->last_rounded_frac_m,
					dd->last_rounded_n,
					dd->post_div_m2);
		if (!ret)
			new_parent = dd->clk_ref;
	}
	if (!ret) {
		/*
		 * Switch the parent clock in the hierarchy, and make sure
		 * that the new parent's usecount is correct.  Note: we
		 * enable the new parent before disabling the old to avoid
		 * any unnecessary hardware disable->enable transitions.
		 */
		if (clk->usecount) {
			omap2_clk_enable(new_parent);
			omap2_clk_disable(clk->parent);
		}
		clk_reparent(clk, new_parent);
		clk->rate = dd->last_rounded_rate;
	}
	omap2_clk_disable(dd->clk_ref);
	omap2_clk_disable(dd->clk_bypass);

	return 0;
}
