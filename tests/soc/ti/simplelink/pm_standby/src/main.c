/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LPF3 (CC23x0/CC27xx) standby smoke test.
 *
 * Verifies that the SoC actually reaches hardware standby when the kernel
 * idles, observed via the TI Power driver's AWAKE_STANDBY notification, and
 * that a stale RTC channel 0 state (interrupt mask left set with the channel
 * disarmed and an old compare value) does not prevent standby entry.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <ti/drivers/Power.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_rtc.h>

static Power_NotifyObj notify_obj;
static volatile uint32_t awake_standby_count;

static int_fast16_t awake_notify_cb(uint_fast16_t eventType, uintptr_t eventArg,
				    uintptr_t clientArg)
{
	ARG_UNUSED(eventType);
	ARG_UNUSED(eventArg);
	ARG_UNUSED(clientArg);

	awake_standby_count++;

	return Power_NOTIFYDONE;
}

static void *pm_standby_setup(void)
{
	Power_registerNotify(&notify_obj, PowerLPF3_AWAKE_STANDBY, awake_notify_cb, 0);

	/* Give the LF crystal time to settle; standby entry is gated on the
	 * LFINC filter (errata SYS_207 / CKM handling in the Power driver).
	 */
	k_sleep(K_SECONDS(2));

	return NULL;
}

ZTEST(pm_standby, test_standby_entered)
{
	awake_standby_count = 0;

	k_sleep(K_MSEC(500));

	zassert_true(awake_standby_count > 0,
		     "SoC did not enter standby during a 500 ms sleep");
}

ZTEST(pm_standby, test_standby_with_stale_rtc_imask)
{
	/* Emulate an RTC channel 0 user that armed a compare, was disarmed,
	 * but left IMASK set: compare value ~100 ms in the past (inside the
	 * 1 s immediate-event window, TRM 12.4.2), channel disarmed.
	 * The standby policy must key off the armed state; a policy keying
	 * off IMASK computes a zero delta from the stale compare value and
	 * falls back to WFI instead of standby for the whole sleep.
	 */
	HWREG(RTC_BASE + RTC_O_CH0CC8U) = HWREG(RTC_BASE + RTC_O_TIME8U) - 12500U;
	HWREG(RTC_BASE + RTC_O_ARMCLR) = RTC_ARMCLR_CH0_CLR;
	HWREG(RTC_BASE + RTC_O_ICLR) = RTC_ICLR_EV0_CLR;
	HWREG(RTC_BASE + RTC_O_IMSET) = RTC_IMSET_EV0_SET;

	awake_standby_count = 0;

	k_sleep(K_MSEC(500));

	/* Clean up before asserting */
	HWREG(RTC_BASE + RTC_O_IMCLR) = RTC_IMCLR_EV0_CLR;

	zassert_true(awake_standby_count > 0,
		     "standby was skipped because of stale RTC CH0 IMASK/compare state");
}

ZTEST_SUITE(pm_standby, NULL, pm_standby_setup, NULL, NULL, NULL);
