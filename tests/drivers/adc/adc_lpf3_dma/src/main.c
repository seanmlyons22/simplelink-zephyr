/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression test for adc_lpf3 DMA-driven 3-channel sequences (extra-6).
 *
 * A 3-channel sequence produces a burst length of 3 items. The cc23x0/cc27xx
 * DMA driver requires the burst (uDMA arbitration size) to be a power of two,
 * so dma_config() returned -EINVAL inside adc_context_start_sampling() (which
 * cannot report errors) and adc_read() then blocked forever on the completion
 * semaphore. With the burst rounded up to the next power of two, the read
 * completes. Input voltages are irrelevant: the test only asserts the read
 * returns, so no wiring is needed.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/ztest.h>

#define ADC_NODE DT_NODELABEL(adc0)

static const struct adc_dt_spec ch0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec ch1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec ch2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);

ZTEST(adc_lpf3_dma, test_three_channel_dma_read)
{
	uint16_t buf[3] = {0};
	struct adc_sequence seq = {
		.channels = BIT(ch0.channel_id) | BIT(ch1.channel_id) |
			    BIT(ch2.channel_id),
		.buffer = buf,
		.buffer_size = sizeof(buf),
		.resolution = ch0.resolution,
	};
	int err;

	zassert_true(adc_is_ready_dt(&ch0), "ADC device not ready");
	zassert_equal(adc_channel_setup_dt(&ch0), 0, "ch0 setup failed");
	zassert_equal(adc_channel_setup_dt(&ch1), 0, "ch1 setup failed");
	zassert_equal(adc_channel_setup_dt(&ch2), 0, "ch2 setup failed");

	/* Without the fix this never returns (config error -> semaphore hang),
	 * so the suite times out. With the fix it returns 0.
	 */
	err = adc_read(ch0.dev, &seq);
	zassert_equal(err, 0, "3-channel DMA read failed: %d", err);
}

ZTEST_SUITE(adc_lpf3_dma, NULL, NULL, NULL, NULL, NULL);
