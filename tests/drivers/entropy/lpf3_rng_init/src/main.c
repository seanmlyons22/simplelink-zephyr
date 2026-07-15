/*
 * Copyright (c) 2026 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/ztest.h>

/*
 * Override the weak RCL wrapper symbol: report a failed noise capture while
 * leaving plausible-looking noise in the buffer (varied 5-bit arithmetic
 * codes with bits 15/31 clear, so the RCT/APT health tests pass).
 *
 * The entropy driver must reject the capture based on the returned status,
 * not on the buffer contents. Regression: the driver overwrote the RCL
 * status with the health-test result, so a failed capture whose residue
 * passed the health tests was hashed into the entropy pool as if valid.
 */
int_fast16_t RCL_AdcNoise_get_samples_blocking(uint32_t *buffer, uint32_t numWords)
{
	uint32_t x = 0x12345678;

	for (uint32_t i = 0; i < numWords; i++) {
		/* xorshift32 */
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		buffer[i] = x & 0x7FFF7FFF;
	}

	return -1; /* capture failed */
}

ZTEST(entropy_lpf3_rng_init, test_init_fails_on_rcl_error)
{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

	zassert_false(device_is_ready(dev),
		      "entropy device ready despite failed noise capture");
}

ZTEST_SUITE(entropy_lpf3_rng_init, NULL, NULL, NULL, NULL, NULL);
