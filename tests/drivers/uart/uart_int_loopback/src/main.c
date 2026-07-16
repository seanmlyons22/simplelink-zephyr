/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * Standalone interrupt-driven UART loopback integrity test.
 *
 * Drives ONLY the interrupt-driven API (uart_fifo_fill / uart_fifo_read /
 * uart_irq_*) on a TX->RX self-loopback: TX a sequential byte pattern, receive
 * it back, and assert every byte arrives in order with none lost. Deliberately
 * does NOT mix poll_out/async, so a failure isolates to the interrupt path.
 *
 * A sticky OVERRUN (uart_err_check) is tolerated as an expected hardware loss
 * without flow control (the 8-entry RX FIFO can overrun when the CPU is briefly
 * behind); the test then only requires forward progress + in-order data up to
 * the point of loss. With no overrun it enforces strict zero-loss.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>

#define DUT   DEVICE_DT_GET(DT_NODELABEL(uart0))
#define TOTAL 4096

static const struct device *const dut = DUT;
static uint8_t rx_buf[TOTAL];
static volatile size_t rx_count;
static volatile size_t tx_count;
static volatile int overruns;
static volatile bool order_ok = true;   /* exact sequential (strict, no loss) */
static volatile bool mono_ok = true;    /* strictly forward: tolerates drops, catches dup/reorder */
static volatile uint8_t prev_byte;
static struct k_sem done;

/* First few mismatches, to reveal the corruption pattern (dup/drop/bitflip). */
static volatile uint16_t mism_idx[10];
static volatile uint8_t mism_exp[10];
static volatile uint8_t mism_got[10];
static volatile int mism_n;

static void uart_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	/*
	 * Deliberately uses the common `while (uart_irq_update && is_pending)`
	 * idiom (same as h4.c and uart_mix_fifo_poll): it regression-guards that
	 * uart_irq_is_pending() surfaces TX-ready, so the TX bootstrap actually
	 * starts. A driver whose is_pending never reports the transition-triggered
	 * TX interrupt would leave this loop transmitting nothing (tx=0).
	 */
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[16];
			int n = uart_fifo_read(dev, buf, sizeof(buf));

			for (int i = 0; i < n; i++) {
				if (rx_count < TOTAL) {
					if (buf[i] != (uint8_t)(rx_count & 0xFF)) {
						if (mism_n < (int)ARRAY_SIZE(mism_idx)) {
							mism_idx[mism_n] = rx_count;
							mism_exp[mism_n] = rx_count & 0xFF;
							mism_got[mism_n] = buf[i];
							mism_n++;
						}
						order_ok = false;
					}
					/*
					 * Strictly-forward check: a clean byte advances the
					 * sawtooth by 1, an overrun DROP advances by >1 (still
					 * forward, tolerated), but a DUPLICATE (gap 0) or a
					 * REORDER (backward, large mod-256 gap) is real
					 * corruption. gap in [1,127] = forward.
					 */
					if (rx_count > 0) {
						uint8_t gap = (uint8_t)(buf[i] - prev_byte);

						if (gap == 0 || gap >= 128) {
							mono_ok = false;
						}
					}
					prev_byte = buf[i];
					rx_buf[rx_count++] = buf[i];
				}
			}
			if (uart_err_check(dev) & UART_ERROR_OVERRUN) {
				overruns++;
			}
			if (rx_count >= TOTAL) {
				k_sem_give(&done);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[8];
			int m = 0;

			while (m < (int)sizeof(buf) && (tx_count + m) < TOTAL) {
				buf[m] = (uint8_t)((tx_count + m) & 0xFF);
				m++;
			}
			if (m > 0) {
				tx_count += uart_fifo_fill(dev, buf, m);
			}
			if (tx_count >= TOTAL) {
				uart_irq_tx_disable(dev);
			}
		}
	}
}

ZTEST(uart_int_loopback, test_loopback)
{
	int ret;

	zassert_true(device_is_ready(dut), "UART device not ready");

	k_sem_init(&done, 0, 1);
	rx_count = 0;
	tx_count = 0;
	overruns = 0;
	order_ok = true;
	mono_ok = true;
	prev_byte = 0;
	mism_n = 0;

	uart_irq_callback_set(dut, uart_cb);
	uart_irq_rx_enable(dut);
	uart_irq_tx_enable(dut);

	ret = k_sem_take(&done, K_SECONDS(10));

	uart_irq_tx_disable(dut);
	uart_irq_rx_disable(dut);

	printk("int_loopback: tx=%zu rx=%zu overruns=%d order_ok=%d\n",
	       tx_count, rx_count, overruns, order_ok);
	for (int i = 0; i < mism_n; i++) {
		printk("  mismatch[%d]: idx=%u expected=0x%02x got=0x%02x\n",
		       i, mism_idx[i], mism_exp[i], mism_got[i]);
	}

	if (overruns == 0) {
		/* No flow control needed at this rate: strict zero-loss. */
		zassert_equal(ret, 0, "timed out: rx=%zu/%d (tx=%zu)", rx_count,
			      TOTAL, tx_count);
		zassert_equal(rx_count, TOTAL, "lost bytes: rx=%zu != %d", rx_count,
			      TOTAL);
		zassert_true(order_ok, "out-of-order / corrupted byte with no overrun");
	} else {
		/*
		 * Overruns => the RX FIFO dropped bytes (no flow control). Still
		 * require forward progress and that every byte we DID receive was
		 * in order (no silent corruption), just not the full count.
		 */
		printk("int_loopback: %d overrun(s) -> tolerating drops, checking no dup/reorder\n",
		       overruns);
		zassert_true(rx_count > 0, "no bytes received at all");
		zassert_true(mono_ok, "duplicate/reorder in RX stream (not explained by overrun drop)");
	}
}

ZTEST_SUITE(uart_int_loopback, NULL, NULL, NULL, NULL, NULL);
