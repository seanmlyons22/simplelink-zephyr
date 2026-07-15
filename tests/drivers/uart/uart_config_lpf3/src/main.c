/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * LPF3 UART runtime line-configuration test.
 *
 * Exercises uart_configure() across parity / stop-bit / data-bit / baud
 * combinations. For each: verifies uart_config_get() round-trips the setting,
 * then loops a byte back DIO20->DIO22 to confirm the line actually frames and
 * receives correctly at that config (a wrong parity/stop/data setup would
 * corrupt the byte or raise a framing/parity error). Test bytes are 7-bit-safe
 * so the same values are valid under the 7-data-bit config.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>

#define DUT DEVICE_DT_GET(DT_NODELABEL(uart0))

static bool loopback(const struct device *dev, uint8_t tx)
{
	int64_t end = k_uptime_get() + 100;
	uint8_t rx;

	while (uart_poll_in(dev, &rx) == 0) {
	}
	uart_poll_out(dev, tx);
	while (k_uptime_get() < end) {
		if (uart_poll_in(dev, &rx) == 0) {
			return rx == tx;
		}
	}
	return false;
}

static void try_config(const struct device *dev, const struct uart_config *cfg,
		       const char *name)
{
	struct uart_config got;

	zassert_ok(uart_configure(dev, cfg), "configure(%s) failed", name);
	zassert_ok(uart_config_get(dev, &got), "config_get(%s) failed", name);
	zassert_equal(got.baudrate, cfg->baudrate, "%s: baud %u != %u", name,
		      got.baudrate, cfg->baudrate);
	zassert_equal(got.parity, cfg->parity, "%s: parity mismatch", name);
	zassert_equal(got.stop_bits, cfg->stop_bits, "%s: stop_bits mismatch", name);
	zassert_equal(got.data_bits, cfg->data_bits, "%s: data_bits mismatch", name);

	/* 7-bit-safe test bytes so they are valid under UART_CFG_DATA_BITS_7 too. */
	zassert_true(loopback(dev, 0x3C), "%s: loopback 0x3C failed", name);
	zassert_true(loopback(dev, 0x55), "%s: loopback 0x55 failed", name);
	zassert_true(loopback(dev, 0x00), "%s: loopback 0x00 failed", name);
	printk("uart_config_lpf3: %s OK\n", name);
}

ZTEST(uart_config_lpf3, test_line_configs)
{
	const struct device *dev = DUT;
	struct uart_config cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	zassert_true(device_is_ready(dev), "UART not ready");

	try_config(dev, &cfg, "8N1-115200");

	cfg.parity = UART_CFG_PARITY_EVEN;
	try_config(dev, &cfg, "8E1");

	cfg.parity = UART_CFG_PARITY_ODD;
	try_config(dev, &cfg, "8O1");

	cfg.parity = UART_CFG_PARITY_NONE;
	cfg.stop_bits = UART_CFG_STOP_BITS_2;
	try_config(dev, &cfg, "8N2");

	cfg.stop_bits = UART_CFG_STOP_BITS_1;
	cfg.data_bits = UART_CFG_DATA_BITS_7;
	cfg.parity = UART_CFG_PARITY_EVEN;
	try_config(dev, &cfg, "7E1");

	cfg.data_bits = UART_CFG_DATA_BITS_8;
	cfg.parity = UART_CFG_PARITY_NONE;
	cfg.baudrate = 9600;
	try_config(dev, &cfg, "8N1-9600");

	cfg.baudrate = 1000000;
	try_config(dev, &cfg, "8N1-1M");

	printk("uart_config_lpf3: all line configs OK\n");
}

ZTEST_SUITE(uart_config_lpf3, NULL, NULL, NULL, NULL, NULL);
