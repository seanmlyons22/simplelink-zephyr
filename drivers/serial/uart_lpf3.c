/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Copyright (c) 2024 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Unified UART driver for TI LPF3 family (CC23x0, CC27xx).
 * Both SOC lines share the same IP block and driverlib API.
 */

#define DT_DRV_COMPAT ti_lpf3_uart

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/arch/cpu.h>

#include <errno.h>

#include <driverlib/uart.h>
#include <driverlib/clkctl.h>

#include <inc/hw_memmap.h>

#ifdef CONFIG_PM
#include <ti/drivers/Power.h>
#endif

#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
#include <driverlib/udma.h>

#define UART_LPF3_REG_GET(base, offset) ((base) + (offset))
/*
 * Burst-only uDMA servicing, mirroring TI's UART2LPF3 SDK driver.
 *
 * Errata UDMA_01 (SWRZ161; the SDK applies it to the whole LPF3 family): when a
 * uDMA access is held up in the interconnect write buffers after an arbitration
 * loss, the peripheral can raise a spurious second single/burst request. If the
 * uDMA services single requests, this corrupts the stream (RX: a DR read races
 * the FIFO fill and returns torn data; TX: a write is dropped on a full FIFO).
 * The symptom is data corruption with the item count preserved and no error
 * flag, worsening with sustained throughput and baud rate.
 *
 * Workaround per TI: respond to burst requests only (UDMA_ATTR_USEBURST) with
 * the SDK-validated watermark/arbitration pairing:
 *   RX: IFLS 6/8 (burst request at >= 6 of 8 FIFO entries), arbitration 4
 *   TX: IFLS 2/8 (burst request at <= 2 filled / 6 empty), arbitration 2
 * With single requests disabled, a sub-watermark RX tail never raises a burst
 * request, so the CPU drains those stragglers on the RX-timeout interrupt
 * (UART_INT_RT), exactly like the SDK driver.
 */
#define UART_LPF3_RX_BURST_LEN 2
#define UART_LPF3_TX_BURST_LEN 2
#endif

struct uart_lpf3_config {
	uint32_t reg;
	uint32_t sys_clk_freq;
	uint32_t clkctl_id;
#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN
	unsigned int irq;
#endif
#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
	const struct device *dma_dev;
	uint8_t dma_channel_tx;
	uint8_t dma_trigsrc_tx;
	uint8_t dma_channel_rx;
	uint8_t dma_trigsrc_rx;
#endif
};

enum uart_lpf3_pm_locks {
	UART_LPF3_PM_LOCK_TX,
	UART_LPF3_PM_LOCK_RX,
	UART_LPF3_PM_LOCK_COUNT,
};

struct uart_lpf3_data {
	struct uart_config uart_config;
	const struct pinctrl_dev_config *pcfg;
#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN */
#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
	const struct device *dev;

	uart_callback_t async_callback;
	void *async_user_data;

	struct k_work_delayable tx_timeout_work;
	const uint8_t *tx_buf;
	size_t tx_len;

	uint8_t *rx_buf;
	size_t rx_len;
	size_t rx_processed_len;
	uint8_t *rx_next_buf;
	size_t rx_next_len;

	int32_t rx_timeout_us;
#endif /* CONFIG_UART_LPF3_DMA_DRIVEN */
#ifdef CONFIG_PM_DEVICE
	ATOMIC_DEFINE(pm_lock, UART_LPF3_PM_LOCK_COUNT);
#endif
#ifdef CONFIG_PM
	Power_NotifyObj pm_notify;
#endif
};

static inline void uart_lpf3_pm_policy_state_lock_get(struct uart_lpf3_data *data,
						       enum uart_lpf3_pm_locks pm_lock_type)
{
#ifdef CONFIG_PM_DEVICE
	if (!atomic_test_and_set_bit(data->pm_lock, pm_lock_type)) {
		pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
#endif
}

static inline void uart_lpf3_pm_policy_state_lock_put(struct uart_lpf3_data *data,
						       enum uart_lpf3_pm_locks pm_lock_type)
{
#ifdef CONFIG_PM_DEVICE
	if (atomic_test_and_clear_bit(data->pm_lock, pm_lock_type)) {
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
		pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	}
#endif
}

static int uart_lpf3_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_lpf3_config *config = dev->config;

	if (!UARTCharAvailable(config->reg)) {
		return -1;
	}

	*c = UARTGetCharNonBlocking(config->reg);

	return 0;
}

static void uart_lpf3_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_lpf3_config *config = dev->config;
	unsigned int key;

	/*
	 * Claim a TX FIFO slot atomically. UARTPutChar()'s "wait for space then
	 * write" is not atomic, so two concurrent poll_out() callers can both see
	 * the last free entry and both write it -- silently dropping one byte (a
	 * TX FIFO overflow is not flagged). Lock only around the check+write, not
	 * the wait, so interrupts are not disabled while spinning.
	 */
	while (true) {
		key = irq_lock();
		if (UARTSpaceAvailable(config->reg)) {
			UARTPutCharNonBlocking(config->reg, c);
			irq_unlock(key);
			break;
		}
		irq_unlock(key);
	}

#ifdef CONFIG_PM_DEVICE
	/* Wait for character to be transmitted to ensure CPU
	 * does not enter standby when UART is busy
	 */
	while (UARTBusy(config->reg)) {
	}
#endif
}

static int uart_lpf3_err_check(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;
	uint32_t flags = UARTGetRxError(config->reg);
	int error = 0;

	error |= (flags & UART_RXERROR_FRAMING) ? UART_ERROR_FRAMING : 0;
	error |= (flags & UART_RXERROR_PARITY) ? UART_ERROR_PARITY : 0;
	error |= (flags & UART_RXERROR_BREAK) ? UART_BREAK : 0;
	error |= (flags & UART_RXERROR_OVERRUN) ? UART_ERROR_OVERRUN : 0;

	UARTClearRxError(config->reg);

	return error;
}

static int uart_lpf3_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;
	uint32_t line_ctrl = 0;
	bool flow_ctrl;

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		line_ctrl |= UART_CONFIG_PAR_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		line_ctrl |= UART_CONFIG_PAR_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		line_ctrl |= UART_CONFIG_PAR_EVEN;
		break;
	case UART_CFG_PARITY_MARK:
		line_ctrl |= UART_CONFIG_PAR_ONE;
		break;
	case UART_CFG_PARITY_SPACE:
		line_ctrl |= UART_CONFIG_PAR_ZERO;
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		line_ctrl |= UART_CONFIG_STOP_ONE;
		break;
	case UART_CFG_STOP_BITS_2:
		line_ctrl |= UART_CONFIG_STOP_TWO;
		break;
	case UART_CFG_STOP_BITS_0_5:
	case UART_CFG_STOP_BITS_1_5:
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		line_ctrl |= UART_CONFIG_WLEN_5;
		break;
	case UART_CFG_DATA_BITS_6:
		line_ctrl |= UART_CONFIG_WLEN_6;
		break;
	case UART_CFG_DATA_BITS_7:
		line_ctrl |= UART_CONFIG_WLEN_7;
		break;
	case UART_CFG_DATA_BITS_8:
		line_ctrl |= UART_CONFIG_WLEN_8;
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		flow_ctrl = false;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		flow_ctrl = true;
		break;
	case UART_CFG_FLOW_CTRL_DTR_DSR:
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	/* Disables UART before setting control registers */
	UARTConfigSetExpClk(config->reg, config->sys_clk_freq, cfg->baudrate, line_ctrl);

	if (flow_ctrl) {
		UARTEnableCts(config->reg);
		UARTEnableRts(config->reg);
	} else {
		UARTDisableCts(config->reg);
		UARTDisableRts(config->reg);
	}

	/* Re-enable UART */
	UARTEnable(config->reg);

	/* Make use of the FIFO to reduce chances of data being lost */
	UARTEnableFifo(config->reg);

#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
	/* Watermarks paired with the uDMA arbitration sizes (see UDMA_01 note) */
	UARTSetFifoLevel(config->reg, UART_FIFO_TX2_8, UART_FIFO_RX6_8);
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
	/*
	 * Interrupt-driven RX: trigger the RX interrupt as early as possible
	 * (1/4 full = 2 of 8 entries) to maximize the drain headroom before an
	 * overrun on the shallow 8-entry FIFO when the ISR is delayed (e.g. by
	 * concurrent RF activity). The receive-timeout interrupt (UART_INT_RT)
	 * still flushes the sub-watermark tail.
	 */
	UARTSetFifoLevel(config->reg, UART_FIFO_TX4_8, UART_FIFO_RX2_8);
#endif

	data->uart_config = *cfg;

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_lpf3_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_lpf3_data *data = dev->data;

	*cfg = data->uart_config;
	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_lpf3_fifo_fill(const struct device *dev, const uint8_t *buf, int len)
{
	const struct uart_lpf3_config *config = dev->config;
	int n = 0;

	while (n < len) {
		if (!UARTSpaceAvailable(config->reg)) {
			break;
		}
		UARTPutCharNonBlocking(config->reg, buf[n]);
		n++;
	}

	return n;
}

static int uart_lpf3_fifo_read(const struct device *dev, uint8_t *buf, const int len)
{
	const struct uart_lpf3_config *config = dev->config;
	int c, n;

	n = 0;
	while (n < len) {
		if (!UARTCharAvailable(config->reg)) {
			break;
		}
		c = UARTGetCharNonBlocking(config->reg);
		buf[n++] = c;
	}

	return n;
}

static void uart_lpf3_irq_tx_enable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	/* When TX IRQ is enabled, it is implicit that we are expecting to transmit
	 * using the UART, hence we should no longer go into standby
	 */
	uart_lpf3_pm_policy_state_lock_get(dev->data, UART_LPF3_PM_LOCK_TX);

	UARTEnableInt(config->reg, UART_INT_TX);

	/* TI UART hardware doesn't generate TX interrupts when interrupts are enabled
	 * while the TX FIFO is already empty/ready. Interrupts only fire on state
	 * transitions (ready -> busy -> ready). If TX is currently ready, manually
	 * trigger the interrupt to start the transmission flow.
	 */
	if (UARTSpaceAvailable(config->reg)) {
		NVIC_SetPendingIRQ(config->irq);
	}
}

static void uart_lpf3_irq_tx_disable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	UARTDisableInt(config->reg, UART_INT_TX);

	uart_lpf3_pm_policy_state_lock_put(dev->data, UART_LPF3_PM_LOCK_TX);
}

static int uart_lpf3_irq_tx_ready(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	return UARTSpaceAvailable(config->reg) ? 1 : 0;
}

static void uart_lpf3_irq_rx_enable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	/* When RX IRQ is enabled, it is implicit that we are expecting to receive
	 * from the UART, hence we can no longer go into standby
	 */
	uart_lpf3_pm_policy_state_lock_get(dev->data, UART_LPF3_PM_LOCK_RX);

	/* Trigger the ISR on both RX and Receive Timeout. This is to allow
	 * the use of the hardware FIFOs for more efficient operation
	 */
	UARTEnableInt(config->reg, UART_INT_RX | UART_INT_RT);
}

static void uart_lpf3_irq_rx_disable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	UARTDisableInt(config->reg, UART_INT_RX | UART_INT_RT);

	uart_lpf3_pm_policy_state_lock_put(dev->data, UART_LPF3_PM_LOCK_RX);
}

static int uart_lpf3_irq_tx_complete(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	return UARTBusy(config->reg) ? 0 : 1;
}

static int uart_lpf3_irq_rx_ready(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	return UARTCharAvailable(config->reg) ? 1 : 0;
}

static void uart_lpf3_irq_err_enable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	return UARTEnableInt(config->reg, UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
}

static void uart_lpf3_irq_err_disable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	return UARTDisableInt(config->reg, UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
}

static int uart_lpf3_irq_is_pending(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;

	/* Read masked interrupt status */
	uint32_t status = UARTIntStatus(config->reg, true);

	return status ? 1 : 0;
}

static int uart_lpf3_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void uart_lpf3_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					void *user_data)
{
	struct uart_lpf3_data *data = dev->data;

	data->callback = cb;
	data->user_data = user_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if CONFIG_UART_LPF3_DMA_DRIVEN

static int uart_lpf3_async_callback_set(const struct device *dev, uart_callback_t callback,
					void *user_data)
{
	struct uart_lpf3_data *data = dev->data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->async_callback = NULL;
	data->async_user_data = NULL;
#else
	data->async_callback = callback;
	data->async_user_data = user_data;
#endif

	return 0;
}

static int uart_lpf3_async_tx(const struct device *dev, const uint8_t *buf, size_t len,
			      int32_t timeout)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;
	unsigned int key;
	int ret;

	struct dma_block_config block_cfg_tx = {
		.source_address = (uint32_t)buf,
		.dest_address = UART_LPF3_REG_GET(config->reg, UART_O_DR),
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.block_size = len,
	};

	struct dma_config dma_cfg_tx = {
		.dma_slot = config->dma_trigsrc_tx,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.block_count = 1,
		.head_block = &block_cfg_tx,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = UART_LPF3_TX_BURST_LEN,
		.dest_burst_length = UART_LPF3_TX_BURST_LEN,
		.dma_callback = NULL,
		.user_data = NULL,
	};

	/*
	 * Hold the transfer setup atomic against a concurrent tx_abort/timeout:
	 * otherwise tx_halt could stop the channel between dma_start and
	 * UARTEnableDMA and we would then re-start a transfer the caller was told
	 * was aborted. On any setup error, clear the TX state so a later uart_tx()
	 * is not wedged at -EBUSY.
	 */
	key = irq_lock();

	if (data->tx_len) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->tx_buf = buf;
	data->tx_len = len;

	ret = dma_config(config->dma_dev, config->dma_channel_tx, &dma_cfg_tx);
	if (ret) {
		goto err;
	}

	/* Respond to burst requests only (UDMA_01 workaround) */
	uDMAEnableChannelAttribute(BIT(config->dma_channel_tx), UDMA_ATTR_USEBURST);

	/* Disable DMA trigger */
	UARTDisableDMA(config->reg, UART_DMA_TX);

	/* Start DMA channel */
	ret = dma_start(config->dma_dev, config->dma_channel_tx);
	if (ret) {
		goto err;
	}

	/* Schedule timeout work */
	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&data->tx_timeout_work, K_USEC(timeout));
	}

	/* Lock PM */
	uart_lpf3_pm_policy_state_lock_get(data, UART_LPF3_PM_LOCK_TX);

	/*
	 * Clear any latched EOT so RIS.EOT reflects only THIS transfer. The ISR
	 * completes the transfer (fires TX_DONE) on EOT -- once the TX FIFO + line
	 * have fully drained -- so a chained 2nd uart_tx re-arms on an empty FIFO
	 * (see the TXDMADONE handler; serial-3 UDMA_01 chained-TX corruption).
	 */
	UARTClearInt(config->reg, UART_INT_EOT);

	/* Enable DMA trigger to start the transfer */
	UARTEnableDMA(config->reg, UART_DMA_TX);

	irq_unlock(key);

	return 0;

err:
	data->tx_buf = NULL;
	data->tx_len = 0;
	irq_unlock(key);
	return ret;
}

static int uart_lpf3_tx_halt(struct uart_lpf3_data *data)
{
	const struct uart_lpf3_config *config = data->dev->config;
	struct dma_status status;
	struct uart_event evt;
	size_t total_len;
	unsigned int key;

	key = irq_lock();

	total_len = data->tx_len;

	evt.type = UART_TX_ABORTED;
	evt.data.tx.buf = data->tx_buf;
	evt.data.tx.len = 0;

	data->tx_buf = NULL;
	data->tx_len = 0;

	dma_stop(config->dma_dev, config->dma_channel_tx);

	/*
	 * Mask the TX DMA request and clear any latched TXDMADONE: a completion
	 * that raced this abort must not fire into the next transfer. Compute the
	 * aborted length while still locked so a new uart_tx() cannot race the
	 * pending-count read.
	 */
	UARTDisableDMA(config->reg, UART_DMA_TX);
	UARTClearInt(config->reg, UART_INT_TXDMADONE);

	/*
	 * Also mask+clear EOT: a transfer waiting for its EOT completion
	 * (TXDMADONE seen, drain pending) must not fire TX_DONE after this abort.
	 */
	UARTDisableInt(config->reg, UART_INT_EOT);
	UARTClearInt(config->reg, UART_INT_EOT);

	if (dma_get_status(config->dma_dev, config->dma_channel_tx, &status) == 0) {
		evt.data.tx.len = total_len - status.pending_length;
	}

	irq_unlock(key);

	if (total_len) {
		if (data->async_callback) {
			data->async_callback(data->dev, &evt, data->async_user_data);
		}

		/* Unlock PM */
		uart_lpf3_pm_policy_state_lock_put(data, UART_LPF3_PM_LOCK_TX);
	} else {
		return -EINVAL;
	}

	return 0;
}

static void uart_lpf3_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_lpf3_data *data = CONTAINER_OF(dwork, struct uart_lpf3_data, tx_timeout_work);

	uart_lpf3_tx_halt(data);
}

static int uart_lpf3_async_tx_abort(const struct device *dev)
{
	struct uart_lpf3_data *data = dev->data;

	k_work_cancel_delayable(&data->tx_timeout_work);

	return uart_lpf3_tx_halt(data);
}

static int uart_lpf3_async_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				     int32_t timeout)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;
	struct uart_event evt;
	unsigned int key;
	int ret;

	struct dma_block_config block_cfg_rx = {
		.source_address = UART_LPF3_REG_GET(config->reg, UART_O_DR),
		.dest_address = (uint32_t)buf,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.block_size = len,
	};

	struct dma_config dma_cfg_rx = {
		.dma_slot = config->dma_trigsrc_rx,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.block_count = 1,
		.head_block = &block_cfg_rx,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = UART_LPF3_RX_BURST_LEN,
		.dest_burst_length = UART_LPF3_RX_BURST_LEN,
		.dma_callback = NULL,
		.user_data = NULL,
	};

	key = irq_lock();

	if (data->rx_len) {
		ret = -EBUSY;
		goto unlock;
	}

	ret = dma_config(config->dma_dev, config->dma_channel_rx, &dma_cfg_rx);
	if (ret) {
		goto unlock;
	}

	/* Respond to burst requests only (UDMA_01 workaround) */
	uDMAEnableChannelAttribute(BIT(config->dma_channel_rx), UDMA_ATTR_USEBURST);

	/* Disable DMA trigger */
	UARTDisableDMA(config->reg, UART_DMA_RX);

	/*
	 * Start clean: drop any residue left in the RX FIFO by a previous
	 * session (e.g. an aborted read) and clear latched errors, so the new
	 * transfer is not shifted by stale bytes.
	 */
	while (UARTCharAvailable(config->reg)) {
		(void)UARTGetCharNonBlocking(config->reg);
	}
	UARTClearRxError(config->reg);

	/* Start DMA channel */
	ret = dma_start(config->dma_dev, config->dma_channel_rx);
	if (ret) {
		goto unlock;
	}

	/* Lock PM */
	uart_lpf3_pm_policy_state_lock_get(data, UART_LPF3_PM_LOCK_RX);

	/* Enable DMA trigger to start the transfer */
	UARTEnableDMA(config->reg, UART_DMA_RX);

	/*
	 * With burst-only servicing (UDMA_01 workaround) a sub-watermark RX tail
	 * never raises a burst request, so bytes below the FIFO watermark are left
	 * in the FIFO. Scavenge them by CPU on the RX-timeout interrupt (UART_INT_RT,
	 * asserted after 32 idle bit times); that handler only moves bytes into the
	 * buffer, it never notifies, so it does not double-serve the poll below.
	 */
	UARTClearInt(config->reg, UART_INT_RT | UART_INT_OE);
	UARTEnableInt(config->reg, UART_INT_RT | UART_INT_OE);

	/* Surface an RX overrun to the application instead of dropping silently. */
	UARTClearRxError(config->reg);

	data->rx_buf = buf;
	data->rx_len = len;
	data->rx_processed_len = 0;
	data->rx_timeout_us = timeout;

	/*
	 * Delivery is driven inline by the RX-timeout (UART_INT_RT) interrupt,
	 * which fires only when the line goes idle. With burst-only servicing a
	 * partial buffer always strands a few sub-watermark bytes in the FIFO, so
	 * RT is guaranteed to fire for it; the ISR then delivers what has been
	 * received. Reading the DMA count only at idle (never mid-transfer) is
	 * what avoids the uDMA arbitration-loss torn reads a periodic poll caused
	 * (TRM 15.3.3; the TI SDK is likewise purely RT/DMADONE driven). Nothing
	 * to schedule here.
	 */

	/* Request next buffer */
	if (data->async_callback) {
		evt.type = UART_RX_BUF_REQUEST;

		data->async_callback(dev, &evt, data->async_user_data);
	}

unlock:
	irq_unlock(key);

	return ret;
}

static int uart_lpf3_async_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_lpf3_data *data = dev->data;
	unsigned int key;
	int ret = 0;

	key = irq_lock();

	if (data->rx_len == 0) {
		ret = -EACCES;
		goto unlock;
	}

	if (data->rx_next_len) {
		ret = -EBUSY;
		goto unlock;
	}

	data->rx_next_buf = buf;
	data->rx_next_len = len;

unlock:
	irq_unlock(key);

	return ret;
}

static void uart_lpf3_notify_rx_processed(struct uart_lpf3_data *data, size_t processed)
{
	struct uart_event evt;

	if (!data->async_callback || data->rx_processed_len == processed) {
		return;
	}

	evt.type = UART_RX_RDY;
	evt.data.rx.buf = data->rx_buf;
	evt.data.rx.offset = data->rx_processed_len;
	evt.data.rx.len = processed - data->rx_processed_len;

	data->rx_processed_len = processed;

	data->async_callback(data->dev, &evt, data->async_user_data);
}

static int uart_lpf3_async_rx_disable(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;
	struct dma_status status;
	struct uart_event evt;
	size_t rx_processed;
	unsigned int key;
	int ret = 0;

	key = irq_lock();

	if (data->rx_len == 0) {
		ret = -EINVAL;
		goto unlock;
	}

	UARTDisableInt(config->reg, UART_INT_RT | UART_INT_OE);

	dma_stop(config->dma_dev, config->dma_channel_rx);

	/*
	 * Mask the RX DMA request and clear any latched RXDMADONE/RT so a
	 * completion that raced this disable cannot fire into a subsequently
	 * re-enabled session (which would "complete" a brand-new empty buffer).
	 */
	UARTDisableDMA(config->reg, UART_DMA_RX);
	UARTClearInt(config->reg, UART_INT_RXDMADONE | UART_INT_RT);

	/* Unlock PM */
	uart_lpf3_pm_policy_state_lock_put(data, UART_LPF3_PM_LOCK_RX);

	if (dma_get_status(config->dma_dev, config->dma_channel_rx, &status) == 0) {
		rx_processed = data->rx_len - status.pending_length;

		uart_lpf3_notify_rx_processed(data, rx_processed);
	}

	if (data->async_callback) {
		evt.type = UART_RX_BUF_RELEASED;
		evt.data.rx_buf.buf = data->rx_buf;

		data->async_callback(dev, &evt, data->async_user_data);
	}

	data->rx_buf = NULL;
	data->rx_len = 0;

	if (data->rx_next_len) {
		if (data->async_callback) {
			evt.type = UART_RX_BUF_RELEASED;
			evt.data.rx_buf.buf = data->rx_next_buf;

			data->async_callback(dev, &evt, data->async_user_data);
		}

		data->rx_next_buf = NULL;
		data->rx_next_len = 0;
	}

	if (data->async_callback) {
		evt.type = UART_RX_DISABLED;

		data->async_callback(dev, &evt, data->async_user_data);
	}

unlock:
	irq_unlock(key);

	return ret;
}

/*
 * Complete the current RX buffer: deliver the remaining bytes, release the
 * buffer, then either chain to the next buffer or end reception. Must be called
 * with interrupts locked and data->rx_len != 0. Shared by the RXDMADONE
 * (buffer full) and RX-timeout straggler (buffer drained early) paths.
 */
static void uart_lpf3_rx_buf_complete(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;
	struct uart_event evt;

	uart_lpf3_notify_rx_processed(data, data->rx_len);

	if (data->async_callback) {
		evt.type = UART_RX_BUF_RELEASED;
		evt.data.rx.buf = data->rx_buf;

		data->async_callback(dev, &evt, data->async_user_data);
	}

	if (data->rx_next_len == 0) {
		/* If no next buffer, end the transfer */
		data->rx_buf = NULL;
		data->rx_len = 0;

		UARTDisableInt(config->reg, UART_INT_RT | UART_INT_OE);

		if (data->async_callback) {
			evt.type = UART_RX_DISABLED;

			data->async_callback(dev, &evt, data->async_user_data);
		}

		/* Unlock PM */
		uart_lpf3_pm_policy_state_lock_put(data, UART_LPF3_PM_LOCK_RX);
	} else {
		/* Otherwise, load next buffer and start the transfer */
		data->rx_buf = data->rx_next_buf;
		data->rx_len = data->rx_next_len;
		data->rx_next_buf = NULL;
		data->rx_next_len = 0;
		data->rx_processed_len = 0;

		/*
		 * Gate the UART DMA requests across the swap. The uDMA clears
		 * USEBURST on transfer completion, so it must be re-applied
		 * BEFORE the channel is enabled: otherwise, with RXDMAE still
		 * set and FIFO stragglers present, the channel services single
		 * requests for the few instructions until USEBURST is written --
		 * re-opening the UDMA_01 torn/duplicated-read window at every
		 * buffer boundary.
		 */
		UARTDisableDMA(config->reg, UART_DMA_RX);

		dma_reload(config->dma_dev, config->dma_channel_rx,
			   (uint32_t)UART_LPF3_REG_GET(config->reg, UART_O_DR),
			   (uint32_t)data->rx_buf, data->rx_len);

		uDMAEnableChannelAttribute(BIT(config->dma_channel_rx), UDMA_ATTR_USEBURST);

		dma_start(config->dma_dev, config->dma_channel_rx);

		UARTEnableDMA(config->reg, UART_DMA_RX);

		/* Request a new buffer */
		if (data->async_callback) {
			evt.type = UART_RX_BUF_REQUEST;

			data->async_callback(dev, &evt, data->async_user_data);
		}
	}
}

#endif /* CONFIG_UART_LPF3_DMA_DRIVEN */

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN

static void uart_lpf3_isr(const struct device *dev)
{
	struct uart_lpf3_data *data = dev->data;
#if CONFIG_UART_LPF3_DMA_DRIVEN
	const struct uart_lpf3_config *config = dev->config;
	struct uart_event evt;
	const uint8_t *tx_buf;
	size_t tx_len;
	unsigned int key;
	uint32_t int_status = UARTIntStatus(config->reg, true);
#endif

#if CONFIG_UART_INTERRUPT_DRIVEN
	if (data->callback) {
		data->callback(dev, data->user_data);
	}
#endif

#if CONFIG_UART_LPF3_DMA_DRIVEN
	/*
	 * When a peripheral channel is used (which is the case here for UART),
	 * the DMA transfer completion is signaled on the peripheral's interrupt only.
	 * It is not signaled on the DMA dedicated interrupt.
	 */
	if (int_status & UART_INT_TXDMADONE) {
		/*
		 * DMA has finished filling the TX FIFO, but bytes may still be in
		 * the FIFO/shift register. Defer TX_DONE to the EOT interrupt, which
		 * fires once the TX FIFO AND the TX line are fully drained (TRM 19.6
		 * RIS.EOT). This makes TX_DONE mean "transmitted" and, crucially,
		 * guarantees a 2nd uart_tx() chained from the callback re-arms the DMA
		 * on an EMPTY TX FIFO. Re-arming into the 1st transfer's still-full
		 * FIFO at high baud hits the UDMA_01 dropped-write window and garbles
		 * the 2nd buffer (serial-3). RIS.EOT was cleared in uart_lpf3_async_tx,
		 * so unmasking it now fires exactly once for this transfer -- either
		 * on a later ISR entry once drain completes, or right away if the
		 * short transfer already drained.
		 */
		UARTClearInt(config->reg, UART_INT_TXDMADONE);
		UARTEnableInt(config->reg, UART_INT_EOT);
	}

	if (int_status & UART_INT_EOT) {
		UARTDisableInt(config->reg, UART_INT_EOT);
		UARTClearInt(config->reg, UART_INT_EOT);

		k_work_cancel_delayable(&data->tx_timeout_work);

		key = irq_lock();

		tx_buf = data->tx_buf;
		tx_len = data->tx_len;

		data->tx_buf = NULL;
		data->tx_len = 0;

		/* Unlock PM */
		uart_lpf3_pm_policy_state_lock_put(data, UART_LPF3_PM_LOCK_TX);

		/*
		 * Clear the TX state before running the callback, so that a new
		 * uart_tx() chained from the UART_TX_DONE callback is not
		 * rejected with -EBUSY.
		 */
		if (tx_len && data->async_callback) {
			evt.type = UART_TX_DONE;
			evt.data.tx.buf = tx_buf;
			evt.data.tx.len = tx_len;

			data->async_callback(dev, &evt, data->async_user_data);
		}

		irq_unlock(key);
	}

	if (int_status & UART_INT_RXDMADONE) {
		UARTClearInt(config->reg, UART_INT_RXDMADONE);

		key = irq_lock();

		if (data->rx_len == 0) {
			/* RX already stopped by uart_lpf3_async_rx_disable() */
			irq_unlock(key);
			return;
		}

		uart_lpf3_rx_buf_complete(dev);

		irq_unlock(key);
	}

	if (int_status & UART_INT_RT) {
		UARTClearInt(config->reg, UART_INT_RT);

		key = irq_lock();

		if (data->rx_len != 0) {
			struct dma_status rx_stat;
			size_t pos;

			/*
			 * Single requests are disabled (UDMA_01 workaround), so
			 * bytes below the RX FIFO watermark never trigger the DMA.
			 * On RX idle, pause the DMA, drain the stragglers by CPU
			 * into the buffer, deliver everything received so far, then
			 * resume the DMA over the remainder. Delivering here is
			 * contention-free (idle, no burst in flight) and, unlike a
			 * deferred timer, cannot race an active transfer after a
			 * mid-stream buffer swap.
			 */
			UARTDisableDMA(config->reg, UART_DMA_RX);
			dma_stop(config->dma_dev, config->dma_channel_rx);

			if (dma_get_status(config->dma_dev, config->dma_channel_rx,
					   &rx_stat) == 0) {
				pos = data->rx_len - rx_stat.pending_length;

				while (pos < data->rx_len &&
				       UARTCharAvailable(config->reg)) {
					data->rx_buf[pos++] =
						UARTGetCharNonBlocking(config->reg);
				}

				if (pos == data->rx_len) {
					uart_lpf3_rx_buf_complete(dev);
				} else {
					/*
					 * Deliver the partial buffer on inactivity,
					 * unless the caller asked for no timeout
					 * (SYS_FOREVER: buffer-driven delivery only).
					 */
					if (data->rx_timeout_us != SYS_FOREVER_US) {
						uart_lpf3_notify_rx_processed(data, pos);
					}

					dma_reload(config->dma_dev,
						   config->dma_channel_rx,
						   (uint32_t)UART_LPF3_REG_GET(
							   config->reg, UART_O_DR),
						   (uint32_t)&data->rx_buf[pos],
						   data->rx_len - pos);

					/*
					 * Re-apply USEBURST (cleared by the uDMA
					 * on completion) BEFORE enabling the
					 * channel, so no single request is ever
					 * serviced (UDMA_01). RXDMAE is already
					 * gated off above and re-enabled below.
					 */
					uDMAEnableChannelAttribute(
						BIT(config->dma_channel_rx),
						UDMA_ATTR_USEBURST);
					dma_start(config->dma_dev,
						  config->dma_channel_rx);
				}
			}

			if (data->rx_len != 0) {
				UARTEnableDMA(config->reg, UART_DMA_RX);
			}
		}

		irq_unlock(key);
	}

	if (int_status & UART_INT_OE) {
		bool stopped = false;

		/* Clear the RX error interrupt latches (RSR is cleared via err_check). */
		UARTClearInt(config->reg,
			     UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);

		key = irq_lock();

		if (data->rx_len != 0) {
			struct dma_status rx_stat;
			size_t rx_count = data->rx_len;

			if (dma_get_status(config->dma_dev, config->dma_channel_rx,
					   &rx_stat) == 0) {
				rx_count = data->rx_len - rx_stat.pending_length;
			}

			if (data->async_callback) {
				evt.type = UART_RX_STOPPED;
				evt.data.rx_stop.reason = uart_lpf3_err_check(dev);
				evt.data.rx_stop.data.buf = data->rx_buf;
				evt.data.rx_stop.data.offset = data->rx_processed_len;
				evt.data.rx_stop.data.len = rx_count - data->rx_processed_len;

				data->async_callback(dev, &evt, data->async_user_data);
			}

			/* Delivered above with UART_RX_STOPPED; don't re-deliver on stop. */
			data->rx_processed_len = rx_count;
			stopped = true;
		}

		irq_unlock(key);

		/*
		 * An overrun desynchronizes the RX stream, so stop the transfer
		 * (release buffers + UART_RX_DISABLED); the application re-enables.
		 * Done outside the lock since rx_disable takes its own.
		 */
		if (stopped) {
			(void)uart_lpf3_async_rx_disable(dev);
		}
	}
#endif
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN */

static const struct uart_driver_api uart_lpf3_driver_api = {
	.poll_in = uart_lpf3_poll_in,
	.poll_out = uart_lpf3_poll_out,
	.err_check = uart_lpf3_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_lpf3_configure,
	.config_get = uart_lpf3_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_lpf3_fifo_fill,
	.fifo_read = uart_lpf3_fifo_read,
	.irq_tx_enable = uart_lpf3_irq_tx_enable,
	.irq_tx_disable = uart_lpf3_irq_tx_disable,
	.irq_tx_ready = uart_lpf3_irq_tx_ready,
	.irq_rx_enable = uart_lpf3_irq_rx_enable,
	.irq_rx_disable = uart_lpf3_irq_rx_disable,
	.irq_tx_complete = uart_lpf3_irq_tx_complete,
	.irq_rx_ready = uart_lpf3_irq_rx_ready,
	.irq_err_enable = uart_lpf3_irq_err_enable,
	.irq_err_disable = uart_lpf3_irq_err_disable,
	.irq_is_pending = uart_lpf3_irq_is_pending,
	.irq_update = uart_lpf3_irq_update,
	.irq_callback_set = uart_lpf3_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_LPF3_DMA_DRIVEN
	.callback_set = uart_lpf3_async_callback_set,
	.tx = uart_lpf3_async_tx,
	.tx_abort = uart_lpf3_async_tx_abort,
	.rx_enable = uart_lpf3_async_rx_enable,
	.rx_buf_rsp = uart_lpf3_async_rx_buf_rsp,
	.rx_disable = uart_lpf3_async_rx_disable,
#endif /* CONFIG_UART_LPF3_DMA_DRIVEN */
};

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN
#define UART_LPF3_IRQ_CFG(n)									\
	do {											\
		const struct uart_lpf3_config *config = dev->config;				\
												\
		UARTClearInt(config->reg, UART_INT_RX);						\
		UARTClearInt(config->reg, UART_INT_RT);						\
												\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_lpf3_isr,		\
			    DEVICE_DT_INST_GET(n), 0);						\
		irq_enable(DT_INST_IRQN(n));							\
	} while (false)

#define UART_LPF3_IRQ_INIT(n) .irq = DT_INST_IRQN(n),
#define UART_LPF3_INT_FIELDS .callback = NULL, .user_data = NULL,
#else
#define UART_LPF3_IRQ_CFG(n)
#define UART_LPF3_IRQ_INIT(n)
#define UART_LPF3_INT_FIELDS
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_LPF3_DMA_DRIVEN */

static int uart_lpf3_init_common(const struct device *dev)
{
	const struct uart_lpf3_config *config = dev->config;
	struct uart_lpf3_data *data = dev->data;

	CLKCTLEnable(CLKCTL_BASE, config->clkctl_id);

#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
	if (!device_is_ready(config->dma_dev)) {
		return -ENODEV;
	}

	UARTEnableInt(config->reg, UART_INT_TXDMADONE | UART_INT_RXDMADONE);

	/*
	 * One-time init only: init_common() also runs on PM resume, and
	 * re-initializing a possibly-scheduled work item is undefined. data->dev
	 * is zero-initialized, so a NULL here marks the very first call.
	 */
	if (data->dev == NULL) {
		k_work_init_delayable(&data->tx_timeout_work, uart_lpf3_async_tx_timeout);
		data->dev = dev;
	}
#endif

#ifdef CONFIG_PM_DEVICE
	atomic_clear_bit(data->pm_lock, UART_LPF3_PM_LOCK_RX);
	atomic_clear_bit(data->pm_lock, UART_LPF3_PM_LOCK_TX);
#endif

	/* Configure and enable UART */
	return uart_lpf3_configure(dev, &data->uart_config);
}

#ifdef CONFIG_PM
/*
 * Standby powers down the peripheral domain: the UART's non-retained registers
 * (baud, line control, FIFO, DMA/interrupt config) are lost and must be
 * reprogrammed on wakeup -- mirrors the TI SDK UART2LPF3 AWAKE_STANDBY notify.
 * The PM_DEVICE resume action only runs for system-managed device PM; this
 * covers the configs where the device is not suspended around standby (e.g.
 * CONFIG_PM_DEVICE_RUNTIME), where the console/DUT UART would otherwise be
 * dead after the first standby cycle. init_common() is the same restore path
 * PM_DEVICE_ACTION_RESUME uses (pinctrl/IOC is retained across standby).
 */
static int_fast16_t uart_lpf3_awake_notify(uint_fast16_t event_type, uintptr_t event_arg,
					   uintptr_t client_arg)
{
	ARG_UNUSED(event_type);
	ARG_UNUSED(event_arg);

	(void)uart_lpf3_init_common((const struct device *)client_arg);

	return Power_NOTIFYDONE;
}
#endif /* CONFIG_PM */

#define UART_LPF3_INIT_FUNC(n)									\
	static int uart_lpf3_init_##n(const struct device *dev)					\
	{											\
		struct uart_lpf3_data *data = dev->data;					\
		int ret;									\
												\
		ret = pinctrl_apply_state(data->pcfg, PINCTRL_STATE_DEFAULT);			\
		if (ret) {									\
			return ret;								\
		}										\
												\
		ret = uart_lpf3_init_common(dev);						\
		if (ret) {									\
			return ret;								\
		}										\
												\
		IF_ENABLED(CONFIG_PM, (						                \
			Power_registerNotify(&data->pm_notify, PowerLPF3_AWAKE_STANDBY,	        \
					     uart_lpf3_awake_notify, (uintptr_t)dev);))		\
												\
		/* Enable interrupts */								\
		UART_LPF3_IRQ_CFG(n);								\
												\
		return 0;									\
	}

#ifdef CONFIG_PM_DEVICE

static int uart_lpf3_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct uart_lpf3_config *config = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
		/*
		 * Refuse to suspend mid-transfer: UARTDisable() clears LCRH.FEN
		 * (flushing the RX FIFO) and drops DMACTL/IMSC, which would wedge
		 * an in-flight async transfer (no completion ever arrives).
		 */
		{
			struct uart_lpf3_data *data = dev->data;

			if (data->rx_len || data->tx_len) {
				return -EBUSY;
			}
		}
#endif
		UARTDisable(config->reg);
		CLKCTLDisable(CLKCTL_BASE, config->clkctl_id);
		return 0;
	case PM_DEVICE_ACTION_RESUME:
		return uart_lpf3_init_common(dev);
	default:
		return -ENOTSUP;
	}
}

#endif /* CONFIG_PM_DEVICE */

#ifdef CONFIG_UART_LPF3_DMA_DRIVEN
#define UART_LPF3_DMA_INIT(n)							\
	.dma_dev = DEVICE_DT_GET(TI_CC23X0_CC27XX_DT_INST_DMA_CTLR(n, tx)),	\
	.dma_channel_tx = TI_CC23X0_CC27XX_DT_INST_DMA_CHANNEL(n, tx),		\
	.dma_trigsrc_tx = TI_CC23X0_CC27XX_DT_INST_DMA_TRIGSRC(n, tx),		\
	.dma_channel_rx = TI_CC23X0_CC27XX_DT_INST_DMA_CHANNEL(n, rx),		\
	.dma_trigsrc_rx = TI_CC23X0_CC27XX_DT_INST_DMA_TRIGSRC(n, rx),
#else
#define UART_LPF3_DMA_INIT(n)
#endif /* CONFIG_UART_LPF3_DMA_DRIVEN */

#define UART_LPF3_DEVICE_DEFINE(n)								\
												\
	DEVICE_DT_INST_DEFINE(n, uart_lpf3_init_##n,						\
			      PM_DEVICE_DT_INST_GET(n),						\
			      &uart_lpf3_data_##n, &uart_lpf3_config_##n, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_lpf3_driver_api)

#define UART_LPF3_INIT(n)									\
	PINCTRL_DT_INST_DEFINE(n);								\
	PM_DEVICE_DT_INST_DEFINE(n, uart_lpf3_pm_action);					\
	UART_LPF3_INIT_FUNC(n);									\
												\
	static const struct uart_lpf3_config uart_lpf3_config_##n = {				\
		.reg = DT_INST_REG_ADDR(n),							\
		.sys_clk_freq = DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency),		\
		.clkctl_id = DT_INST_PROP(n, ti_clkctl_id),					\
		UART_LPF3_IRQ_INIT(n)								\
		UART_LPF3_DMA_INIT(n)};								\
												\
	static struct uart_lpf3_data uart_lpf3_data_##n = {					\
		.uart_config =									\
			{									\
				.baudrate = DT_INST_PROP(n, current_speed),			\
				.parity = UART_CFG_PARITY_NONE,					\
				.stop_bits = UART_CFG_STOP_BITS_1,				\
				.data_bits = UART_CFG_DATA_BITS_8,				\
				.flow_ctrl = DT_INST_PROP(n, hw_flow_control)			\
						? UART_CFG_FLOW_CTRL_RTS_CTS			\
						: UART_CFG_FLOW_CTRL_NONE,			\
			},									\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		UART_LPF3_INT_FIELDS};								\
												\
	UART_LPF3_DEVICE_DEFINE(n);

DT_INST_FOREACH_STATUS_OKAY(UART_LPF3_INIT)
