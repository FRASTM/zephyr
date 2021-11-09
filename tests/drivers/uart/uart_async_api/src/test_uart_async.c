/*
 *  Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_uart.h"

K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(tx_aborted, 0, 1);
K_SEM_DEFINE(rx_rdy, 0, 1);
K_SEM_DEFINE(rx_buf_released, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

ZTEST_BMEM volatile bool failed_in_isr;
ZTEST_BMEM static const struct device *uart_dev;

#if CONFIG_NOCACHE_MEMORY
/* When dma is activated, the tx_buf and rx_buf used for uart transfers
 * are reserved in the non-cached memory area, to avoid cache manipulation
 */
#endif

void init_test(void)
{
	uart_dev = device_get_binding(UART_DEVICE_NAME);
}

#ifdef CONFIG_USERSPACE
void set_permissions(void)
{
	k_thread_access_grant(k_current_get(), &tx_done, &tx_aborted,
			      &rx_rdy, &rx_buf_released, &rx_disabled,
			      uart_dev);
}
#endif

void test_single_read_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		(*(uint32_t *)user_data)++;
		break;
	case UART_RX_RDY:
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

ZTEST_BMEM volatile uint32_t tx_aborted_count;

void test_single_read_setup(void)
{
	uart_callback_set(uart_dev,
			  test_single_read_callback,
			  (void *) &tx_aborted_count);
}

void test_single_read(void)
{
#if CONFIG_NOCACHE_MEMORY
	const uint8_t TX_BUF[] = "test";

	static __aligned(32) uint8_t tx_buf[5] __used
		__attribute__((__section__(".sram_dma")));
	static __aligned(32) uint8_t rx_buf[10] __used
		__attribute__((__section__(".sram_dma")));

	memset(tx_buf, 0, sizeof(tx_buf));
	memcpy(tx_buf, TX_BUF, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));
#else
	uint8_t rx_buf[10] = {0};
	uint8_t tx_buf[5] = "test";
#endif

	zassert_not_equal(memcmp(tx_buf, rx_buf, 5), 0,
			  "Initial buffer check failed");

	uart_rx_enable(uart_dev, rx_buf, 10, 50 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");

	uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");


	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");
	zassert_not_equal(memcmp(tx_buf, rx_buf+5, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(1000)), 0,
		      "RX_DISABLED timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");

	zassert_equal(memcmp(tx_buf, rx_buf+5, 5), 0, "Buffers not equal");
	zassert_equal(tx_aborted_count, 0, "TX aborted triggered");
}

#if CONFIG_NOCACHE_MEMORY
static __aligned(32) uint8_t chained_read_buf0[10] __used
	__attribute__((__section__(".sram_dma")));
static __aligned(32) uint8_t chained_read_buf1[20] __used
	__attribute__((__section__(".sram_dma")));
static __aligned(32) uint8_t chained_read_buf2[30] __used
	__attribute__((__section__(".sram_dma")));
#else
/* this src memory shall be in RAM to support using as a DMA source pointer.*/
ZTEST_BMEM uint8_t chained_read_buf0[10];
ZTEST_BMEM uint8_t chained_read_buf1[20];
ZTEST_BMEM uint8_t chained_read_buf2[30];
#endif

ZTEST_DMEM uint8_t buf_num = 1U;
ZTEST_BMEM uint8_t *read_ptr;
ZTEST_BMEM volatile size_t read_len;

void test_chained_read_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		read_ptr = evt->data.rx.buf + evt->data.rx.offset;
		read_len = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_REQUEST:
		if (buf_num == 1U) {
			uart_rx_buf_rsp(uart_dev,
					chained_read_buf1,
					20);
			buf_num = 2U;
		} else if (buf_num == 2U) {
			uart_rx_buf_rsp(uart_dev,
					chained_read_buf2,
					30);
			buf_num = 0U;
		}
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

void test_chained_read_setup(void)
{
	uart_callback_set(uart_dev, test_chained_read_callback, NULL);
}

void test_chained_read(void)
{
#if CONFIG_NOCACHE_MEMORY
	static __aligned(32) uint8_t tx_buf[10] __used
		__attribute__((__section__(".sram_dma")));

	memset(tx_buf, 0, sizeof(tx_buf));
#else
	uint8_t tx_buf[10];
#endif

	uart_rx_enable(uart_dev, chained_read_buf0, 10, 50 * USEC_PER_MSEC);

	for (int i = 0; i < 6; i++) {
		zassert_not_equal(k_sem_take(&rx_disabled, K_MSEC(10)),
				  0,
				  "RX_DISABLED occurred");
		snprintf(tx_buf, sizeof(tx_buf), "Message %d", i);
		uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
		zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0,
			      "TX_DONE timeout");
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(1000)), 0,
			      "RX_RDY timeout");
		size_t read_len_temp = read_len;

		zassert_equal(read_len_temp, 10,
			      "Incorrect read length");
		zassert_equal(memcmp(tx_buf, read_ptr, 10),
			      0,
			      "Buffers not equal");
	}
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}
#if CONFIG_NOCACHE_MEMORY
static __aligned(32) uint8_t double_buffer[2][12] __used
	__attribute__((__section__(".sram_dma")));
static uint8_t *next_buf = double_buffer[1];
#else
ZTEST_BMEM uint8_t double_buffer[2][12];
ZTEST_DMEM uint8_t *next_buf = double_buffer[1];
#endif

void test_double_buffer_callback(const struct device *uart_dev,
				 struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		read_ptr = evt->data.rx.buf + evt->data.rx.offset;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_REQUEST:
		uart_rx_buf_rsp(uart_dev, next_buf, sizeof(double_buffer[0]));
		break;
	case UART_RX_BUF_RELEASED:
		next_buf = evt->data.rx_buf.buf;
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

void test_double_buffer_setup(void)
{
	uart_callback_set(uart_dev, test_double_buffer_callback, NULL);
}

void test_double_buffer(void)
{
#if CONFIG_NOCACHE_MEMORY
static __aligned(32) uint8_t tx_buf[4] __used
	__attribute__((__section__(".sram_dma")));
#else
	uint8_t tx_buf[4];
#endif

	zassert_equal(uart_rx_enable(uart_dev,
				     double_buffer[0],
				     sizeof(double_buffer[0]),
				     50 * USEC_PER_MSEC),
		      0,
		      "Failed to enable receiving");

	for (int i = 0; i < 100; i++) {
		snprintf(tx_buf, sizeof(tx_buf), "%03d", i);
		uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
		zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0,
			      "TX_DONE timeout");
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0,
			      "RX_RDY timeout");
		zassert_equal(memcmp(tx_buf, read_ptr, 4),
			      0,
			      "Buffers not equal");
	}
	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}

void test_read_abort_callback(const struct device *dev,
			      struct uart_event *evt, void *user_data)
{
	int err;

	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		err = k_sem_take(&rx_rdy, K_NO_WAIT);
		failed_in_isr |= (err < 0);
		break;
	case UART_RX_DISABLED:
		err = k_sem_take(&rx_buf_released, K_NO_WAIT);
		failed_in_isr |= (err < 0);
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

void test_read_abort_setup(void)
{
	failed_in_isr = false;
	uart_callback_set(uart_dev, test_read_abort_callback, NULL);

	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_buf_released);
	k_sem_reset(&rx_disabled);
	k_sem_reset(&tx_done);
}

void test_read_abort(void)
{
#if CONFIG_NOCACHE_MEMORY
	static __aligned(32) uint8_t tx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
	static __aligned(32) uint8_t rx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
#else
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];
#endif

	memset(rx_buf, 0, sizeof(rx_buf));
	memset(tx_buf, 1, sizeof(tx_buf));

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, tx_buf, 5, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, 95, 100 * USEC_PER_MSEC);

	/* Wait for at least one character. RX_RDY event will be generated only
	 * if there is pending data.
	 */
	k_busy_wait(1000);

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
	zassert_false(failed_in_isr, "Unexpected order of uart events");
	zassert_not_equal(memcmp(tx_buf, rx_buf, 100), 0, "Buffers equal");

	/* Read out possible other RX bytes
	 * that may affect following test on RX
	 */
	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);
	while (k_sem_take(&rx_rdy, K_MSEC(1000)) != -EAGAIN)
		;
	uart_rx_disable(uart_dev);
}

ZTEST_BMEM volatile size_t sent;
ZTEST_BMEM volatile size_t received;

void test_write_abort_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

void test_write_abort_setup(void)
{
	uart_callback_set(uart_dev, test_write_abort_callback, NULL);
}

void test_write_abort(void)
{
#if CONFIG_NOCACHE_MEMORY
	static __aligned(32) uint8_t tx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
	static __aligned(32) uint8_t rx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
#else
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];
#endif

	memset(rx_buf, 0, 100);
	memset(tx_buf, 1, 100);

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, tx_buf, 5, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");

	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, 95, 100 * USEC_PER_MSEC);
	uart_tx_abort(uart_dev);
	zassert_equal(k_sem_take(&tx_aborted, K_MSEC(100)), 0,
		      "TX_ABORTED timeout");
	if (sent != 0) {
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0,
			      "RX_RDY timeout");
		zassert_equal(sent, received, "Sent is not equal to received.");
	}
	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}


void test_forever_timeout_callback(const struct device *dev,
				   struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

void test_forever_timeout_setup(void)
{
	uart_callback_set(uart_dev, test_forever_timeout_callback, NULL);
}

void test_forever_timeout(void)
{
#if CONFIG_NOCACHE_MEMORY
	static __aligned(32) uint8_t tx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
	static __aligned(32) uint8_t rx_buf[100] __used
		__attribute__((__section__(".sram_dma")));
#else
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];
#endif

	memset(rx_buf, 0, 100);
	memset(tx_buf, 1, 100);

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), SYS_FOREVER_US);

	uart_tx(uart_dev, tx_buf, 5, SYS_FOREVER_US);
	zassert_not_equal(k_sem_take(&tx_aborted, K_MSEC(1000)), 0,
			  "TX_ABORTED timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_not_equal(k_sem_take(&rx_rdy, K_MSEC(1000)), 0,
			  "RX_RDY timeout");

	uart_tx(uart_dev, tx_buf, 95, SYS_FOREVER_US);

	zassert_not_equal(k_sem_take(&tx_aborted, K_MSEC(1000)), 0,
			  "TX_ABORTED timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");


	zassert_equal(memcmp(tx_buf, rx_buf, 100), 0, "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}

ZTEST_DMEM bool chained_write_next_buf = true;
ZTEST_BMEM volatile uint8_t tx_sent;
#if CONFIG_NOCACHE_MEMORY
static const uint8_t TX_DATA0[] = "Message 1";
static const uint8_t TX_DATA1[] = "Message 2";
static __aligned(32) uint8_t chained_write_tx_bufs[2][10] __used
	__attribute__((__section__(".sram_dma")));
#else
ZTEST_DMEM uint8_t chained_write_tx_bufs[2][10] = {"Message 1", "Message 2"};
#endif

void test_chained_write_callback(const struct device *uart_dev,
				 struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		if (chained_write_next_buf) {
			uart_tx(uart_dev, chained_write_tx_bufs[1], 10, 100 * USEC_PER_MSEC);
			chained_write_next_buf = false;
		}
		tx_sent = 1;
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

void test_chained_write_setup(void)
{
	uart_callback_set(uart_dev, test_chained_write_callback, NULL);
}

void test_chained_write(void)
{
#if CONFIG_NOCACHE_MEMORY
	static __aligned(32) uint8_t rx_buf[20] __used
		__attribute__((__section__(".sram_dma")));

	memset(chained_write_tx_bufs[0], 0, sizeof(chained_write_tx_bufs[0]));
	memcpy(chained_write_tx_bufs[0], TX_DATA0, sizeof(TX_DATA0));
	memset(chained_write_tx_bufs[1], 0, sizeof(chained_write_tx_bufs[1]));
	memcpy(chained_write_tx_bufs[1], TX_DATA1, sizeof(TX_DATA1));
#else
	uint8_t rx_buf[20];
#endif

	memset(rx_buf, 0, 20);

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, chained_write_tx_bufs[0], 10, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(chained_write_next_buf, false, "Sent no message");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(memcmp(chained_write_tx_bufs[0], rx_buf, 10),
		      0,
		      "Buffers not equal");
	zassert_equal(memcmp(chained_write_tx_bufs[1], rx_buf + 10, 10),
		      0,
		      "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}
#if CONFIG_NOCACHE_MEMORY
static __aligned(32) uint8_t long_rx_buf[1024] __used
	__attribute__((__section__(".sram_dma")));
static __aligned(32) uint8_t long_rx_buf2[1024] __used
	__attribute__((__section__(".sram_dma")));
static __aligned(32) uint8_t long_tx_buf[1000] __used
	__attribute__((__section__(".sram_dma")));
#else
ZTEST_BMEM uint8_t long_rx_buf[1024];
ZTEST_BMEM uint8_t long_rx_buf2[1024];
ZTEST_BMEM uint8_t long_tx_buf[1000];
#endif
ZTEST_BMEM volatile uint8_t evt_num;
ZTEST_BMEM size_t long_received[2];

void test_long_buffers_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	static bool next_buf = true;

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		long_received[evt_num] = evt->data.rx.len;
		evt_num++;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	case UART_RX_BUF_REQUEST:
		if (next_buf) {
			uart_rx_buf_rsp(uart_dev, long_rx_buf2, 1024);
			next_buf = false;
		}
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

void test_long_buffers_setup(void)
{
	uart_callback_set(uart_dev, test_long_buffers_callback, NULL);
}

void test_long_buffers(void)
{
	memset(long_rx_buf, 0, 1024);
	memset(long_tx_buf, 1, 1000);

	uart_rx_enable(uart_dev, long_rx_buf, sizeof(long_rx_buf), 10 * USEC_PER_MSEC);

	uart_tx(uart_dev, long_tx_buf, 500, 200 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(200)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(long_received[0], 500, "Wrong number of bytes received.");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf, 500),
		      0,
		      "Buffers not equal");

	evt_num = 0;
	uart_tx(uart_dev, long_tx_buf, 1000, 200 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(200)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(long_received[0], 524, "Wrong number of bytes received.");
	zassert_equal(long_received[1], 476, "Wrong number of bytes received.");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf + 500, long_received[0]),
		      0,
		      "Buffers not equal");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf2, long_received[1]),
		      0,
		      "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}
