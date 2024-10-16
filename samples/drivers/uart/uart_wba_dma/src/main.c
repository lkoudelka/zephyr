/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 256



static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	print_uart("UART RX interrupt");

}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

int main(void)
{
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	//int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	int ret = uart_callback_set (uart_dev, serial_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Error setting UART callback - Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("Error setting UART callback -UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}

	ret= uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf),  SYS_FOREVER_US);	//50 * USEC_PER_MSEC

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Error setting rx_enable: Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("Error setting rx_enable: UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting rx_enable: %d\n", ret);
		}
		return 0;
	}
//	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");

	/* indefinitely wait for input from the user */
	while (1){}
	return 0;
}
