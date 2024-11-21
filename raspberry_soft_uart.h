#ifndef RASPBERRY_SOFT_UART_H
#define RASPBERRY_SOFT_UART_H

#include <linux/tty.h>

int raspberry_soft_uart_init(const int gpio_tx, const int gpio_rx, const int invert, const int invert_tx, const int invert_rx, const int stop_bits, const int break_tx, const int break_rx);
int raspberry_soft_uart_finalize(void);
int raspberry_soft_uart_open(struct tty_struct* tty);
int raspberry_soft_uart_close(void);
int raspberry_soft_uart_set_baudrate(const int baudrate);
int raspberry_soft_uart_send_string(const unsigned char* string, int string_size);
int raspberry_soft_uart_get_tx_queue_room(void);
int raspberry_soft_uart_get_tx_queue_size(void);
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char));

#endif
