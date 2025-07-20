#ifndef __DEV_USART_H__
#define __DEV_USART_H__

#include <stdint.h>

int dev_usart_init(void);
int dev_usart_write_queue(const uint8_t* buf, uint16_t len);
void dev_usart_periodic(void);
int dev_usart_read();
void dev_usart_get_rx_buffer(uint8_t* buf, uint16_t* size);
uint32_t dev_usart_get_tx_overflow_count(void);

#endif