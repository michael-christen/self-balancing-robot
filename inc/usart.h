#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include <stdint.h>

void usart_configure(uint32_t baud_rate);
char usart_block_receive_char();
void usart_send_char(char c);
void usart_send_string(const char *c_str);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __USART_H
