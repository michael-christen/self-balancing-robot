/*
 * TODO:
 * - configure interrupts to fill and dump buffers
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

void usart_configure(uint32_t baud_rate) {
	USART_InitTypeDef USART_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	//Configure USART2 pins:  Rx and Tx ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Configure USART2 setting:         ----------------------------
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2,ENABLE);
}

char usart_block_receive_char() {
	// Wait to receive a character
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USART2);
}

char usart_nonblock_receive_char() {
	if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
	    return USART_ReceiveData(USART2);
    } else {
        return (char) 0;
    }
}

float usart_block_receive_float() {
    char c_str[256];
    uint8_t idx = 0;
    for(;;) {
        char c = usart_block_receive_char();
        c_str[idx++] = c;
        if (c == '$') {
            c_str[idx-1] = '\0';
            break;
        }
    }
    return atof(c_str);
}

void usart_send_char(char c) {
	// Wait until we can send a character
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, c);
}

void usart_send_string(const char *c_str) {
	while(*c_str) {
		usart_send_char(*c_str++);
	}
}
