#include <stdio.h>
#include <stdint.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#define BLINK_DELAY_MS	(500)

volatile uint32_t tickMs = 0;

void init() {
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &(GPIO_InitTypeDef){GPIO_Pin_9, GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
}

int main(void) {
	uint32_t nextBlink;
	uint32_t blinkState = 0;

	init();

	nextBlink = tickMs + BLINK_DELAY_MS;
	for(;;) {

		if(tickMs > nextBlink) {
			nextBlink = tickMs + BLINK_DELAY_MS;
			if(blinkState) {
				GPIO_SetBits(GPIOC, GPIO_Pin_9);
			} else {
				GPIO_ResetBits(GPIOC, GPIO_Pin_9);
			}
			blinkState ^= 1;
		}

		__WFI();
	}

	return 0;
}

void SysTick_Handler(void)
{
	tickMs++;
}
