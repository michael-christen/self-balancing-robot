#include <stdio.h>
#include <stdint.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#define BLINK_DELAY_US	(20)

volatile uint32_t tickUs = 0;

void init() {
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000000)) {
		// Capture error
		while (1){};
	}
}

typedef struct stepper_t {
    uint32_t next_step;
    uint8_t state;
    uint32_t step_delay;
    GPIO_TypeDef *gpio_port;
    uint32_t gpio_pin;
} stepper_t;


stepper_t stepper_init(GPIO_TypeDef *gpio_port, uint32_t gpio_pin, uint32_t step_delay) {
    stepper_t this_stepper = {
        .next_step = tickUs + step_delay,
        .state = 0,
        .step_delay = step_delay,
        .gpio_port = gpio_port,
        .gpio_pin = gpio_pin,
    };
    return this_stepper;
}

void next_action(stepper_t *stepper) {
	if(tickUs > stepper->next_step) {
        stepper->next_step = stepper->step_delay + tickUs;
		if(stepper->state) {
			GPIO_SetBits(stepper->gpio_port, stepper->gpio_pin);
		} else {
			GPIO_ResetBits(stepper->gpio_port, stepper->gpio_pin);
		}
		stepper->state ^= 1;
	}
}


int main(void) {

	init();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(
        GPIOC,
        &(GPIO_InitTypeDef){
            GPIO_Pin_9,
            GPIO_Mode_OUT,
            GPIO_Speed_2MHz,
            GPIO_OType_PP,
            GPIO_PuPd_NOPULL
        });
    stepper_t stepper0 = stepper_init(GPIOC, GPIO_Pin_9, BLINK_DELAY_US);
	for(;;) {
        next_action(&stepper0);
		__WFI();
	}

	return 0;
}

void SysTick_Handler(void)
{
	tickUs++;
}
