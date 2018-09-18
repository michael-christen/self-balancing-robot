#include <stdio.h>
#include <stdint.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#define BLINK_DELAY_US	(50)
#define false (0)
#define true (1)
#define bool uint8_t

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
    uint32_t step_pin;
    uint32_t dir_pin;
} stepper_t;

void set_dir(stepper_t *stepper, bool forward);

stepper_t stepper_init(
                GPIO_TypeDef *gpio_port,
                uint32_t step_pin, uint32_t dir_pin, uint32_t step_delay, bool forward) {
    stepper_t this_stepper = {
        .next_step = tickUs + step_delay,
        .state = 0,
        .step_delay = step_delay,
        .gpio_port = gpio_port,
        .step_pin = step_pin,
        .dir_pin = dir_pin,
    };
    set_dir(&this_stepper, forward);
    return this_stepper;
}

void set_dir(stepper_t *stepper, bool forward) {
    if (forward) {
        GPIO_SetBits(stepper->gpio_port, stepper->dir_pin);
    } else {
        GPIO_ResetBits(stepper->gpio_port, stepper->dir_pin);
    }
}

void next_action(stepper_t *stepper) {
	if(tickUs > stepper->next_step) {
        stepper->next_step = stepper->step_delay + tickUs;
		if(stepper->state) {
			GPIO_SetBits(stepper->gpio_port, stepper->step_pin);
		} else {
			GPIO_ResetBits(stepper->gpio_port, stepper->step_pin);
		}
		stepper->state ^= 1;
	}
}

void set_speed_and_dir(stepper_t *stepper, uint32_t step_delay, bool forward) {
    set_dir(stepper, forward);
    // Only update delay on next cycle
    stepper->step_delay = step_delay;
}


int main(void) {

	init();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(
        GPIOC,
        &(GPIO_InitTypeDef){
            GPIO_Pin_9 | GPIO_Pin_8,
            GPIO_Mode_OUT,
            GPIO_Speed_2MHz,
            GPIO_OType_PP,
            GPIO_PuPd_NOPULL
        });
    bool forward = true;
    uint32_t last_count = 0;
    stepper_t stepper0 = stepper_init(GPIOC, GPIO_Pin_9, GPIO_Pin_8, BLINK_DELAY_US, forward);
	for(;;) {
        if (tickUs / 1000000 > last_count) {
          last_count += 1;
          forward = !forward;
          set_dir(&stepper0, forward);
        }
        next_action(&stepper0);
		__WFI();
	}

	return 0;
}

void SysTick_Handler(void)
{
	tickUs++;
}
