#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/stepper.h"


stepper_t stepper_init(
    GPIO_TypeDef *gpio_port,
    uint32_t step_pin,
    uint32_t dir_pin,
    uint32_t step_delay,
    bool forward,
    uint32_t current_ticks) {
    stepper_t this_stepper = {
        .next_step = current_ticks + step_delay,
        .state = 0,
        .step_delay = step_delay,
        .gpio_port = gpio_port,
        .step_pin = step_pin,
        .dir_pin = dir_pin,
    };
    stepper_set_dir(&this_stepper, forward);
    return this_stepper;
}


void stepper_set_dir(stepper_t *stepper, bool forward) {
    if (forward) {
        GPIO_SetBits(stepper->gpio_port, stepper->dir_pin);
    } else {
        GPIO_ResetBits(stepper->gpio_port, stepper->dir_pin);
    }
}

void stepper_next_action(stepper_t *stepper, uint32_t current_ticks) {
	if(current_ticks > stepper->next_step) {
        stepper->next_step = stepper->step_delay + current_ticks;
		if(stepper->state) {
			GPIO_SetBits(stepper->gpio_port, stepper->step_pin);
		} else {
			GPIO_ResetBits(stepper->gpio_port, stepper->step_pin);
		}
		stepper->state ^= 1;
	}
}
