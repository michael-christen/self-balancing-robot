#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include <stdint.h>

#include "stm32f0xx.h"

#include "inc/std_utils.h"

/* TODO:
 * - set_delay
 * - TIMER based
 *
 */

typedef struct stepper_t {
    uint32_t next_step;
    uint8_t state;
    uint32_t step_delay;
    GPIO_TypeDef *gpio_port;
    uint32_t step_pin;
    uint32_t dir_pin;
} stepper_t;

stepper_t stepper_init(
    GPIO_TypeDef *gpio_port,
    uint32_t step_pin,
    uint32_t dir_pin,
    uint32_t step_delay,
    bool forward,
    uint32_t current_ticks);

void stepper_set_dir(stepper_t *stepper, bool forward);
void stepper_set_speed(stepper_t *stepper, uint16_t frequency);

void stepper_next_action(stepper_t *stepper, uint32_t current_ticks);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __STEPPER_H
