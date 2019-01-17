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
    TIM_TypeDef *timer;
    uint32_t dir_pin;
    uint32_t enable_pin;
} stepper_t;

stepper_t stepper_init(
    TIM_TypeDef *timer,
    uint32_t dir_pin,
    uint32_t enable_pin);

void stepper_set_dir(stepper_t *stepper, bool forward);
void stepper_set_speed(stepper_t *stepper, uint16_t frequency);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __STEPPER_H
