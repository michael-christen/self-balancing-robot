#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/stepper.h"


stepper_t stepper_init(
    GPIO_TypeDef *gpio_port,
    uint32_t step_pin,
    uint32_t dir_pin,
    uint32_t enable_pin,
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
        .enable_pin = enable_pin,
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

void stepper_set_speed(stepper_t *stepper, uint16_t frequency) {
    if (frequency == 0) {
        // Allow the steppers to spin freely by disabling them
        GPIO_SetBits(stepper->gpio_port, stepper->enable_pin);
        // If no enable pin is there, stop the timer from ticking
        TIM_OC1Init(TIM1, &(TIM_OCInitTypeDef){
            TIM_OCMode_Inactive,     // OCMode
            TIM_OutputState_Enable,  // OutputState
            TIM_OutputNState_Enable, // OutputNState
            0,                       // Pulse
            TIM_OCPolarity_Low,      // Polarity
            TIM_OCNPolarity_High,    // NPolarity
            TIM_OCIdleState_Set,     // IdleState
            TIM_OCIdleState_Reset,   // NIdleState
        });
        // Configure Channel 2 too
        TIM_OC2Init(TIM1, &(TIM_OCInitTypeDef){
            TIM_OCMode_Inactive,     // OCMode
            TIM_OutputState_Enable,  // OutputState
            TIM_OutputNState_Enable, // OutputNState
            0,                       // Pulse
            TIM_OCPolarity_Low,      // Polarity
            TIM_OCNPolarity_High,    // NPolarity
            TIM_OCIdleState_Set,     // IdleState
            TIM_OCIdleState_Reset,   // NIdleState
        });
        return;
    }
    GPIO_ResetBits(stepper->gpio_port, stepper->enable_pin);
    /* Compute the value to be set in ARR regiter to generate signal frequency  */
    uint16_t timer_period = (SystemCoreClock / frequency ) - 1;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
    uint16_t ch1_pulse = (uint16_t) (
        ((uint32_t) 5 * (timer_period - 1)) / 10);

    /* Time Base configuration */
    TIM_TimeBaseInit(TIM1, &(TIM_TimeBaseInitTypeDef){
        0,                  // Prescaler
        TIM_CounterMode_Up, // CounterMode
        timer_period,       // Period
        0,                  // Clock Division
        0                   // RepetitionCounter
    });

    /* Configuration in PWM mode */
    TIM_OC1Init(TIM1, &(TIM_OCInitTypeDef){
        TIM_OCMode_PWM1,         // OCMode
        TIM_OutputState_Enable,  // OutputState
        TIM_OutputNState_Enable, // OutputNState
        ch1_pulse,               // Pulse
        TIM_OCPolarity_Low,      // Polarity
        TIM_OCNPolarity_High,    // NPolarity
        TIM_OCIdleState_Set,     // IdleState
        TIM_OCIdleState_Reset,   // NIdleState
    });
    // Configure Channel 2 too
    TIM_OC2Init(TIM1, &(TIM_OCInitTypeDef){
        TIM_OCMode_PWM1,         // OCMode
        TIM_OutputState_Enable,  // OutputState
        TIM_OutputNState_Enable, // OutputNState
        ch1_pulse,               // Pulse
        TIM_OCPolarity_Low,      // Polarity
        TIM_OCNPolarity_High,    // NPolarity
        TIM_OCIdleState_Set,     // IdleState
        TIM_OCIdleState_Reset,   // NIdleState
    });
};
