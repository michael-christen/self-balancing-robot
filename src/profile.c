#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/profile.h"

static uint8_t profile_state;


void profile_init() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(
        GPIOC,
        &(GPIO_InitTypeDef){
            GPIO_Pin_10,
            GPIO_Mode_OUT,
            GPIO_Speed_2MHz,
            GPIO_OType_PP,
            GPIO_PuPd_NOPULL
        });
    profile_state = 0;
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);
}

void profile_toggle() {
    profile_state = !profile_state;
    if(profile_state) {
        GPIO_SetBits(GPIOC, GPIO_Pin_10);
    } else {
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
    }
}
