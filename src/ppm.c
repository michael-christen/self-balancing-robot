#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

static const MAX_NUM_CHANNELS = 4;


/*
 * Configure PPM receiving for 1 channel.
 * Use Channel 1 to restart every rising edge
 * Use Channel 2 to capture the duty cycle, which is what we're looking for.
 */
void ppm_configure(uint8_t num_channels) {
	// TODO: Add support for channels
	//  - Configure 1st channel's rising edge to start timer
	//  - Configure every channel's falling edge to capture
	//  - Configure last channel's falling edge to trigger interrupt to override the
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	// Setup GPIO input to map to capture channels
	GPIO_Init(
			GPIOA,
			&(GPIO_InitTypeDef){
			GPIO_Pin_0,
			GPIO_Mode_AF,
			GPIO_Speed_50MHz,
			GPIO_OType_PP,
			GPIO_PuPd_UP
			});
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_2);
	NVIC_Init(&(NVIC_InitTypeDef){
			TIM2_IRQn,                 // Channel
			0,                         // Priority (0-3)
			ENABLE                     // Command
			});
	// Initialize timer
	TIM_TimeBaseInit(TIM2, &(TIM_TimeBaseInitTypeDef){
			47,                  // Prescaler (want 1us ticks) 48 ticks / us by default (48MHZ sysclk)
			TIM_CounterMode_Up, // CounterMode
			// TODO: Revisit
			60000,              // Period
			0,                  // Clock Division
			0                   // RepetitionCounter
			});
	TIM_ICInit(TIM2, &(TIM_ICInitTypeDef){
			TIM_Channel_1,             // Channel
			TIM_ICPolarity_Rising,     // Polarity
			TIM_ICSelection_DirectTI,  // Selection
			TIM_ICPSC_DIV1,            // Prescaler
			0                          // Filter
			});
	// TODO: Using as pwm, ideally I don't need to waste a capture like this.
	TIM_ICInit(TIM2, &(TIM_ICInitTypeDef){
			TIM_Channel_2,             // Channel
			TIM_ICPolarity_Falling,     // Polarity
			TIM_ICSelection_IndirectTI,  // Selection
			TIM_ICPSC_DIV1,            // Prescaler
			0                          // Filter
			});
  // Select the TIM2 Input Trigger: TI2FP2
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
  // Select the slave Mode: Reset Mode
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

  // TIM enable counter
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}

static uint16_t clock_val = 0;
static uint16_t num_updates = 0;

void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
		clock_val = TIM_GetCapture2(TIM2);
	} else {
		num_updates ++;
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_Update);
}

uint16_t ppm_get_ch(uint8_t channel) {
	// Mocking out the CC
	// TIM_GenerateEvent(TIM2, TIM_EventSource_CC1);
	return clock_val;
}

uint16_t ppm_get_updates() {
	uint16_t val = num_updates;
	num_updates = 0;
	return val;
}
