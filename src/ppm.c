#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#define MAX_NUM_CHANNELS 4
#define DEFAULT_VAL 1500

static uint16_t clock_vals[MAX_NUM_CHANNELS] = {0};
static uint16_t num_updates = 0;


/*
 * Configure PPM receiving for 1 channel.
 * Use Channel 1 to restart every rising edge
 * Use Channel 2 to capture the duty cycle, which is what we're looking for.
 */
void ppm_configure(uint8_t num_channels) {
	for (int i = 0; i < MAX_NUM_CHANNELS; ++i) {
		clock_vals[i] = DEFAULT_VAL;
	}
	// TODO: Add support for channels
	//  - Configure 1st channel's rising edge to start timer
	//  - Configure every channel's falling edge to capture
	//  - Configure last channel's falling edge to trigger interrupt to override the
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	// Setup GPIO input to map to capture channels
	GPIO_Init(
			GPIOB,
			&(GPIO_InitTypeDef){
			GPIO_Pin_4,
			GPIO_Mode_AF,
			GPIO_Speed_50MHz,
			GPIO_OType_PP,
			GPIO_PuPd_UP
			});
	GPIO_Init(
			GPIOB,
			&(GPIO_InitTypeDef){
			GPIO_Pin_0,
			GPIO_Mode_AF,
			GPIO_Speed_50MHz,
			GPIO_OType_PP,
			GPIO_PuPd_UP
			});
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
	NVIC_Init(&(NVIC_InitTypeDef){
			TIM3_IRQn,                 // Channel
			0,                         // Priority (0-3)
			ENABLE                     // Command
			});
	// Initialize timer
	TIM_TimeBaseInit(TIM3, &(TIM_TimeBaseInitTypeDef){
			47,                  // Prescaler (want 1us ticks) 48 ticks / us by default (48MHZ sysclk)
			TIM_CounterMode_Up, // CounterMode
			// TODO: Revisit
			60000,              // Period
			0,                  // Clock Division
			0                   // RepetitionCounter
			});
	TIM_ICInit(TIM3, &(TIM_ICInitTypeDef){
			TIM_Channel_1,             // Channel
			TIM_ICPolarity_Rising,     // Polarity
			TIM_ICSelection_DirectTI,  // Selection
			TIM_ICPSC_DIV1,            // Prescaler
			0                          // Filter
			});
	// TODO: Using as pwm, ideally I don't need to waste a capture like this.
	TIM_ICInit(TIM3, &(TIM_ICInitTypeDef){
			TIM_Channel_2,             // Channel
			TIM_ICPolarity_Falling,     // Polarity
			TIM_ICSelection_IndirectTI,  // Selection
			TIM_ICPSC_DIV1,            // Prescaler
			0                          // Filter
			});
	// Configure 3rd capture (2nd channel)
	TIM_ICInit(TIM3, &(TIM_ICInitTypeDef){
			TIM_Channel_3,             // Channel
			TIM_ICPolarity_Falling,    // Polarity
			TIM_ICSelection_DirectTI,  // Selection
			TIM_ICPSC_DIV1,            // Prescaler
			0                          // Filter
			});
  // Select the TIM3 Input Trigger: TI2FP2
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
  // Select the slave Mode: Reset Mode
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

  // TIM enable counter
	TIM_Cmd(TIM3, ENABLE);

	// Configure the last capture as the interrupt
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}

void TIM3_IRQHandler(void) {
	if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
		clock_vals[0] = TIM_GetCapture2(TIM3);
		clock_vals[1] = TIM_GetCapture3(TIM3) - clock_vals[0];
	} else {
		for (int i = 0; i < MAX_NUM_CHANNELS; ++i) {
			clock_vals[i] = DEFAULT_VAL;
		}
		num_updates ++;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update);
}

uint16_t ppm_get_ch(uint8_t channel) {
	// Mocking out the CC
	// TIM_GenerateEvent(TIM3, TIM_EventSource_CC1);
	return clock_vals[channel];
}

uint16_t ppm_get_updates() {
	uint16_t val = num_updates;
	num_updates = 0;
	return val;
}
