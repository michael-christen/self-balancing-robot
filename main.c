#include <stdio.h>
#include <stdint.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/stepper.h"
#include "inc/usart.h"
#include "inc/i2c.h"
#include "inc/std_utils.h"

#define BLINK_DELAY_US	(50)

volatile uint32_t tickUs = 0;


void init() {
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000000)) {
		// Capture error
		while (1){};
	}
}


int vain3(void) {
    usart_configure(9600);
    uint16_t val;
    usart_send_string("HELLO WORLD");
    char c_str[32];
    ftoa(c_str, 13.24567, 7);
    while(1) {
        val = usart_block_receive_char();
        usart_send_string(c_str);
        usart_send_string("\r\n");
    }
}


int vain(void) {

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
    stepper_t stepper0 = stepper_init(GPIOC, GPIO_Pin_9, GPIO_Pin_8, BLINK_DELAY_US, forward, tickUs);
	for(;;) {
        if (tickUs / 1000000 > last_count) {
          last_count += 1;
          forward = !forward;
          stepper_set_dir(&stepper0, forward);
        }
        stepper_next_action(&stepper0, tickUs);
		__WFI();
	}

	return 0;
}

// TODO: Check if we need to shift down by 1
#define MPU6050_ADDRESS (0xD0 << 1)
#define ARDUINO_SLAVE (0x08 << 1)

int main(void) {
    uint8_t read_buf[255];
    uint8_t write_buf[255];
    write_buf[0] = 0x00;
    write_buf[1] = 0xDE;
    write_buf[2] = 0xAD;
    write_buf[3] = 0xBE;
    write_buf[4] = 0xEF;

    init();
    i2c_configure();
    // i2c_read_reg(MPU6050_ADDRESS, 117, 1, read_buf);
    // i2c_read_reg(MPU6050_ADDRESS, 107, 1, read_buf+1);
    i2c_send(ARDUINO_SLAVE, 5, write_buf, true);
    write_buf[0] = 0xFF;
    i2c_send(ARDUINO_SLAVE, 1, write_buf, false);
    i2c_receive(ARDUINO_SLAVE, 8, read_buf, true);
    return 0;
}

void SysTick_Handler(void)
{
	tickUs++;
}
