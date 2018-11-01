#include <stdio.h>
#include <stdint.h>

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/stepper.h"
#include "inc/usart.h"
#include "inc/i2c.h"
#include "inc/std_utils.h"
#include "inc/quaternion_filters.h"
#include "inc/mpu_9250.h"
#include "inc/imu.h"
#include "inc/profile.h"

#define BLINK_DELAY_US	(50)

volatile uint32_t tickUs = 0;


void init() {
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000000)) {
		// Capture error
		while (1){};
	}
}


int usart_example(void) {
    usart_configure(9600);
    usart_send_string("HELLO WORLD");
    char c_str[32];
    ftoa(c_str, 13.24567, 7);
    while(1) {
        usart_block_receive_char();
        usart_send_string(c_str);
        usart_send_string("\r\n");
    }
}


int stepper_example(void) {
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


int i2c_example(void) {
    const uint16_t ARDUINO_SLAVE = (0x08 << 1);
    uint8_t read_buf[255];
    uint8_t write_buf[255];
    write_buf[0] = 0x00;
    write_buf[1] = 0xDE;
    write_buf[2] = 0xAD;
    write_buf[3] = 0xBE;
    write_buf[4] = 0xEF;

    init();
    i2c_configure();
    i2c_send(ARDUINO_SLAVE, 5, write_buf, true);
    write_buf[0] = 0xFF;
    i2c_send(ARDUINO_SLAVE, 1, write_buf, false);
    i2c_receive(ARDUINO_SLAVE, 8, read_buf, true);
    return 0;
}

int delay_profile_example(void) {
    init();
    profile_init();
    for(;;) {
        delay(10);
        profile_toggle();
        delay(100);
        profile_toggle();
        delay(200);
        profile_toggle();
    }
}


int main(void) {
    euler_t angles;
    char c_str[32];

    init();
    usart_configure(9600);
    usart_send_string("Configuring I2C\r\n");
    i2c_configure();
    usart_send_string("Initializing IMU\r\n");
    int imu_init_err = imu_init();
    if (imu_init_err) {
        usart_send_string("Failed to initialize IMU: ");
        itoa(c_str, imu_init_err, 5); usart_send_string(c_str);
        for(;;);
    }
    usart_send_string("Starting loop, retrieving Orientations\r\n");

    uint32_t last_display = tickUs;
    
    for (;;) {
        imu_get_euler_orientation(&angles);
        // TODO: Use PID loop to determine speed to set motors

        // Every 0.5 second serially print the angles
        if (tickUs - last_display  > 500000) {
            /*
            // Accelerometer data
            usart_send_string("ACC: ");
            ftoa(c_str, orientation.ax, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.ay, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.az, 2); usart_send_string(c_str);
            usart_send_string("\r\n");

            // Magnetometer data
            usart_send_string("MAG: ");
            ftoa(c_str, orientation.mx, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.my, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.mz, 2); usart_send_string(c_str);
            usart_send_string("\r\n");

            // Quaternion
            float *q = getQ();
            usart_send_string("QTN: ");
            ftoa(c_str, q[0], 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, q[1], 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, q[2], 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, q[3], 2); usart_send_string(c_str);
            usart_send_string("\r\n");
            */

            // Yaw, Pitch, Roll
            usart_send_string("Orientation: ");
            ftoa(c_str, angles.yaw, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, angles.pitch, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, angles.roll, 2); usart_send_string(c_str);
            usart_send_string("\r\n");

            last_display = tickUs;
        }
    }
    for(;;);
    return 0;
}


void SysTick_Handler(void)
{
	tickUs++;
}
