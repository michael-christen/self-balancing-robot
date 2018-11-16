#include <stdio.h>
#include <stdint.h>
// #include <stdlib.h>
// #include <string.h>

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

#define RAD_TO_DEG ( 180.0f / M_PI )
#define DEG_TO_RAD ( M_PI / 180.0f )

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
    usart_configure(9600);
    usart_send_string("Enter a speed value, then hit enter:\r\n");
    // Configure direction pin/s & enable pin
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(
        GPIOC,
        &(GPIO_InitTypeDef){
            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_7,
            GPIO_Mode_OUT,
            GPIO_Speed_2MHz,
            GPIO_OType_PP,
            GPIO_PuPd_NOPULL
        });
    bool forward = true;
    uint32_t last_count = 0;
    stepper_t stepper0 = stepper_init(
        GPIOC,
        GPIO_Pin_9,
        GPIO_Pin_9 | GPIO_Pin_8,
        GPIO_Pin_7,
        BLINK_DELAY_US,
        forward,
        tickUs);

    // Timer based configuration
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init(
        GPIOA,
        &(GPIO_InitTypeDef){
            GPIO_Pin_8 | GPIO_Pin_9,
            GPIO_Mode_AF,
            GPIO_Speed_50MHz,
            GPIO_OType_PP,
            GPIO_PuPd_UP
        });
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

    uint16_t frequency = 400;  // Hz

    stepper_set_speed(&stepper0, frequency);

    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    float speed = 0;
    char c_str[256];
    for(;;) {
        uint8_t idx = 0;
        for(;;) {
            char c = usart_block_receive_char();
            usart_send_char(c);
            c_str[idx++] = c;
            if (c == '$') {
                c_str[idx-1] = '\0';
                break;
            }
        }
        speed = atof(c_str);
        usart_send_string("\r\nSetting speed to: ");
        ftoa(c_str, speed, 0); usart_send_string(c_str);
        usart_send_string("\r\n");
        uint16_t motor_speed = (uint16_t) fabs(speed);
        bool dir = speed < 0;
        stepper_set_dir(&stepper0, dir);
        stepper_set_speed(&stepper0, motor_speed);
    }

    uint16_t speeds[] = {
            400,
            800,
            1000,
            2000,
            4000,
            6000,
            4000,
            2000,
            1000,
            800,
            400,
    };
    uint8_t speed_index = 0;
	for(;;) {
        if (tickUs / 1000000 > last_count) {
          last_count += 1;
          forward = !forward;
          // stepper_set_dir(&stepper0, forward);
          stepper_set_speed(&stepper0, speeds[speed_index++]);
          speed_index %= 11;
        }
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


// TODO: Improve
#define PERIOD 13000  // uS ~ 75 Hz


int16_t constr(int16_t value, int16_t smallest, int16_t biggest) {
    if (value < smallest) {
        return smallest;
    } else if (value > biggest) {
        return biggest;
    } else {
        return value;
    }
}


float constrf(float value, float smallest, float biggest) {
    if (value < smallest) {
        return smallest;
    } else if (value > biggest) {
        return biggest;
    } else {
        return value;
    }
}


int vain(void) {
    init();
    usart_configure(9600);

	char c_str[32];
  
	for (float x = -1; x < 1.5; x += 2) {
        for (float y = -10.0; y < 10.0; y += 0.1) {
          float z = atan2(y, x);
          usart_send_string("ATAN2(");
	      ftoa(c_str, y, 2); usart_send_string(c_str);
	      usart_send_string("/");
	      ftoa(c_str, x, 2); usart_send_string(c_str);
          usart_send_string(") = ");
	      ftoa(c_str, z, 2); usart_send_string(c_str);
          usart_send_string("\r\n");
	      // delay(100);
        }
	}
  
  /* z approaches -pi as y goes from -10 to 0 */
  /* z approaches +pi as y goes from +10 to 0 */
}

const float MAX_PID_OUTPUT = 4000;
const float MAX_SPEED = 20000;
const float MIN_SPEED = 200;
const float MAX_ERROR = 30.0;
const float MIN_ERROR = 1.0;
// const float MAX_SPEED_DIFF = 200;

float get_motor_speed_from_pid(float pid_output, float pid_error, float last_speed) {
    float speed = (constrf(pid_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT) *
                    (MAX_SPEED / MAX_PID_OUTPUT));
    // float diff = constrf(speed - last_speed, -MAX_SPEED_DIFF, MAX_SPEED_DIFF);
    // speed = last_speed + diff;
    if (fabs(pid_error) < MIN_ERROR) {
        return 0;
    }
    if (fabs(speed) < MIN_SPEED) {
        return 0;
    }
    return speed;
}


int main(void) {
    euler_t angles;
    char c_str[32];

    init();
    usart_configure(9600);
    usart_send_string("Clock Frequency: ");
    ftoa(c_str, SystemCoreClock, 2); usart_send_string(c_str);
    usart_send_string("\r\n");
    usart_send_string("Configuring I2C\r\n");
    i2c_configure();
    usart_send_string("Initializing IMU\r\n");
    int imu_init_err = imu_init();
    if (imu_init_err) {
        usart_send_string("Failed to initialize IMU: ");
        my_itoa(c_str, imu_init_err, 5); usart_send_string(c_str);
        for(;;);
    }
    usart_send_string("Starting loop, retrieving Orientations\r\n");

    // Configure direction pin/s and enable pin
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(
        GPIOC,
        &(GPIO_InitTypeDef){
            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_7,
            GPIO_Mode_OUT,
            GPIO_Speed_2MHz,
            GPIO_OType_PP,
            GPIO_PuPd_NOPULL
        });
    bool forward = true;
    stepper_t stepper0 = stepper_init(
        GPIOC,
        GPIO_Pin_9,
        GPIO_Pin_9 | GPIO_Pin_8,
        GPIO_Pin_7,
        BLINK_DELAY_US,
        forward,
        tickUs);

    // Timer based configuration
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init(
        GPIOA,
        &(GPIO_InitTypeDef){
            GPIO_Pin_8 | GPIO_Pin_9,
            GPIO_Mode_AF,
            GPIO_Speed_50MHz,
            GPIO_OType_PP,
            GPIO_PuPd_UP
        });
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);


    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    profile_init();

    uint32_t last_display = tickUs;
    uint32_t last_loop;

    float integral_error = 0;
    float last_pid_error = 0;
    float pid_output = 0;
    const float Kp = 700.0;
    const float Ki = 0.0;
    const float Kd = 0.0;
    /*
    // Establish setpoint
    float roll_sum = 0;
    for (int i = 0; i < 200; ++i) {
        imu_update_quaternion();
        imu_get_euler_orientation(&angles);
        roll_sum += angles.roll;
    }
    float setpoint = roll_sum / 200.0;
    */
    float setpoint = 2.25;
    float last_speed = 0;
    for (;;) {
        profile_toggle();
        last_loop = tickUs;

        imu_update_quaternion();
        imu_get_euler_orientation(&angles);

        // TODO: Proper PID
        float pid_error = angles.roll - setpoint;
        integral_error += Ki * pid_error;
        integral_error = constrf(integral_error, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
        float error_derivative = pid_error - last_pid_error;

        pid_output = Kp * pid_error + integral_error + Kd * error_derivative;

        // Remove integral_error when crossover
        if (last_pid_error * pid_error < 0) {
            integral_error = 0;
        }

        last_pid_error = pid_error;

        if (fabs(pid_error) > MAX_ERROR) {
            pid_output = 0;
            integral_error = 0;
        }

        float speed = get_motor_speed_from_pid(pid_output, pid_error, last_speed);
        last_speed = speed;
        stepper_set_dir(&stepper0, (speed < 0));
        stepper_set_speed(&stepper0, (uint16_t) fabs(speed));

        // Every 0.5 second serially print the angles
        if (tickUs - last_display  > 500000) {
            /*
            float hz = imu_get_hz();
            // Accelerometer data
            usart_send_string("ACC: ");
            ftoa(c_str, orientation.ax, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.ay, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.az, 2); usart_send_string(c_str);
            usart_send_string("\r\n");
    
            // Gyroscopic Data
            usart_send_string("GYR: ");
            ftoa(c_str, orientation.gx, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.gy, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.gz, 2); usart_send_string(c_str);
            usart_send_string("\r\n");
    
            // Magnetometer data
            usart_send_string("MAG: ");
            ftoa(c_str, orientation.mx, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.my, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, orientation.mz, 2); usart_send_string(c_str);
            usart_send_string("\r\n");
    
            // Yaw, Pitch, Roll
            usart_send_string("Orientation: ");
            ftoa(c_str, angles.yaw, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, angles.pitch, 2); usart_send_string(c_str);
            usart_send_string(" ");
            ftoa(c_str, angles.roll, 2); usart_send_string(c_str);
            usart_send_string("\r\n");

            usart_send_string("Error: ");
            ftoa(c_str, pid_error, 2); usart_send_string(c_str);
            usart_send_string("\r\n");


            usart_send_string("Speed: ");
            ftoa(c_str, speed, 0); usart_send_string(c_str);
            usart_send_string("\r\n");

            usart_send_string("HZ: ");
            ftoa(c_str, hz, 2); usart_send_string(c_str);
            usart_send_string("\r\n");
            */

            last_display = tickUs;
        }
        /*
        if ((tickUs - last_loop) > PERIOD) {
            usart_send_string("ERROR loop too short\r\n");
        }
        */
        while((tickUs - last_loop) < PERIOD);
    }
    for(;;);
    return 0;
}


void SysTick_Handler(void)
{
	tickUs++;
}
