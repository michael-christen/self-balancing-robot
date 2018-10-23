#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"

#include "inc/std_utils.h"

#include <stdint.h>

static volatile uint32_t i2cErr = 0;

void I2C2_IRQHandler(void) {
    i2cErr = I2C2->ISR;
    I2C_ITConfig(I2C2, I2C_IT_ERRI, DISABLE);
}


// Adapted from silta i2c
void i2c_configure() {
    I2C_InitTypeDef i2cConfig;
	GPIO_InitTypeDef GPIO_InitStructure;

    // GPIO Init
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    // TODO: Possibly configure DMA

    // Setup Pins
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);

    // Configure I2C
    I2C_StructInit(&i2cConfig);

    i2cConfig.I2C_Ack = I2C_Ack_Enable;
    i2cConfig.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    i2cConfig.I2C_DigitalFilter = 0;
    i2cConfig.I2C_Timing = 0x10805E89;

    I2C_DeInit(I2C2);
    I2C_Init(I2C2, &i2cConfig);

    // TODO: Configure I2c interrupts
    // I2C_ITConfig(I2C2, I2C_IT_ERRI, ENABLE);
    // NVIC_EnableIRQ(I2C2_IRQn);

    // Enable I2C
    I2C_Cmd(I2C2, ENABLE);
}


void i2c_send(uint8_t address, uint16_t write_length, uint8_t *write_buffer, bool stop) {
    // Ensure I2C isn't busy
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET);
    // Start the read
    // Note: SoftEnd_Mode allows us to explicitly send stop later
    I2C_TransferHandling(I2C2, address, write_length, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    for(uint16_t i = 0; i < write_length; ++i) {
        // Ensure transmit interrupted flag is set
        while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET);
        I2C_SendData(I2C2, write_buffer[i]);
    }
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) == RESET);
    // Wait for stop condition
    if (stop) {
        I2C_TransferHandling(I2C2, address, 0, I2C_AutoEnd_Mode, I2C_Generate_Stop);
        while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET);
        // Clear the stop flag
        I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);
    }
}

void i2c_receive(uint8_t address, uint16_t read_length, uint8_t *read_buffer) {
    // Ensure I2C isn't busy
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET);
    // Start the read
    // Automatically end the transmission
    I2C_TransferHandling(I2C2, address, read_length, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
    // Fill the buffer
    for(uint16_t i = 0; i < read_length; ++i) {
        // Wait until RX has data
        while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET);
        read_buffer[i] = I2C_ReceiveData(I2C2);
    }
    // Wait for stop condition
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET);
    // Clear the stop flag
    I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);
}


static uint8_t _temp_write_buffer[] = {0, 0};


/*
 * START, ADDR(W), REG ADDR, START, ADDR(R), <data>, ..., NACK, STOP
 */
void i2c_read_reg(uint8_t address, uint8_t reg, uint16_t read_length, uint8_t *read_buffer) {
    _temp_write_buffer[0] = reg;
    i2c_send(address, 1, _temp_write_buffer, false);
    i2c_receive(address, read_length, read_buffer);
}
