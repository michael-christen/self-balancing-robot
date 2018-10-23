#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include <stdint.h>

void i2c_configure();
void i2c_send(uint8_t address, uint16_t write_length, uint8_t *write_buffer);
void i2c_receive(uint8_t address, uint16_t read_length, uint8_t *read_buffer);
void i2c_read_reg(uint8_t address, uint8_t reg, uint16_t read_length, uint8_t *read_buffer);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __I2C_H
