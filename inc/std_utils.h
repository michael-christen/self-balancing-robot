#ifndef __STD_UTILS_H
#define __STD_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include <math.h>
#include <stdint.h>

#define false (0)
#define true (1)
#define bool uint8_t

int my_itoa(char *c_str, uint32_t val, uint32_t num_decimals);
int ftoa(char *c_str, float val, uint32_t num_decimals);
void delay(int duration);

extern volatile uint32_t tickUs;

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __STD_UTILS_H
