#include <stdint.h>

#include "inc/std_utils.h"

int itoa(char *c_str, uint32_t val, uint32_t num_decimals) {
	int i = 0;
    int total_len = 0;
	if(val == 0) {
		c_str[i++] = '0';
	}
	while(val) {
		c_str[i++] = '0' + (val % 10);
		val /= 10;
	}
    while(i < num_decimals) {
        c_str[i++] = '0';
    }
	c_str[i] = '\0';  // Terminate
    total_len = i;
	// Reverse
	i --;
	for(int j = 0; j < i / 2 + 1; ++j) {
		char temp = c_str[j];
		c_str[j] = c_str[i-j];
		c_str[i-j] = temp;
	}
    return total_len;
}



/* Overwrite c_str with the string value of a floating point number.
 *
 */
int ftoa(char *c_str, float val, uint32_t num_decimals) {
    int i = 0;
    if (val < 0) {
        c_str[i++] = '-';
        val *= -1;
    }
    // assert(val >= 0);
    // Separate out integer from fraction
    uint32_t int_val = (uint32_t)val;
    float f_val = val - (float)int_val;
    i = itoa(c_str + i, int_val, 0);
    if (num_decimals > 0) {
        c_str[i++] = '.';
        i += itoa(c_str + i, (uint32_t)(f_val * pow(10, num_decimals)), num_decimals);
    }
    return i;
}


void delay(int duration) {
   uint32_t cur_ticks = tickUs;
   // TODO: Deal with overflow
   while(tickUs > cur_ticks + (duration * 1000));
}
