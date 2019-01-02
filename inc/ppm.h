#ifndef __PPM_H
#define __PPM_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// TODO: Possibly configure a timeout and a timeout return code
// TODO: return status code if properly initialized
void ppm_configure(uint8_t num_channels);
uint16_t ppm_get_ch(uint8_t channel);
uint16_t ppm_get_updates();

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __PPM_H

