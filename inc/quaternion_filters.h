#ifndef __QUATERNION_FILTERS_H
#define __QUATERNION_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __QUATERNION_FILTERS_H

