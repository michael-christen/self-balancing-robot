#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

typedef struct euler_t {
    float yaw, pitch, roll;
} euler_t;


int imu_init(void);
int imu_get_euler_orientation(euler_t *angles);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // _IMU_H

