#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

typedef struct euler_t {
    float yaw, pitch, roll;
} euler_t;

typedef struct imu_t {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
} imu_t;


int imu_init(void);
int imu_update_quaternion();
int imu_get_euler_orientation(euler_t *angles);

extern imu_t orientation;

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // _IMU_H

