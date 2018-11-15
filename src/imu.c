#include <math.h>

#include "inc/imu.h"
#include "inc/mpu_9250.h"
#include "inc/quaternion_filters.h"
#include "inc/std_utils.h"
#include "inc/usart.h"


#define ERR_MPU_NOT_FOUND     (1)
#define ERR_TRIM_TOO_HIGH     (2)
#define ERR_MPU_MAG_NOT_FOUND (3)

#define RAD_TO_DEG ( 180.0f / M_PI )
#define DEG_TO_RAD ( M_PI / 180.0f )

imu_t orientation;
static float mag_calibration[3];
static float delta_t, sum;
uint32_t last_update, sum_count;
static int16_t accel_vals[3];
static int16_t gyro_vals[3];
static int16_t mag_vals[3];

int imu_init(void) {
    // Check Connection
    uint8_t val = mpu_read_byte(WHO_AM_I_MPU9250);
    if(val != 0x71) {
        return ERR_MPU_NOT_FOUND;
    }
    usart_send_string("MPU9250 Connected\r\n");
    // Check Trim
    float trim_percentage[6];
    mpu_self_test(trim_percentage);
    usart_send_string("Self Test Completed\r\n");
    for (int i=0; i < 6; ++i) {
        if (trim_percentage[i] > 10.0) {
            return ERR_TRIM_TOO_HIGH;
        }
    }
    // Calibrate
    /*
    float gyro_bias[3];
    float accelerometer_bias[3];
    char c_str[32];
    mpu_calibrate(gyro_bias, accelerometer_bias);
    usart_send_string("Calibration Completed\r\n");
    usart_send_string("GYRO Bias: ");
    for (int i = 0; i < 3; ++i) {
        ftoa(c_str, gyro_bias[i], 2); usart_send_string(c_str);
        usart_send_string(" ");
    }
    usart_send_string("\r\n");
    usart_send_string("ACC Bias: ");
    for (int i = 0; i < 3; ++i) {
        ftoa(c_str, accelerometer_bias[i], 2); usart_send_string(c_str);
        usart_send_string(" ");
    }
    usart_send_string("\r\n");
    */
    // Initialize
    mpu_init();
    usart_send_string("MPU Init Completed\r\n");
    // Check Magnet connection
    uint8_t who_is_mag = mpu_mag_read_byte(WHO_AM_I_AK8963);
    if (who_is_mag != 0x48) {
        return ERR_MPU_MAG_NOT_FOUND;
    }
    usart_send_string("MPU Mag found\r\n");
    mpu_init_mag(mag_calibration);
    usart_send_string("MPU Mag initialized\r\n");
    // Reset orientation
    orientation.ax = orientation.ay = orientation.az = 0;
    orientation.gx = orientation.gy = orientation.gz = 0;
    orientation.mx = orientation.my = orientation.mz = 0;
    // Reset raw arrays
    accel_vals[0] = 0;
    accel_vals[1] = 0;
    accel_vals[2] = 0;
    gyro_vals[0] = 0;
    gyro_vals[1] = 0;
    gyro_vals[2] = 0;
    mag_vals[0] = 0;
    mag_vals[1] = 0;
    mag_vals[2] = 0;
    // Reset last_update
    last_update = tickUs;
    return 0;
}

int imu_update_quaternion() {
    if (mpu_read_byte(INT_STATUS) & 0x01) {
        // Accelerometer
        mpu_read_accel_data(accel_vals);
        mpu_get_ares();
        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        orientation.ax = (float)accel_vals[0] * aRes; // - accelBias[0];
        orientation.ay = (float)accel_vals[1] * aRes; // - accelBias[1];
        orientation.az = (float)accel_vals[2] * aRes; // - accelBias[2];
        mpu_read_gyro_data(gyro_vals);
        mpu_get_gres();
        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        orientation.gx = (float)gyro_vals[0] * gRes;
        orientation.gy = (float)gyro_vals[1] * gRes;
        orientation.gz = (float)gyro_vals[2] * gRes;

        mpu_read_mag_data(mag_vals);
        mpu_get_mres();
        float mag_bias[3] = {
            // User environmental x-axis correction in milliGauss, should be
            // automatically calculated
            +470.,
            // User environmental x-axis correction in milliGauss TODO axis??
            +120.,
            // User environmental x-axis correction in milliGauss
            +125.,
        };
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        // Get actual magnetometer value, this depends on scale being set
        orientation.mx = (float)mag_vals[0] * mRes * mag_calibration[0] - mag_bias[0];
        orientation.my = (float)mag_vals[1] * mRes * mag_calibration[1] - mag_bias[1];
        orientation.mz = (float)mag_vals[2] * mRes * mag_calibration[2] - mag_bias[2];
    }
    uint32_t now = tickUs;
    // Set integration time by elapzed since last filter update
    delta_t = ((now - last_update) / 1000000.0f);
    last_update = now;

    // Keep track of HZ
    sum += delta_t;
    sum_count++;

    MahonyQuaternionUpdate(
        orientation.ax, orientation.ay, orientation.az,
        orientation.gx * DEG_TO_RAD, orientation.gy * DEG_TO_RAD, orientation.gz * DEG_TO_RAD,
        orientation.my, orientation.mx, orientation.mz,
        delta_t
    );
    return 0;
}

int imu_get_euler_orientation(euler_t *angles) {
    const float *q = getQ();
    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    angles->yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                        q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    angles->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    angles->roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                          q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    // Raw accelerometer values, no yaw
    /*
    angles->pitch = atan2(orientation.ay,
                          sqrt((orientation.ax * orientation.ax) +
                               (orientation.az * orientation.az)));
    angles->roll = atan2(-orientation.ax,
                         sqrt((orientation.ay * orientation.ay) +
                              (orientation.az * orientation.az)));

    // OR

    rollAcc = asin((float)orientation.ax) * RAD_TO_DEG;
    pitchAcc = asin((float)orientation.ay) * RAD_TO_DEG;

    roll -= orientation.gy * DEG_TO_RAD * (1000000.0 / PERIOD);
    pitch += orientation.gx * DEG_TO_RAD * (1000000.0 / PERIOD);

    // NOTE: roll ended up as 0xFFFFFF...
    roll = roll * 0.999 + rollAcc * 0.01;
    pitch = pitch * 0.999 + pitchAcc * 0.001;

    */
    angles->roll  *= RAD_TO_DEG;
    angles->pitch *= RAD_TO_DEG;
    angles->yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    // Declination of zipcode 94118
    // 	13° 25' E  ± 0° 20' (or 13.4°) on 2018-11-14
    angles->yaw   -= 13.4;
    return 0;
}
