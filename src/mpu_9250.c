/*
 * TODO:
 * - test
 * - possibly wrap whole thing into imu package, get_euler_angles_from_imu
 * - use structs instead of 3-6 length data arrays
 *
 */
#include "inc/mpu_9250.h"
#include "inc/i2c.h"
#include "inc/std_utils.h"


// TODO: Change when change set pin
#define MPU9250_ADDRESS (MPU9250_ADDRESS_AD0 << 1)
#define MAG_ADDRESS (AK8963_ADDRESS << 1)

static uint8_t tmp_buf[16] = {0};
typedef enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
} Gscale;
typedef enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
} Ascale;
typedef enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
} Mscale;
typedef enum Mmode {
  M_8HZ = 0x02,  // 8 Hz update
  M_100HZ = 0x06 // 100 Hz continuous magnetometer
} Mmode;

Gscale G_SCALE = GFS_250DPS;
Ascale A_SCALE = AFS_2G;
Mscale M_SCALE = MFS_16BITS;
Mmode M_MODE = M_8HZ;

float aRes, gRes, mRes;


uint8_t mpu_read_byte(uint8_t subaddress) {
    i2c_read_reg(MPU9250_ADDRESS, subaddress, 1, tmp_buf);
    return tmp_buf[0];
}

void mpu_write_byte(uint8_t subaddress, uint8_t byte) {
    tmp_buf[0] = subaddress;
    tmp_buf[1] = byte;
    i2c_send(MPU9250_ADDRESS, 2, tmp_buf, true);
}

uint8_t mpu_mag_read_byte(uint8_t subaddress) {
    i2c_read_reg(MAG_ADDRESS, subaddress, 1, tmp_buf);
    return tmp_buf[0];
}

void mpu_mag_write_byte(uint8_t subaddress, uint8_t byte) {
    tmp_buf[0] = subaddress;
    tmp_buf[1] = byte;
    i2c_send(MAG_ADDRESS, 2, tmp_buf, true);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void mpu_self_test(float *destination) {
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = GFS_250DPS;

  mpu_write_byte(SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  mpu_write_byte(CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  mpu_write_byte(GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  mpu_write_byte(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  mpu_write_byte(ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g
  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    i2c_read_reg(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    i2c_read_reg(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  mpu_write_byte(ACCEL_CONFIG, 0xE0);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  mpu_write_byte(GYRO_CONFIG,  0xE0);
  delay(25);  // Delay a while to let the device stabilize

  // Get average self-test values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
    i2c_read_reg(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    i2c_read_reg(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  mpu_write_byte(ACCEL_CONFIG, 0x00);
  mpu_write_byte(GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize


  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X-axis accel self-test results
  selfTest[0] = mpu_read_byte(SELF_TEST_X_ACCEL);
  // Y-axis accel self-test results
  selfTest[1] = mpu_read_byte(SELF_TEST_Y_ACCEL);
  // Z-axis accel self-test results
  selfTest[2] = mpu_read_byte(SELF_TEST_Z_ACCEL);
  // X-axis gyro self-test results
  selfTest[3] = mpu_read_byte(SELF_TEST_X_GYRO);
  // Y-axis gyro self-test results
  selfTest[4] = mpu_read_byte(SELF_TEST_Y_GYRO);
  // Z-axis gyro self-test results
  selfTest[5] = mpu_read_byte(SELF_TEST_Z_GYRO);

  // Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
      - 100.;
    // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
      - 100.;
  }
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void mpu_calibrate(float * gyroBias, float * accelBias) {
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  mpu_write_byte(PWR_MGMT_1, READ_FLAG);
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  mpu_write_byte(PWR_MGMT_1, 0x01);
  mpu_write_byte(PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  // Disable all interrupts
  mpu_write_byte(INT_ENABLE, 0x00);
  // Disable FIFO
  mpu_write_byte(FIFO_EN, 0x00);
  // Turn on internal clock source
  mpu_write_byte(PWR_MGMT_1, 0x00);
  // Disable I2C master
  mpu_write_byte(I2C_MST_CTRL, 0x00);
  // Disable FIFO and I2C master modes
  mpu_write_byte(USER_CTRL, 0x00);
  // Reset FIFO and DMP
  mpu_write_byte(USER_CTRL, 0x0C);
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  mpu_write_byte(CONFIG, 0x01);
  // Set sample rate to 1 kHz
  mpu_write_byte(SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  mpu_write_byte(GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  mpu_write_byte(ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  mpu_write_byte(USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // MPU-9150)
  mpu_write_byte(FIFO_EN, 0x78);
  delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  mpu_write_byte(FIFO_EN, 0x00);
  // Read FIFO sample count
  i2c_read_reg(MPU9250_ADDRESS, FIFO_COUNTH, 2, data);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    i2c_read_reg(MPU9250_ADDRESS, FIFO_R_W, 12, data);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  mpu_write_byte(XG_OFFSET_H, data[0]);
  mpu_write_byte(XG_OFFSET_L, data[1]);
  mpu_write_byte(YG_OFFSET_H, data[2]);
  mpu_write_byte(YG_OFFSET_L, data[3]);
  mpu_write_byte(ZG_OFFSET_H, data[4]);
  mpu_write_byte(ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  i2c_read_reg(MPU9250_ADDRESS, XA_OFFSET_H, 2, data);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2c_read_reg(MPU9250_ADDRESS, YA_OFFSET_H, 2, data);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2c_read_reg(MPU9250_ADDRESS, ZA_OFFSET_H, 2, data);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  mpu_write_byte(XA_OFFSET_H, data[0]);
  mpu_write_byte(XA_OFFSET_L, data[1]);
  mpu_write_byte(YA_OFFSET_H, data[2]);
  mpu_write_byte(YA_OFFSET_L, data[3]);
  mpu_write_byte(ZA_OFFSET_H, data[4]);
  mpu_write_byte(ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void mpu_init(void) {
  // wake up device
  // Clear sleep mode bit (6), enable all sensors
  mpu_write_byte(PWR_MGMT_1, 0x00);
  delay(100); // Wait for all registers to reset

  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  mpu_write_byte(PWR_MGMT_1, 0x01);
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  mpu_write_byte(CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  mpu_write_byte(SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // get current GYRO_CONFIG register value
  uint8_t c = mpu_read_byte(GYRO_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | G_SCALE << 3; // Set full scale range for the gyro
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  // Write new GYRO_CONFIG value to register
  mpu_write_byte(GYRO_CONFIG, c );

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  c = mpu_read_byte(ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | A_SCALE << 3; // Set full scale range for the accelerometer
  // Write new ACCEL_CONFIG register value
  mpu_write_byte(ACCEL_CONFIG, c);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  // Get current ACCEL_CONFIG2 register value
  c = mpu_read_byte(ACCEL_CONFIG2);
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // Write new ACCEL_CONFIG2 register value
  mpu_write_byte(ACCEL_CONFIG2, c);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master.
  mpu_write_byte(INT_PIN_CFG, 0x22);
  // Enable data ready (bit 0) interrupt
  mpu_write_byte(INT_ENABLE, 0x01);
  delay(100);

  // TODO: Possibly
  // setupMagForSPI();
}


// TODO:
void mpu_init_mag(float *destination) {
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  // TODO: Test this!! Likely doesn't work
  mpu_mag_write_byte(AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  mpu_mag_write_byte(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);

  // Read the x-, y-, and z-axis calibration values
  i2c_read_reg(MAG_ADDRESS, AK8963_ASAX, 3, rawData);

  // Return x-axis sensitivity adjustment values, etc.
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  mpu_mag_write_byte(AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);

  // Configure the magnetometer for continuous read and highest resolution.
  // Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mmode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

  // Set magnetometer data resolution and sample ODR
  mpu_mag_write_byte(AK8963_CNTL, M_SCALE << 4 | M_MODE);
  delay(10);
}

void mpu_read_accel_data(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  i2c_read_reg(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void mpu_get_ares() {
  switch (A_SCALE)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}

void mpu_read_gyro_data(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  i2c_read_reg(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void mpu_get_gres() {
  switch (G_SCALE)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void mpu_read_mag_data(int16_t *destination) {
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  uint8_t rawData[7];
  // Wait for magnetometer data ready bit to be set
  if (mpu_mag_read_byte(AK8963_ST1) & 0x01)
  {
    // Read the six raw data and ST2 registers sequentially into data array
    i2c_read_reg(MAG_ADDRESS, AK8963_XOUT_L, 7, rawData);
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
  }
}

void mpu_get_mres() {
  switch (M_SCALE)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

// NOTE: UNUSED
int16_t read_temp_data(void) {
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  i2c_read_reg(MPU9250_ADDRESS, TEMP_OUT_H, 2, rawData);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================
//
#ifdef __cplusplus

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void MPU9250::magCalMPU9250(float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3]  = {-32768, -32768, -32768},		// Wrote out decimal (signed) values to remove a conversion warning
          mag_min[3]  = {32767, 32767, 32767},
          mag_temp[3] = {0, 0, 0};

  // Make sure resolution has been calculated
  getMres();

  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
  Serial.println(
      F("  4 seconds to get ready followed by 15 seconds of sampling)"));
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == M_8HZ)
  {
    sample_count = 128;
  }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  }

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }

    if (Mmode == M_8HZ)
    {
      delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
    }
    if (Mmode == M_100HZ)
    {
      delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = (float)mag_bias[0] * mRes * factoryMagCalibration[0];
  bias_dest[1] = (float)mag_bias[1] * mRes * factoryMagCalibration[1];
  bias_dest[2] = (float)mag_bias[2] * mRes * factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println(F("Mag Calibration done!"));
}
#endif  // __cplusplus
