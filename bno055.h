#include "bno055_hal.h"
#include <stdint.h>

typedef struct bno055 {
    uint8_t i2c_address;
    uint8_t last_page;
    uint8_t reset_pin;
} bno055_t;

// Below is a list of the different operating modes for the BNO055.
// All descriptions are from the datasheet

/**
*   Configuration mode
*   This mode is used to configure BNO, wherein all output data is reset to zero and sensor
*   fusion is halted. This is the only mode in which all the writable register map entries can be
*   changed. (Exceptions from this rule are the interrupt registers (INT and INT_MSK) and the
*   operation mode register (OPR_MODE), which can be modified in any operation mode.)
*   As being said, this mode is the default operation mode after power-on or RESET. Any other
*   mode must be chosen to be able to read any sensor data.
*/
#define CONFIGMODE 0b0000

/**
*   ACCONLY
*   If the application requires only raw accelerometer data, this mode can be chosen. In this
*   mode the other sensors (magnetometer, gyro) are suspended to lower the power
*   consumption. In this mode, the BNO055 behaves like a stand-alone acceleration sensor
*/
#define ACCONLY 0b0001

/**
*   MAGONLY
*   In MAGONLY mode, the BNO055 behaves like a stand-alone magnetometer, with
*   acceleration sensor and gyroscope being suspended.
*/
#define MAGONLY 0b0010

/**
*   GYROONLY
*   In GYROONLY mode, the BNO055 behaves like a stand-alone gyroscope, with acceleration
*   sensor and magnetometer being suspended.
*/
#define GYROONLY 0b0011

/**
*   ACCMAG
*   Both accelerometer and magnetometer are switched on, the user can read the data from
*   these two sensors.
*/
#define ACCMAG 0b0100

/**
*   ACCGYRO
*   Both accelerometer and gyroscope are switched on; the user can read the data from these
*   two sensors.
*/
#define ACCGYRO 0b0101

/**
*   Both magnetometer and gyroscope are switched on, the user can read the data from these
*   two sensors.
*/
#define MAGGYRO 0b0110

/**
*   AMG (ACC-MAG-GYRO)
*   All three sensors accelerometer, magnetometer and gyroscope are switched on.
*/
#define AMG 0b0111

/**
*   IMU (Inertial Measurement Unit)
*   In the IMU mode the relative orientation of the BNO055 in space is calculated from the
*   accelerometer and gyroscope data. The calculation is fast (i.e. high output data rate).
*/
#define IMU 0b1000

/**
*   COMPASS
*   The COMPASS mode is intended to measure the magnetic earth field and calculate the
*   geographic direction.
*   The earth magnetic field is a vector with the horizontal components x,y and the vertical z
*   component. It depends on the position on the globe and natural iron occurrence. For
*   heading calculation (direction of compass pointer) only the horizontal components x and y
*   are used. Therefore the vector components of the earth magnetic field must be transformed
*   in the horizontal plane, which requires the knowledge of the direction of the gravity vector.
*   To summarize, the heading can only be calculated when considering gravity and magnetic
*   field at the same time.
*   However, the measurement accuracy depends on the stability of the surrounding magnetic
*   field. Furthermore, since the earth magnetic field is usually much smaller than the magnetic
*   fields that occur around and inside electronic devices, the compass mode requires
*   calibration
*/
#define COMPASS 0b1001

/**
*   M4G (Magnet for Gyroscope)
*   The M4G mode is similar to the IMU mode, but instead of using the gyroscope signal to
*   detect rotation, the changing orientation of the magnetometer in the magnetic field is used.
*   Since the magnetometer has much lower power consumption than the gyroscope, this mode
*   is less power consuming in comparison to the IMU mode. There are no drift effects in this
*   mode which are inherent to the gyroscope.
*   However, as for compass mode, the measurement accuracy depends on the stability of the
*   surrounding magnetic field.
*   For this m
*/
#define M4G 0b1010

/**
*   NDOF_FMC_OFF
*   This fusion mode is same as NDOF mode, but with the Fast Magnetometer Calibration
*   turned ‘OFF’. 
*/
#define NDOF_FMC_OFF 0b1011

/**
*   NDOF
*   This is a fusion mode with 9 degrees of freedom where the fused absolute orientation data
*   is calculated from accelerometer, gyroscope and the magnetometer. The advantages of
*   combining all three sensors are a fast calculation, resulting in high output data rate, and high
*   robustness from magnetic field distortions. In this mode the Fast Magnetometer calibration
*   is turned ON and thereby resulting in quick calibration of the magnetometer and higher
*   output data accuracy. The current consumption is slightly higher in comparison to the
*   NDOF_FMC_OFF fusion mode.
*/
#define NDOF 0b1100

// Accelerometer G ranges
#define BNO055_G_RANGE_2G 0b00
#define BNO055_G_RANGE_4G 0b01
#define BNO055_G_RANGE_8G 0b10
#define BNO055_G_RANGE_16G 0b11
// Gyroscope G ranges
#define BNO055_G_RANGE_2000_DPS 0b000
#define BNO055_G_RANGE_1000_DPS 0b001
#define BNO055_G_RANGE_500_DPS 0b010
#define BNO055_G_RANGE_250_DPS 0b011
#define BNO055_G_RANGE_125_DPS 0b100


// Accelerometer bandwidth values
#define BNO055_G_BANDWIDTH_7HZ81 0b000
#define BNO055_G_BANDWIDTH_15HZ63 0b001
#define BNO055_G_BANDWIDTH_31HZ25 0b010
#define BNO055_G_BANDWIDTH_62HZ5 0b011
#define BNO055_G_BANDWIDTH_125HZ 0b100
#define BNO055_G_BANDWIDTH_250HZ 0b101
#define BNO055_G_BANDWIDTH_500HZ 0b110
#define BNO055_G_BANDWIDTH_1000HZ 0b111
// Gyroscope bandwidth values
#define BNO055_G_BANDWIDTH_523_HZ 0b000
#define BNO055_G_BANDWIDTH_230_HZ 0b001
#define BNO055_G_BANDWIDTH_116_HZ 0b010
#define BNO055_G_BANDWIDTH_47_HZ 0b011
#define BNO055_G_BANDWIDTH_23_HZ 0b100
#define BNO055_G_BANDWIDTH_12_HZ 0b101
#define BNO055_G_BANDWIDTH_64_HZ 0b110
#define BNO055_G_BANDWIDTH_32_HZ 0b111

// Accelerometer operating modes
#define BNO055_ACCELEROMETER_OP_MODE_NORMAL 0b000
#define BNO055_ACCELEROMETER_OP_MODE_SUSPEND 0b001
#define BNO055_ACCELEROMETER_OP_MODE_LOW_POWER_1 0b010
#define BNO055_ACCELEROMETER_OP_MODE_STANDBY 0b011
#define BNO055_ACCELEROMETER_OP_MODE_LOW_POWER_2 0b100
#define BNO055_ACCELEROMETER_OP_MODE_DEEP_SUSPEND 0b101
// Gyroscope operating modes
#define BNO055_GYROSCOPE_OP_MODE_NORMAL 0b000
#define BNO055_GYROSCOPE_OP_MODE_FAST_POWERUP 0b001
#define BNO055_GYROSCOPE_OP_MODE_DEEP_SUSPEND 0b010
#define BNO055_GYROSCOPE_OP_MODE_SUSPEND 0b011
#define BNO055_GYROSCOPE_OP_MODE_ADVANCED_POWERSAVE 0b100

// Unit selection
#define ACCELERATION_UNIT_MS2 0
#define ACCELERATION_UNIT_MG 1
#define ANGULAR_RATE_UNIT_DPS 0
#define ANGULAR_RATE_UNIT_RPS 1
#define EULAR_ANGLES_UNIT_DEG 0
#define EULAR_ANGLS_UNIT_RAD 1
#define TEMPERATURE_UNIT_C 0
#define TEMPERATURE_UNIT_F 1

void bno055_set_reset_pin(bno055_t * instance, uint8_t reset_pin);

void bno055_reset(bno055_t * instance);

/**
*   @brief Set the I2C address of a BNO055 instance. Default address is 0x29
*   @param instance A pointer to the bno055_t instance
*   @param address The I2C slave address to be assignd that instance
*/
void bno055_set_i2c_address(bno055_t * instance, uint8_t address);

/**
*   @brief Initialize the bno055_t instance. If the device is not at address 0x29
*          it is important to call bno055_set_i2c_address() before initializing
*   @param instance Pointer to the bno055_t instance
*/
int bno055_initialize(bno055_t * instance);

/**
*   @brief Set the operating mode of the BNO055
*   @param instance Pointer to the bno055_t instance
*   @param mode The operation mode
*/
int bno055_set_operating_mode(bno055_t * instance, uint8_t mode);

/**
*   @brief Set accelerometer g range
*   @param instance Pointer to the bno055_t instance
*   @param range The accelerometer g range. Choices are:
*                   - BNO055_G_RANGE_2G (Default)
*                   - BNO055_G_RANGE_4G
*                   - BNO055_G_RANGE_8G
*                   - BNO055_G_RANGE_16G
*/
int bno055_set_accelerometer_g_range(bno055_t * instance, uint8_t range);

/**
*   @brief Set accelerometer bandwidth
*   @param instance Pointer to the bno055_t instance
*   @param bandwidth The accelerometer bandwidth. Choices are:
*                       - BNO055_G_BANDWIDTH_7HZ81  (Default)
*                       - BNO055_G_BANDWIDTH_15HZ63
*                       - BNO055_G_BANDWIDTH_31HZ25
*                       - BNO055_G_BANDWIDTH_62HZ5
*                       - BNO055_G_BANDWIDTH_125HZ
*                       - BNO055_G_BANDWIDTH_250HZ
*                       - BNO055_G_BANDWIDTH_500HZ
*                       - BNO055_G_BANDWIDTH_1000HZ
*/
int bno055_set_accelerometer_bandwidth(bno055_t * instance, uint8_t bandwidth);

/**
*   @brief Set accelerometer operating mode
*   @param instance Pointer to the bno055_t instance
*   @param op_mode The operating mode. Choices are:
*                       - BNO055_ACCELEROMETER_OP_MODE_NORMAL   (Default)
*                       - BNO055_ACCELEROMETER_OP_MODE_SUSPEND
*                       - BNO055_ACCELEROMETER_OP_MODE_LOW_POWER_1
*                       - BNO055_ACCELEROMETER_OP_MODE_STANDBY
*                       - BNO055_ACCELEROMETER_OP_MODE_LOW_POWER_2
*                       - BNO055_ACCELEROMETER_OP_MODE_DEEP_SUSPEND
*/
int bno055_set_accelerometer_operating_mode(bno055_t * instance, uint8_t op_mode);

/**
*   @brief Set gyroscope g range
*   @param instance Pointer to the bno055_t instance
*   @param range The gyroscope g range. Choices are:
*                   - BNO055_G_RANGE_2000_DPS   (Default)
*                   - BNO055_G_RANGE_1000_DPS
*                   - BNO055_G_RANGE_500_DPS
*                   - BNO055_G_RANGE_250_DPS
*                   - BNO055_G_RANGE_125_DPS
*/
int bno055_set_gyroscope_g_range(bno055_t * instance, uint8_t range);

/**
*   @brief Set gyroscope bandwidth
*   @param instance Pointer to the bno055_t instance
*   @param bandwidth The gyroscopes bandwidth. Choices are:
*                       - BNO055_G_BANDWIDTH_523_HZ (Default)
*                       - BNO055_G_BANDWIDTH_230_HZ
*                       - BNO055_G_BANDWIDTH_116_HZ
*                       - BNO055_G_BANDWIDTH_47_HZ
*                       - BNO055_G_BANDWIDTH_23_HZ
*                       - BNO055_G_BANDWIDTH_12_HZ
*                       - BNO055_G_BANDWIDTH_64_HZ
*                       - BNO055_G_BANDWIDTH_32_HZ
*/
int bno055_set_gyroscope_bandwidth(bno055_t * instance, uint8_t bandwidth);

/**
*   @brief Set gyroscope operating mode
*   @param instance Pointer to the bno055_t instance
*   @param op_mode The gyroscope operating mode. Choices are:
*                       - BNO055_GYROSCOPE_OP_MODE_NORMAL   (Default)
*                       - BNO055_GYROSCOPE_OP_MODE_FAST_POWERUP
*                       - BNO055_GYROSCOPE_OP_MODE_DEEP_SUSPEND
*                       - BNO055_GYROSCOPE_OP_MODE_SUSPEND
*                       - BNO055_GYROSCOPE_OP_MODE_ADVANCED_POWERSAVE
*/
int bno055_set_gyroscope_operating_mode(bno055_t * instance, uint8_t op_mode);

/**
*   @brief Set acceleration unit
*   @param instance Pointer to the bno055_t instance
*   @param unit The unit. Choices are:
*                   - ACCELERATION_UNIT_MS2
*                   - ACCELERATION_UNIT_MG
*/
int bno055_set_acceleration_unit(bno055_t * instance, uint8_t unit);

/**
*   @brief Set angular rate unit
*   @param instance Pointer to the bno055_t instance
*   @param unit The unit. Choices are:
*                   - ANGULAR_RATE_UNIT_DPS
*                   - ANGULAR_RATE_UNIT_RPS
*/
int bno055_set_angular_rate_unit(bno055_t * instance, uint8_t unit);

/**
*   @brief Set eular angles unit
*   @param instance Pointer to the bno055_t instance
*   @param unit The unit. Choices are:
*                   - EULAR_ANGLES_UNIT_DEG
*                   - EULAR_ANGLS_UNIT_RAD
*/
int bno055_set_eular_angles_unit(bno055_t * instance, uint8_t unit);

/**
*   @brief Set temperature unit
*   @param instance Pointer to the bno055_t instance
*   @param unit The unit. Choices are:
*                   - TEMPERATURE_UNIT_C
*                   - TEMPERATURE_UNIT_F
*/
int bno055_set_temperature_unit(bno055_t * instance, uint8_t unit);

/**
*   @brief Read accelerometer data in x axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t accelerometer data in selected unit
*/
int16_t bno055_read_accelerometer_x_data(bno055_t * instance);

/**
*   @brief Read accelerometer data in y axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t accelerometer data in selected unit
*/
int16_t bno055_read_accelerometer_y_data(bno055_t * instance);

/**
*   @brief Read accelerometer data in z axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t accelerometer data in selected unit
*/
int16_t bno055_read_accelerometer_z_data(bno055_t * instance);


/**
*   @brief Read magnetc field strength data in x axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t magnetc data
*/
int16_t bno055_read_magnetic_field_strength_x_data(bno055_t * instance);

/**
*   @brief Read magnetc field strength data in y axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t magnetc data
*/
int16_t bno055_read_magnetic_field_strength_y_data(bno055_t * instance);

/**
*   @brief Read magnetc field strength data in z axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t magnetc data
*/
int16_t bno055_read_magnetic_field_strength_z_data(bno055_t * instance);


/**
*   @brief Read gyro data in x axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gyro data in selected unit
*/
int16_t bno055_read_gyro_x_data(bno055_t * instance);

/**
*   @brief Read gyro data in y axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gyro data in selected unit
*/
int16_t bno055_read_gyro_y_data(bno055_t * instance);

/**
*   @brief Read gyro data in z axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gyro data in selected unit
*/
int16_t bno055_read_gyro_z_data(bno055_t * instance);


/**
*   @brief Read eular heading
*   @param instance Pointer to the bno055_t instance
*   @return int16_t eular heading in selected unit
*/
int16_t bno055_read_eular_heading(bno055_t * instance);

/**
*   @brief Read eular pitch
*   @param instance Pointer to the bno055_t instance
*   @return int16_t eular pitch in selected unit
*/
int16_t bno055_read_eular_pitch(bno055_t * instance);

/**
*   @brief Read eular roll
*   @param instance Pointer to the bno055_t instance
*   @return int16_t eular roll in selected unit
*/
int16_t bno055_read_eular_roll(bno055_t * instance);


/**
*   @brief Read quaternion w data
*   @param instance Pointer to the bno055_t instance
*   @return int16_t quaternion data
*/
int16_t bno055_read_quaternion_w_data(bno055_t * instance);

/**
*   @brief Read quaternion x data
*   @param instance Pointer to the bno055_t instance
*   @return int16_t quaternion data
*/
int16_t bno055_read_quaternion_x_data(bno055_t * instance);

/**
*   @brief Read quaternion y data
*   @param instance Pointer to the bno055_t instance
*   @return int16_t quaternion data
*/
int16_t bno055_read_quaternion_y_data(bno055_t * instance);

/**
*   @brief Read quaternion z data
*   @param instance Pointer to the bno055_t instance
*   @return int16_t quaternion data
*/
int16_t bno055_read_quaternion_z_data(bno055_t * instance);


/**
*   @brief Read linear acceleration in x axis (Only available in fusion mode)
*   @param instance Pointer to the bno055_t instance
*   @return int16_t linear acceleration data
*/
int16_t bno055_read_linear_acceleration_x_data(bno055_t * instance);

/**
*   @brief Read linear acceleration in y axis (Only available in fusion mode)
*   @param instance Pointer to the bno055_t instance
*   @return int16_t linear acceleration data
*/
int16_t bno055_read_linear_acceleration_y_data(bno055_t * instance);

/**
*   @brief Read linear acceleration in z axis (Only available in fusion mode)
*   @param instance Pointer to the bno055_t instance
*   @return int16_t linear acceleration data
*/
int16_t bno055_read_linear_acceleration_z_data(bno055_t * instance);

/**
*   @brief Read graity vector x axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gravity vector data
*/
int16_t bno055_read_gravity_vector_x_data(bno055_t * instance);

/**
*   @brief Read graity vector y axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gravity vector data
*/
int16_t bno055_read_gravity_vector_y_data(bno055_t * instance);

/**
*   @brief Read graity vector z axis
*   @param instance Pointer to the bno055_t instance
*   @return int16_t gravity vector data
*/
int16_t bno055_read_gravity_vector_z_data(bno055_t * instance);

int8_t bno055_read_temperature_data(bno055_t * instance);

// Enable interrupts
int bno055_enable_accelerometer_no_motion_interrupt(bno055_t * instance);
int bno055_disable_accelerometer_no_motion_interrupt(bno055_t * instance);
int bno055_enable_accelerometer_any_motion_interrupt(bno055_t * instance);
int bno055_disable_accelerometer_any_motion_interrupt(bno055_t * instance);
int bno055_enable_accelerometer_high_g_interrupt(bno055_t * instance);
int bno055_disable_accelerometer_high_g_interrupt(bno055_t * instance);
int bno055_enable_gyro_high_rate_interrupt(bno055_t * instance);
int bno055_disable_gyro_high_rate_interrupt(bno055_t * instance);
int bno055_enable_gyro_any_motion_interrupt(bno055_t * instance);
int bno055_disable_gyro_any_motion_interrupt(bno055_t * instance);