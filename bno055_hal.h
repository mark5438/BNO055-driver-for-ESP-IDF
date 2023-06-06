#include <stdint.h>

// PAGE 0 REGISTERS

// Radiuses
#define MAG_RADIUS_MSB 0x6A
#define MAG_RADIUS_LSB 0x69
#define MAG_ACC_MSB 0x68
#define MAG_ACC_LSB 0x67

// Offsets
#define GYR_OFFSET_Z_MSB 0x66
#define GYR_OFFSET_Z_LSB 0x65
#define GYR_OFFSET_Y_MSB 0x64
#define GYR_OFFSET_Y_LSB 0x63
#define GYR_OFFSET_X_MSB 0x62
#define GYR_OFFSET_X_LSB 0x61

#define MAG_OFFSET_Z_MSB 0x60
#define MAG_OFFSET_Z_LSB 0x5F
#define MAG_OFFSET_Y_MSB 0x5E
#define MAG_OFFSET_Y_LSB 0x5D
#define MAG_OFFSET_X_MSB 0x5C
#define MAG_OFFSET_X_LSB 0x5B

#define ACC_OFFSET_Z_MSB 0x5A
#define ACC_OFFSET_Z_LSB 0x59
#define ACC_OFFSET_Y_MSB 0x58
#define ACC_OFFSET_Y_LSB 0x57
#define ACC_OFFSET_X_MSB 0x56
#define ACC_OFFSET_X_LSB 0x55


#define AXIS_MAP_SIGN 0x42
#define AXIS_MAP_CONFIG 0x41
#define TEMP_SOURCE 0x40
#define SYS_TRIGGER 0x3F
#define PWR_MODE 0x3E
#define OPR_MODE 0x3D
#define UNIT_SEL 0x3B
// BITS:
#define ACC_UNIT 0
#define GYR_UNIT 1
#define EUL_UNIT 2
#define TEMP_UNIT 4
#define ORI_ANDROID_WINDOWS 7

#define SYS_ERR 0x3A
#define SYS_STATUS 0x39
#define SYS_CLK_STATUS 0x38
#define INT_STA 0x37
#define ST_RESULT 0x36
#define CALIB_STAT 0x35
#define TEMP 0x34

// Data registers
// Gravity Data
#define GRV_DATA_Z_MSB 0x33
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_X_LSB 0x2E

// Linear acceleration
#define LIA_DATA_Z_MSB 0x2D
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_X_LSB 0x28

// Quaternion acceleration
#define QUA_DATA_Z_MSB 0x27
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_W_LSB 0x20

// Attitude data
#define EUL_PITCH_MSB 0x1F
#define EUL_PITCH_LSB 0x1E
#define EUL_ROLL_MSB 0x1D
#define EUL_ROLL_LSB 0x1C
#define EUL_HEADING_MSB 0x1B
#define EUL_HEADING_LSB 0x1A

// Gyroscope data
#define GYR_DATA_Z_MSB 0x19
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_X_LSB 0x14

// Magnetometer data
#define MAG_DATA_Z_MSB 0x13
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_X_MSB 0xF
#define MAG_DATA_X_LSB 0xE

// Acceleration data
#define ACC_DATA_Z_MSB 0xD
#define ACC_DATA_Z_LSB 0xC
#define ACC_DATA_Y_MSB 0xB
#define ACC_DATA_Y_LSB 0xA
#define ACC_DATA_X_MSB 0x9
#define ACC_DATA_X_LSB 0x8

// META data
#define PAGE_ID 0x7
#define BL_REV_ID 0x6
#define SW_REV_ID_MSB 0x5
#define SW_REV_ID_LSB 0x4
#define GYR_ID 0x3
#define MAG_ID 0x2
#define ACC_ID 0x1
#define CHIP_ID 0x0

// Page 1
#define GYR_AM_SET 0x1F
#define GYR_AM_THRES 0x1E
#define GYR_DUR_Z 0x1D
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Y 0x1B
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_X 0x19
#define GYR_HR_X_SET 0x18
#define GYR_INT_SETING 0x17
#define ACC_NM_SET 0x16
#define ACC_NM_THRES 0x15
#define ACC_HG_THRES 0x14
#define ACC_HG_DURATION 0x13
#define ACC_INT_SETTINGS 0x12
#define ACC_AM_THRES 0x11
#define INT_EN 0x10
//BITS:
#define GYRO_AM 2
#define GYRO_HIGH_RATE 3
#define ACC_HIGH_G 5
#define ACC_AM 6
#define ACC_NM 7

#define INT_MASK 0xF
#define GYRO_SLEEP_CONFIG 0xD
#define ACC_SLEEP_CONFIG 0xC
#define GYRO_CONFIG_1 0xB
#define GYRO_CONFIG_0 0xA
//BITS:
#define GYRO_CONFIG_G_RANGE_FIRST_BIT 0
#define GYRO_CONFIG_G_RANGE_N_BITS 3
#define GYRO_CONFIG_BANDWIDTH_FIRST_BIT 3
#define GYRO_CONFIG_BANDWIDTH_N_BITS 3
#define GYRO_CONFIG_OPERATION_MODE_FIRST_BIT 0
#define GYRO_CONFIG_OPERATION_MODE_N_BITS 3

#define MAG_CONFIG 0x9

#define ACC_CONFIG 0x8
//BITS:
#define ACC_CONFIG_G_RANGE_FIRST_BIT 0
#define ACC_CONFIG_G_RANGE_N_BITS 2
#define ACC_CONFIG_BANDWIDTH_FIRST_BIT 2
#define ACC_CONFIG_BANDWIDTH_N_BITS 3
#define ACC_CONFIG_OPERATION_MODE_FIRST_BIT 5
#define ACC_CONFIG_OPERATION_MODE_N_BITS 3