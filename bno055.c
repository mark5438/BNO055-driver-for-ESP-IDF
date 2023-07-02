#include "bno055.h"

#include "i2c_master.h"

#include "esp_log.h"

#include "gpio.h"

/**
*   @brief All the bno055 registers are multiplexed to have 2 functions. When the page_id is set to 0,
*   they have function 0, and when page_id is set to 1 they have function 0;
*
*   @param page The pagenumber. 0x00 or 0x01
*/
int set_page(bno055_t * instance, uint8_t page)
{
    if(page == instance->last_page)
        return 0;   // Device already on correct page. No need to change
    if(!(page == 0x00 || page == 0x01))
        return -1;  // Page must be 1 or 0
    int ret = i2c_master_write_register(instance->i2c_address, PAGE_ID, &page, 1);
    instance->last_page = page;
    return ret;
}

/**
*   @brief Assign a value to specific bits in a register, and leave all other bits untouched
*   @param instance A pointer to a bno055 object
*   @param page The page the register address is on
*   @param reg_address The register address
*   @param first_bit The integer value of the first bit o be changed
*   @param n_bits The number of bits to be changed
*   @param value The value to assign the bits in the register
*
*   Function automatically checks whether the value is too big for the number of bits
*
*   @return int non-zero for error
*/
int bno055_set_register_bits_vlaue(bno055_t * instance, uint8_t page, uint8_t reg_address, uint8_t first_bit, uint8_t n_bits, uint8_t value)
{
    if(value > (1 << (n_bits + 1)) - 1)
        return -1;
    set_page(instance, page);
    uint8_t old_value;
    i2c_master_read_register(instance->i2c_address, reg_address, &old_value, 1);
    for(int i = first_bit; i < first_bit + n_bits; i++)
        old_value &= ~(1 << i);
    old_value |= (value << n_bits);
    i2c_master_write_register(instance->i2c_address, reg_address, &old_value, 1);
    return 0;
}

/**
 *  @brief Read data value from 2 registers, and combine to int16
 *  @param instance A pointer to the bno055 object
 *  @param lsb_reg The address to the data register holding the least significant byte
 *  @param msb_reg The address to the data register holding the most signigicant byte
*/
int16_t read_two_registers(bno055_t * instance, uint8_t lsb_reg, uint8_t msb_reg)
{
    uint8_t buf;
    i2c_master_read_register(instance->i2c_address, lsb_reg, &buf, 1);
    int16_t ret = buf;
    i2c_master_read_register(instance->i2c_address, msb_reg, &buf, 1);
    ret |= buf << 8;
    return ret;
}

void bno055_set_reset_pin(bno055_t * instance, uint8_t reset_pin)
{
    instance->reset_pin = reset_pin;
    configure_gpio_output(instance->reset_pin);
    gpio_high(instance->reset_pin);
}

void bno055_reset(bno055_t * instance)
{
    gpio_low(instance->reset_pin);
    gpio_high(instance->reset_pin);
}

void bno055_set_i2c_address(bno055_t * instance, uint8_t address)
{
    instance->i2c_address = address;
}

int bno055_initialize(bno055_t * instance)
{
    instance->last_page = 0; // Device has page 0 selected as default
    int ret =  i2c_master_init();
    return ret;
}

int bno055_set_operating_mode(bno055_t * instance, uint8_t mode)
{
    int err = set_page(instance, 0);
    if(err)
        return err;
    return i2c_master_write_register(instance->i2c_address, OPR_MODE, &mode, 1);
}

int bno055_set_accelerometer_g_range(bno055_t * instance, uint8_t range)
{
    return bno055_set_register_bits_vlaue(instance, 1, ACC_CONFIG, ACC_CONFIG_G_RANGE_FIRST_BIT, ACC_CONFIG_G_RANGE_N_BITS, range);
}

int bno055_set_accelerometer_bandwidth(bno055_t * instance, uint8_t bandwidth)
{
    return bno055_set_register_bits_vlaue(instance, 1, ACC_CONFIG, ACC_CONFIG_BANDWIDTH_FIRST_BIT, ACC_CONFIG_BANDWIDTH_N_BITS, bandwidth);
}

int bno055_set_accelerometer_operating_mode(bno055_t * instance, uint8_t op_mode)
{
    if(op_mode > 5)
        return -1;  // Unknown mode
    return bno055_set_register_bits_vlaue(instance, 1, ACC_CONFIG, ACC_CONFIG_OPERATION_MODE_FIRST_BIT, ACC_CONFIG_OPERATION_MODE_N_BITS, op_mode);
}

int bno055_set_gyroscope_g_range(bno055_t * instance, uint8_t range)
{
    return bno055_set_register_bits_vlaue(instance, 1, GYRO_CONFIG_0, GYRO_CONFIG_G_RANGE_FIRST_BIT, GYRO_CONFIG_G_RANGE_N_BITS, range);
}

int bno055_set_gyroscope_bandwidth(bno055_t * instance, uint8_t bandwidth)
{
    return bno055_set_register_bits_vlaue(instance, 1, GYRO_CONFIG_0, GYRO_CONFIG_BANDWIDTH_FIRST_BIT, GYRO_CONFIG_BANDWIDTH_N_BITS, bandwidth);
}

int bno055_set_gyroscope_operating_mode(bno055_t * instance, uint8_t op_mode)
{
    if(op_mode > 5)
        return -1;  // Unknown mode
    return bno055_set_register_bits_vlaue(instance, 1, GYRO_CONFIG_1, GYRO_CONFIG_OPERATION_MODE_FIRST_BIT, GYRO_CONFIG_OPERATION_MODE_N_BITS, op_mode);
}

int bno055_set_acceleration_unit(bno055_t * instance, uint8_t unit)
{
    return bno055_set_register_bits_vlaue(instance, 0, UNIT_SEL, ACC_UNIT, 1, unit);
}

int bno055_set_angular_rate_unit(bno055_t * instance, uint8_t unit)
{
    return bno055_set_register_bits_vlaue(instance, 0, UNIT_SEL, GYR_UNIT, 1, unit);
}

int bno055_set_eular_angles_unit(bno055_t * instance, uint8_t unit)
{
    return bno055_set_register_bits_vlaue(instance, 0, UNIT_SEL, EUL_UNIT, 1, unit);
}

int bno055_set_temperature_unit(bno055_t * instance, uint8_t unit)
{
    return bno055_set_register_bits_vlaue(instance, 0, UNIT_SEL, TEMP_UNIT, 1, unit);
}

int16_t bno055_read_accelerometer_x_data(bno055_t * instance)
{
    return read_two_registers(instance, ACC_DATA_X_LSB, ACC_DATA_X_MSB);
}

int16_t bno055_read_accelerometer_y_data(bno055_t * instance)
{
    return read_two_registers(instance, ACC_DATA_Y_LSB, ACC_DATA_Y_MSB);
}

int16_t bno055_read_accelerometer_z_data(bno055_t * instance)
{
    return read_two_registers(instance, ACC_DATA_Z_LSB, ACC_DATA_Z_MSB);
}

int16_t bno055_read_magnetic_field_strength_x_data(bno055_t * instance)
{
    return read_two_registers(instance, MAG_DATA_X_LSB, MAG_DATA_X_MSB);
}

int16_t bno055_read_magnetic_field_strength_y_data(bno055_t * instance)
{
    return read_two_registers(instance, MAG_DATA_Y_LSB, MAG_DATA_Y_MSB);
}

int16_t bno055_read_magnetic_field_strength_z_data(bno055_t * instance)
{
    return read_two_registers(instance, MAG_DATA_Z_LSB, MAG_DATA_Z_MSB);
}

int16_t bno055_read_gyro_x_data(bno055_t * instance)
{
    return read_two_registers(instance, GYR_DATA_X_LSB, GYR_DATA_X_MSB);
}

int16_t bno055_read_gyro_y_data(bno055_t * instance)
{
    return read_two_registers(instance, GYR_DATA_Y_LSB, GYR_DATA_Y_MSB);
}

int16_t bno055_read_gyro_z_data(bno055_t * instance)
{
    return read_two_registers(instance, GYR_DATA_Z_LSB, GYR_DATA_Z_MSB);
}

int16_t bno055_read_eular_heading(bno055_t * instance)
{
    return read_two_registers(instance, EUL_HEADING_LSB, EUL_HEADING_MSB);
}

int16_t bno055_read_eular_pitch(bno055_t * instance)
{
    return read_two_registers(instance, EUL_PITCH_LSB, EUL_PITCH_MSB);
}

int16_t bno055_read_eular_roll(bno055_t * instance)
{
    return read_two_registers(instance, EUL_ROLL_LSB, EUL_ROLL_MSB);
}

int16_t bno055_read_quaternion_w_data(bno055_t * instance)
{
    return read_two_registers(instance, QUA_DATA_W_LSB, QUA_DATA_W_MSB);
}

int16_t bno055_read_quaternion_x_data(bno055_t * instance)
{
    return read_two_registers(instance, QUA_DATA_X_LSB, QUA_DATA_X_MSB);
}

int16_t bno055_read_quaternion_y_data(bno055_t * instance)
{
    return read_two_registers(instance, QUA_DATA_Y_LSB, QUA_DATA_Y_MSB);
}

int16_t bno055_read_quaternion_z_data(bno055_t * instance)
{
    return read_two_registers(instance, QUA_DATA_Z_LSB, QUA_DATA_Z_MSB);
}

int16_t bno055_read_linear_acceleration_x_data(bno055_t * instance)
{
    return read_two_registers(instance, LIA_DATA_X_LSB, LIA_DATA_X_MSB);
}

int16_t bno055_read_linear_acceleration_y_data(bno055_t * instance)
{
    return read_two_registers(instance, LIA_DATA_Y_LSB, LIA_DATA_Y_MSB);
}

int16_t bno055_read_linear_acceleration_z_data(bno055_t * instance)
{
    return read_two_registers(instance, LIA_DATA_Z_LSB, LIA_DATA_Z_MSB);
}

int16_t bno055_read_gravity_vector_x_data(bno055_t * instance)
{
    return read_two_registers(instance, GRV_DATA_X_LSB, GRV_DATA_X_MSB);
}

int16_t bno055_read_gravity_vector_y_data(bno055_t * instance)
{
    return read_two_registers(instance, GRV_DATA_Y_LSB, GRV_DATA_Y_MSB);
}

int16_t bno055_read_gravity_vector_z_data(bno055_t * instance)
{
    return read_two_registers(instance, GRV_DATA_Z_LSB, GRV_DATA_Z_MSB);
}

int8_t bno055_read_temperature_data(bno055_t * instance)
{
    uint8_t buf;
    i2c_master_read_register(instance->i2c_address, TEMP, &buf, 1);
    return buf;
}

int bno055_enable_accelerometer_no_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_NM, 1, 1);
}

int bno055_disable_accelerometer_no_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_NM, 1, 0);
}

int bno055_enable_accelerometer_any_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_AM, 1, 1);
}

int bno055_disable_accelerometer_any_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_AM, 1, 0);
}

int bno055_enable_accelerometer_high_g_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_HIGH_G, 1, 1);
}

int bno055_disable_accelerometer_high_g_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, ACC_HIGH_G, 1, 0);
}

int bno055_enable_gyro_high_rate_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, GYRO_HIGH_RATE, 1, 1);
}

int bno055_disable_gyro_high_rate_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, GYRO_HIGH_RATE, 1, 0);
}

int bno055_enable_gyro_any_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, GYRO_AM, 1, 1);
}

int bno055_disable_gyro_any_motion_interrupt(bno055_t * instance)
{
    return bno055_set_register_bits_vlaue(instance, 1, INT_EN, GYRO_AM, 1, 0);
}
