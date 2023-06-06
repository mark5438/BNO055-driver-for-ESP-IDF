#include "driver/i2c.h"

#include "i2c_master.h"

#define I2C_MASTER_SCL_IO           19    /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           18    /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1

int i2c_master_init()
{
    // Configure the I2C bus
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    
    i2c_set_timeout(0, 1048575);

    // Install the I2C driver
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

int i2c_master_write_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, unsigned int size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, unsigned int size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Send the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Read the register value
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute the I2C command
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
}