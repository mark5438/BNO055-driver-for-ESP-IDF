int i2c_master_init();
int i2c_master_write_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, unsigned int size);
void i2c_master_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, unsigned int size);