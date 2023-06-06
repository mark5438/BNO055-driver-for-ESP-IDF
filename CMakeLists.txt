idf_component_register(SRCS "gpio.c" "bno055.c"
                    INCLUDE_DIRS "."
                    REQUIRES i2c_master driver)
