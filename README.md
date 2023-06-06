# BNO055 Driver for ESP-IDF projects

A driver to configure and read data from a BNO055 using an ESP32 or other ESP-IDF capable boards.
This driver is also super easy to port. Only the functions in the files i2c_master.c and gpio.c need to be changed

## Installation

Clone this repo into your projects /components folder and add BNO055-driver-for-ESP-IDF to required components from all other components that needs to communicate with the BNO055

## Usage

```c
#include "bno055.h"

void app_main()
{
    bno055_t bno;

    bno055_set_i2c_address(&bno, 0x29); // Or 0x28
    bno055_initialize(&bno);
    bno055_set_operating_mode(NDOF);

    while (1)
    {
        int16_t accel_x = bno055_read_accelerometer_x_data();
    }
}
```

## License

[MIT](https://choosealicense.com/licenses/mit/)