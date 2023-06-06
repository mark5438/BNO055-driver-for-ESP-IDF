#include "gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

void configure_gpio_output(uint8_t pin)
{
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
}

void gpio_high(uint8_t pin)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, 1));
}

void gpio_low(uint8_t pin)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}