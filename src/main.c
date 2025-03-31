#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#define RED_LED_PIN 7
#define USER_BUTTON 10
#define MOTOR_DRIVER_SDA 4
#define MOTOR_DRIVER_SCL 5

#define MOTOR_DRIVER_ADDR (0x22)

#define OFF 0
#define ON 1

#define BIT(n)  (1u<<(n)) // Macro to define a bitmask

struct motor_driver_values {
    uint8_t motor_number;
    uint8_t speed;
    uint8_t direction;
};

void RedLEDTask(void* param) {
    while (1) {
        // Blink LED
        gpio_put(RED_LED_PIN, ON);
        vTaskDelay(1000);
        gpio_put(RED_LED_PIN, OFF);
        vTaskDelay(1000);
    }
}

void DriveMotorsTask(void* param) {

    struct motor_driver_values* values = (struct motor_driver_values*) param;

    while (1) {
        uint8_t data[5];
        data[0] = (0x26);
        data[1] = (*values).motor_number;
        data[2] = (*values).speed;
        data[3] = (*values).direction;
        data[4] = (*values).motor_number ^ (*values).speed ^ (*values).direction;

        i2c_write_blocking(i2c_default, MOTOR_DRIVER_ADDR, data, 5, false);
    }

}

int main() {
    stdio_init_all();

    // Initialize Red LED pin
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);

    // Initialize User Button
    gpio_init(USER_BUTTON);
    gpio_set_dir(USER_BUTTON, GPIO_IN);
    gpio_pull_down(USER_BUTTON);

    // Inintialize Motor Driver
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(MOTOR_DRIVER_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MOTOR_DRIVER_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MOTOR_DRIVER_SDA);
    gpio_pull_up(MOTOR_DRIVER_SCL);

    TaskHandle_t rLEDTask = NULL;
    TaskHandle_t dMotorsTask = NULL;

    struct motor_driver_values motor_vals;
    motor_vals.motor_number = 1;
    motor_vals.speed = 150;
    motor_vals.direction = 1;


    uint32_t status = xTaskCreate(
                    RedLEDTask,
                    "Red LED",
                    1024,
                    NULL,
                    0,
                    &rLEDTask);

   status = xTaskCreate(
            DriveMotorsTask,
            "Drive Motors",
            1024,
            (void*) &motor_vals,
            0,
            &dMotorsTask);
    
    while (gpio_get(USER_BUTTON) == OFF) {

    }
    sleep_ms(300); // to avoid debounce
    vTaskStartScheduler();
        
    while(1) {
        // should never reach this
    }
}