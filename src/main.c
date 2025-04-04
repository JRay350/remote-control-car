#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include "nec_receive.h"

#define RED_LED_PIN 7
#define USER_BUTTON 10
#define MOTOR_DRIVER_SDA 4
#define MOTOR_DRIVER_SCL 5
#define RX_GPIO 0

#define MOTOR_DRIVER_ADDR (0x22)

#define OFF 0
#define ON 1

#define UP_BUTTON 0x18
#define DOWN_BUTTON 0x52
#define LEFT_BUTTON 0x08
#define RIGHT_BUTTON 0x5a
#define STOP_BUTTON 0x1c

#define BIT(n)  (1u<<(n)) // Macro to define a bitmask

void DriveMotor(uint8_t motor_number, uint8_t speed, uint8_t direction) {
    uint8_t data[5];
    data[0] = (0x26);
    data[1] = motor_number;
    data[2] = speed;
    data[3] = direction;
    data[4] = motor_number ^ speed ^ direction;

    i2c_write_blocking(i2c_default, MOTOR_DRIVER_ADDR, data, 5, false);
}

void DriveMotorForward() {
    DriveMotor(1, 255, 1);
    DriveMotor(2, 255, 1);
}

void DriveMotorLeft() {
    DriveMotor(1, 255, 1);
}

void DriveMotorRight() {
    DriveMotor(2, 255, 1);
}

void DriveMotorReverse() {
    DriveMotor(1, 255, 0);
    DriveMotor(2, 255, 0);
}

void DriveMotorStop() {
    DriveMotor(1, 0, 0);
    DriveMotor(2, 0, 0);
}

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
    PIO pio = pio0;
    int rx_sm = nec_rx_init(pio, RX_GPIO);

    uint8_t rx_address = 0x00;
    uint8_t rx_data = 0x00;

    while (1) {
        if (!pio_sm_is_rx_fifo_empty(pio, rx_sm)) {
            uint32_t rx_frame = pio_sm_get(pio, rx_sm);
            if (nec_decode_frame(rx_frame, &rx_address, &rx_data)) {
                switch(rx_data) {
                    case UP_BUTTON:
                        DriveMotorStop();
                        DriveMotorForward();
                        break;
                    case RIGHT_BUTTON:
                        DriveMotorStop();
                        DriveMotorRight();
                        break;
                    case DOWN_BUTTON:
                        DriveMotorStop();
                        DriveMotorReverse();
                        break;
                    case LEFT_BUTTON:
                        DriveMotorStop();
                        DriveMotorLeft();
                        break;
                    case STOP_BUTTON:
                        DriveMotorStop();
                        break;
                    default:
                        break;
                }
                rx_data = 0;
            }
        }
        vTaskDelay(10);
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

    PIO pio = pio0;
    int rx_sm = nec_rx_init(pio, RX_GPIO);

    uint8_t rx_address = 0x00;
    uint8_t rx_data = 0x00;

    TaskHandle_t rLEDTask = NULL;
    TaskHandle_t dMotorsTask = NULL;

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
            NULL,
            1,
            &dMotorsTask);
    
    while (gpio_get(USER_BUTTON) == OFF) {

    }
    sleep_ms(300); // to avoid debounce
    vTaskStartScheduler();
        
    while(1) {
        // should never reach this
    }
}