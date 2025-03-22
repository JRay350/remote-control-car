#include <stdio.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define RED_LED_PIN 7
#define GREEN_LED_PIN PICO_DEFAULT_LED_PIN
#define OFF 0
#define ON 1

void RedLEDTask(void* param) {
    while (1) {
        // Blink LED
        gpio_put(RED_LED_PIN, ON);
        vTaskDelay(1000);
        gpio_put(RED_LED_PIN, OFF);
        vTaskDelay(1000);
    }
}

void GreenLEDTask(void* param) {
    while (1) {
        // Blink LED
        gpio_put(GREEN_LED_PIN, ON);
        vTaskDelay(1000);
        gpio_put(GREEN_LED_PIN, OFF);
        vTaskDelay(1000);
    }
}

int main() {
    stdio_init_all();

    // Initialize Red LED pin
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);

    // Initialize Green LED pin
    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

    TaskHandle_t rLEDTask = NULL;
    TaskHandle_t gLEDTask = NULL;

    uint32_t status = xTaskCreate(
                    RedLEDTask,
                    "Red LED",
                    1024,
                    NULL,
                    tskIDLE_PRIORITY,
                    &rLEDTask);
                    
    status = xTaskCreate(
                    GreenLEDTask,
                    "Green LED",
                    1024,
                    NULL,
                    1,
                    &gLEDTask);   

    vTaskStartScheduler();
        
    while(1) {
        // should never reach this
    }
}