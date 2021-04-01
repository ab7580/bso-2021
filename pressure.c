#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"


void showTasks(void *pvParameters)
{
    while(1) {
    	static char msg[500];
    	vTaskList(msg);
    	puts(msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void CountToThree(void) {
	while(1) {
	    	puts("1");
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	        puts("2");
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	        puts("3");
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	    }

	    vTaskDelete(NULL);
}

void alenTask(void) {
	while (1) {
		puts("alen");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

const int gpio = 2;

/* This task uses the high level GPIO API (esp_gpio.h) to blink an LED.
 *
 */
void blinkenTask(void *pvParameters)
{
    gpio_enable(gpio, GPIO_OUTPUT);
    while(1) {
        gpio_write(gpio, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_write(gpio, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    xTaskCreate(CountToThree, "CountToThree", 256, NULL, 2, NULL);
    xTaskCreate(alenTask, "alenTask", 256, NULL, 2, NULL);
    xTaskCreate(blinkenTask, "blink", 256, NULL, 2, NULL);
    // xTaskCreate(pxTaskCode, pcName, usStackDpeth, pvParameter, uxPriority, pxCreatedTask)
    /*
     * xTaskCreate(params).
     * Vrne, ali je uspeˇsno ali ne.
     * Params:
     * –pvTaskCode:  katera funkcija bo izvajala opravilo–pcName:  ime opravila
     * –uStackDepth:   globina  sklada  (koliko  elementov  lahko  uporablja  to  opravilo).   Dobra  privzetavrednost je 1000
     * –pvParameters:  parametri, ki jih lahko poˇsljemo v funkcijo pvTaskCode()
     * –uxPriority:  prioriteta
     * –pxCreatedTask:  kazalec na opravilo
     */

    // this is a comment
}
