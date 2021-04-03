/* The classic "blink" example
 *
 * This sample code is in the public domain.
 */
#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

const int gpio = 2;

#define I2C_BUS		0
#define SCL_PIN		14
#define SDA_PIN		12
#define PCF_ADDRESS 0x38

#define button1		0x20	// 0b ??0? ????
#define button2		0x10	// 0b ???0 ????
#define button3		0x80	// 0b 0??? ????
#define button4		0x40	// 0b ?0?? ????

#define led1 		0xfe	// 0b ???? ???0
#define led2 		0xfd	// 0b ???? ??0?
#define led3 		0xfb	// 0b ???? ?0??
#define led4 		0xf7	// 0b ???? 0???
#define leds_off	0xff

bmp280_t bmp280_dev;

// read BMP280 sensor values
float read_bmp280() {

	float temperature, pressure;

	//bmp280_force_measurement(&bmp280_dev);
	// wait for measurement to complete
	//while (bmp280_is_measuring(&bmp280_dev)) {
	//};
	bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL);

	return pressure;
}

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

//task to check if button is pressed and turn its light on
void komunikacijaTask(void *pvParameters)
{
	uint8_t data;
	uint8_t dataoff = leds_off;
	uint8_t data1 = led1;
	uint8_t data2 = led2;
	uint8_t data3 = led3;
	uint8_t data4 = led4;

    // BMP280 configuration
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_NORMAL;
	params.filter = BMP280_FILTER_4;
	params.oversampling_pressure = BMP280_STANDARD;
	params.standby = BMP280_STANDBY_500;
	bmp280_dev.i2c_dev.bus = I2C_BUS;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);
	while(1){
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &dataoff, 1);
		i2c_slave_read(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		printf("\nAUTO: %.2f Pa", read_bmp280());
		if((data & button1) == 0){
			i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data1, 1);
			printf("\nBTN/LED1");
			printf("\nPressure: %.2f Pa", read_bmp280());
		}
		if((data & button2) == 0){
			i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data2, 1);
			printf("\nBTN/LED2");
		}
		if((data & button3) == 0){
			i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data3, 1);
			printf("\nBTN/LED3");
		}
		if((data & button4) == 0){
			i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data4, 1);
			printf("\nBTN/LED4");
		}
        vTaskDelay(500 / portTICK_PERIOD_MS);
    	//uint8_t datax = led1 & led2 & led3 & led4;
		//i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
        //vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


/* This task demonstrates an alternative way to use raw register
   operations to blink an LED.

   The step that sets the iomux register can't be automatically
   updated from the 'gpio' constant variable, so you need to change
   the line that sets IOMUX_GPIO2 if you change 'gpio'.

   There is no significant performance benefit to this way over the
   blinkenTask version, so it's probably better to use the blinkenTask
   version.

   NOTE: This task isn't enabled by default, see the commented out line in user_init.
*/
void blinkenRegisterTask(void *pvParameters)
{
    GPIO.ENABLE_OUT_SET = BIT(gpio);
    IOMUX_GPIO2 = IOMUX_GPIO2_FUNC_GPIO | IOMUX_PIN_OUTPUT_ENABLE; /* change this line if you change 'gpio' */
    while(1) {
        GPIO.OUT_SET = BIT(gpio);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        GPIO.OUT_CLEAR = BIT(gpio);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);

    //xTaskCreate(blinkenTask, "blinkenTask", 256, NULL, 2, NULL);
    xTaskCreate(komunikacijaTask, "komunikacijaTask", 256, NULL, 2, NULL);
    //xTaskCreate(blinkenRegisterTask, "blinkenRegisterTask", 256, NULL, 2, NULL);
}
