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
#include <math.h>

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

float pZero = 101325; // sea level pressure

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

// read BMP280 sensor values
float read_bmp280(bmp280_quantity quantity) {
	float temperature, pressure;
	bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL);
	if (quantity == BMP280_TEMPERATURE) {
		return temperature;
	}
	return pressure;
}

float FromTemperatureAndPressure(float t, float p) {
    return ((pow(pZero/p, 1/5.257)-1)*(t + 273.15))/(float)0.0065;
}

float FromTemperatureAndPressure2(float t, float p) {
    float base = pZero/(float)p;
    float exp = 1/(float)5.257;
    float power = pow(base, exp);
    return ((power-1)*(t + 273.15))/(float)0.0065;
}

//task to check if button is pressed and turn its light on
void komunikacijaTask(void *pvParameters)
{
	uint8_t data;
	uint8_t dataoff = leds_off;
	uint8_t data1 = led1;
	//uint8_t data2 = led2;
	//uint8_t data3 = led3;
	uint8_t data4 = led4;
	float bmpstart = 0;
	float bmpcurr = 0;
	float diff = 0;
	int precision = 30;

	while(1){
		i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &dataoff, 1);
		i2c_slave_read(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
		bmpcurr = read_bmp280(BMP280_PRESSURE);
		printf("\nAUTO: %.2f Pa", bmpcurr);
		printf("\nTEMP: %.2f C", read_bmp280(BMP280_TEMPERATURE));
		//printf("\nHEIGHT: %.2f m", FromTemperatureAndPressure2(read_bmp280(BMP280_TEMPERATURE), bmpcurr));
		if((data & button1) == 0){
			//i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data1, 1);
			printf("\nBTN/LED1");
			bmpstart = bmpcurr;
			printf("\nStart: %.2f Pa", bmpstart);
		}
		/*if((data & button2) == 0){
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
		}*/
		diff = bmpcurr-bmpstart;
		// - pomeni, da je trenutno visje
		if(diff < 0){
			if(diff>-1*precision){
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data1, 1);
			}
			else if(diff>-2*precision){
				uint8_t datax = led1 & led2;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
			else if(diff>-3*precision){
				uint8_t datax = led1 & led2 & led3;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
			else{
				uint8_t datax = led1 & led2 & led3 & led4;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
		}
		// + pomeni, da je trenutno nizje
		else{
			if(diff<precision){
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data4, 1);
			}
			else if(diff<2*precision){
				uint8_t datax = led4 & led3;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
			else if(diff<3*precision){
				uint8_t datax = led4 & led3 & led2;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
			else{
				uint8_t datax = led1 & led2 & led3 & led4;
				i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
			}
		}
        vTaskDelay(500 / portTICK_PERIOD_MS);
    	//uint8_t datax = led1 & led2 & led3 & led4;
		//i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &datax, 1);
        //vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);

    // BMP280 configuration
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_NORMAL;
	params.filter = BMP280_FILTER_4;
	params.oversampling_pressure = BMP280_STANDARD;
	params.oversampling_temperature = BMP280_FILTER_4;
	params.standby = BMP280_STANDBY_500;
	bmp280_dev.i2c_dev.bus = I2C_BUS;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);

    xTaskCreate(komunikacijaTask, "komunikacijaTask", 256, NULL, 2, NULL);
}
