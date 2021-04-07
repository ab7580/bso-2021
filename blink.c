#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"
#include "semphr.h"

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

const int pressuresSize = 10;
float pressures[10];
int pressureIndex = 0;

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

void write_byte_pcf(uint8_t data) {
	i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
}


uint8_t read_byte_pcf() {
	uint8_t data;
	i2c_slave_read(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
	return data;
}

void WriteToPressuresArray(float pressure) {
	int i = pressureIndex % pressuresSize;
	pressures[i] = pressure;
	pressureIndex += 1;
}

void ReadPressureEverySecond(void *pvParameters) {
	while (1) {
		float pressure = read_bmp280(BMP280_PRESSURE);
		printf("\nPressure: %.2f Pa", pressure);

		WriteToPressuresArray(pressure);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void GetLastTenPressuresAverage(void *pvParameters) {
	while (1) {
		// if pressures[9] == 0 then we still haven't made 10 measurements
		if (pressures[pressuresSize-1] == 0) {
			puts("array not filled yet");
			vTaskDelay((pressuresSize * 1000) / portTICK_PERIOD_MS);
			continue;
		}

		float avg = 0;
		for (int i=0; i<pressuresSize; ++i) {
			avg += pressures[i];
		}
		avg = avg/pressuresSize;
		printf("\n Average Pressure over last %d measurements: %.2f Pa",pressuresSize, avg);

		vTaskDelay((pressuresSize * 1000) / portTICK_PERIOD_MS);
	}
}

void InitPressuresArray() {
	for (int i=0; i<pressuresSize; i += 1){
		pressures[i] = 0;
	}
}
void komunikacijaTask(void *pvParameters)
{
	uint8_t data;
	float bmpstart = 0;
	float bmpcurr = 0;
	float diff = 0;
	int precision = 30;

	while(1){
		write_byte_pcf(leds_off);

		data = read_byte_pcf();

		bmpcurr = read_bmp280(BMP280_PRESSURE);

		printf("\nAUTO: %.2f Pa", bmpcurr);
		printf("\nTEMP: %.2f C", read_bmp280(BMP280_TEMPERATURE));

		if((data & button1) == 0){
			printf("\nBTN/LED1");
			bmpstart = bmpcurr;
			printf("\nStart: %.2f Pa", bmpstart);
		}
		diff = bmpcurr-bmpstart;

		// - pomeni, da je trenutno visje
		if(diff < 0){
			if(diff>-1*precision){
				write_byte_pcf(led1);
			}
			else if(diff>-2*precision){
				write_byte_pcf(led1 & led2);
			}
			else if(diff>-3*precision){
				write_byte_pcf(led1 & led2 & led3);
			}
			else{
				write_byte_pcf(led1 & led2 & led3 & led4);
			}
		}
		// + pomeni, da je trenutno nizje
		else{
			if(diff<precision){
				write_byte_pcf(led4);
			}
			else if(diff<2*precision){
				write_byte_pcf(led4 & led3);
			}
			else if(diff<3*precision){
				write_byte_pcf(led4 & led3 & led2);
			}
			else{
				write_byte_pcf(led1 & led2 & led3 & led4);
			}
		}
        vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

SemaphoreHandle_t xSemaphoreA = NULL;
SemaphoreHandle_t xSemaphoreB = NULL;

 void TaskA( void * pvParameters )
 {
	while(1) {
	    if( xSemaphoreA != NULL )
	    {
	        if( xSemaphoreTake( xSemaphoreA, 0 ) == pdTRUE )
	        {
	        	xSemaphoreGive(xSemaphoreA);
	            puts("taskA");
	        }
	    }
	    vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
 }
 void TaskB( void * pvParameters )
 {
	while(1) {
	    if( xSemaphoreB != NULL )
	    {
	        if( xSemaphoreTake( xSemaphoreB, 0 ) == pdTRUE )
	        {
	        	xSemaphoreGive(xSemaphoreB);
	            puts("taskB");
	        }
	    }
	    vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
 }
 void MasterTask( void * pvParameters )
 {
	int8_t data;
	xSemaphoreA = xSemaphoreCreateBinary();
	xSemaphoreB = xSemaphoreCreateBinary();
	xSemaphoreTake(xSemaphoreA, 0);
	xSemaphoreTake(xSemaphoreB, 0);
	xTaskCreate(TaskA, "TaskA", 256, NULL, 2, NULL);
	xTaskCreate(TaskB, "TaskB", 256, NULL, 2, NULL);
	while(1) {
		data = read_byte_pcf();
		if((data & button1) == 0){
			printf("\nBTN/LED1 - enable task B, disable task A");
			xSemaphoreTake(xSemaphoreA, 10);
			xSemaphoreGive(xSemaphoreB);
		}
		if ((data & button2) == 0) {
			printf("\nBTN/LED2 - enable task A, disable task B");
			xSemaphoreTake(xSemaphoreB, 10);
			xSemaphoreGive(xSemaphoreA);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
 }

void user_init(void)
{
	InitPressuresArray();

    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);

	bmp280_params_t params;
	bmp280_init_default_params(&params);
	// params.mode = BMP280_MODE_FORCED;
	params.mode = BMP280_MODE_NORMAL;
	params.filter = BMP280_FILTER_4; // for better accuracy
	params.oversampling_pressure = BMP280_STANDARD;
	params.oversampling_temperature = BMP280_FILTER_4;
	params.standby = BMP280_STANDBY_500;
	bmp280_dev.i2c_dev.bus = I2C_BUS;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);
    // xTaskCreate(ReadPressureEverySecond, "ReadPressureEverySecond", 256, NULL, 2, NULL);
    // xTaskCreate(GetLastTenPressuresAverage, "GetLastTenPressuresAverage", 256, NULL, 3, NULL);
	// xTaskCreate(komunikacijaTask, "komunikacijaTask", 256, NULL, 2, NULL);
	xTaskCreate(MasterTask, "MasterTask", 256, NULL, 3, NULL);
}

