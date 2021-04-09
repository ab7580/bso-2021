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

const int pressuresSize = 10;
float pressures[10];
int pressureIndex = 0;
int measureSpeed = 500;
float bmpstart = 0;
short pascalPerMeter = 12;
int multiplier = 30;

int globalIndex = 0;

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
		printf("\n %.2f;%d",pressure, globalIndex);
		globalIndex += 1;
		// WriteToPressuresArray(pressure);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}


void ReadPressureAndTemperatureEverySecond(void *pvParameters) {
	while (1) {
		float pressure = read_bmp280(BMP280_PRESSURE);
		float temperature = read_bmp280(BMP280_TEMPERATURE);
		printf("\n %.2f;%.2f;%d",pressure, temperature, globalIndex);
		globalIndex += 1;
		// WriteToPressuresArray(pressure);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void InitPressuresArray() {
	for (int i=0; i<pressuresSize; i += 1){
		pressures[i] = 0;
	}
	pressureIndex = 0;
}

void GetPressuresAverage(void *pvParameters) {
	while (1) {
		// if pressures[9] == 0 then we still haven't made 10 measurements
		if (pressures[pressuresSize-1] == 0) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}

		float avg = 0;
		for (int i = 0; i < pressuresSize; ++i) {
			avg += pressures[i];
		}
		avg = avg/pressuresSize;
		// printf("\n Average Pressure over last %d measurements: %.2f Pa",pressuresSize, avg);
		printf("\n %d;%.2f",globalIndex, avg);

		// place all 0s in the array now, so that you wait for full 10 new measurements before getting another average
		InitPressuresArray();

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}


void readerTask(void *pvParameters)
{
	float bmpcurr = 0;
	float diff = 0;
	short i = 0;
	int precision;

	while(1){
		write_byte_pcf(leds_off);

		bmpcurr = read_bmp280(BMP280_PRESSURE);

		//printf("AUTO: %.2f Pa\n", bmpcurr);
		//printf("TEMP: %.2f C\n", read_bmp280(BMP280_TEMPERATURE));
		printf("%.2f;%.2f\n", bmpcurr, read_bmp280(BMP280_TEMPERATURE));

		diff = bmpcurr-bmpstart;

		if(bmpstart == 0 && i == 3){
			bmpstart = bmpcurr;
		}
		else if(i<3){
			i++;
		}

		precision = multiplier * pascalPerMeter;

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
        vTaskDelay(measureSpeed / portTICK_PERIOD_MS);
	}
}

//every 30s reset the start altitude so lights show change constantly / with time
void resetPressure( void * pvParameters )
{
	while(1){
		bmpstart = read_bmp280(BMP280_PRESSURE);
		printf("AUTO Pressure reset\n");
		vTaskDelay(30000 / portTICK_PERIOD_MS);
	}
}

//slower, larger difference for LED display, less accurate
void initB()
{
	printf("initB ");
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_NORMAL;
	params.filter = BMP280_FILTER_4;
	params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
	params.oversampling_temperature = BMP280_FILTER_2;
	params.standby = BMP280_STANDBY_500;
	bmp280_init(&bmp280_dev, &params);
	measureSpeed = 500;
	multiplier = 30; // 1 LED = 30 meter difference
	printf(", complete\n");
}

//faster, smaller difference for LED display, best accuracy
void initA()
{
	printf("initA ");
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_NORMAL;
	params.filter = BMP280_FILTER_16;
	params.oversampling_pressure = BMP280_STANDARD;
	params.oversampling_temperature = BMP280_FILTER_4;
	params.standby = BMP280_STANDBY_250;
	bmp280_init(&bmp280_dev, &params);
	measureSpeed = 250;
	multiplier = 2; // 1 LED = 2 meter difference
	printf(", complete\n");
}

void buttonTask( void * pvParameters )
{
	int8_t data;
	bool tA = false;
	TaskHandle_t xHandle;
	bool pressureAuto = false;
	while(1) {
		data = read_byte_pcf();
		if((data & button1) == 0 && tA){
			printf("BTN1 - change to std accuracy\n");
			initB();
			tA = false;
		}
		if ((data & button2) == 0 && !tA) {
			printf("BTN2 - change to high accuracy\n");
			initA();
			tA = true;
		}
		// manual pressure reset
		if((data & button3) == 0){
			printf("BTN3 - set pressure\n");
			bmpstart = read_bmp280(BMP280_PRESSURE);
			printf("Pressure: %.2f Pa\n", bmpstart);
			if(pressureAuto){
				printf("End auto reset\n");
				vTaskDelete(xHandle);
				pressureAuto = false;
			}
		}
		// automatic pressure reset every 30s
		if((data & button4) == 0 && !pressureAuto){
			printf("BTN4 - start auto reset\n");
			xTaskCreate(resetPressure, "resetPressure", 256, NULL, 4, &xHandle);
			pressureAuto = true;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void user_init(void)
{
	//InitPressuresArray();

    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);

	bmp280_dev.i2c_dev.bus = I2C_BUS;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	initB();

    // xTaskCreate(ReadPressureEverySecond, "ReadPressureEverySecond", 256, NULL, 2, NULL);
    // xTaskCreate(GetPressuresAverage, "GetPressuresAverage", 256, NULL, 3, NULL);
	xTaskCreate(buttonTask, "buttonTask", 256, NULL, 3, NULL);
	xTaskCreate(readerTask, "readerTask", 256, NULL, 2, NULL);
}

