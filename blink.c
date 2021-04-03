#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"
#include "math.h"

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

// read BMP280 sensor values
float read_bmp280_forced() {

	float temperature, pressure;

	bmp280_force_measurement(&bmp280_dev);
	// wait for measurement to complete
	while (bmp280_is_measuring(&bmp280_dev)) {
	};
	bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL);

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

void ReadPressureEverySecond(void) {
	while (1) {
		float pressure = read_bmp280_forced();
		printf("\nPressure: %.2f Pa", pressure);

		WriteToPressuresArray(pressure);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void GetLastTenPressuresAverage(void) {
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

void WriteToPressuresArray(float pressure) {
	int i = pressureIndex % pressuresSize;
	pressures[i] = pressure;
	pressureIndex += 1;
}

void InitPressuresArray() {
	for (int i=0; i<pressuresSize; i += 1){
		pressures[i] = 0;
	}
}



void user_init(void)
{
	InitPressuresArray();

    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
	gpio_enable(SCL_PIN, GPIO_OUTPUT);

	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_FORCED;
	params.filter = BMP280_FILTER_4; // for better accuracy
	params.oversampling_pressure = BMP280_STANDARD;
	params.standby = BMP280_STANDBY_500;
	bmp280_dev.i2c_dev.bus = I2C_BUS;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);

    xTaskCreate(ReadPressureEverySecond, "ReadPressureEverySecond", 256, NULL, 2, NULL);
    xTaskCreate(GetLastTenPressuresAverage, "GetLastTenPressuresAverage", 256, NULL, 3, NULL);
}

