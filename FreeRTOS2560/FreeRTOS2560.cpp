#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
	init();

	Serial.begin(9600);
	Wire.begin();	
/*	Serial.println("Initializing I2C devices...");
	accelgyro.initialize();

	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	pinMode(LED_PIN, OUTPUT);*/
}


void task1(void*p)
{
	Serial.println("Hello");
	MPU6050 accelgyro;

	accelgyro.initialize();

	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	
	accelgyro.getAcceleration(&ax, &ay, &az);
	
	Serial.print("a/g:\t");
	Serial.print(ax); Serial.print("\t");
	Serial.print(ay); Serial.print("\t");
	Serial.print(az); Serial.print("\t");
	Serial.print(gx); Serial.print("\t");
	Serial.print(gy); Serial.print("\t");
	Serial.println(gz);
	vTaskDelay(150);
}


#define STACK_DEPTH 512

void vApplicationIdleHook()
{
	//
}

int main(void)
{
	TaskHandle_t t1 ;
	
	xTaskCreate(task1, "Task 1", STACK_DEPTH, NULL,6,&t1);
	//xTaskCreate(task2,"Task 2", STACK_DEPTH, NULL, 5, &t2);
	
	vTaskStartScheduler();
	
	/*while(1) {
		Serial.println("Hello");		
		accelgyro.getAcceleration(&ax, &ay, &az);
		
		Serial.print("a/g:\t");
		Serial.print(ax); Serial.print("\t");
		Serial.print(ay); Serial.print("\t");
		Serial.print(az); Serial.print("\t");
		Serial.print(gx); Serial.print("\t");
		Serial.print(gy); Serial.print("\t");
		Serial.println(gz);
		
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}*/
	
	
}