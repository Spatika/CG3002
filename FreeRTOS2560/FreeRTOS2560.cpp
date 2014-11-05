#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
	
	Wire.begin();
	Serial.begin(115200);
	
	//Serial.println("Initializing I2C devices...");
	accelgyro.initialize();

	//Serial.println("Testing device connections...");
	//Serial.println(accelgyro.testConnection() ? "successful" : "failed");
		
}


void task1(void*p)
{
	
	Serial.println("start") ;
	accelgyro.getAcceleration(&ax, &ay, &az);
	
	Serial.print("a/g:\t");
	Serial.print(ax); Serial.print("\t");
	Serial.print(ay); Serial.print("\t");
	Serial.print(az); Serial.print("\t");
	
	//Serial.println("end") ;
	delayMicroseconds(100);
	
}


#define STACK_DEPTH 2048

void vApplicationIdleHook()
{
	//
}

int main(void)
{
	init() ;
	setup() ;
	
	TaskHandle_t t1 ;
	
	xTaskCreate(task1, "Task 1", STACK_DEPTH, NULL,6,&t1);
	
	vTaskStartScheduler();
	
	/*
	while(1) {
		Serial.println("start") ;
		accelgyro.getAcceleration(&ax, &ay, &az);
	
		Serial.print("a/g:\t");
		Serial.print(ax); Serial.print("\t");
		Serial.print(ay); Serial.print("\t");
		Serial.print(az); Serial.print("\t");
	
		//Serial.println("end") ;
		delayMicroseconds(100);
	} */
	
}