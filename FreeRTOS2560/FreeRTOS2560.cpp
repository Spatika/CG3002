#include <avr/io.h>
#include <stdlib.h>
#include <Arduino.h>

#include <util/delay.h>


#include "semphr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include "NewPing.h"
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define TRIGGER_PIN1  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     11  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN2  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     7 // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN3  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3     11  // Arduino pin tied to echo pin on the ultrasonic sensor.

//Left
int motor1 = 8;

//Right
int motor2 = 9;


#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


xSemaphoreHandle printing_semphr ;

#define PRINTING_TIMEOUT ( (portTickType) 25 )

void setup() {
	
	Wire.begin();
	Serial.begin(115200);
	
	//Serial.println("Initializing I2C devices...");
	accelgyro.initialize();	
}

//US 1
void task1(void*p) {
	while(1) {
		if ( xSemaphoreTake( printing_semphr, PRINTING_TIMEOUT ) == pdTRUE ) {
			//Serial.println("1: Task Loop");
			
			
			unsigned int uS = sonar1.ping(); // Send ping, get ping time in microseconds (uS).
			Serial.print("Ping 1: ");
			Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
			Serial.println("cm");
			
			
			
			/* Give up semaphore reserving print resource */
			xSemaphoreGive( printing_semphr );
			/* Yield so that task2 can be scheduled */
			vPortYield();
			vTaskDelay(1000) ;
			} else {
			/* If the semaphore take timed out, something has gone wrong. */
			Serial.println("** Task 1 Error: could not take semaphore **");
			/* Hang thread rather than continue. */
			for(;;);
		}	
	}
}

//US 2
void task2(void *p) {
	while(1) {
		if ( xSemaphoreTake( printing_semphr, PRINTING_TIMEOUT ) == pdTRUE ) {
			//Serial.println("2: Task Loop");
			
			unsigned int uS = sonar2.ping(); // Send ping, get ping time in microseconds (uS).
			Serial.print("Ping 2: ");
			Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
			Serial.println("cm");
			 
			
			/* Give up semaphore reserving print resource */
			xSemaphoreGive( printing_semphr );
			/* Yield so that task2 can be scheduled */
			vPortYield();
			vTaskDelay(1000) ;
		} 
			
		else {
			/* If the semaphore take timed out, something has gone wrong. */
			Serial.println("** Task 2 Error: could not take semaphore **");
			/* Hang thread rather than continue. */
			for(;;);
		}
		
	}	
}

//MPU6050 
void task3(void * p) {
	
 while(1) {	
	if ( xSemaphoreTake( printing_semphr, PRINTING_TIMEOUT ) == pdTRUE ) {
		
	  accelgyro.getAcceleration(&ax, &ay, &az);
	  
	  Serial.print("a/g:\t");
	  Serial.print(ax); Serial.print("\t");
	  Serial.print(ay); Serial.print("\t");
	  Serial.print(az); Serial.print("\t");
	  Serial.print(gx); Serial.print("\t");
	  Serial.print(gy); Serial.print("\t");
	  Serial.println(gz);
	  
	  xSemaphoreGive( printing_semphr );
	  vPortYield() ;
	  vTaskDelay(2000) ;
	}
	
	else {
		/* If the semaphore take timed out, something has gone wrong. */
		Serial.println("** Task 3 Error: could not take semaphore **");
		/* Hang thread rather than continue. */
		for(;;);
	}
	
 }
}

void task4(void * p) {
	
}

void task5(void * p) {
	
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
	
	TaskHandle_t t1, t2, t3, t4, t5 ; 
	
	
	printing_semphr =  xSemaphoreCreateMutex() ;
	
	
	xTaskCreate(task1, "UltraSonic 1", STACK_DEPTH, NULL, 1, &t1) ;
	xTaskCreate(task2, "UltraSonic 2", STACK_DEPTH, NULL, 2, &t2) ;
	//xTaskCreate(task3, "MPU6050", STACK_DEPTH, NULL, 3, &t3) ; 
	
	//xTaskCreate(task4, "Handshake", STACK_DEPTH, NULL, 4, &t4) ; 
	//xTaskCreate(task5, "VibeMotors", STACK_DEPTH, NULL, 1, &t5) ;
	
	vTaskStartScheduler();
	
	return 0 ;
	
}


void MotorVibe(int i, int on){
	if(i == 1){
		if(on == 1)
		digitalWrite(motor1, HIGH);	// turn the motor on
		else
		digitalWrite(motor1, LOW);
	}
	else if(i == 2){
		if(on == 1)
		digitalWrite(motor2, HIGH);	// turn the motor on
		else
		digitalWrite(motor2, LOW);	// turn the motor off
	}
	else{
		if(on == 1){
			digitalWrite(motor1, HIGH);	// turn the motor on
			digitalWrite(motor2, HIGH);
		}
		else {
			digitalWrite(motor1, LOW);	// turn the motor off
			digitalWrite(motor2, LOW);
		}

	}
}
