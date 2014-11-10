#include <avr/io.h>
#include <stdlib.h>
#include <Arduino.h>
#include <Math.h>
#include <util/delay.h>
#include <string.h>


#include "semphr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "Kalman.h"
#include "Wire.h"

#include "NewPing.h"

using namespace std ; 


/*US Setup */
#define TRIGGER_PIN1  42  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     43  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN2  44  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     45 // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN3  46  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3     47  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN4  48  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN4     49  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN5  50  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN5     51  // Arduino pin tied to echo pin on the ultrasonic sensor.


#define SONAR_NUM     5 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 20 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//1 - left 60
//2 - right 60
//3 top front 80
//4 front mid 80
//5 front low 80
//unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

int flagM1 = 0; int flagM2 = 0;
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
	NewPing(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
	NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE),
	NewPing(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE),
	NewPing(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE),
	NewPing(TRIGGER_PIN5, ECHO_PIN5, MAX_DISTANCE)
};

uint32_t PingTimer;
uint32_t count;


/*US Setup */


/*Motor Setup*/
//Left
int motor1 = 8;

//Right
int motor2 = 9;
/*Motor Setup*/


xSemaphoreHandle printing_semphr ;

#define PRINTING_TIMEOUT ( (portTickType) 25 )

/*IMU Setup*/

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1 //set to 1

#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5 //set to 0

#define MPU6050_RA_INT_PIN_CFG      0x37

Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter


uint32_t timer;
uint8_t i2cData[14];

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = {
(MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

/*IMU Setup*/



/*I2C Functions*/
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
	Wire.beginTransmission(address);
	Wire.write(registerAddress);
	Wire.write(data, length);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
	if (rcode) {
		Serial.print(F("i2cWrite failed: "));
		Serial.println(rcode);
	}
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data, bool sendStop) {
	return i2cWrite(address, registerAddress, &data, 1, sendStop); // Returns 0 on success
}


uint8_t i2cRead(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
	uint32_t timeOutTimer;
	Wire.beginTransmission(address);
	Wire.write(registerAddress);
	uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
	if (rcode) {
		Serial.print(F("i2cRead failed: "));
		Serial.println(rcode);
		return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
	}
	Wire.requestFrom(address, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++) {
		if (Wire.available())
		data[i] = Wire.read();
		else {
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if (Wire.available())
			data[i] = Wire.read();
			else {
				Serial.println(F("i2cRead timeout"));
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	return 0; // Success
}




void updateMPU6050() {
	while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
	accX = ((i2cData[0] << 8) | i2cData[1]);
	accY = -((i2cData[2] << 8) | i2cData[3]);
	accZ = ((i2cData[4] << 8) | i2cData[5]);
	tempRaw = (i2cData[6] << 8) | i2cData[7];
	gyroX = -(i2cData[8] << 8) | i2cData[9];
	gyroY = (i2cData[10] << 8) | i2cData[11];
	gyroZ = -(i2cData[12] << 8) | i2cData[13];
}

void updateHMC5883L() {
	while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
	magX = ((i2cData[0] << 8) | i2cData[1]);
	magZ = ((i2cData[2] << 8) | i2cData[3]);
	magY = ((i2cData[4] << 8) | i2cData[5]);
}

void updatePitchRoll() {
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -? to ? (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
	roll = atan(accY / sqrt(accX  accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
	magZ *= -1;

	magX *= magGain[0];
	magY *= magGain[1];
	magZ *= magGain[2];

	magX -= magOffset[0];
	magY -= magOffset[1];
	magZ -= magOffset[2];

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
	double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
	yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	yaw *= -1;
}

void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
	i2cWrite(HMC5883L, 0x00, 0x11, true);
	delay(100); // Wait for sensor to get ready
	updateHMC5883L(); // Read positive bias values

	int16_t magPosOff[3] = {
	magX, magY, magZ                 };

	i2cWrite(HMC5883L, 0x00, 0x12, true);
	delay(100); // Wait for sensor to get ready
	updateHMC5883L(); // Read negative bias values

	int16_t magNegOff[3] = {
	magX, magY, magZ                 };

	i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal

	magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
	magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
	magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

}


/*I2C Functions*/


/*NewPing functions */


//motor 1, no 8, LHS
//motor 2, no 9, RHS

void MotorVibe(int i, int on){
	if(i == 1) {
		if(on == 1)
		digitalWrite(motor1, HIGH);	// turn the motor on
		else
		digitalWrite(motor1, LOW);
	}
	
	else if(i == 2) {
		if(on == 1)
		digitalWrite(motor2, HIGH);	// turn the motor on
		else
		digitalWrite(motor2, LOW);	// turn the motor off
	}
	
	else {
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


int sensorRead(int i) {
  unsigned int uS = sonar[i].ping(); // Send ping, get ping time in microseconds (uS).
  float reading  = uS / US_ROUNDTRIP_CM;
  //Serial.println(uS / US_ROUNDTRIP_CM);
  //Serial.println(reading);
  /*Serial.print(i);
  Serial.print(": ");
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");*/
  switch(i + 1){
    
   //left motor vibrate 
  case 1 : 
    if(reading < 60 && reading > 0) {
	  flagM1 = 1;
    }
    break;
    
  //right motor vibrate
  case 2 : 
    if(reading < 60 && reading > 0) {
	  flagM2 = 1;
    }
    break;
  //top facing middle sensor  
  case 3:
  
  //front middle
  case 4: 
  
  
    if(reading < 80 && reading > 0){
	  flagM2 = 1;
	  flagM1 = 1;
    }
    break;
    
  //front low - 3 small vibrates 
  case 5:
	break ;
    /*if(countH<10){
      height[countH] = reading;
      avg = height[0];
      countH++;
    }
    if(countH == 10){
      for(int i=0;i<10;i++){
        avg += height[i];
      }
      if(i==10)
      {
        avg/=10;
      }
    }
        
    //Step
    if(reading < avg && reading > avg -15){
      MotorVibe(3, 1 );
     /* delay(150);
      MotorVibe(3, 0 );
      delay(150);
      MotorVibe(3, 1 );*/
    //}
    //Obstacle
    /*else if(reading < avg -15) {
      MotorVibe(3, 1);
    }
    //floor
    else {
      MotorVibe(3,0);
    }
    break; */
  }
  
}

void setup() {
	
	  Wire.begin();
	  Serial.begin(115200);
	  
  // ----------------------------------For handshake-------------------------------
  //wait for SOMETHINGGGGG from RPi
  int count = 0 ; 
  while(1) {
   
   if(Serial.available()) {
   
     // read the incoming byte:
     char readChar = Serial.read();
     count++ ;
     
     if(count==1)
       Serial.println(readChar) ;
   
     if(count==2)
       break;
     }
   }

	
	  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

	  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

	  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once

	  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode


	  while (i2cRead(MPU6050, 0x75, i2cData, 1));
	  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
		  Serial.print(F("Error reading sensor"));
		  while (1);
	  }

	  //enable i2c pass through to access magnetometer
	  //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0

	  while (i2cWrite(MPU6050, 0x37, 0x02, true));

	  while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode

	  calibrateMag();

	  delay(100); // Wait for sensors to stabilize

	  /* Set Kalman and gyro starting angle */
	  updateMPU6050();
	  updateHMC5883L();
	  updatePitchRoll();
	  updateYaw();

	  kalmanX.setAngle(roll); // First set roll starting angle
	  gyroXangle = roll;
	  compAngleX = roll;

	  kalmanY.setAngle(pitch); // Then pitch
	  gyroYangle = pitch;
	  compAngleY = pitch;

	  kalmanZ.setAngle(yaw); // And finally yaw
	  gyroZangle = yaw;
	  compAngleZ = yaw;	
	  
	  
  timer = micros(); // Initialize the timer 
  PingTimer = millis() + 100;
  count = 0;
  pinMode(motor1, HIGH);
  pinMode(motor2, HIGH);
  
  //-------------------SECOND HANDSHAKE-----------------------
 /* while(1) {
   
   if(Serial.available()) {
   
   // read the incoming byte:
   char readChar = Serial.read();
   break;
   }
  }*/

}

//US1
void task1(void*p) {
	while(1) {
		if ( xSemaphoreTake( printing_semphr, PRINTING_TIMEOUT ) == pdTRUE ) {
			flagM1 = 0;
			flagM2 = 0;
			 
			 sensorRead(0);
			 sensorRead(1);
			 sensorRead(2);
			 sensorRead(3);
			 sensorRead(4);
			 if(flagM1 == 1){
				 MotorVibe(1, 1);
			 }
			 if(flagM2 == 1){
				 MotorVibe(2, 1 );
			 }
			 if(flagM1 == 0){
				 MotorVibe(1, 0 );
			 }
			 if(flagM2 == 0){
				 MotorVibe(2, 0 );
			 }
			 
			
			/* Give up semaphore reserving print resource */
			
			xSemaphoreGive( printing_semphr );
			/* Yield so that task2 can be scheduled */
			vPortYield();
			vTaskDelay(500) ;
			
			} else {
			/* If the semaphore take timed out, something has gone wrong. */
			Serial.println("** Task 1 Error: could not take semaphore **");
			/* Hang thread rather than continue. */
			for(;;);
		}	
	}
}

//MPU6050 + HMC
void task2(void *p) {
	while(1) {
		if ( xSemaphoreTake(printing_semphr, 2*PRINTING_TIMEOUT ) == pdTRUE ) {
				//Serial.println("2: Task Loop");
			
			   updateMPU6050() ;
			   updateHMC5883L() ;

			   double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
			   timer = micros();


			   /* Roll and pitch estimation */
			   updatePitchRoll();
			   double gyroXrate = gyroX / 131.0; // Convert to deg/s
			   double gyroYrate = gyroY / 131.0; // Convert to deg/s

			   #ifdef RESTRICT_PITCH
			   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			   if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
				   kalmanX.setAngle(roll);
				   compAngleX = roll;
				   kalAngleX = roll;
				   gyroXangle = roll;
			   }
			   
			   else
			   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

			   if (abs(kalAngleX) > 90)
			   gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
			   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
			   #else
			   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			   if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
				   kalmanY.setAngle(pitch);
				   compAngleY = pitch;
				   kalAngleY = pitch;
				   gyroYangle = pitch;
			   }
			   else
			   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

			   if (abs(kalAngleY) > 90)
			   gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
			   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
			   #endif


			   /* Yaw estimation */
			   updateYaw();
			   double gyroZrate = gyroZ / 131.0; // Convert to deg/s
			   // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
			   if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
				   kalmanZ.setAngle(yaw);
				   compAngleZ = yaw;
				   kalAngleZ = yaw;
				   gyroZangle = yaw;
			   }
			   else
			   kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


			   /* Estimate angles using gyro only */
			   gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
			   gyroYangle += gyroYrate * dt;
			   gyroZangle += gyroZrate * dt;

			   /* Estimate angles using complimentary filter */
			   compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
			   compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
			   compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

			   // Reset the gyro angles when they has drifted too much
			   if (gyroXangle < -180 || gyroXangle > 180)
			   gyroXangle = kalAngleX;
			   if (gyroYangle < -180 || gyroYangle > 180)
			   gyroYangle = kalAngleY;
			   if (gyroZangle < -180 || gyroZangle > 180)
			   gyroZangle = kalAngleZ;

				

			   char output[100];
			   double acclx = (accX/ 16384.0)*9.8, accly = (accY/ 16384.0)*9.8, acclz = (accZ/ 16384.0)*9.8;
			   double compassVal = fmod(yaw+360, 360);

				char acclxCh[10], acclyCh[10], acclzCh[10], comp[10], mil[30] ; 
				dtostrf(acclx, 3, 2, acclxCh) ;
				dtostrf(accly, 3, 2, acclyCh);
				dtostrf(acclz,3,2,acclzCh);
				dtostrf(compassVal,3,2,comp);
				
				
				ltoa(millis(), mil, 10);
				strcpy(output,"(accelerometerReading = ( x = ");
				strcat(output, acclxCh);
				strcat(output, ", y = ");
				strcat(output, acclyCh);
				
				strcat(output, ", z = ");
				strcat(output, acclzCh);
				strcat(output, ") compassReading = ");
				strcat(output, comp);
				strcat(output, ", currentTime = " );
				strcat(output, mil);
				strcat(output, ")");
				Serial.println(output);
				
			/* Give up semaphore reserving print resource */
			xSemaphoreGive( printing_semphr );
			/* Yield so that task1 can be scheduled */
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

#define STACK_DEPTH 512

void vApplicationIdleHook()
{
	//
}

int main(void)
{
	
	init() ;
	
	//includes handshake
	setup() ;
	
	TaskHandle_t t1, t2, t3 ; 
	
	//printing_semphr =  xSemaphoreCreateMutex() ;
	
	vSemaphoreCreateBinary(printing_semphr) ;
	
	xTaskCreate(task1, "UltraSonics", STACK_DEPTH, NULL, 1, &t1) ;
	
	xTaskCreate(task2, "IMU", STACK_DEPTH, NULL, 2, &t2) ;
	
	vTaskStartScheduler() ;
	
	return 0 ;
	
}


/*Other IMU Functions*/



/*Other IMU Functions*/