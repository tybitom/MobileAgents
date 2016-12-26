/*
 * distanceSensor.h
 *
 *  Created on: 20.11.2016
 *      Author: Tomek
 */

#ifndef SOURCE_DISTANCESENSOR_H_
#define SOURCE_DISTANCESENSOR_H_

#include <Arduino.h>
#include "Defines.h"

//////////////// For Ultrasonic distance sensor //////////////////////

// converts microseconds from distance sensor measurement to centimeters
float microsecondsToCentimeters(long microseconds) {
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The ping travels out and back, so to find the distance of the
	// object we take half of the distance travelled.
	return ((float)microseconds) / 29 / 2;
}

// the function that triggers the distance measurement
// should not be triggered more often than 150 ms
// measurement can last up to 100 ms!
void triggerDistanceSensorSignal() {
	PinController::getInstance()->setPinState(distanceSensorSignalPin, LOW); // Send low pulse
	delayMicroseconds(2); // Wait for 2 microseconds
	PinController::getInstance()->setPinState(distanceSensorSignalPin, HIGH); // Send high pulse
	delayMicroseconds(5); // Wait for 5 microseconds
	PinController::getInstance()->setPinState(distanceSensorSignalPin, LOW); // Holdoff
}

// attaches interrupt to the sensor
void initializeDistanceSensor() {
	pinMode(distanceSensorInterruptPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(distanceSensorInterruptPin),
			distanceSensorInterrupt, CHANGE);
}

// when the data are ready (measurement has finished), prints the distance out
// watch out that measurement needs to be triggered with function triggerDistanceSensorSignal()
void getDistanceSensorMeasurement() {
	if (isDistanceSensorDataReady) {
		// convert the time into a distance
		Serial.print("Dist1: ");
		Serial.println(microsecondsToCentimeters(distanceSensorSignalDuration));
		isDistanceSensorDataReady = false;
	}
	/*else
		Serial.println("no result");*/
}

#endif /* SOURCE_DISTANCESENSOR_H_ */
