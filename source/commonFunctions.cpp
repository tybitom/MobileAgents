/*
 * commonFunctions.cpp
 *
 *  Created on: 29.10.2016
 *      Author: TTYBISZE
 */
#include "commonFunctions.h"

#include "Arduino.h"

//extern MotorSpeedController leftWheel;
extern MotorSpeedController rightWheel;

/////////////// REPEATED FUNCTIONS FOR TASKS ///////////////////////

void blinkLed() {
	PinController::getInstance()->setPinState(LED_BUILTIN, HIGH);
	delay(300);
	PinController::getInstance()->setPinState(LED_BUILTIN, LOW);
	delay(300);
}

void printEncoderValues() {
	Serial.print("Encoder counter values: L: ");
	//Serial.print(leftWheel.getEncoderCounterValue(), DEC);
	Serial.print("\tR: ");
	Serial.println(rightWheel.getEncoderCounterValue(), DEC);
}

void plotEncoderValues() {
	//plot2(Serial, leftWheel.getEncoderCounterValue(), rightWheel.getEncoderCounterValue());
}

void printPIDcontrol() {
	/*Serial.print(leftWheel.getDs());
	Serial.print('\t');
	Serial.print(leftWheel.getMeasuredSpeed());
	Serial.print('\t');
	Serial.print(leftWheel.getMotorOutput());
	Serial.print("\t|\t");*/
	Serial.print(rightWheel.getDs());
	Serial.print('\t');
	Serial.print(rightWheel.getMeasuredSpeed());
	Serial.print('\t');
	Serial.println(rightWheel.getMotorOutput());
}

void plotPIDcontrol() {
	/*plot6(Serial, leftWheel.getDs(), (int ) leftWheel.getMeasuredSpeed(),
			(int ) leftWheel.getMotorOutput(), rightWheel.getDs(),
			(int ) rightWheel.getMeasuredSpeed(),
			(int ) rightWheel.getMotorOutput());*/
}

///////////////////////// ONE CALL FUNCTIONS /////////////////////////

void getNumberOfPinsAviableToSet() {
	Serial.print("Number of pins available (software coded) to set is: ");
	Serial.println(PinController::getInstance()->getNumberOfPinsAviable());
}
