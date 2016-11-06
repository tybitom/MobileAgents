/*
 * commonFunctions.cpp
 *
 *  Created on: 29.10.2016
 *      Author: TTYBISZE
 */
#include "commonFunctions.h"

#include "Arduino.h"

extern MotorSpeedController leftWheel;
extern MotorSpeedController rightWheel;

/*void printEncoderValues() {
	//Serial.print("Encoder counter value: ");
	//Serial.println(leftWheel.getControlState());
	Serial.print(leftWheel.getEncoderCounterValue(), DEC);
	Serial.print('\t');
	Serial.println(rightWheel.getEncoderCounterValue(), DEC);
}

void printMeasuredSpeed() {
	Serial.print(leftWheel.getMeasuredSpeed(), DEC);
	Serial.print('\t');
	Serial.println(rightWheel.getMeasuredSpeed(), DEC);
}

void printAll() {
	Serial.print(leftWheel.getEncoderCounterValue(), DEC);
	Serial.print('\t');
	Serial.print(rightWheel.getEncoderCounterValue(), DEC);
	Serial.print('\t');
	Serial.print(leftWheel.getMeasuredSpeed(), DEC);
	Serial.print('\t');
	Serial.println(rightWheel.getMeasuredSpeed(), DEC);
}*/

void blinkLed() {
	/*if(PinController::getInstance()->getPinUsage(LED_BUILTIN) == DIGITAL_OUPUT) {
		if(digitalRead(LED_BUILTIN) == LOW) {
			digitalWrite(LED_BUILTIN, HIGH);
		}
		else {
			digitalWrite(LED_BUILTIN, LOW);
		}
	}*/
	PinController::getInstance()->setPinState(LED_BUILTIN, HIGH);
	delay(300);
	PinController::getInstance()->setPinState(LED_BUILTIN, LOW);
	delay(300);
}

void printPID() {
	Serial.print(leftWheel.getMeasuredSpeed());
	Serial.print('\t');
	Serial.print(leftWheel.getMotorOutput());
	Serial.println();
}

void plotPID() {
	plot2(Serial, leftWheel.getMeasuredSpeed()/1000, leftWheel.getMotorOutput());
	Serial.println();
}
