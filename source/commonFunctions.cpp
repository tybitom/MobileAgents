/*
 * commonFunctions.cpp
 *
 *  Created on: 29.10.2016
 *      Author: TTYBISZE
 */
#include "commonFunctions.h"
#include "MemoryFree/MemoryFree.h"

#include "Arduino.h"

extern AgentDriveController agentDriveController;

/////////////// REPEATED FUNCTIONS FOR TASKS ///////////////////////

void blinkLed() {
	PinController::getInstance()->setPinState(LED_BUILTIN, !PinController::getInstance()->getPinValue(LED_BUILTIN));
}

void printEncoderValues() {
	// Serial.print("Encoder counter values: L: ");
	Serial.print("ECV L: ");
	Serial.print(agentDriveController.getLeftEncoderCounterValue(), DEC);
	Serial.print("\tR: ");
	Serial.println(agentDriveController.getRightEncoderCounterValue(), DEC);
}

void plotEncoderValues() {
	plot2(Serial, agentDriveController.getLeftEncoderCounterValue()/10000, agentDriveController.getRightEncoderCounterValue()/10000);
}

void printPIDcontrol() {
	Serial.print("PID: ");
	Serial.print(agentDriveController.getDs());
	Serial.print('\t');
	Serial.print(agentDriveController.getMeasuredSpeed());
	Serial.print('\t');
	Serial.print(agentDriveController.getLeftMotorOutput());
	Serial.print("\t|\t");
	/*Serial.print(rightWheel.getDs());
	Serial.print('\t');
	Serial.print(rightWheel.getMeasuredSpeed());
	Serial.print('\t');*/
	Serial.println(agentDriveController.getRightMotorOutput());
}

void plotPIDcontrol() {
	plot4(Serial, agentDriveController.getDs(), (int ) agentDriveController.getMeasuredSpeed(),
			(int ) agentDriveController.getLeftMotorOutput(), (int ) agentDriveController.getRightMotorOutput());
}

void printPIDcontrolParts() {
	Serial.print("PARTS V: ");
	Serial.print(agentDriveController.getVelPart());
	Serial.print("\tR: ");
	Serial.println(agentDriveController.getRotPart());
}

void printPinValue(uint8_t pinNumber) {
	Serial.print("PV: ");
	Serial.println(PinController::getInstance()->getPinValue(pinNumber));
}

///////////////////////// ONE CALL FUNCTIONS /////////////////////////

void getNumberOfPinsAviableToSet() {
	// Serial.print("Number of pins available (software coded) to set is: ");
	Serial.print("NPA: ");
	Serial.println(PinController::getInstance()->getNumberOfPinsAviable());
}

void printFreeMemory() {
	Serial.print("I|cF|fm|FM");
	Serial.println(freeMemory());
}
