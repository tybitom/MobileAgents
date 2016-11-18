#include "Arduino.h"

#include "source/Defines.h"
#include "source/MotorControl/AgentDriveController.h"

#include "source/PinControl/TaskManager.h"

AgentDriveController agentDriveController;

#include "source/commonFunctions.h"

#include "source/SerialInterpreter.h"

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!! IMPORTANT BEFORE YOU RUN THE PROGRAM !!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// PLEASE EDIT FILE HardwareSerial.h EDITING THE FOLLOWING LINE:
// #define SERIAL_RX_BUFFER_SIZE 100 // 64
// BIGGER BUFFER IS NEEDED WHILE JSON MESSAGES ARE LONGER THAN 64 CHARACTERS
// MAX_RECEIVED_STRING_LEN set to 100
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

volatile uint8_t counter = 0;

void setup() {
	Serial.begin(9600); //38400

	PinController::getInstance()->setPinUsage(LED_BUILTIN, DIGITAL_OUTPUT);

	agentDriveController.initializeController(encoder0PinA, encoder0PinB,
			encoder1PinA, encoder1PinB, leftEncoderCounter, rightEncoderCounter,
			motorLPWMPin, motorLDirPin, motorRPWMPin, motorRDirPin);

	printFreeMemory();

	//TaskManager::getInstance()->addTask(0, 100, plotPIDcontrol);
	//TaskManager::getInstance()->addTask(1, 400, printPIDcontrol);
	//TaskManager::getInstance()->addTask(2, 50, plotEncoderValues);
	//TaskManager::getInstance()->addTask(1, 170, printEncoderValues);

	//agentDriveController.setSetSpeed(1000);
	delay(500);

	//agentDriveController.enableController();
}

void loop() {

	agentDriveController.controlSpeed();

	TaskManager::getInstance()->realizeTasks();

	if (counter == 0) {
		printFreeMemory();
		counter++;
	}
}

void serialEvent() {
	String inputString;
	inputString = Serial.readString();
	if (inputString.length() >= MAX_RECEIVED_STRING_LEN) {
		// Serial.println("JSON message too long! Over 100 chars!");
		Serial.println("S|MA|sE|tl"); // to long
	} else if (inputString != "") {
		Serial.println(inputString);
		if (interpreteMessage(inputString)) {
			Serial.println("I|MA|sE|OK");
		} else {
			Serial.println("S|MA|sE|NOK"); // not ok
			printFreeMemory();
		}
		inputString = "";
	}
}
