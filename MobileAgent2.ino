#include "Arduino.h"

#include "source/Defines.h"

#include "source/MotorControl/MotorSpeedController.h"
#include "source/PinControl/TaskManager.h"

MotorSpeedController leftWheel;
MotorSpeedController rightWheel;

#include "source/commonFunctions.h"
#include "source/distanceSensor.h"

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
	Serial.begin(38400);

	PinController::getInstance()->setPinUsage(LED_BUILTIN, DIGITAL_OUTPUT);

	leftWheel.initializeController(encoder0PinA, encoder0PinB,
			leftEncoderCounter, motorLPWMPin, motorLDirPin, 0.2, 0.05, 0.001,
			50);
	leftWheel.setSetSpeed(0);

	rightWheel.initializeController(encoder1PinA, encoder1PinB,
			rightEncoderCounter, motorRPWMPin, motorRDirPin, 0.2, 0.05, 0.01,
			50);
	rightWheel.setSetSpeed(0);

	printFreeMemory();

	initializeDistanceSensor();

	TaskManager::getInstance()->addTask(0, 50, plotPIDcontrol);
	TaskManager::getInstance()->addTask(1, 1000, triggerDistanceSensorSignal);

	delay(500); // delay the start of the program for a while

	leftWheel.enableController();
	rightWheel.enableController();
}

void loop() {

	leftWheel.controlSpeed();
	rightWheel.controlSpeed();

	getDistanceSensorMeasurement();

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
	/*while (Serial.available()) {
	 // get the new byte:
	 char inChar = (char)Serial.read();
	 // add it to the inputString:
	 inputString += inChar;
	 // if the incoming character is a newline, set a flag
	 // so the main loop can do something about it:
	 if (inChar == '\n') {
	 stringComplete = true;
	 }
	 }*/
}
