#include "Arduino.h"

#include "source/Defines.h"

#include "source/MotorControl/MotorSpeedController.h"
#include "source/PinControl/ScheduledTask.h"
#include "source/PinControl/TaskManager.h"
#include "source/Simplot/Simplot.h"

MotorSpeedController leftWheel;
//MotorSpeedController rightWheel;

#include "source/commonFunctions.h"
#include "source/SerialInterpreter.h"

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!! IMPORTANT BEFORE YOU RUN THE PROGRAM !!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// PLEASE EDIT FILE HardwareSerial.h EDITING TWO FOLLOWING LINES:
// #define SERIAL_TX_BUFFER_SIZE 120 // 64
// #define SERIAL_RX_BUFFER_SIZE 120 // 64
// BIGGER BUFFER IS NEEDED WHILE JSON MESSAGES ARE LONGER THAN 64 CHARACTERS
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void setup() {
	Serial.begin(38400);

	leftWheel.initializeController(encoder0PinA, encoder0PinB,
			leftEncoderCounter, motorLPWMPin, motorLDirPin, 0.01, 0.01, 0.01,
			200);
	leftWheel.setSetSpeed(1000);

	/*rightWheel.initializeController(encoder1PinA, encoder1PinB,
			rightEncoderCounter, motorRPWMPin, motorRDirPin, 0.5, 0.01, 0.01,
			100);
	rightWheel.setSetSpeed(1000);*/

	leftWheel.setControlState(CONTROL_DISABLED);
	//rightWheel.setControlState(CONTROL_DISABLED);
}

void loop() {

	leftWheel.controllSpeed();
	//rightWheel.controllSpeed();

	TaskManager::getInstance()->realizeTasks();
}

void serialEvent() {
	String inputString;
	//inputString.reserve(100);
	inputString = Serial.readString();
	Serial.println(inputString.length());
	if (inputString != "") {
		Serial.println(inputString);
		//if (SerialInterpreter::getInstance()->interpreteMessage(inputString)) {
		if (interpreteMessage(inputString)) {
			Serial.println("OK");
		} else {
			Serial.println("NOT OK");
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
