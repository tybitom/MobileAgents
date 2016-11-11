#include "Arduino.h"

#include "source/Defines.h"

#include "source/MotorControl/MotorSpeedController.h"
#include "source/PinControl/ScheduledTask.h"
#include "source/PinControl/TaskManager.h"
#include "source/Simplot/Simplot.h"

MotorSpeedController rightWheel;
//MotorSpeedController leftWheel;

#include "source/commonFunctions.h"
#include "source/SerialInterpreter.h"

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!! IMPORTANT BEFORE YOU RUN THE PROGRAM !!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// PLEASE EDIT FILE HardwareSerial.h EDITING TWO FOLLOWING LINES:
// #define SERIAL_TX_BUFFER_SIZE 120 // 64
// #define SERIAL_RX_BUFFER_SIZE 120 // 64
// BIGGER BUFFER IS NEEDED WHILE JSON MESSAGES ARE LONGER THAN 64 CHARACTERS
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void setup() {
	Serial.begin(38400);

	rightWheel.initializeController(encoder1PinA, encoder1PinB,
			rightEncoderCounter, motorRPWMPin, motorRDirPin, 0.2, 0.05, 0.01,
			50);
	rightWheel.setSetSpeed(1000);

	//leftWheel.initializeController(encoder0PinA, encoder0PinB,
	//		leftEncoderCounter, motorLPWMPin, motorLDirPin, 0.01, 0.05, 0.002,
	//		50);
	//leftWheel.setSetSpeed(1000);

	//TaskManager::getInstance()->addTask(0, 50, plotPIDcontrol);

	//delay(500);

	//rightWheel.enableController();
	//leftWheel.enableController();
}

void loop() {

	//leftWheel.controlSpeed();
	rightWheel.controlSpeed();

	TaskManager::getInstance()->realizeTasks();
}

void serialEvent() {
	String inputString;
	//inputString.reserve(100);
	inputString = Serial.readString();
	if(inputString.length() >= 100) {
		Serial.println("JSON message too long! Over 100 chars!");
	}
	else if (inputString != "") {
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
