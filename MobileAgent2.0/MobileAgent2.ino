#include "Arduino.h"

#include "source/Defines.h"

#include "source/MotorControl/MotorSpeedController.h"
#include "source/PinControl/ScheduledTask.h"
#include "source/PinControl/TaskManager.h"
#include "source/Simplot/Simplot.h"

MotorSpeedController leftWheel;
MotorSpeedController rightWheel;

#include "source/printFunctions.h"

int counter = 0;

void setup() {
	Serial.begin(38400);

	leftWheel.initializeController(encoder0PinA, encoder0PinB,
			leftEncoderCounter, motorLPWMPin, motorLDirPin, 0.5, 0.01, 0.01,
			300);
	 leftWheel.setSetSpeed(1000);

	 rightWheel.initializeController(encoder1PinA, encoder1PinB,
			rightEncoderCounter, motorRPWMPin, motorRDirPin, 0.5, 0.01, 0.01,
			100);
	 rightWheel.setSetSpeed(1000);

	//PinController::getInstance()->setPinUsage(LED_BUILTIN, DIGITAL_OUPUT);
	//task = new ScheduledTask(0, 1000, &blinkLed);
	 //ScheduledTask task(0, 500, &printMeasuredSpeed);
	 //TaskManager::getInstance()->addTask(task);
}

void loop() {
	leftWheel.controllSpeed();
	// rightWheel.controllSpeed();

	//TaskManager::getInstance()->realizeTasks();

	/*PinController::getInstance()->setPinState(LED_BUILTIN, HIGH);
	delay(300);
	PinController::getInstance()->setPinState(LED_BUILTIN, LOW);
	delay(300);*/
}

/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
	static String inputString = "";
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char) Serial.read();
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			//stringComplete = true;
			inputString = "";
		}
	}
}

