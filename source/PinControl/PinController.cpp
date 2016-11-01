/*
 * PinController.cpp
 *
 *  Created on: 28.10.2016
 *      Author: TTYBISZE
 */

#include "PinController.h"

PinController *PinController::pinControllerInstance = nullptr;

PinController* PinController::getInstance() {
	if (pinControllerInstance == nullptr)
		pinControllerInstance = new PinController();
	return pinControllerInstance;
}
// Sets pin usage:
// enum PIN_TYPE {NO_FUNC_PIN,DIGITAL_OUPUT,DIGITAL_INPUT,DIGITAL_INPUT_NO_PULLUP,PWM_PIN,ANALOG_INPUT,ANALOG_OUTPUT}
bool PinController::setPinUsage(uint8_t pinNumber, PIN_TYPE pinType) {
	if (pinNumber < NUMBER_OF_PINS_AVIABLE) {
		if (!getPinUsage(pinNumber)) {
			pinsUsage[pinNumber] = pinType;
			switch (pinType) {
			case DIGITAL_OUPUT: {
				pinMode(pinNumber, OUTPUT);
				break;
			}
			case DIGITAL_INPUT: {
				pinMode(pinNumber, INPUT);
				digitalWrite(pinNumber, HIGH);
				break;
			}
			case DIGITAL_INPUT_NO_PULLUP: {
				pinMode(pinNumber, INPUT);
				break;
			}
			case PWM_PIN: {
				pinMode(pinNumber, OUTPUT);
				break;
			}
			case ANALOG_INPUT: {
				// no setup needs to be done (https://www.arduino.cc/en/Reference/AnalogRead)
				break;
			}
			case ANALOG_OUTPUT: {
				pinMode(pinNumber, OUTPUT);
				break;
			}
			default: { // NO_FUNC_PIN
				// Log that no function was assigned to the pin
				break;
			}
			}
			return true;
		}
		// else Log that this pin has been already reserved
	}
	// else Log that not so many pins are aviable
	return false;
}
// Returns pin usage:
// enum PIN_TYPE {NO_FUNC_PIN,DIGITAL_OUPUT,DIGITAL_INPUT,DIGITAL_INPUT_NO_PULLUP,PWM_PIN,ANALOG_INPUT,ANALOG_OUTPUT}
PIN_TYPE PinController::getPinUsage(uint8_t pinNumber) {
	if (pinNumber < NUMBER_OF_PINS_AVIABLE) {
		return pinsUsage[pinNumber];
	}
	// else Log that not so many pins are aviable
	return NO_FUNC_PIN;
}
// Returns number of pins aviable
int PinController::getNumberOfPinsAviable() {
	return NUMBER_OF_PINS_AVIABLE;
}
// Pin needs to be initialized firstly by function setPinUsage(uint8_t pinNumber, PIN_TYPE pinType)
// Sets pin value. State can be LOW or HIGH
void PinController::setPinState(uint8_t pinNumber, uint8_t state) {
	if (pinNumber < NUMBER_OF_PINS_AVIABLE) {
		if (pinsUsage[pinNumber] == DIGITAL_OUPUT) {
			digitalWrite(pinNumber, state);
		}
		// Log that this pin purpose was not set to pwm pin
	}
	// else Log that not so many pins are aviable
}
// Pin needs to be initialized firstly by function setPinUsage(uint8_t pinNumber, PIN_TYPE pinType)
// Sets pin pwm output.
void PinController::setPWM(uint8_t pinNumber, int value) {
	if (pinNumber < NUMBER_OF_PINS_AVIABLE) {
		if ((pinsUsage[pinNumber] == PWM_PIN)
				|| (pinsUsage[pinNumber] == ANALOG_OUTPUT)) {
			analogWrite(pinNumber, value);
		}
		// Log that this pin purpose was not set to pwm pin
	}
	// else Log that not so many pins are aviable
}

