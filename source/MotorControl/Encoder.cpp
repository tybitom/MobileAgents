/*
 * Encoder.cpp
 *
 *  Created on: 28.10.2016
 *      Author: TTYBISZE
 */

#include "Encoder.h"

#include "../PinControl/PinController.h"

uint8_t Encoder::encoderInstanceCounter = 0;

Encoder* initializeEncoder(uint8_t a, uint8_t b, volatile unsigned long &counterValue) {
	return new Encoder(a, b, counterValue);
}

Encoder::Encoder(uint8_t a, uint8_t b, volatile unsigned long &cv) {
	encoderPinA = a;
	encoderPinB = b;
	counterValue = &cv;

	// pinMode(encoderPinA, INPUT);
	// digitalWrite(encoderPinA, HIGH);       // turn on pullup resistor
	// pinMode(encoderPinB, INPUT);
	// digitalWrite(encoderPinB, HIGH);       // turn on pullup resistor
	PinController::getInstance()->setPinUsage(encoderPinA, DIGITAL_INPUT);
	PinController::getInstance()->setPinState(encoderPinA, HIGH);
	PinController::getInstance()->setPinUsage(encoderPinB, DIGITAL_INPUT);
	PinController::getInstance()->setPinState(encoderPinB, HIGH);


	switch (encoderInstanceCounter) {
	case 0: {
		attachInterrupt(0, countImpulsesInterrupt0, CHANGE);
		encoderInstanceCounter++;
		break;
	}
	case 1: {
		attachInterrupt(1, countImpulsesInterrupt1, CHANGE);
		encoderInstanceCounter++;
		break;
	}
	default: {
		// log a message that no more encoders can be added
	}
	}
}

volatile unsigned long Encoder::getCounterValue() const {
	return *counterValue;
}

uint8_t Encoder::getEncoderPinA() const {
	return encoderPinA;
}

uint8_t Encoder::getEncoderPinB() const {
	return encoderPinB;
}

