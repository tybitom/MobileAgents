/*
 * MotorSpeedController.h
 *
 *  Created on: 25.10.2016
 *      Author: Tomek
 */

#ifndef MOTORSPEEDCONTROLLER_H_
#define MOTORSPEEDCONTROLLER_H_

#include "elapsedMillis.h"
#include <PID_v1.h>

#include "Encoder.h"
#include "Motor.h"
#include "../Simplot/Simplot.h"

class MotorSpeedController {
private:
	elapsedMicros timeElapsed;
	unsigned long encoderPreviousValue = 0;
	unsigned int sampleTime = 100; // a default value the same like in PID.h
	enum ControlState {
		CONTOL_ENABLED, CONTOL_DISABLED
	};
	ControlState controlState = CONTOL_DISABLED;

	// Regulation Parameters (double>=0)
	double Kp = 0.8;
	double Kd = 0.01;
	double Ki = 0.2;

	double measuredSpeed; // [imp/s]
	double motorOutput; // PWM [0-255]
	double setSpeed = 0;

	Encoder* encoder = nullptr;
	Motor* motor;
	PID* myPID; //Specify the links and initial tuning parameters
public:
	void initializeController(uint8_t encPinA, uint8_t encPinB,
			volatile unsigned long &encoderCounter, uint8_t motPinL,
			uint8_t motPinR) {
		encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
		motor = new Motor(motPinL, motPinR);
		myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, Kp, Ki, Kd,
		DIRECT);
		myPID->SetMode(AUTOMATIC); //turn the PID on
		controlState = CONTOL_ENABLED;
	}

	void initializeController(uint8_t encPinA, uint8_t encPinB,
			volatile unsigned long &encoderCounter, uint8_t motPinL,
			uint8_t motPinR, double kp, double ki, double kd) {
		encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
		motor = new Motor(motPinL, motPinR);
		myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, kp, ki, kd,
		DIRECT);
		myPID->SetMode(AUTOMATIC); //turn the PID on
		controlState = CONTOL_ENABLED;
	}

	void initializeController(uint8_t encPinA, uint8_t encPinB,
			volatile unsigned long &encoderCounter, uint8_t motPinL,
			uint8_t motPinR, double kp, double ki, double kd, int sampleTime) {
		encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
		motor = new Motor(motPinL, motPinR);
		myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, kp, ki, kd,
		DIRECT);
		this->sampleTime = sampleTime;
		myPID->SetSampleTime(sampleTime);

		myPID->SetMode(AUTOMATIC); //turn the PID on
		controlState = CONTOL_ENABLED;
	}

	void controllSpeed() {
		if (controlState == CONTOL_ENABLED) {
			if (timeElapsed >= sampleTime) {
				unsigned long encoderCounterValue = encoder->getCounterValue();

				Serial.println(timeElapsed, DEC);
				Serial.print('\t');

				measuredSpeed = (encoderCounterValue - encoderPreviousValue)
						* (1000000.0 / timeElapsed); // [imp/s]
				timeElapsed = 0;

				//Serial.println(measuredSpeed, DEC);

				encoderPreviousValue = encoderCounterValue;

				// real PID calculation
				if (myPID->Compute()) {
					motor->setMotorSpeed(motorOutput);
				}

				plot2(Serial, measuredSpeed, motorOutput);
				Serial.print('\t');
				Serial.print(measuredSpeed, DEC);
				Serial.print('\t');
				Serial.println(motorOutput, DEC);
			}
		}
		//else log that encoder was not initialized
	}

	unsigned long getEncoderCounterValue() const {
		if (controlState == CONTOL_ENABLED) {
			return encoder->getCounterValue();
		}
		return 0;
	}

	void resetPID() {
		controlState = CONTOL_DISABLED;
		delete myPID;
		myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, Kp, Ki, Kd,
		DIRECT);
		myPID->SetMode(AUTOMATIC); //turn the PID on
		controlState = CONTOL_ENABLED;
	}

	double getKd() const {
		return Kd;
	}

	void setKd(double kd = 0.01) {
		Kd = kd;
		resetPID();
	}

	double getKi() const {
		return Ki;
	}

	void setKi(double ki = 0.2) {
		Ki = ki;
		resetPID();
	}

	double getKp() const {
		return Kp;
	}

	void setKp(double kp = 0.6) {
		Kp = kp;
		resetPID();
	}

	void setPIDParameters(double kp, double ki, double kd) {
		Kp = kp;
		Ki = ki;
		Kd = kd;
		resetPID();
	}

	// sets the frequency, in Milliseconds, with which
	// the PID calculation is performed.  default is 100
	void setSampleTime(int sampleTime) {
		this->sampleTime = sampleTime;
		myPID->SetSampleTime(sampleTime);
	}

	double getMeasuredSpeed() const {
		return measuredSpeed;
	}

	double getMotorOutput() const {
		return motorOutput;
	}

	double getSetSpeed() const {
		return setSpeed;
	}

	void setSetSpeed(double setSpeed) {
		if (setSpeed >= 0) {
			motor->changeDirection();
			this->setSpeed = setSpeed;
		} else {
			this->setSpeed = (-1) * setSpeed;
		}
	}

	void changeMotorDirection() {
		motor->changeDirection();
	}

	String getControlState() {
		if (controlState == CONTOL_ENABLED) {
			return "CONTOL_ENABLED";
		}
		// else
		return "CONTOL_DISABLED";
	}
};

#endif /* MOTORSPEEDCONTROLLER_H_ */
