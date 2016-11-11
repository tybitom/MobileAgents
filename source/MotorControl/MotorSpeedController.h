/*
 * MotorSpeedController.h
 *
 *  Created on: 25.10.2016
 *      Author: Tomek
 */

#ifndef MOTORSPEEDCONTROLLER_H_
#define MOTORSPEEDCONTROLLER_H_

//#include "elapsedMillis.h"
//#include <PID_v1.h>

#include "Encoder.h"
#include "Motor.h"
#include "../Simplot/Simplot.h"

#define outMax 250
#define outMin 0

class MotorSpeedController {
private:
	// ControlState for MotorSpeedController
	// when CONTROL_DISABLED function controllSpeed() does nothing
	enum ControlState {
		CONTROL_ENABLED, CONTROL_DISABLED
	};

	ControlState controlState = CONTROL_DISABLED;

	unsigned long lastTime;
	unsigned long encoderPreviousValue = 0;
	double lastMeasuredSpeed;
	double ITerm;

	int ds = 0; // distance change in time dt (since the last )

	// Regulation Parameters (double>=0)
	double Kp = 0.01;
	double Kd = 0.05;
	double Ki = 0.002;

	int sampleTime = 200; // a default value the same like in PID.h

	double measuredSpeed = 0; // [imp/s]
	double motorOutput = 0; // PWM [0-255]
	double setSpeed = 0;

	Encoder* encoder = nullptr;
	Motor* motor;
public:
	void initializeController(uint8_t encPinA, uint8_t encPinB,
			volatile unsigned long &encoderCounter, uint8_t motPinPWM,
			uint8_t motPinDIR, double kp, double ki, double kd,
			int sampleTime) {
		encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
		motor = new Motor(motPinPWM, motPinDIR);

		Kp = kp;
		Kd = ki;
		Ki = kd;

		this->sampleTime = sampleTime;

		ITerm = 0;

		lastTime = millis() - sampleTime;
		Serial.println("INFO: Motor controller was successfully initialized!");
	}

	void controlSpeed() {
		if (controlState == CONTROL_ENABLED) {
			unsigned long now = millis();
			int timeChange = (now - lastTime);
			if (timeChange >= sampleTime) {
				unsigned long encoderCounterValue = encoder->getCounterValue();

				if(timeChange > (1.5 * sampleTime)) {
					Serial.println("WARNING! dt for control speed was higher than (1,5 * sampleTime).");
				}

				ds = encoderCounterValue - encoderPreviousValue;
				measuredSpeed = (ds) * (1000.0 / timeChange); // [imp/s]

				lastTime = now;

				encoderPreviousValue = encoderCounterValue;

				// Real PID regulation calculation:
				double error = setSpeed - measuredSpeed;
				ITerm += (Ki * error);
				if (ITerm > outMax) {
					ITerm = outMax;
				} else if (ITerm < outMin) {
					ITerm = outMin;
				}

				// Compute PID Output:
				motorOutput = Kp * error + ITerm
						- Kd * (measuredSpeed - lastMeasuredSpeed);

				if (motorOutput > outMax) {
					motorOutput = outMax;
				} else if (motorOutput < outMin) {
					motorOutput = outMin;
				}

				lastMeasuredSpeed = measuredSpeed;

				motor->setMotorSpeed(motorOutput);
			}
		}
	}

	unsigned long getEncoderCounterValue() const {
		if (controlState == CONTROL_ENABLED) {
			return encoder->getCounterValue();
		}
		return 0;
	}

	double getKd() const {
		return Kd;
	}

	void setKd(double kd = 0.01) {
		Kd = kd;
	}

	double getKi() const {
		return Ki;
	}

	void setKi(double ki = 0.2) {
		Ki = ki;
	}

	double getKp() const {
		return Kp;
	}

	void setKp(double kp = 0.6) {
		Kp = kp;
	}

	void setPIDParameters(double kp, double ki, double kd) {
		Kp = kp;
		Ki = ki;
		Kd = kd;
		Serial.println("INFO: Parameters set to ");
		Serial.print(kp);
		Serial.print(", ");
		Serial.print(ki);
		Serial.print(", ");
		Serial.print(kd);
		Serial.println('.');
	}

	// sets the frequency, in Milliseconds, with which
	// the PID calculation is performed.  default is 100
	void setSampleTime(int sampleTime) {
		this->sampleTime = sampleTime;
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

	void stopMotor() {
		motor->stopMotor();
	}

	String getControlState() {
		if (controlState == CONTROL_ENABLED) {
			return "CONTROL_ENABLED";
		}
		return "CONTROL_DISABLED";
	}

	void enableController() {
		encoderPreviousValue = encoder->getCounterValue();
		lastMeasuredSpeed = measuredSpeed; // because, when controller is disabled, motor speed is 0
		ITerm = motorOutput;
		controlState = CONTROL_ENABLED;
	}

	void disableController() {
		controlState = CONTROL_DISABLED;
	}

	int getDs() const {
		return ds;
	}
};

#endif /* MOTORSPEEDCONTROLLER_H_ */
