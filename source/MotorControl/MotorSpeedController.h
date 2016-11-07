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
	// when CONTROL_DISABLED function controllSpeed does nothing
	enum ControlState {
		CONTROL_ENABLED, CONTROL_DISABLED
	};

	//elapsedMillis timeElapsed;
	unsigned long lastTime;
	unsigned long encoderPreviousValue = 0;
	double lastMeasuredSpeed;
	double ITerm;
	ControlState controlState = CONTROL_DISABLED;

	// Regulation Parameters (double>=0)
	double Kp = 0.01;
	double Kd = 0.05;
	double Ki = 0.002;

	int sampleTime = 200; // a default value the same like in PID.h

	double measuredSpeed; // [imp/s]
	double motorOutput; // PWM [0-255]
	double setSpeed = 0;

	Encoder* encoder = nullptr;
	Motor* motor;
	//PID* myPID; //Specify the links and initial tuning parameters
public:
	/*void initializeController(uint8_t encPinA, uint8_t encPinB,
	 volatile unsigned long &encoderCounter, uint8_t motPinL,
	 uint8_t motPinDIR) {
	 encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
	 motor = new Motor(motPinL, motPinR);
	 myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, Kp, Ki, Kd,
	 DIRECT);
	 myPID->SetMode(AUTOMATIC); //turn the PID on
	 controlState = CONTROL_ENABLED;
	 printPIDParameters();
	 }

	 void initializeController(uint8_t encPinA, uint8_t encPinB,
	 volatile unsigned long &encoderCounter, uint8_t motPinPWM,
	 uint8_t motPinR, double kp, double ki, double kd) {
	 encoder = initializeEncoder(encPinA, encPinB, encoderCounter);
	 motor = new Motor(motPinL, motPinR);
	 myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, kp, ki, kd,
	 DIRECT);
	 myPID->SetMode(AUTOMATIC); //turn the PID on
	 controlState = CONTROL_ENABLED;
	 printPIDParameters();
	 }*/

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
	}

	void controlSpeed() {
		if (controlState == CONTROL_ENABLED) {
			unsigned long now = millis();
			int timeChange = (now - lastTime);
			if (timeChange >= sampleTime) {
				unsigned long encoderCounterValue = encoder->getCounterValue();

				//Serial.println(timeChange);
				/*if (timeElapsed > (1.5 * sampleTime)) {
				 Serial.println("Motor control is not called on time!");
				 }*/
				//plot1(Serial, timeElapsed);
				int ds = encoderCounterValue - encoderPreviousValue;
				//Serial.println(ds);
				measuredSpeed = (ds) * (1000.0 / timeChange); // [imp/s]

				lastTime = now;
				//Serial.println(measuredSpeed);

				//Serial.println(measuredSpeed, DEC);

				encoderPreviousValue = encoderCounterValue;

				// real PID calculation
				double error = setSpeed - measuredSpeed;
				ITerm += (Ki * error);
				if (ITerm > outMax) {
					ITerm = outMax;
				} else if (ITerm < outMin) {
					ITerm = outMin;
				}

				/*Compute PID Output*/
				motorOutput = Kp * error + ITerm
						- Kd * (measuredSpeed - lastMeasuredSpeed);

				if (motorOutput > outMax) {
					motorOutput = outMax;
				} else if (motorOutput < outMin) {
					motorOutput = outMin;
				}

				lastMeasuredSpeed = measuredSpeed;
				//lastTime = now;

				motor->setMotorSpeed(motorOutput);

				//Serial.println(motorOutput);
				plot3(Serial, ds, measuredSpeed, motorOutput);
				//Serial.println();

				/*Serial.print('\t');
				 Serial.print(measuredSpeed, DEC);
				 Serial.print('\t');
				 Serial.println(motorOutput, DEC);*/
			}
		}
		//else log that encoder was not initialized
	}

	unsigned long getEncoderCounterValue() const {
		if (controlState == CONTROL_ENABLED) {
			return encoder->getCounterValue();
		}
		return 0;
	}

	/*void resetPID(double kp, double ki, double kd) {
	 controlState = CONTROL_DISABLED;
	 delete myPID;
	 myPID = new PID(&measuredSpeed, &motorOutput, &setSpeed, kp, ki, kd,
	 DIRECT);
	 myPID->SetMode(AUTOMATIC); //turn the PID on
	 }*/

	/*double getKd() const {
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
	 }*/

	void setPIDParameters(double kp, double ki, double kd) {
		Kp = kp;
		Ki = ki;
		Kd = kd;
		//resetPID(kp, ki, kd);
		//printPIDParameters();
	}

	/*void printPIDParameters() {
	 Serial.print("Parameters were set: ");
	 Serial.print(myPID->GetKp());
	 Serial.print(", ");
	 Serial.print(myPID->GetKi());
	 Serial.print(", ");
	 Serial.println(myPID->GetKd());
	 }*/

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
		// else
		return "CONTROL_DISABLED";
	}

	void enableController() {
		encoderPreviousValue = encoder->getCounterValue();
		lastMeasuredSpeed = 0; // because, when controller is disabled, motor speed is 0
		ITerm = motorOutput;
		controlState = CONTROL_ENABLED;
	}

	void disableController() {
		controlState = CONTROL_DISABLED;
	}

	/*void setControlState(ControlState controlState = CONTROL_DISABLED) {
	 if(encoder != nullptr) {
	 this->controlState = controlState;
	 }
	 }*/
};

#endif /* MOTORSPEEDCONTROLLER_H_ */
