/*
 * AgentDriveController.h
 *
 *  Created on: 25.10.2016
 *      Author: Tomek
 */

#ifndef AgentDriveController_H_
#define AgentDriveController_H_

#include "Encoder.h"
#include "Motor.h"

#define outMax 250
#define outMin 0

class AgentDriveController {
private:
	// ControlState for AgentDriveController
	// when CONTROL_DISABLED function controllSpeed() does nothing
	enum ControlState {
		CONTROL_ENABLED, CONTROL_DISABLED
	};

	ControlState controlState = CONTROL_DISABLED;

	unsigned long lastTime;
	unsigned long leftEncoderPreviousValue = 0;
	unsigned long rightEncoderPreviousValue = 0;
	double lastMeasuredSpeed; // overall
	double previousRotationError;
	double ITerm;
	double ITermRot;

	int ds = 0; // distance change in time dt (since the last )

	// Regulation Parameters (double>=0)
	double Kp = 0.2;
	double Kd = 0.02;
	double Ki = 0.001;

	// Regulation Parameters (double>=0)
	double Kprot = 1;
	double Kdrot = 0.01;
	double Kirot = 0.1;

	int sampleTime = 100; // a default value the same like in PID.h

	double measuredSpeed = 0; // [imp/s]
	double leftMotorOutput = 0; // PWM [0-255]
	double rightMotorOutput = 0; // PWM [0-255]
	double setSpeed = 0;

	Encoder* encoderL = nullptr;
	Motor* motorL;
	Encoder* encoderR = nullptr;
	Motor* motorR;

	double velPart;
	double rotPart;
public:
	void initializeController(uint8_t lEncPinA, uint8_t lEncPinB,
			uint8_t rEncPinA, uint8_t rEncPinB,
			volatile unsigned long &lEncoderCounter,
			volatile unsigned long &rEncoderCounter, uint8_t lMotPinPWM,
			uint8_t lMotPinDIR, uint8_t rMotPinPWM, uint8_t rMotPinDIR);
	void controlSpeed();
	unsigned long getLeftEncoderCounterValue() const;
	unsigned long getRightEncoderCounterValue() const;
	double getKd() const;
	double getKi() const;
	double getKp() const;
	void setPIDParameters(double kp, double ki, double kd);
	void setPIDrotParameters(double kp, double ki, double kd);
	void setSampleTime(int sampleTime);
	double getMeasuredSpeed() const;
	double getLeftMotorOutput() const;
	double getRightMotorOutput() const;
	double getSetSpeed() const;
	void setSetSpeed(double setSpeed);
	void setLeftMotorPWMvalue(int val);
	void setRightMotorPWMvalue(int val);
	void changeDirection();
	void stopAgent();
	String getControlState();
	void enableController();
	void disableController();
	int getDs() const;
	void clearITerm();
	double getRotPart() const;
	double getVelPart() const;
};

#endif /* AgentDriveController_H_ */
