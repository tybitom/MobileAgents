/*
 * AgentDriveController.cpp
 *
 *  Created on: 12.11.2016
 *      Author: TTYBISZE
 */

#include "AgentDriveController.h"

void AgentDriveController::initializeController(uint8_t lEncPinA,
		uint8_t lEncPinB, uint8_t rEncPinA, uint8_t rEncPinB,
		volatile unsigned long &lEncoderCounter,
		volatile unsigned long &rEncoderCounter, uint8_t lMotPinPWM,
		uint8_t lMotPinDIR, uint8_t rMotPinPWM, uint8_t rMotPinDIR) {
	encoderL = initializeEncoder(lEncPinA, lEncPinB, lEncoderCounter);
	motorL = new Motor(lMotPinPWM, lMotPinDIR);

	encoderR = initializeEncoder(rEncPinA, rEncPinB, rEncoderCounter);
	motorR = new Motor(rMotPinPWM, rMotPinDIR);

	ITerm = 0;
	ITermRot = 0;
	previousRotationError = 0;

	lastTime = millis() - sampleTime;
	// Serial.println("INFO: Motor controller was successfully initialized!");
	Serial.println("I|ADC|iC|succ");
}

void AgentDriveController::controlSpeed() {
	if (controlState == CONTROL_ENABLED) {
		unsigned long now = millis();
		int timeChange = (now - lastTime);
		if (timeChange >= sampleTime) {
			unsigned long lencValue = encoderL->getCounterValue();
			unsigned long rencValue = encoderR->getCounterValue();

			if (timeChange > (1.5 * sampleTime)) {
				// Serial.println("WARNING! dt for control speed was higher than (1,5 * sampleTime).");
				Serial.println("W|ADC|cS|dth"); // dt to long
			}

			ds = ((lencValue - leftEncoderPreviousValue)
					+ (rencValue - rightEncoderPreviousValue)) / 2;
			measuredSpeed = (ds) * (1000.0 / timeChange); // [imp/s]

			int rotError = (lencValue - leftEncoderPreviousValue)
					- (rencValue - rightEncoderPreviousValue);

			lastTime = now;

			leftEncoderPreviousValue = lencValue;
			rightEncoderPreviousValue = rencValue;

			// Real PID regulation calculation:
			double error = setSpeed - measuredSpeed;
			ITerm += (Ki * error);
			if (ITerm > outMax) {
				ITerm = outMax;
			} else if (ITerm < outMin) {
				ITerm = outMin;
			}

			ITermRot += (Kirot * rotError);
			if (ITermRot > outMax) {
				ITermRot = outMax;
			} else if (ITermRot < outMin) {
				ITermRot = outMin;
			}

			velPart = Kp * error + ITerm
					- Kd * (measuredSpeed - lastMeasuredSpeed);

			rotPart = Kprot * rotError + ITermRot
					- Kdrot * (rotError - previousRotationError);
			/*Serial.print(velPart);
			 Serial.print("\t|\t");
			 Serial.println(rotPart);*/

			previousRotationError = rotError;

			// Compute PID Output:
			leftMotorOutput = velPart - rotPart;
			rightMotorOutput = velPart + rotPart;

			if (leftMotorOutput > outMax) {
				leftMotorOutput = outMax;
			} else if (leftMotorOutput < outMin) {
				leftMotorOutput = outMin;
			}

			if (rightMotorOutput > outMax) {
				rightMotorOutput = outMax;
			} else if (rightMotorOutput < outMin) {
				rightMotorOutput = outMin;
			}

			lastMeasuredSpeed = measuredSpeed;

			/*Serial.print(leftMotorOutput);
			 Serial.print("\t|\t");
			 Serial.println(rightMotorOutput);*/

			motorL->setMotorPWMvalue(leftMotorOutput);
			motorR->setMotorPWMvalue(rightMotorOutput);
		}
	}
}

unsigned long AgentDriveController::getLeftEncoderCounterValue() const {
	return encoderL->getCounterValue();
}

unsigned long AgentDriveController::getRightEncoderCounterValue() const {
	return encoderR->getCounterValue();
}

double AgentDriveController::getKd() const {
	return Kd;
}

double AgentDriveController::getKi() const {
	return Ki;
}

double AgentDriveController::getKp() const {
	return Kp;
}

void AgentDriveController::setPIDParameters(double kp, double ki, double kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
	/*Serial.println("INFO: Parameters set to ");
	 Serial.print(kp);
	 Serial.print(", ");
	 Serial.print(ki);
	 Serial.print(", ");
	 Serial.print(kd);
	 Serial.println('.');*/
	Serial.println("I|ADC|sPID|succ"); // parameters set successfully
}

void AgentDriveController::setPIDrotParameters(double kp, double ki,
		double kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
	Serial.println("I|ADC|sPIDrot|succ"); // parameters set successfully
}

// sets the frequency, in Milliseconds, with which
// the PID calculation is performed.  default is 100
void AgentDriveController::setSampleTime(int sampleTime) {
	this->sampleTime = sampleTime;
}

double AgentDriveController::getMeasuredSpeed() const {
	return measuredSpeed;
}

double AgentDriveController::getLeftMotorOutput() const {
	return leftMotorOutput;
}

double AgentDriveController::getRightMotorOutput() const {
	return rightMotorOutput;
}

double AgentDriveController::getSetSpeed() const {
	return setSpeed;
}

void AgentDriveController::setSetSpeed(double setSpeed) {
	if (setSpeed >= 0) {
		this->setSpeed = setSpeed;
	} else {
		motorL->changeDirection();
		motorR->changeDirection();
		this->setSpeed = (-1) * setSpeed;
	}
}

void AgentDriveController::setLeftMotorPWMvalue(int val) {
	disableController();
	motorL->setMotorPWMvalue(val);
}

void AgentDriveController::setRightMotorPWMvalue(int val) {
	disableController();
	motorR->setMotorPWMvalue(val);
}

void AgentDriveController::changeDirection() {
	motorL->changeDirection();
	motorR->changeDirection();
}

void AgentDriveController::stopAgent() {
	motorL->stopMotor();
	motorR->stopMotor();
}

String AgentDriveController::getControlState() {
	if (controlState == CONTROL_ENABLED) {
		return "CONTROL_ENABLED";
	}
	return "CONTROL_DISABLED";
}

void AgentDriveController::enableController() {
	leftEncoderPreviousValue = encoderL->getCounterValue();
	rightEncoderPreviousValue = encoderR->getCounterValue();
	lastMeasuredSpeed = measuredSpeed; // because, when controller is disabled, motor speed is 0
	ITerm = 0;
	ITermRot = 0;
	controlState = CONTROL_ENABLED;
}

void AgentDriveController::disableController() {
	controlState = CONTROL_DISABLED;
}

int AgentDriveController::AgentDriveController::getDs() const {
	return ds;
}

void AgentDriveController::clearITerm() {
	ITerm = 0;
	ITermRot = 0;
	lastMeasuredSpeed = 0;
	previousRotationError = 0;
	ds = 0;
	measuredSpeed = 0;
	leftMotorOutput = 0;
	rightMotorOutput = 0;
}

double AgentDriveController::getRotPart() const {
	return rotPart;
}

double AgentDriveController::getVelPart() const {
	return velPart;
}
