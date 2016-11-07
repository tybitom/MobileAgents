/*
 * SerialInterpretter.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Tomek
 */

#include "Defines.h"
#include "SerialInterpreter.h"
#include "commonFunctions.h"
#include "MotorControl/MotorSpeedController.h"
#include "PinControl/PinController.h"
#include "PinControl/TaskManager.h"

extern MotorSpeedController leftWheel;
extern MotorSpeedController rightWheel;

// json object pattern:
// {
//    "type": typeString,
//    "element": valueOrString,
//    ...
// }
// available types: pinCTRL, task, ...
// elements and their number depend on type
// They should be treated like commands, therefore elements depends on functions they call.
// ---------------------------------------------
// example: it calls setPinUsage(uint8_t pinNumber,	PIN_TYPE pinType);
// {
// 	  "type": setPinUsage,
//    "pinNumber": 5,
//    "pinType": DIGITAL_OUPUT
// }
bool interpreteMessage(String &json) {
	bool result = true;
	StaticJsonBuffer<170> jsonBuffer;
	JsonObject& jsonObject = jsonBuffer.parseObject(json);
	if (jsonObject.success()) {
		const String msgType = jsonObject["type"].asString();
		if (msgType == "pinCTRL") {
			result = interpretePinCTRL(jsonObject);
		} else if (msgType == "task") {
			result = interpreteTask(jsonObject);
		} else if (msgType == "motorL") {
			result = interpreteMotorCTRL(jsonObject, true);
		} else if (msgType == "motorR") {
			result = interpreteMotorCTRL(jsonObject, false);
		} else {
			Serial.print("ERROR! JSON type unknown! ");
			result = false;
		}
		if (!result) {
			Serial.print(" Readed JSON data: ");
			for (JsonObject::iterator it = jsonObject.begin();
					it != jsonObject.end(); ++it) {
				Serial.print(it->key);
				Serial.print(": ");
				Serial.print(it->value.asString());
				Serial.print(", ");
			}
			Serial.println();
		}
	} else {
		Serial.println("Parsing JSON message failed");
		result = false;
	}
	return result;
}

bool interpretePinCTRL(JsonObject& jsonObject) {
	bool result = true;
	String fun = jsonObject["fun"]; // fun like function
	if (fun == "setPinUsage") {
		uint8_t pinNumber = jsonObject["pinNumber"];
		String pinType = jsonObject["pinType"];
		if (!PinController::getInstance()->setPinUsage(pinNumber, pinType)) {
			Serial.println("ERROR! Pin Usage could not be set!");
			result = false;
		}
	} else if (fun == "setPinState") {
		uint8_t pinNumber = jsonObject["pinNumber"];
		uint8_t state = jsonObject["state"];
		if (!PinController::getInstance()->setPinState(pinNumber, state)) {
			Serial.println("ERROR! Pin State could not be set!");
			result = false;
		}
	} else if (fun == "setPWM") {
		uint8_t pinNumber = jsonObject["pinNumber"];
		int value = jsonObject["value"];
		if (!PinController::getInstance()->setPWM(pinNumber, value)) {
			Serial.println("ERROR! PWM value could not be set!");
			result = false;
		}
	} else {
		Serial.println("ERROR! Pin control command unknown!");
		result = false;
	}
	return result;
}

bool interpreteTask(JsonObject& jsonObject) {
	bool result = true;
	bool activateTask = jsonObject["activateTask"];
	if (activateTask) {
		String fun = jsonObject["fun"]; // fun like function
		int id = jsonObject["id"];
		unsigned long sampleTime = jsonObject["sampleTime"];
		Serial.print("Setting task ");
		Serial.print(fun);
		Serial.print("with parameters: ");
		Serial.print(id);
		Serial.print(", ");
		Serial.print(sampleTime);
		Serial.println("...");
		if (fun == "blinkLed") {
			if (!PinController::getInstance()->setPinUsage(LED_BUILTIN,
					DIGITAL_OUPUT)
					|| !TaskManager::getInstance()->addTask(id, sampleTime,
							&blinkLed)) {
				Serial.println("ERROR! Adding task failed!");
				result = false;
			}
		} else if (fun == "plotPID") {
			if (!TaskManager::getInstance()->addTask(id, sampleTime,
					&plotPID)) {
				Serial.println("ERROR! Adding task failed!");
				result = false;
			}
		} else if (fun == "printPID") {
			if (!TaskManager::getInstance()->addTask(id, sampleTime,
					&printPID)) {
				Serial.println("ERROR! Adding task failed!");
				result = false;
			}
		} else {
			Serial.println("ERROR! Function for task not known!");
			result = false;
		}
	} else {
		int id = jsonObject["id"];
		if (TaskManager::getInstance()->deactivateTask(id)) {
			result = true;
		} else {
			result = false;
			// Log that no task with provided id was found
		}
	}
	return result;
}

bool interpreteMotorCTRL(JsonObject& jsonObject, bool leftMotor) {
	bool result = true;
	String cmd = jsonObject["cmd"];
	Serial.println(cmd);
	/*if (cmd == "stop") {
	 if (leftMotor) {
	 leftWheel.stopMotor();
	 } else {
	 rightWheel.stopMotor();
	 }
	 } else*/
	if (cmd == "state") {
		bool activate = jsonObject["activate"];
		if (activate)
			Serial.println("Activate: Yes");
		else
			Serial.println("Activate: No");
		if (activate) {
			if (leftMotor) {
				leftWheel.enableController();
			} else {
				rightWheel.enableController();
			}
		} else {
			if (leftMotor) {
				leftWheel.disableController();
				leftWheel.stopMotor();
			} else {
				rightWheel.disableController();
				rightWheel.stopMotor();
			}
		}
	} else if (cmd == "PID") {
		double kp = jsonObject["kp"];
		double ki = jsonObject["ki"];
		double kd = jsonObject["kd"];
		int sampleTime = jsonObject["dt"];
		Serial.println("Setting new PID parameters...");
		if (leftMotor) {
			//leftWheel.stopMotor();
			leftWheel.setPIDParameters(kp, ki, kd);
			leftWheel.setSampleTime(sampleTime);
		} else {
			//rightWheel.stopMotor();
			rightWheel.setPIDParameters(kp, ki, kd);
			rightWheel.setSampleTime(sampleTime);
		}
	} else if (cmd == "speed") {
		int setSpeed = jsonObject["setSpeed"];
		if (leftMotor) {
			leftWheel.setSetSpeed(setSpeed);
		} else {
			//rightWheel.setSetSpeed(setSpeed);
		}
	} else {
		Serial.println("ERROR! Motor control command unknown!");
		result = false;
	}
	return result;
}
