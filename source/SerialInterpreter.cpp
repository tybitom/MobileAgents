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
		if (msgType == "") {
			//Serial.println("SEVERE! JSON could not be interpreted. msgType empty! Probably lack of memory on Arduino.");
			Serial.println("S|SI|iM|te"); // type empty
			result = false;
		} else {
			if (msgType == "pinCTRL") {
				result = interpretePinCTRL(jsonObject);
			} else if (msgType == "task") {
				result = interpreteTask(jsonObject);
			} else if (msgType == "agent") {
				result = interpreteAgentCTRL(jsonObject);
			} else if (msgType == "motorL") {
				result = interpreteMotorCTRL(jsonObject, true);
			} else if (msgType == "motorR") {
				result = interpreteMotorCTRL(jsonObject, false);
			} else if (msgType == "quest") {
				result = interpreteQuestion(jsonObject);
			} else {
				// Serial.println("SEVERE! JSON type unknown! ");
				Serial.println("S|SI|iM|tu"); // type unknown
				result = false;
			}
			/*if (!result) {
			 Serial.print("INFO: Readed JSON data: ");
			 for (JsonObject::iterator it = jsonObject.begin();
			 it != jsonObject.end(); ++it) {
			 Serial.print(it->key);
			 Serial.print(": ");
			 Serial.print(it->value.asString());
			 Serial.print(", ");
			 }
			 Serial.println();
			 }*/
		}
	} else {
		// Serial.println("Parsing JSON message failed");
		Serial.println("S|SI|iM|pf"); // parsing failed
		result = false;
	}
	return result;
}

bool interpretePinCTRL(JsonObject& jsonObject) {
	bool result = true;
	String fun = jsonObject["fun"]; // fun like function
	if (fun == "") {
		// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iPC|fe"); // fun empty
		result = false;
	} else {
		if (fun == "setPinUsage") {
			uint8_t pinNumber = jsonObject["pinNumber"];
			String pinType = jsonObject["pinType"];
			if (pinType == "") {
				// Serial.println("SEVERE! JSON could not be interpreted. pinType empty! Probably lack of memory on Arduino.");
				Serial.println("S|SI|iPC|pTe"); // pinType empty
				result = false;
			} else {
				if (!PinController::getInstance()->setPinUsage(pinNumber,
						pinType)) {
					// Serial.println("SEVERE! Pin Usage could not be set!");
					Serial.println("S|SI|iPC|pus"); // Pin Usage could not be set
					result = false;
				}
			}
		} else if (fun == "setPinState") {
			uint8_t pinNumber = jsonObject["pinNumber"];
			uint8_t state = jsonObject["state"];
			if (!PinController::getInstance()->setPinState(pinNumber, state)) {
				// Serial.println("SEVERE! Pin State could not be set!");
				Serial.println("S|SI|iPC|pss"); // Pin State could not be set
				result = false;
			}
		} else if (fun == "setPWM") {
			uint8_t pinNumber = jsonObject["pinNumber"];
			int value = jsonObject["value"];
			if (!PinController::getInstance()->setPWM(pinNumber, value)) {
				// Serial.println("SEVERE! PWM value could not be set!");
				Serial.println("S|SI|iPC|PWMs"); // PWM could not be set
				result = false;
			}
		} else {
			// Serial.println("SEVERE! Pin control command unknown!");
			Serial.println("S|SI|iPC|cu"); // command unknown
			result = false;
		}
	}
	return result;
}

bool interpreteTask(JsonObject& jsonObject) {
	bool result = true;
	bool activateTask = jsonObject["activateTask"];
	if (activateTask) {
		String fun = jsonObject["fun"]; // fun like function
		if (fun == "") {
			// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
			Serial.println("S|SI|iT|fe"); // fun empty
			result = false;
		} else {
			int id = jsonObject["id"];
			unsigned long sampleTime = jsonObject["sampleTime"];

			if (fun == "blinkLed") {
				if (!TaskManager::getInstance()->addTask(id, sampleTime,
						&blinkLed)) {
					//Serial.println("SEVERE! Adding task failed!");
					Serial.println("S|SI|iT|af"); // Adding task failed!
					result = false;
				}
			} else if (fun == "printEncoderValues") {
				if (!TaskManager::getInstance()->addTask(id, sampleTime,
						&printEncoderValues)) {
					//Serial.println("SEVERE! Adding task failed!");
					Serial.println("S|SI|iT|af"); // Adding task failed!
					result = false;
				}
			} else if (fun == "plotEncoderValues") {
				if (!TaskManager::getInstance()->addTask(id, sampleTime,
						&plotEncoderValues)) {
					//Serial.println("SEVERE! Adding task failed!");
					Serial.println("S|SI|iT|af"); // Adding task failed!
					result = false;
				}
			} else if (fun == "printPID") {
				if (!TaskManager::getInstance()->addTask(id, sampleTime,
						&printPIDcontrol)) {
					//Serial.println("SEVERE! Adding task failed!");
					Serial.println("S|SI|iT|af"); // Adding task failed!
					result = false;
				}
			} else if (fun == "plotPID") {
				if (!TaskManager::getInstance()->addTask(id, sampleTime,
						&plotPIDcontrol)) {
					//Serial.println("SEVERE! Adding task failed!");
					Serial.println("S|SI|iT|af"); // Adding task failed!
					result = false;
				}
			} else {
				// Serial.println("SEVERE! Function for task unknown!");
				Serial.println("S|SI|iT|fu"); // function unknown
				result = false;
			}
		}
	} else {
		int id = jsonObject["id"];
		if (TaskManager::getInstance()->deactivateTask(id)) {
			result = true;
		} else {
			result = false;
			// Serial.print("SEVERE! No task with provided id ");
			// Serial.print(id);
			// Serial.println(" was found.");
			Serial.print("S|SI|iT|tnf"); // task not found
			Serial.println(id);
		}
	}
	return result;
}

bool interpreteAgentCTRL(JsonObject& jsonObject) {
	bool result = true;
	String cmd = jsonObject["cmd"];
	if (cmd == "") {
		// Serial.println("SEVERE! JSON could not be interpreted. cmd empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iAC|ce"); // cmd empty
		result = false;
	} else {
		if (cmd == "stop") {
			leftWheel.disableController();
			leftWheel.stopMotor();
			rightWheel.disableController();
			rightWheel.stopMotor();
		} else if (cmd == "speed") {
			int val = jsonObject["setSpeed"];
			leftWheel.setSetSpeed(val);
			rightWheel.setSetSpeed(val);
		} else {
			//Serial.println("SEVERE! Motor control command unknown!");
			Serial.println("S|SI|iAC|cu"); // command unknown
			result = false;
		}
	}
	return result;
}

bool interpreteMotorCTRL(JsonObject& jsonObject, bool leftMotor) {
	bool result = true;
	String cmd = jsonObject["cmd"];
	if (cmd == "") {
		// Serial.println("SEVERE! JSON could not be interpreted. cmd empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iMC|ce"); // cmd empty
		result = false;
	} else {
		if (cmd == "state") {
			bool activate = jsonObject["activate"];
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
			//Serial.println("INFO: Setting new PID parameters...");
			if (leftMotor) {
				leftWheel.setPIDParameters(kp, ki, kd);
				leftWheel.setSampleTime(sampleTime);
			} else {
				rightWheel.setPIDParameters(kp, ki, kd);
				rightWheel.setSampleTime(sampleTime);
			}
		} else if (cmd == "speed") {
			int setSpeed = jsonObject["setSpeed"];
			if (leftMotor) {
				leftWheel.setSetSpeed(setSpeed);
			} else {
				rightWheel.setSetSpeed(setSpeed);
			}
		} else if (cmd == "pwm") {
			int setpwm = jsonObject["setPWM"];
			if (leftMotor) {
				leftWheel.setMotorPWMvalue(setpwm);
			} else {
				rightWheel.setMotorPWMvalue(setpwm);
			}
		} else {
			//Serial.println("SEVERE! Motor control command unknown!");
			Serial.println("S|SI|iMC|cu"); // command unknown
			result = false;
		}
	}
	return result;
}

bool interpreteQuestion(JsonObject& jsonObject) {
	bool result = true;
	String fun = jsonObject["fun"];
	if (fun == "") {
		// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iQ|fe"); // fun empty
		result = false;
	} else {
		if (fun == "printEncoderValues") {
			printEncoderValues();
		} else if (fun == "plotEncoderValues") {
			plotEncoderValues();
		} else if (fun == "printPIDcontrol") {
			printPIDcontrol();
		} else if (fun == "plotPIDcontrol") {
			plotPIDcontrol();
		} else if (fun == "getNumberOfPinsAviableToSet") {
			getNumberOfPinsAviableToSet();
		} else {
			// Serial.println("SEVERE! Function unknown!");
			Serial.println("S|SI|iQ|fu"); // fun unknown
			result = false;
		}
	}
	return result;
}
