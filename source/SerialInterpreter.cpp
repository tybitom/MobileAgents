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

#include "JSONinterpreter.h"

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
	uint8_t pos = 0;
	String key, value;

	if (!getNext(json, pos, key, value) && key == "type") {
		//Serial.println("SEVERE! JSON could not be interpreted. msgType empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iM|te"); // type empty
		result = false;
	} else {
		if (value == "pinCTRL") {
			result = interpretePinCTRL(json, pos, key, value);
		} else if (value == "task") {
			result = interpreteTask(json, pos, key, value);
		} else if (value == "agent") {
			result = interpreteAgentCTRL(json, pos, key, value);
		} else if (value == "motorL") {
		 result = interpreteMotorCTRL(json, pos, key, value, true);
		 } else if (value == "motorR") {
		 result = interpreteMotorCTRL(json, pos, key, value, false);
		 } else if (value == "quest") {
			result = interpreteQuestion(json, pos, key, value);
		} else {
			// Serial.println("SEVERE! JSON type unknown! ");
			Serial.println("S|SI|iM|tu"); // type unknown
			Serial.print(key);
			Serial.print(": ");
			Serial.println(value);
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
	return result;
}

bool interpretePinCTRL(const String &json, uint8_t &pos, String &key,
		String &value) {
	bool result = true;

	if (!getNext(json, pos, key, value) && key == "fun") { // fun like function
	// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iPC|fe"); // fun empty
		result = false;
	} else {
		if (value == "setPinUsage") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			uint8_t pinNumber = atoi(value.c_str());
			if (!getNext(json, pos, key, value) && key == "pinType") {
				// Serial.println("SEVERE! JSON could not be interpreted. pinType empty! Probably lack of memory on Arduino.");
				Serial.println("S|SI|iPC|pTe"); // pinType empty
				result = false;
			} else {
				if (!PinController::getInstance()->setPinUsage(pinNumber,
						value)) {
					// Serial.println("SEVERE! Pin Usage could not be set!");
					Serial.println("S|SI|iPC|pus"); // Pin Usage could not be set
					result = false;
				}
			}
		} else if (value == "setPinState") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			uint8_t pinNumber = atoi(value.c_str());
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			uint8_t state = atoi(value.c_str());
			if (!PinController::getInstance()->setPinState(pinNumber, state)) {
				// Serial.println("SEVERE! Pin State could not be set!");
				Serial.println("S|SI|iPC|pss"); // Pin State could not be set
				result = false;
			}
		} else if (value == "setPWM") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			uint8_t pinNumber = atoi(value.c_str());
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			int pwmValue = atoi(value.c_str());
			if (!PinController::getInstance()->setPWM(pinNumber, pwmValue)) {
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

bool interpreteTask(const String &json, uint8_t &pos, String &key,
		String &value) {
	bool result = true;
	getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????

	if (atoi(value.c_str()) != 0) {
		getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
		int id = atoi(value.c_str());

		if (!getNext(json, pos, key, value) && key == "fun") { // fun like function
		// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
			Serial.println("S|SI|iT|fe"); // fun empty
			result = false;
		} else {
			String fun = value;
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			unsigned long sampleTime = atoi(value.c_str());

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
		getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
		uint8_t id = atoi(value.c_str());
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

bool interpreteAgentCTRL(const String &json, uint8_t &pos, String &key,
		String &value) {
	bool result = true;

	if (!getNext(json, pos, key, value) && key == "cmd") {
		// Serial.println("SEVERE! JSON could not be interpreted. cmd empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iAC|ce"); // cmd empty
		result = false;
	} else {
		if (value == "stop") {
			leftWheel.disableController();
			leftWheel.clearITerm();
			leftWheel.stopMotor();

			rightWheel.disableController();
			rightWheel.clearITerm();
			rightWheel.stopMotor();
		} else if (value == "speed") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			int val = atoi(value.c_str());
			if (val == 0) {
				leftWheel.disableController();
				leftWheel.clearITerm();
				leftWheel.stopMotor();

				rightWheel.disableController();
				rightWheel.clearITerm();
				rightWheel.stopMotor();
			} else {
				leftWheel.setSetSpeed(val);
				leftWheel.enableController();
				rightWheel.setSetSpeed(val);
				rightWheel.enableController();
			}
		} else {
			//Serial.println("SEVERE! Motor control command unknown!");
			Serial.println("S|SI|iAC|cu"); // command unknown
			result = false;
		}
	}
	return result;
}

bool interpreteMotorCTRL(const String &json, uint8_t &pos, String &key,
		String &value, bool leftMotor) {
	bool result = true;

	if (!getNext(json, pos, key, value) && key == "cmd") {
		// Serial.println("SEVERE! JSON could not be interpreted. cmd empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iMC|ce"); // cmd empty
		result = false;
	} else {
		if (value == "state") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			bool activate = atoi(value.c_str());
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
		} else if (value == "PID") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			double kp = atof(value.c_str());
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			double ki = atof(value.c_str());
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			double kd = atof(value.c_str());
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			int sampleTime = atoi(value.c_str());
			//Serial.println("INFO: Setting new PID parameters...");
			if (leftMotor) {
				leftWheel.setPIDParameters(kp, ki, kd);
				leftWheel.setSampleTime(sampleTime);
			} else {
				rightWheel.setPIDParameters(kp, ki, kd);
				rightWheel.setSampleTime(sampleTime);
			}
		} else if (value == "speed") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			int setSpeed = atoi(value.c_str());
			if (leftMotor) {
				leftWheel.setSetSpeed(setSpeed);
			} else {
				rightWheel.setSetSpeed(setSpeed);
			}
		} else if (value == "pwm") {
			getNext(json, pos, key, value); //////////////////////////?????????????????????????????????????????
			int setpwm = atoi(value.c_str());
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

bool interpreteQuestion(const String &json, uint8_t &pos, String &key,
		String &value) {
	bool result = true;

	if (!getNext(json, pos, key, value) && key == "fun") { // fun like function
	// Serial.println("SEVERE! JSON could not be interpreted. fun empty! Probably lack of memory on Arduino.");
		Serial.println("S|SI|iQ|fe"); // fun empty
		result = false;
	} else {
		if (value == "printEncoderValues") {
			printEncoderValues();
		} else if (value == "plotEncoderValues") {
			plotEncoderValues();
		} else if (value == "printPIDcontrol") {
			printPIDcontrol();
		} else if (value == "plotPIDcontrol") {
			plotPIDcontrol();
		} else if (value == "getNumberOfPinsAviableToSet") {
			getNumberOfPinsAviableToSet();
		} else if (value == "RAM") {
			printFreeMemory();
		} else {
			// Serial.println("SEVERE! Function unknown!");
			Serial.println("S|SI|iQ|fu"); // fun unknown
			result = false;
		}
	}
	return result;
}
