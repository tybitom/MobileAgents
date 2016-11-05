/*
 * SerialInterpretter.cpp
 *
 *  Created on: 04.11.2016
 *      Author: Tomek
 */

#include "SerialInterpreter.h"
#include "commonFunctions.h"
#include "PinControl/PinController.h"
#include "PinControl/TaskManager.h"

SerialInterpreter *SerialInterpreter::serialInterpreterInstance = nullptr;

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
SerialInterpreter* SerialInterpreter::getInstance() {
	if (serialInterpreterInstance == nullptr) {
		serialInterpreterInstance = new SerialInterpreter();
	}
	return serialInterpreterInstance;
}

bool SerialInterpreter::interpreteMessage(String &json) {
	bool result = true;
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonObject = jsonBuffer.parseObject(json);
	if (jsonObject.success()) {
		String type = jsonObject["type"];

		if (type == "pinCTRL") {
			result = interpretePinCTRL(jsonObject);
		} else if (type == "task") {
			result = interpreteTask(jsonObject);
		} else if (type == "motorCTRL") {
			result = interpreteMotorCTRL(jsonObject);
		} else {
			Serial.println("ERROR! JSON type not known! Readed JSON data: ");
			result = false;
		}
		if (!result) {
			Serial.print("Interpreted JSON: ");
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

bool SerialInterpreter::interpreteMessage(char json[]) {
	bool result = true;
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& jsonObject = jsonBuffer.parseObject(json);
	if (jsonObject.success()) {
		String type = jsonObject["type"];
		Serial.println(type);

		if (type == "pinCTRL") {
			result = interpretePinCTRL(jsonObject);
		} else if (type == "task") {
			result = interpreteTask(jsonObject);
		} else {
			Serial.println("ERROR! JSON type not known! Readed JSON data: ");
			result = false;
		}
		if (!result) {
			Serial.print("Interpreted JSON: ");
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

bool SerialInterpreter::interpretePinCTRL(JsonObject& jsonObject) {
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

bool SerialInterpreter::interpreteTask(JsonObject& jsonObject) {
	bool result = true;
	bool activateTask = jsonObject["activateTask"];
	if (activateTask) {
		String fun = jsonObject["fun"]; // fun like function
		if (fun == "blinkLed") {
			int id = jsonObject["id"];
			unsigned long sampleTime = jsonObject["sampleTime"];
			Serial.print("Setting task blinkLed with parameters: ");
			Serial.print(id);
			Serial.print(", ");
			Serial.print(sampleTime);
			Serial.println("...");
			if (!PinController::getInstance()->setPinUsage(LED_BUILTIN,
					DIGITAL_OUPUT)
					|| !TaskManager::getInstance()->addTask(id, sampleTime,
							&blinkLed)) {
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

bool SerialInterpreter::interpreteMotorCTRL(JsonObject& jsonObject) {
	bool result = true;
	String motor = jsonObject["motor"];
	String cmd = jsonObject["cmd"];
	if (cmd == "state") {
		bool activate = jsonObject["activate"];
		ControlState state;
		activate ? state = CONTROL_ENABLED : state = CONTROL_DISABLED;
		if (motor == "L") {
			leftWheel.setControlState(state);
		} else if (motor == "R") {
			rightWheel.setControlState(state);
		} else {
			Serial.println(
					"ERROR! Not specified, if it is right or left motor!");
		}
	} else if (cmd == "PID") {
		double kp = jsonObject["kp"];
		double ki = jsonObject["ki"];
		double kd = jsonObject["kd"];
		unsigned long sampleTime = jsonObject["dt"];
		if (motor == "L") {
			leftWheel.setPIDParameters(kp, ki, kd);
			leftWheel.setSampleTime(sampleTime);
		} else if (motor == "R") {
			rightWheel.setPIDParameters(kp, ki, kd);
			rightWheel.setSampleTime(sampleTime);
		} else {
			Serial.println(
					"ERROR! Not specified, if it is right or left motor!");
		}
	} else if (cmd == "speed") {
		int setSpeed = jsonObject["setSpeed"];
		if (motor == "L") {
			leftWheel.setSetSpeed(setSpeed);
		} else if (motor == "R") {
			rightWheel.setSetSpeed(setSpeed);
		} else {
			Serial.println(
					"ERROR! Not specified, if it is right or left motor!");
		}
	} else {
		Serial.println("ERROR! Motor control command unknown!");
		result = false;
	}
	return result;
}
