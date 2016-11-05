/*
 * serialInterpreter.h
 *
 *  Created on: 04.11.2016
 *      Author: Tomek
 */

#ifndef SOURCE_SERIALINTERPRETER_H_
#define SOURCE_SERIALINTERPRETER_H_

#include "ArduinoJson.h"

class SerialInterpreter {
private:
	static SerialInterpreter *serialInterpreterInstance;
public:
	static SerialInterpreter* getInstance();
	bool interpreteMessage(String &json);
	bool interpreteMessage(char json[]);
	bool interpretePinCTRL(JsonObject& jsonObject);
	bool interpreteTask(JsonObject& jsonObject);
	bool interpreteMotorCTRL(JsonObject& jsonObject);
};

#endif /* SOURCE_SERIALINTERPRETER_H_ */
