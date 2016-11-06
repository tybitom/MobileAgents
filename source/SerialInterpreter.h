/*
 * serialInterpreter.h
 *
 *  Created on: 04.11.2016
 *      Author: Tomek
 */

#ifndef SOURCE_SERIALINTERPRETER_H_
#define SOURCE_SERIALINTERPRETER_H_

#include "ArduinoJson.h"

bool interpreteMessage(String &json);
bool interpretePinCTRL(JsonObject& jsonObject);
bool interpreteTask(JsonObject& jsonObject);
bool interpreteMotorCTRL(JsonObject& jsonObject, bool leftMotor);

#endif /* SOURCE_SERIALINTERPRETER_H_ */
