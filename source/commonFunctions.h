/*
 * commonFunctions.h
 *
 *  Created on: 05.11.2016
 *      Author: Tomek
 */

#ifndef SOURCE_COMMONFUNCTIONS_H_
#define SOURCE_COMMONFUNCTIONS_H_

#include "MotorControl/AgentDriveController.h"
#include "PinControl/PinController.h"
#include "Simplot/Simplot.h"

void blinkLed();
void printEncoderValues();
void plotEncoderValues();
void printPIDcontrol();
void plotPIDcontrol();
void printPIDcontrolParts();
void printFreeMemory();

void getNumberOfPinsAviableToSet();

#endif /* SOURCE_COMMONFUNCTIONS_H_ */
