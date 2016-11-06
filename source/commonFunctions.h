/*
 * commonFunctions.h
 *
 *  Created on: 05.11.2016
 *      Author: Tomek
 */

#ifndef SOURCE_COMMONFUNCTIONS_H_
#define SOURCE_COMMONFUNCTIONS_H_

#include "PinControl/PinController.h"
#include "MotorControl/MotorSpeedController.h"
#include "Simplot/Simplot.h"

/*void printEncoderValues();
void printMeasuredSpeed();
void printControlResults();
void printAll();*/

void blinkLed();
void printPID();
void plotPID();

#endif /* SOURCE_COMMONFUNCTIONS_H_ */
