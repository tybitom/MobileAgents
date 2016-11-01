/*
 * ScheduledTask.h
 *
 *  Created on: 29.10.2016
 *      Author: TTYBISZE
 */

#ifndef SCHEDULEDTASK_H_
#define SCHEDULEDTASK_H_

#include "Arduino.h"

class ScheduledTask {
private:
	int id;
	unsigned long sampleTime;
	void (*functionPointer)();

	unsigned long lastTime;
public:
	ScheduledTask() {
		id = -1;
		sampleTime = -1;
		functionPointer = nullptr;
		lastTime = -1;
	}
	ScheduledTask(int id, unsigned long sampleTime, void (*taskFunction)(void)) {
		this->id = id;
		this->sampleTime = sampleTime;
		functionPointer = taskFunction;

		lastTime = millis() - sampleTime;
	}
	void doNothing() {

	}
	bool realizeTask() {
		unsigned long now = millis();
		unsigned long timeChange = (now - lastTime);
		if (timeChange >= sampleTime) {
			lastTime = now;
			functionPointer();
			return true;
		}
		return false;
	}
};

#endif /* SCHEDULEDTASK_H_ */
