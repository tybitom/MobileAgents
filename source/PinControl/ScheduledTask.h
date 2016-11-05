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
	bool activated;
	unsigned long sampleTime; // sampling time in milliseconds
	void (*functionPointer)();

	unsigned long lastTime;
public:
	ScheduledTask() {
		id = -1;
		sampleTime = -1;
		functionPointer = nullptr;
		lastTime = -1;
		activated = false;
	}
	ScheduledTask(int id, unsigned long sampleTime, void (*taskFunction)(void)) {
		this->id = id;
		this->sampleTime = sampleTime;
		functionPointer = taskFunction;

		activated = true;
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
	bool getId() const{
		return id;
	}

	bool isActivated() const {
		return activated;
	}

	void setActivated(bool activated = true) {
		this->activated = activated;
	}
};

#endif /* SCHEDULEDTASK_H_ */
