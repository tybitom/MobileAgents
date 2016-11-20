/*
 * TaskManager.cpp
 *
 *  Created on: 30.10.2016
 *      Author: Tomek
 */

#include "TaskManager.h"

uint8_t TaskManager::taskCounter = 0;
TaskManager *TaskManager::taskManagerInstance = nullptr;

TaskManager* TaskManager::getInstance() {
	if (taskManagerInstance == nullptr) {
		taskManagerInstance = new TaskManager();
	}
	return taskManagerInstance;
}

// adds task to the task list run every sampleTime in milliseconds
// and function realizeTasks() method is called
bool TaskManager::addTask(uint8_t id, unsigned long sampleTime,
		void (*taskFunction)(void)) {
	bool result = false;
	int indexOfExistingTask = checkIfTaskExists(id);
	if ((indexOfExistingTask >= 0) && (indexOfExistingTask < taskCounter)) {
		tasks[indexOfExistingTask] = ScheduledTask(id, sampleTime,
				taskFunction);
		Serial.println("I|TM|aT|tu"); // Task has been updated
		result = true;
	} else if (taskCounter < MAX_NUMBER_OF_TASKS) {
		tasks[taskCounter] = ScheduledTask(id, sampleTime, taskFunction);
		Serial.print("I|TM|aT|tc"); // Task has been created
		Serial.println(id);
		taskCounter++;
		result = true;
	} else {
		Serial.print("I|TM|at|nt");
		Serial.println(MAX_NUMBER_OF_TASKS); // No more than MAX_NUMBER_OF_TASKS can be added
	}
	return result;
}

//removes task from a list
bool TaskManager::deactivateTask(uint8_t id) {
	bool result = false;
	uint8_t indexOfExistingTask = checkIfTaskExists(id);
	if ((indexOfExistingTask >= 0) && (indexOfExistingTask < taskCounter)) {
		tasks[indexOfExistingTask].setActivated(false);
		result = true;
	}
	return result;
}

// realizes all previously added tasks
void TaskManager::realizeTasks() {
	for (int i = 0; i < taskCounter; ++i) {
		if (tasks[i].isActivated() == true) {
			tasks[i].realizeTask();
		}
	}
}

// if the task with provided id already exists, its parameters will be updated, while adding task
uint8_t TaskManager::checkIfTaskExists(uint8_t id) {
	for (int i = 0; i < taskCounter; ++i) {
		if (tasks[i].getId() == id) {
			Serial.print("I|TM|ct|te"); // Task exists on index
			Serial.println(id);
			return i;
		}
	}
	Serial.println("I|TM|ct|nf"); // No task with provided id was found
	return -1;
}
