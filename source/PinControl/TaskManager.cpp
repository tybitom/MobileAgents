/*
 * TaskManager.cpp
 *
 *  Created on: 30.10.2016
 *      Author: Tomek
 */

#include "TaskManager.h"

int TaskManager::taskCounter = 0;
TaskManager *TaskManager::taskManagerInstance = nullptr;

TaskManager* TaskManager::getInstance() {
	if (taskManagerInstance == nullptr) {
		taskManagerInstance = new TaskManager();
	}
	return taskManagerInstance;
}

// adds task to the task list run every time realizeTasks() method is called
bool TaskManager::addTask(ScheduledTask task) {
	bool result = false;
	int indexOfExistingTask = checkIfTaskExists(task.getId());
	if (indexOfExistingTask > 0 && indexOfExistingTask < taskCounter) {
		tasks[indexOfExistingTask] = task;
		result = true;

	} else if (taskCounter < MAX_NUMBER_OF_TASKS) {
		tasks[taskCounter] = task;
		taskCounter++;
		result = true;
	}
	//else Log that no more than MAX_NUMBER_OF_TASKS can be added
	return result;
}

// adds task to the task list run every sampleTime in milliseconds
// and function realizeTasks() method is called
bool TaskManager::addTask(int id, unsigned long sampleTime,
		void (*taskFunction)(void)) {
	bool result = false;
	int indexOfExistingTask = checkIfTaskExists(id);
	if ((indexOfExistingTask >= 0) && (indexOfExistingTask < taskCounter)) {
		Serial.println("Updating task...");
		tasks[indexOfExistingTask] = ScheduledTask(id, sampleTime,
				taskFunction);
		Serial.println("Task updated");
		result = true;
	} else if (taskCounter < MAX_NUMBER_OF_TASKS) {
		Serial.println("Creating task...");
		tasks[taskCounter] = ScheduledTask(id, sampleTime, taskFunction);
		Serial.println("Task created");
		taskCounter++;
		result = true;
	}
	//else Log that no more than MAX_NUMBER_OF_TASKS can be added
	return result;
}

//removes task from a list
bool TaskManager::deactivateTask(int id) {
	bool result = false;
	int indexOfExistingTask = checkIfTaskExists(id);
	if ((indexOfExistingTask >= 0) && (indexOfExistingTask < taskCounter)) {
		tasks[indexOfExistingTask].setActivated(false);
		result = true;
	}
	/*for (int i = 0; i < taskCounter; ++i) {

	 if (tasks[i].getId() == id) {
	 tasks[i].setActivated(false);
	 result = true;
	 }
	 }*/
	//if (result == false) Log that no task with provided id was found
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
int TaskManager::checkIfTaskExists(int id) {
	int result = -1;
	for (int i = 0; i < taskCounter; ++i) {
		if (tasks[i].getId() == id) {
			result = id;
			Serial.print("Task exists on index ");
			Serial.println(result);
		}
	}
	//if (result == -1) Log that no task with provided id was found
	return result;
}
