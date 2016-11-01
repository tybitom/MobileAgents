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

// adds task to the task list runed every time realizeTasks() method is called
bool TaskManager::addTask(ScheduledTask task) {
	if (taskCounter < MAX_NUMBER_OF_TASKS) {
		tasks[taskCounter] = task;
		taskCounter++;
		return true;
	}
	//else Log that no more than MAX_NUMBER_OF_TASKS can be added
	return false;
}

// realizes all previously added tasks
void TaskManager::realizeTasks() {
	for (int i = 0; i < taskCounter; ++i) {
		tasks[i].realizeTask();
	}
}
