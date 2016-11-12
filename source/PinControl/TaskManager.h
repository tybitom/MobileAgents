/*
 * TaskManager.h
 *
 *  Created on: 30.10.2016
 *      Author: Tomek
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_

// edit carefully to do not to run out of Arduino memory
#define MAX_NUMBER_OF_TASKS 5

#include "ScheduledTask.h"

class TaskManager {
private:
	static int taskCounter;
	ScheduledTask tasks[MAX_NUMBER_OF_TASKS];
	static TaskManager *taskManagerInstance;

	int checkIfTaskExists(int id);
public:
	static TaskManager* getInstance();
	//bool addTask(ScheduledTask task);
	bool addTask(int id, unsigned long sampleTime, void (*taskFunction)(void));
	bool deactivateTask(int id);
	void realizeTasks();
};

#endif /* TASKMANAGER_H_ */
