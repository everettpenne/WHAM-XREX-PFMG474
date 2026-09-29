/*
 * tasks.h -- the main loop's periodic work, one function per task, each
 * called once per iteration from App_Poll() (app.c). Bare metal, no RTOS:
 * a task must return quickly and never block.
 */
#ifndef TASKS_H
#define TASKS_H

/* State-independent fault polling (task_faults.c). */
void TaskFaults_Poll(void);

/* Serial command link: dispatch a completed line, if any (task_scpi.c). */
void TaskScpi_Poll(void);

#endif /* TASKS_H */
