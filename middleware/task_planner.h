#ifndef _TASK_PLANNER_H
#define _TASK_PLANNER_H

#include <stdint.h>

typedef enum {
    TASK_IDLE = 0,
    TASK_PHASE1_STRAIGHT,
    TASK_PHASE1_STOPPED
} TaskState;

void      Task_Planner_Init(void);
void      Task_Planner_On_Key(void);
void      Task_Planner_Update(uint16_t *sensor_data);
TaskState Task_Planner_Get_State(void);

#endif
