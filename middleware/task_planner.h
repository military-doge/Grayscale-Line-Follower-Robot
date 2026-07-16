#ifndef _TASK_PLANNER_H
#define _TASK_PLANNER_H

#include <stdint.h>

typedef enum {
    TASK_IDLE = 0,
    TASK_PHASE1_STRAIGHT,
    TASK_PHASE1_STOPPED,
    TASK_PHASE2_STRAIGHT1,
    TASK_PHASE2_TRACK1,
    TASK_PHASE2_ALIGN,
    TASK_PHASE2_STRAIGHT2,
    TASK_PHASE2_TRACK2,
    TASK_PHASE2_DONE
} TaskState;

void      Task_Planner_Init(void);
void      Task_Planner_On_Key(void);
void      Task_Planner_Update(uint16_t *sensor_data);
TaskState Task_Planner_Get_State(void);

#endif
