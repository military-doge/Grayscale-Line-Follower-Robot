#ifndef _TASK_PLANNER_H
#define _TASK_PLANNER_H

#include <stdint.h>

typedef enum {
    TASK_IDLE = 0,
    TASK_PHASE1_STRAIGHT,
    TASK_PHASE1_STOPPED,
    TASK_PHASE2_STRAIGHT1,
    TASK_PHASE2_TRACK1,
    TASK_PHASE2_FWD1,
    TASK_PHASE2_ALIGN,
    TASK_PHASE2_STRAIGHT2,
    TASK_PHASE2_TRACK2,
    TASK_PHASE2_FWD2,
    TASK_PHASE2_DONE,
    TASK_PHASE3_RTURN_ALIGN,
    TASK_PHASE3_STRAIGHT1,
    TASK_PHASE3_TRACK1,
    TASK_PHASE3_FWD1,
    TASK_PHASE3_ALIGN180,
    TASK_PHASE3_LTURN_ALIGN,
    TASK_PHASE3_STRAIGHT2,
    TASK_PHASE3_TRACK2,
    TASK_PHASE3_FWD2,
    TASK_PHASE3_ALIGN0,
    TASK_PHASE3_DONE
} TaskState;

void      Task_Planner_Init(void);
void      Task_Planner_On_Key(void);
void      Task_Planner_Update(uint16_t *sensor_data, int encoder_a, int encoder_b);
TaskState Task_Planner_Get_State(void);

#endif
