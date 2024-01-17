#ifndef __SENSORS_H_
#define __SENSORS_H_

#include "common.h"
#include "quaternion.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

typedef struct 
{
    TriaxialData_t Gyro;
    TriaxialData_t Acc;
    TriaxialData_t Mag;
    AttitudeData_t Angle;
}SensorsData_t;


extern QueueHandle_t SensorsDataMutex;

extern void SensorsTask(void *params);
extern void SensorsGetData(SensorsData_t *data);

#endif /*__SENSORS_H_*/