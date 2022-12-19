#ifndef __MAIN_H_
#define __MAIN_H_


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


extern xQueueHandle RemoteDataQueue;


extern xQueueHandle BaroAltitudeQueue;
extern xQueueHandle AltitudeQueue;
extern xQueueHandle AccQueue;




#endif /*__MAIN_H_*/
