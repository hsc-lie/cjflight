#ifndef __REMOTE_DATA_H_
#define __REMOTE_DATA_H_


#include "common.h"




typedef enum
{
	E_REMOTE_DATA_LEFT_ROCKER_X,
	E_REMOTE_DATA_LEFT_ROCKER_Y,

	E_REMOTE_DATA_RIGHT_ROCKER_X,
	E_REMOTE_DATA_RIGHT_ROCKER_Y,

	E_REMOTE_DATA_ROCKER_TYPE_MAX
}E_REMOTE_DATA_ROCKER_TYPE;



typedef enum
{
	E_REMOTE_DATA_SW1,
	E_REMOTE_DATA_SW2,
	E_REMOTE_DATA_SW3,
	E_REMOTE_DATA_SW4,
	
	E_REMOTE_DATA_SW_TYPE_MAX
}E_REMOTE_DATA_SW_TYPE;



typedef struct
{
	uint8_t SWValue[E_REMOTE_DATA_SW_TYPE_MAX];
	int16_t RockerValue[E_REMOTE_DATA_ROCKER_TYPE_MAX];
}RemoteData_t;



extern void RemoteData_SetRockerValue(RemoteData_t * remote, E_REMOTE_DATA_ROCKER_TYPE rockerType, int16_t Value);
extern int16_t RemoteData_GetRockerValue(RemoteData_t * remote, E_REMOTE_DATA_ROCKER_TYPE rockerType);
extern void RemoteData_SetSWValue(RemoteData_t * remote, E_REMOTE_DATA_SW_TYPE swType, uint8_t Value);
extern uint8_t RemoteData_GetSWValue(RemoteData_t * remote, E_REMOTE_DATA_SW_TYPE swType);



#endif /*__REMOTE_DATA_H_*/
