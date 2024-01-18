#ifndef __REMOTE_DATA_H_
#define __REMOTE_DATA_H_


#include "common.h"


typedef enum
{
	REMOTE_DATA_LEFT_ROCKER_X,
	REMOTE_DATA_LEFT_ROCKER_Y,
	REMOTE_DATA_RIGHT_ROCKER_X,
	REMOTE_DATA_RIGHT_ROCKER_Y,
	REMOTE_DATA_ROCKER_TYPE_MAX
}REMOTE_DATA_ROCKER_TYPE_t;


typedef enum
{
	REMOTE_DATA_SW1,
	REMOTE_DATA_SW2,
	REMOTE_DATA_SW3,
	REMOTE_DATA_SW4,
	REMOTE_DATA_SW_TYPE_MAX
}REMOTE_DATA_SW_TYPE_t;



typedef struct
{
	uint8_t SWValue[REMOTE_DATA_SW_TYPE_MAX];
	int16_t RockerValue[REMOTE_DATA_ROCKER_TYPE_MAX];
}RemoteData_t;



extern void RemoteDataSetRockerValue(RemoteData_t * remote, REMOTE_DATA_ROCKER_TYPE_t rockerType, int16_t Value);
extern int16_t RemoteDataGetRockerValue(RemoteData_t * remote, REMOTE_DATA_ROCKER_TYPE_t rockerType);
extern void RemoteDataSetSWValue(RemoteData_t * remote, REMOTE_DATA_SW_TYPE_t swType, uint8_t Value);
extern uint8_t RemoteDataGetSWValue(RemoteData_t * remote, REMOTE_DATA_SW_TYPE_t swType);



#endif /*__REMOTE_DATA_H_*/
