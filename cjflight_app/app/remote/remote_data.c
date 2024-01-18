#include "remote_data.h"


void RemoteDataSetRockerValue(RemoteData_t * remote, REMOTE_DATA_ROCKER_TYPE_t rockerType, int16_t Value)
{
	if(rockerType >= REMOTE_DATA_ROCKER_TYPE_MAX)
	{
		return;
	}
	else
	{
		remote->RockerValue[rockerType] = Value;
	}
}



int16_t RemoteDataGetRockerValue(RemoteData_t * remote, REMOTE_DATA_ROCKER_TYPE_t rockerType)
{
	if(rockerType >= REMOTE_DATA_ROCKER_TYPE_MAX)
	{
		return 0;
	}
	else
	{
		return remote->RockerValue[rockerType];
	}
}



void RemoteDataSetSWValue(RemoteData_t * remote, REMOTE_DATA_SW_TYPE_t swType, uint8_t Value)
{
	if(swType >= REMOTE_DATA_SW_TYPE_MAX)
	{
		return;
	}
	else
	{
		remote->SWValue[swType] = Value;
	}
}



uint8_t RemoteDataGetSWValue(RemoteData_t * remote, REMOTE_DATA_SW_TYPE_t swType)
{
	if(swType >= REMOTE_DATA_SW_TYPE_MAX)
	{
		return 0;
	}
	else
	{
		return remote->SWValue[swType];
	}
}




