#include "remote_data.h"


void RemoteData_SetRockerValue(RemoteData_t * remote, E_REMOTE_DATA_ROCKER_TYPE rockerType, int16_t Value)
{
	if((NULL == remote)
		&& (rockerType >= E_REMOTE_DATA_ROCKER_TYPE_MAX)
	)
	{
		return;
	}
	else
	{
		remote->RockerValue[rockerType] = Value;
	}
}



int16_t RemoteData_GetRockerValue(RemoteData_t * remote, E_REMOTE_DATA_ROCKER_TYPE rockerType)
{
	if((NULL == remote)
		&& (rockerType >= E_REMOTE_DATA_ROCKER_TYPE_MAX)
	)
	{
		return 0;
	}
	else
	{
		return remote->RockerValue[rockerType];
	}
}



void RemoteData_SetSWValue(RemoteData_t * remote, E_REMOTE_DATA_SW_TYPE swType, uint8_t Value)
{
	if((NULL == remote)
		&& (swType >= E_REMOTE_DATA_SW_TYPE_MAX)
	)
	{
		return;
	}
	else
	{
		remote->SWValue[swType] = Value;
	}
}



uint8_t RemoteData_GetSWValue(RemoteData_t * remote, E_REMOTE_DATA_SW_TYPE swType)
{
	if((NULL == remote)
		&& (swType >= E_REMOTE_DATA_SW_TYPE_MAX)
	)
	{
		return 0;
	}
	else
	{
		return remote->SWValue[swType];
	}
}




