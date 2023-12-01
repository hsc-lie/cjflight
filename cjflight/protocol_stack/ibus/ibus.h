#ifndef __IBUS_H_
#define __IBUS_H_


#include "common.h"



typedef enum
{
    E_IBUS_Channel1,
    E_IBUS_Channel2,
    E_IBUS_Channel3,
    E_IBUS_Channel4,
    E_IBUS_Channel5,
    E_IBUS_Channel6,
    E_IBUS_Channel7,
    E_IBUS_Channel8,
    E_IBUS_Channel_MAX
}E_IBUS_Channel;




typedef enum
{
	E_IBUS_ERROR_OK = 0,
	E_IBUS_ERROR_NULL,
}E_IBUS_ERROR_TYPE;





typedef struct
{

	uint8_t NowDataIndex;
	uint16_t CheckSum;
	
	uint8_t DataBuffer[32];

	uint16_t ChannelData[E_IBUS_Channel_MAX];

	void (* ReceivedPackageFunc)(uint16_t * channelData);
	
}IBUS_t;


E_IBUS_ERROR_TYPE IBUS_AnalysisData(IBUS_t * ibus, uint8_t * data, uint32_t len, uint8_t * const isGetPackage);
uint16_t IBUS_GetChannelData(IBUS_t * ibus, E_IBUS_Channel channel);



#endif /*__IBUS_H_*/
