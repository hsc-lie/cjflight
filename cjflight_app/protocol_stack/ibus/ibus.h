#ifndef __IBUS_H_
#define __IBUS_H_


#include "common.h"



typedef enum
{
    IBUS_Channel1,
    IBUS_Channel2,
    IBUS_Channel3,
    IBUS_Channel4,
    IBUS_Channel5,
    IBUS_Channel6,
    IBUS_Channel7,
    IBUS_Channel8,
    IBUS_Channel_MAX
}IBUS_Channel_t;




typedef enum
{
	IBUS_ERROR_OK = 0,
	IBUS_ERROR_NULL,
}IBUS_ERROR_TYPE_t;





typedef struct
{

	uint8_t NowDataIndex;
	uint16_t CheckSum;
	
	uint8_t DataBuffer[32];

	uint16_t ChannelData[IBUS_Channel_MAX];

	void (* ReceivedPackageFunc)(uint16_t *channelData);
	
}IBus_t;


IBUS_ERROR_TYPE_t IBusAnalysisData(IBus_t *ibus, uint8_t *data, uint32_t len, uint8_t * const isGetPackage);
uint16_t IBusGetChannelData(IBus_t *ibus, IBUS_Channel_t channel);



#endif /*__IBUS_H_*/
