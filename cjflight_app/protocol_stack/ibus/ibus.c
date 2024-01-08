#include "ibus.h"

/*
 * @函数名  IBusAnalysisData
 * @用  途  IBUS数据解析,如果接受到一个包,会通过IBus_t中的ReceivedPackageFunc回调
 * @参  数  ibus:ibus结构体
 *          data:串口接收到的数据
 *          len:串口接收到的数据长度
 *          isGetPackage:是否获取完整的一个包
 * @返回值  限幅后的值
*/
bool IBusAnalysisData(IBus_t *ibus, uint8_t *data, uint32_t len)
{
	uint32_t i;
	uint16_t checkSum;
	bool getPackageFlag = FALSE;

	for(i = 0;i < len;i++)
	{
		//头
		if((0 == ibus->NowDataIndex) && (0x20 == *data))
		{
			ibus->DataBuffer[0] = *data;
			++ibus->NowDataIndex;
		}
		//头
		else if((1 == ibus->NowDataIndex) && (0x40 == *data))
		{
			ibus->DataBuffer[1] = *data;
			ibus->CheckSum = 0;
			++ibus->NowDataIndex;
		}
		//数据
		else if((ibus->NowDataIndex > 1) && (ibus->NowDataIndex < 30))    //30
		{
			ibus->DataBuffer[ibus->NowDataIndex] = *data;
		  	ibus->CheckSum += *data;
		  	++ibus->NowDataIndex;
		}
		else if(30 == ibus->NowDataIndex)
		{
			ibus->DataBuffer[ibus->NowDataIndex] = *data;
			++ibus->NowDataIndex;
		}
		//数据校验
		else if(31 == ibus->NowDataIndex)
		{
			ibus->DataBuffer[ibus->NowDataIndex] = *data;
			ibus->NowDataIndex = 0;
			
			checkSum = (((uint16_t)ibus->DataBuffer[31]) << 8) | ibus->DataBuffer[30];
  			ibus->CheckSum = (ibus->CheckSum + 0x60) ^ 0xffff;
			if(ibus->CheckSum == checkSum)
			{
				for(i = 0;i < IBUS_Channel_MAX;i++)
			    {
					ibus->ChannelData[i] = ((uint16_t)(ibus->DataBuffer[(i << 1) + 3]) & 0x0f) << 8 | (ibus->DataBuffer[(i << 1) + 2]);
			    }

				getPackageFlag = TRUE;

				if(NULL != ibus->ReceivedPackageFunc)
				{
					ibus->ReceivedPackageFunc(ibus->ChannelData);
				}
			}
		}
		else
		{
			ibus->NowDataIndex = 0;
		}
		
		++data;
	}

	return getPackageFlag;
}

/*
 * @函数名  IBusGetChannelData
 * @用  途  获取IBUS的通道数据
 * @参  数  ibus:ibus结构体
 *          channel:通道
 * @返回值  通道的数据
*/
uint16_t IBusGetChannelData(IBus_t *ibus, IBUS_Channel_t channel)
{
	if(NULL == ibus)
	{
		return 0;
	}
	else
	{
		return ibus->ChannelData[channel];
	}
}
