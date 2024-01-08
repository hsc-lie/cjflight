#include "spl06.h"

   
//获取补偿参数
void SPL06GetCalibParams(SPL06_t * spl06)
{

	uint8_t data[18] = {0};

	if(NULL == spl06->I2CReadReg)
	{
		return;
	}


	spl06->I2CReadReg(spl06->DevAddr, 0x10, data, 18);

	
	
	spl06->CalibParams.c0 = ((int16_t)data[0] << 4) | (data[1] >> 4);
	spl06->CalibParams.c0 = (spl06->CalibParams.c0&0x0800)?(0xF000|spl06->CalibParams.c0):spl06->CalibParams.c0;
	

	spl06->CalibParams.c1 = (int16_t)(data[1] & 0x0F) << 8 | data[2];
    spl06->CalibParams.c1 = (spl06->CalibParams.c1 & 0x0800)?(0xF000|spl06->CalibParams.c1):spl06->CalibParams.c1;
	

	spl06->CalibParams.c00 = (int32_t)data[3]<<12 | (int32_t)data[4]<<4 | (int32_t)data[5]>>4;
    spl06->CalibParams.c00 = (spl06->CalibParams.c00&0x080000)?(0xFFF00000|spl06->CalibParams.c00):spl06->CalibParams.c00;

	

	spl06->CalibParams.c10 = (int32_t)data[5] << 16 | (int32_t)data[6]<<8 | data[7];
    spl06->CalibParams.c10 = (spl06->CalibParams.c10&0x080000)?(0xFFF00000|spl06->CalibParams.c10):spl06->CalibParams.c10;


	spl06->CalibParams.c01 = (int16_t)data[8] << 8 | data[9];
    
    spl06->CalibParams.c11 = (int16_t)data[10] << 8 | data[11];
	spl06->CalibParams.c11 = (spl06->CalibParams.c11 & 0x0800) ? (0xF000 | spl06->CalibParams.c11) : spl06->CalibParams.c11;

    spl06->CalibParams.c20 = (int16_t)data[12] << 8 | data[13];
	spl06->CalibParams.c20 = (spl06->CalibParams.c20&0x0800) ? (0xF000|spl06->CalibParams.c20):spl06->CalibParams.c20;
    
	spl06->CalibParams.c21 = (int16_t)data[14] << 8 | data[15];
	spl06->CalibParams.c21 = (spl06->CalibParams.c21&0x0800)?(0xF000|spl06->CalibParams.c21):spl06->CalibParams.c21;

    spl06->CalibParams.c30 = (int16_t)data[16] << 8 | data[17];
	spl06->CalibParams.c30 = (spl06->CalibParams.c30&0x0800)?(0xF000|spl06->CalibParams.c30):spl06->CalibParams.c30;

    
}


static void SPL06Delay()
{
	uint32_t count = 100000;

	while(--count);
}


//SPL06初始化
SPL06_ERROR_t SPL06Init(SPL06_t * spl06)
{

	uint8_t id = 0;
	uint8_t count = 0;

	if((NULL == spl06->I2CWriteReg)
		|| (NULL == spl06->I2CReadReg)
	)
	{
		return SPL06_ERROR_NULL;
	}

	SPL06Delay();

	do
	{
		++count;
		spl06->I2CReadReg(spl06->DevAddr, 0x0D, &id, 1);

		if(count > 5)
		{
			return SPL06_ERROR_DEV_NOT_FOUND;
		}
	}
	while (id != 0x10);
	

	SPL06GetCalibParams(spl06);

	SPL06SetPressureRate(spl06, spl06->PressureRate, spl06->PressurePRC);

	SPL06SetTemperatureRate(spl06, spl06->TemperatureRate, spl06->TemperaturePRC);

	SPL06SetMode(spl06, spl06->Mode);


	return SPL06_ERROR_OK;
}


static int32_t SPL06GetKPKT(SPL06_PRC_t prc) 
{
	int32_t kpkt;

	switch(prc)
    {
    	case SPL06_PRC_TIMES1:
			kpkt = 524288;
			break;
        case SPL06_PRC_TIMES2:
            kpkt = 1572864;
            break;
        case SPL06_PRC_TIMES4:
            kpkt = 3670016;
            break;
        case SPL06_PRC_TIMES8:
            kpkt = 7864320;
            break;
        case SPL06_PRC_TIMES16:
            kpkt = 253952;
            break;
        case SPL06_PRC_TIMES32:
            kpkt = 516096;
            break;
        case SPL06_PRC_TIMES64:
            kpkt = 1040384;
            break;
        case SPL06_PRC_TIMES128:
            kpkt = 2088960;
            break;
        default:
            kpkt = 524288;
            break;
    }

	return kpkt;
	
}


void SPL06SetPressureRate(SPL06_t * spl06, SPL06_RATE_t rate, SPL06_PRC_t prc)
{

	uint8_t writeData = 0;
	uint8_t readData = 0;


	spl06->KP = SPL06GetKPKT(prc);
	writeData = (rate << 4) | prc;
	spl06->I2CWriteReg(spl06->DevAddr, 0x06, &writeData, 1);

    if(prc > SPL06_PRC_TIMES8)
    {
		spl06->I2CReadReg(spl06->DevAddr, 0x09, &readData, 1);	
		writeData = readData | 0x04;
		spl06->I2CWriteReg(spl06->DevAddr, 0x09, &writeData, 1);
    }


}


void SPL06SetTemperatureRate(SPL06_t * spl06, SPL06_RATE_t rate, SPL06_PRC_t prc)
{

	uint8_t writeData = 0;
	uint8_t readData = 0;


	spl06->KT = SPL06GetKPKT(prc);
	
	writeData = (rate << 4) | prc;
	spl06->I2CWriteReg(spl06->DevAddr, 0x07, &writeData, 1);
    if(prc > SPL06_PRC_TIMES8)
    {
		spl06->I2CReadReg(spl06->DevAddr, 0x09, &readData, 1);
		writeData = readData | 0x08;
		spl06->I2CWriteReg(spl06->DevAddr, 0x09, &writeData, 1);
    }

}


//设置模式
void SPL06SetMode(SPL06_t * spl06, SPL06_MODE_t mode)
{
	spl06->I2CWriteReg(spl06->DevAddr, 0x08, &mode, 1);
}


//获取SPL06温度
float SPL06GetTemperature(SPL06_t * spl06)
{
    float tsc;

	uint8_t data[3] = {0};

	if(NULL == spl06->I2CReadReg)
	{
		return 0;
	}

	spl06->I2CReadReg(spl06->DevAddr, 0x03, data, 3);
    spl06->RawTemperature = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    spl06->RawTemperature= (spl06->RawTemperature&0x800000) ? (0xFF000000|spl06->RawTemperature) : spl06->RawTemperature;

    tsc = spl06->RawTemperature / (float)spl06->KT;
	
    return (spl06->CalibParams.c0 * 0.5 + spl06->CalibParams.c1 * tsc);
}

//获取SPL06气压值
float SPL06GetPressure(SPL06_t * spl06)
{
    float tsc, psc;
    float qua2, qua3;

	uint8_t data[3] = {0};

	if(NULL == spl06->I2CReadReg)
	{
		return 0;
	}
	
	spl06->I2CReadReg(spl06->DevAddr, 0x00, data, 3);
    spl06->RawPressure = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    spl06->RawPressure= (spl06->RawPressure & 0x800000) ? (0xFF000000|spl06->RawPressure) : spl06->RawPressure;
	

    tsc = spl06->RawTemperature / (float)spl06->KT;
    psc = spl06->RawPressure / (float)spl06->KP;
    qua2 = spl06->CalibParams.c10 + psc * (spl06->CalibParams.c20 + psc* spl06->CalibParams.c30);
    qua3 = tsc * psc * (spl06->CalibParams.c11 + psc * spl06->CalibParams.c21);

    return (spl06->CalibParams.c00 + psc * qua2 + tsc * spl06->CalibParams.c01 + qua3);
}


//获取SPL06所有数据
void SPL06GetDataAll(SPL06_t * spl06, SPL06Data_t * data)
{
	data->Pressure = SPL06GetPressure(spl06);
	data->Temperature = SPL06GetTemperature(spl06);
}
