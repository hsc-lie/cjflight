#include "spl06.h"


struct spl0601_t spl0601 = {0};
struct spl0601_t *p_spl0601;

spl06_data_t spl06_data = {0};


/*****************************************************************************
 函 数 名  : spl0601_init
 功能描述  : SPL06-01 初始化函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_init(void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    //p_spl0601->i32rawPressure = 0;
    //p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = 0x34;

    spl0601_get_calib_param();
    // sampling rate = 1Hz; Pressure oversample = 2;
    spl0601_rateset(PRESSURE_SENSOR,32, 8);   
    // sampling rate = 1Hz; Temperature oversample = 1; 
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    
}

/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8_t u8OverSmpl  过采样率         Maximal = 128
             uint8_t u8SmplRate  每秒采样次数(Hz) Maximal = 128
             uint8_t iSensor     0: Pressure; 1: Temperature
 输出参数  : 无
 返 回 值  : 无
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月24日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        //spl0601_write(HW_ADR, 0x06, reg);
        simulation_i2c_writereg(SPL06_ADDR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            //reg = spl0601_read(HW_ADR, 0x09);
            simulation_i2c_readregs(SPL06_ADDR, 0x09, 1, &reg);
            //spl0601_write(HW_ADR, 0x09, reg | 0x04);
            simulation_i2c_writereg(SPL06_ADDR, 0x09, reg | 0x04);
        }
    }
    if(iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        //spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        simulation_i2c_writereg(SPL06_ADDR, 0x07, reg|0x80);
        if(u8OverSmpl > 8)
        {
            //reg = spl0601_read(HW_ADR, 0x09);
            simulation_i2c_readregs(SPL06_ADDR, 0x09, 1, &reg);
            //spl0601_write(HW_ADR, 0x09, reg | 0x08);
            simulation_i2c_writereg(SPL06_ADDR, 0x09, reg | 0x08);
        }
    }

}

/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint8_t h;
    uint8_t m;
    uint8_t l;

    //h =  spl0601_read(HW_ADR, 0x10);
    //l  =  spl0601_read(HW_ADR, 0x11);
    simulation_i2c_readregs(SPL06_ADDR, 0x10, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x11, 1, &l);

    p_spl0601->calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    
    //h =  spl0601_read(HW_ADR, 0x11);
    //l  =  spl0601_read(HW_ADR, 0x12);
    simulation_i2c_readregs(SPL06_ADDR, 0x11, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x12, 1, &l);
    p_spl0601->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    
    //h =  spl0601_read(HW_ADR, 0x13);
    //m =  spl0601_read(HW_ADR, 0x14);
    //l =  spl0601_read(HW_ADR, 0x15);
    simulation_i2c_readregs(SPL06_ADDR, 0x13, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x14, 1, &m);
    simulation_i2c_readregs(SPL06_ADDR, 0x15, 1, &l);
    p_spl0601->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    
    //h =  spl0601_read(HW_ADR, 0x15);
    //m =  spl0601_read(HW_ADR, 0x16);
    //l =  spl0601_read(HW_ADR, 0x17);
    simulation_i2c_readregs(SPL06_ADDR, 0x15, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x16, 1, &m);
    simulation_i2c_readregs(SPL06_ADDR, 0x17, 1, &l);
    p_spl0601->calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    
    //h =  spl0601_read(HW_ADR, 0x18);
    //l  =  spl0601_read(HW_ADR, 0x19);
    simulation_i2c_readregs(SPL06_ADDR, 0x18, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x19, 1, &l);
    p_spl0601->calib_param.c01 = (int16_t)h<<8 | l;
    
    //h =  spl0601_read(HW_ADR, 0x1A);
    //l  =  spl0601_read(HW_ADR, 0x1B);
    simulation_i2c_readregs(SPL06_ADDR, 0x1a, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x1b, 1, &l);
    p_spl0601->calib_param.c11 = (int16_t)h<<8 | l;
    
    //h =  spl0601_read(HW_ADR, 0x1C);
    //l  =  spl0601_read(HW_ADR, 0x1D);
    simulation_i2c_readregs(SPL06_ADDR, 0x1c, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x1d, 1, &l);
    p_spl0601->calib_param.c20 = (int16_t)h<<8 | l;
    
    //h =  spl0601_read(HW_ADR, 0x1E);
    //l  =  spl0601_read(HW_ADR, 0x1F);
    simulation_i2c_readregs(SPL06_ADDR, 0x1e, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x1f, 1, &l);
    p_spl0601->calib_param.c21 = (int16_t)h<<8 | l;
    
    //h =  spl0601_read(HW_ADR, 0x20);
    //l  =  spl0601_read(HW_ADR, 0x21);
    simulation_i2c_readregs(SPL06_ADDR, 0x20, 1, &h);
    simulation_i2c_readregs(SPL06_ADDR, 0x21, 1, &l);
    p_spl0601->calib_param.c30 = (int16_t)h<<8 | l;
}


/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_temperature(void)
{
    //spl0601_write(HW_ADR, 0x08, 0x02);
    simulation_i2c_writereg(SPL06_ADDR, 0x08, 0x02);
}

/*****************************************************************************
 函 数 名  : spl0601_start_pressure
 功能描述  : 发起一次压力值测量
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_pressure(void)
{
    //spl0601_write(HW_ADR, 0x08, 0x01);
    simulation_i2c_writereg(SPL06_ADDR, 0x08, 0x01);
}

/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8_t mode  1: pressure; 2: temperature; 3: pressure and temperature
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月25日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_continuous(uint8_t mode)
{
    //spl0601_write(HW_ADR, 0x08, mode+4);
    simulation_i2c_writereg(SPL06_ADDR, 0x08, mode+4);
}




/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

void spl06_get_all_data()
{
    uint8_t data[3] = {0};

    simulation_i2c_readregs(SPL06_ADDR, 0x00, 3, data);
    p_spl0601->i32rawPressure = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;

    simulation_i2c_readregs(SPL06_ADDR, 0x03, 3, data);
    p_spl0601->i32rawTemperature = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;


    spl06_data.temp = spl0601_get_temperature();
    spl06_data.pressure = spl0601_get_pressure();

}

