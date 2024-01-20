#include "gpio_dev.h"

static GPIODev_t *GPIODev = NULL;

/*
 * @函数名  GPIODevRegister
 * @用  途  GPIO设备注册
 * @参  数  gpio:GPIO设备
 * @返回值
*/
void GPIODevRegister(GPIODev_t *gpio)
{
    GPIODev = gpio;
}

/*
 * @函数名  GPIODevUnregister
 * @用  途  GPIO设备注销
 * @参  数
 * @返回值
*/
void GPIODevUnregister()
{
    GPIODev = NULL;
}

/*
 * @函数名  GPIODevInit
 * @用  途  GPIO设备初始化
 * @参  数
 * @返回值
*/
void GPIODevInit()
{
    if((NULL != GPIODev) && (NULL != GPIODev->Init))
    {
        GPIODev->Init();
    }
}

/*
 * @函数名  GPIODevDeInit
 * @用  途  GPIO设备反初始化
 * @参  数
 * @返回值
*/
void GPIODevDeInit()
{
    if((NULL != GPIODev) && (NULL != GPIODev->DeInit))
    {
        GPIODev->DeInit();
    }
}

/*
 * @函数名  GPIODevWritePinOut
 * @用  途  GPIO设备写入引脚高低电平
 * @参  数  type:GPIO类型
 *          value:值
 * @返回值
*/
void GPIODevWritePinOut(GPIO_TYPE_t type, uint8_t value)
{
    if((NULL != GPIODev) && (NULL != GPIODev->WritePinOut))
    {
        GPIODev->WritePinOut(type, value);
    }

}

/*
 * @函数名  GPIODevReadPinIn
 * @用  途  GPIO设备读取引脚高低电平
 * @参  数  type:GPIO类型
 * @返回值  引脚高低电平值
*/
uint8_t GPIODevReadPinIn(GPIO_TYPE_t type)
{
    if((NULL == GPIODev) || (NULL == GPIODev->ReadPinIn))
    {
        return 0xFF;
    }

    return GPIODev->ReadPinIn(type);
}
