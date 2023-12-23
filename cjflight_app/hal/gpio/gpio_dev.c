#include "gpio_dev.h"


static GPIODev_t *GPIODev = NULL;


void GPIODevRegister(GPIODev_t *gpio)
{
    GPIODev = gpio;
}

void GPIODevUnregister(GPIODev_t *gpio)
{
    GPIODev = NULL;
}


void GPIODevInit()
{
    if((NULL != GPIODev) && (NULL != GPIODev->Init))
    {
        GPIODev->Init();
    }
}


void GPIODevDeInit()
{
    if((NULL != GPIODev) && (NULL != GPIODev->DeInit))
    {
        GPIODev->DeInit();
    }
}


void GPIODevWritePinOut(GPIO_TYPE_t type, uint8_t value)
{
    if((NULL != GPIODev) && (NULL != GPIODev->WritePinOut))
    {
        GPIODev->WritePinOut(type, value);
    }

}

uint8_t GPIODevReadPinIn(GPIO_TYPE_t type)
{
    if((NULL == GPIODev) || (NULL == GPIODev->ReadPinIn))
    {
        return 0xFF;
    }

    return GPIODev->ReadPinIn(type);
}
