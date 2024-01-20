#include "bsp_register.h"

#include "at32f4xx.h"

#include "bsp_rcc.h"

#include "bsp_dma.h"

#include "bsp_interrupt.h"

#include "gpio_dev.h"
#include "bsp_gpio.h"

#include "usart_dev.h"
#include "bsp_usart.h"
#include "ring_queue.h"

#include "i2c_dev.h"
#include "simulation_i2c.h"

#include "timer_dev.h"
#include "bsp_timer.h"


extern int main(void);


typedef struct
{
	GPIO_Type * Type;
	uint32_t Pin;
}GPIOPort_t;

static const GPIOPort_t GPIOMapping[GPIO_TYPE_MAX] = 
{
	{LED0_GPIO, LED0_GPIO_PIN},
	{LED1_GPIO, LED1_GPIO_PIN},
	{LED2_GPIO, LED2_GPIO_PIN},
};

static void GPIOWritePinOut(GPIO_TYPE_t type, uint8_t value)
{
	const GPIOPort_t *port = &GPIOMapping[type];

	if(0u == value)
	{
		GPIO_ResetBits(port->Type, port->Pin);
	}
	else
	{
		GPIO_SetBits(port->Type, port->Pin);
	}
}

static uint8_t GPIOReadPinIn(GPIO_TYPE_t type)
{
	const GPIOPort_t *port = &GPIOMapping[type];

	return GPIO_ReadInputDataBit(port->Type, port->Pin);
}

static USART_DEV_ERROR_t USART1Init()
{
	BSPUSART1Init();

	return USART_DEV_OK;
}

static USART_DEV_ERROR_t USART1SendData(uint8_t *data, uint32_t len)
{
	BSPUSARTSendData(USART1, data, len);

	return USART_DEV_OK;
}

static USART_DEV_ERROR_t USART2Init()
{
	BSPDMA1Cannel5Init();
	BSPUSART2Init();

	return USART_DEV_OK;
}

static USART_DEV_ERROR_t USART2ReadData(uint8_t *data, uint32_t readLen, uint32_t *outLen)
{
	*outLen = BSPUSART2ReadData(data, readLen);

	return USART_DEV_OK;
}


#define SIMULATION_I2C_DELAY_COUNT            (16)


static void SimulationI2C1SCLSet0()
{
	SIMULATION_I2C_SCL_GPIO->BRE = SIMULATION_I2C_SCL_GPIO_PIN;
}

static void SimulationI2C1SCLSet1()
{
	SIMULATION_I2C_SCL_GPIO->BSRE = SIMULATION_I2C_SCL_GPIO_PIN;
}

static void SimulationI2C1SDASet0()
{
	SIMULATION_I2C_SDA_GPIO->BRE = SIMULATION_I2C_SDA_GPIO_PIN;
}

static void SimulationI2C1SDASet1()
{
	SIMULATION_I2C_SDA_GPIO->BSRE = SIMULATION_I2C_SDA_GPIO_PIN;
}

static void SimulationI2C1SDASetTX()
{

}

static void SimulationI2C1SDASetRX()
{
	SIMULATION_I2C_SDA_GPIO->BSRE = SIMULATION_I2C_SDA_GPIO_PIN;
}

static uint8_t SimulationI2C1SDARead()
{
	return (SIMULATION_I2C_SDA_GPIO->IPTDT & SIMULATION_I2C_SDA_GPIO_PIN) >> SIMULATION_I2C_SDA_PIN_SOURCE;
}


static SimulationI2C_t SimulationI2C1 = 
{
	.DelayCount = SIMULATION_I2C_DELAY_COUNT,
	.SCLSet0 = SimulationI2C1SCLSet0,
	.SCLSet1 = SimulationI2C1SCLSet1,
	.SDASet0 = SimulationI2C1SDASet0,
	.SDASet1 = SimulationI2C1SDASet1,
	.SDASetTX = SimulationI2C1SDASetTX,
	.SDASetRX = SimulationI2C1SDASetRX,
	.SDARead = SimulationI2C1SDARead,
};

static I2C_DEV_ERROR_t I2C0SendData(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2CWriteData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}

static I2C_DEV_ERROR_t I2C0ReadData(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2CReadData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}

static TIMER_DEV_ERROR_t Timer3Init()
{
	BSPTimer3Init();

	return TIMER_DEV_OK;
}

static TIMER_DEV_ERROR_t Timer3PWMOut(uint8_t channel, uint32_t duty)
{
	TIMER_DEV_ERROR_t ret = TIMER_DEV_OK;

	switch (channel)
	{
		case 1:
			TMR_SetCompare1(TMR3, duty);
			break;
		case 2:
			TMR_SetCompare2(TMR3, duty);
			break;
		case 3:
			TMR_SetCompare3(TMR3, duty);
			break;
		case 4:
			TMR_SetCompare4(TMR3, duty);
			break;
		default:
			ret = TIMER_DEV_ERROR_INVALID_PARAM;
			break;
	}

	return ret;
}

static TIMER_DEV_ERROR_t Timer6Init()
{
	BSPTimer6Init();

	return TIMER_DEV_OK;
}

static TIMER_DEV_ERROR_t Timer6GetCount(uint32_t *count)
{
	*count = TimerGetCount(TMR6);

	return TIMER_DEV_OK;
}

GPIODev_t GPIODev =
{
	.Init = BSPGPIOInitAll,
	.DeInit = NULL,
	.WritePinOut = GPIOWritePinOut,
	.ReadPinIn = GPIOReadPinIn,
};


USARTDev_t USART1Dev = 
{
	.Init = USART1Init,
	.DeInit = NULL,
	.SendData = USART1SendData,
	.ReadData = NULL,
};


USARTDev_t USART2Dev = 
{
	.Init = USART2Init,
	.DeInit = NULL,
	.SendData = NULL,
	.ReadData = USART2ReadData,
};


static I2CDev_t I2CDev0 =
{
	.Init = NULL,
    .DeInit = NULL,
	.SendData = I2C0SendData,
	.ReadData = I2C0ReadData,
};

static TimerDev_t Timer6Dev =
{
	.Init = Timer6Init,
    .DeInit = NULL,
	.GetCount = Timer6GetCount,
	.PWMOut = NULL,
};

static TimerDev_t Timer3Dev =
{
	.Init = Timer3Init,
    .DeInit = NULL,
	.GetCount = NULL,
	.PWMOut = Timer3PWMOut,
};


void BSPMain(void)
{
	BSPInterruptInit();
	BSPRCCInitAll();

	GPIODevRegister(&GPIODev);
	USARTDevRegister(USART_DEV_PRINTF, &USART1Dev);
	USARTDevRegister(USART_DEV_REMOTE, &USART2Dev);
	I2CDevRegister(I2C_DEV_SENSOR, &I2CDev0);
	TimerDevRegister(TIMER_DEV_TEST, &Timer6Dev);
	TimerDevRegister(TIMER_DEV_MOTOR_PWM, &Timer3Dev);
	
	main();
}


