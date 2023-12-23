#include "bsp_register.h"

#include "at32f4xx.h"

#include "bsp_rcc.h"

#include "gpio_dev.h"
#include "bsp_gpio.h"

#include "usart_dev.h"
#include "bsp_usart.h"
#include "circular_queue.h"

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
	
	{SIMULATION_I2C_SCL_GPIO, SIMULATION_I2C_SCL_GPIO_PIN},
	{SIMULATION_I2C_SDA_GPIO, SIMULATION_I2C_SDA_GPIO_PIN},

};

static void GPIOWritePinOut(GPIO_TYPE_t type, uint8_t value)
{
	GPIOPort_t *port = &GPIOMapping[type];

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
	GPIOPort_t *port = &GPIOMapping[type];

	return GPIO_ReadInputDataBit(port->Type, port->Pin);
}

static void USART1SendData(uint8_t * data, uint32_t len)
{
	BSPUSARTSendData(USART1, data, len);
}

static void USART2Init()
{
	BSPDMA1Cannel5Init();
	BSPUSART2Init();
}

static void USART2ReadData(uint8_t * data, uint32_t readLen, uint32_t * outLen)
{
	uint32_t i;
	
	for(i = 0;i < readLen;++i)
	{
	
		if(CIRCULAR_QUEUE_ERROR_OK != CircularQueue_ReadByte(&USART2Queue, data))
		{
			break;
		}
		else
		{
			++data;
		}	
		
	}

	*outLen = i;
}


#define SIMULATION_I2C_DELAY_COUNT            (20)

static void SimulationI2C1SCLSet(uint8_t value)
{
	GPIODevWritePinOut(GPIO_TYPE_SIMULATION_I2C_SCL, value);
}

static void SimulationI2C1SDASet(uint8_t value)
{
	GPIODevWritePinOut(GPIO_TYPE_SIMULATION_I2C_SDA, value);
}

static void SimulationI2C1SDADirSet(SIMULATION_I2C_SDA_DIR_t dir)
{
	if(SIMULATION_I2C_SDA_RX == dir)
	{
		GPIODevWritePinOut(GPIO_TYPE_SIMULATION_I2C_SDA, 1);
	}
}

static uint8_t SimulationI2C1SDARead()
{
	return GPIODevReadPinIn(GPIO_TYPE_SIMULATION_I2C_SDA);
}


static SimulationI2C_t SimulationI2C1 = 
{
	.DelayCount = SIMULATION_I2C_DELAY_COUNT,
	.SCLSet = SimulationI2C1SCLSet,
	.SDASet = SimulationI2C1SDASet,
	.SDADirSet = SimulationI2C1SDADirSet,
	.SDARead = SimulationI2C1SDARead,
};

static int I2C0SendData(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2CSendData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}

static int I2C0ReadData(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2CReadData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}


static void Timer3PWMOut(uint8_t channel, uint32_t duty)
{
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
			break;
	}
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
	.Dev = 
	{
		.ID = 1,
		.Init = BSPUSART1Init,
		.DeInit = NULL,
	},
	.SendData = USART1SendData,
	.ReadData = NULL,
};


USARTDev_t USART2Dev = 
{
	.Dev = 
	{
		.ID = 2,
		.Init = USART2Init,
		.DeInit = NULL,
	},
	.SendData = NULL,
	.ReadData = USART2ReadData,
};


static I2CDev_t I2CDev0 =
{
	.Dev = 
	{
		.ID = 0,
		.Init = NULL,
    	.DeInit = NULL,
	},
	.SendData = I2C0SendData,
	.ReadData = I2C0ReadData,
};


static TimerDev_t Timer1Dev =
{
	.Dev = 
	{
		.ID = 1,
		.Init = BSPTimer3Init,
    	.DeInit = NULL,
	},
	.PWMOut = Timer3PWMOut,
};



static void USARTDevRegisterAll()
{
	USARTDevRegister(&USART1Dev);
	USARTDevRegister(&USART2Dev);
}

static void I2CDevRegisterAll()
{
    I2CDevRegister(&I2CDev0);
}

static void TimerDevRegisterAll()
{
	TimerDevRegister(&Timer1Dev);
}


void BSPMain(void)
{
	/*中断优先级分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	BSPRCCInitAll();

	GPIODevRegister(&GPIODev);

	USARTDevRegisterAll();
    I2CDevRegisterAll();
	TimerDevRegisterAll();

	main();
}


