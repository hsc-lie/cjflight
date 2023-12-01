#include "gpio_cfg.h"
#include "at32f4xx.h"





typedef struct
{
	GPIO_Type * GPIOx;
	uint16_t GPIO_PinSource; 
	uint8_t GPIO_AF;
	GPIO_InitType GPIO_InitType;
	
	
}GPIO_Config_t;








//LED GPIO
const GPIO_Config_t LED_GPIOConfigTable[] = 
{
	//LED1
	{
		.GPIOx = LED1_GPIO,
		.GPIO_PinSource = 0,
		.GPIO_AF = 0,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = LED1_GPIO_PIN,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//LED2
	{
		.GPIOx = LED2_GPIO,
		.GPIO_PinSource = 0,
		.GPIO_AF = 0,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = LED2_GPIO_PIN,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//LED3
	{
		.GPIOx = LED3_GPIO,
		.GPIO_PinSource = 0,
		.GPIO_AF = 0,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = LED3_GPIO_PIN,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
};




//USART1
const GPIO_Config_t USART1_GPIOConfigTable[] = 
{
	//串口1 TX
	{
		.GPIOx = GPIOB,
		.GPIO_PinSource = GPIO_PinsSource6,
		.GPIO_AF = GPIO_AF_0,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_6,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
	//串口1 RX
	{
		.GPIOx = GPIOA,
		.GPIO_PinSource = GPIO_PinsSource6,
		.GPIO_AF = GPIO_AF_0,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_7,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

};

//USART2
const GPIO_Config_t USART2_GPIOConfigTable[] = 
{
	//USART2 RX
	{
		.GPIOx = GPIOA,
		.GPIO_PinSource = GPIO_PinsSource3,
		.GPIO_AF = GPIO_AF_1,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_3,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
};


//软件I2C
const GPIO_Config_t SimulationI2C_GPIOConfigTable[] = 
{
	//软件I2C SCL
	{
		.GPIOx = SIMULATION_I2C_SCL_GPIO,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = SIMULATION_I2C_SCL_GPIO_PIN,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_OD,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//软件I2C SDA
	{
		.GPIOx = SIMULATION_I2C_SDA_GPIO,
		.GPIO_InitType = 
		{
			.GPIO_Pins	= SIMULATION_I2C_SDA_GPIO_PIN,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_OutType = GPIO_OutType_OD,
			.GPIO_Pull = GPIO_Pull_NOPULL,
			.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
	
	
};

//Timer3
const GPIO_Config_t Timer3_GPIOConfigTable[] = 
{
	//Timer3 PWM
	{
		.GPIOx = GPIOB,
		.GPIO_PinSource = GPIO_PinsSource0,
		.GPIO_AF = GPIO_AF_1,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_0,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//Timer3 PWM
	{
		.GPIOx = GPIOB,
		.GPIO_PinSource = GPIO_PinsSource1,
		.GPIO_AF = GPIO_AF_1,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_1,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//Timer3 PWM
	{
		.GPIOx = GPIOB,
		.GPIO_PinSource = GPIO_PinsSource4,
		.GPIO_AF = GPIO_AF_1,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_4,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

	//Timer3 PWM
	{
		.GPIOx = GPIOB,
		.GPIO_PinSource = GPIO_PinsSource5,
		.GPIO_AF = GPIO_AF_1,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_5,
	    	.GPIO_Mode = GPIO_Mode_AF,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
};




void GPIO_ConfigInit(GPIO_Config_t * const gpioConfigTable, uint32_t len)
{
	uint32_t i;

	for(i = 0;i < len;++i)
	{
		GPIO_Init(gpioConfigTable[i].GPIOx, (GPIO_Config_t *)&gpioConfigTable[i].GPIO_InitType);
		GPIO_PinAFConfig(gpioConfigTable[i].GPIOx, gpioConfigTable[i].GPIO_PinSource, gpioConfigTable[i].GPIO_AF);
	}
}


void GPIO_ConfigInitAll(void)
{


	//LED GPIO
	GPIO_ConfigInit(&LED_GPIOConfigTable, sizeof(LED_GPIOConfigTable)/sizeof(GPIO_Config_t));
	//USART1
	GPIO_ConfigInit(&USART1_GPIOConfigTable, sizeof(USART1_GPIOConfigTable)/sizeof(GPIO_Config_t));
	//USART2
	GPIO_ConfigInit(&USART2_GPIOConfigTable, sizeof(USART2_GPIOConfigTable)/sizeof(GPIO_Config_t));

	//软件I2C
	GPIO_ConfigInit(&SimulationI2C_GPIOConfigTable, sizeof(SimulationI2C_GPIOConfigTable)/sizeof(GPIO_Config_t));

	//Timer3
	GPIO_ConfigInit(&Timer3_GPIOConfigTable, sizeof(Timer3_GPIOConfigTable)/sizeof(GPIO_Config_t));
	
	/*
	uint32_t i;
	uint32_t len = sizeof(GPIO_ConfigTable)/sizeof(GPIO_Config_t);

	for(i = 0;i < len;++i)
	{
		GPIO_Init(GPIO_ConfigTable[i].GPIOx, (GPIO_Config_t *)&GPIO_ConfigTable[i].GPIO_InitType);
		GPIO_PinAFConfig(GPIO_ConfigTable[i].GPIOx, GPIO_ConfigTable[i].GPIO_PinSource, GPIO_ConfigTable[i].GPIO_AF);
	}
	*/
}



