#include "gpio_config.h"
#include "at32f4xx.h"



typedef struct
{
	GPIO_Type * GPIOx;
	uint16_t GPIO_PinSource; 
	uint8_t GPIO_AF;
	GPIO_InitType GPIO_InitType;
	
	
}GPIO_Config_t;



const GPIO_Config_t GPIO_ConfigTable[] = 
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

	//软件I2C SDA
	{
		.GPIOx = GPIOA,
		.GPIO_InitType = 
		{
			.GPIO_Pins	= GPIO_Pins_9,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_OutType = GPIO_OutType_OD,
			.GPIO_Pull = GPIO_Pull_NOPULL,
			.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
	
	//软件I2C SCL
	{
		.GPIOx = GPIOA,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_10,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_OD,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},

			
	{
		.GPIOx = GPIOA,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_11,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
	
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

	
			
	{
		.GPIOx = GPIOC,
		.GPIO_InitType = 
		{
			.GPIO_Pins  = GPIO_Pins_14,
	    	.GPIO_Mode = GPIO_Mode_OUT,
	    	.GPIO_OutType = GPIO_OutType_PP,
	    	.GPIO_Pull = GPIO_Pull_NOPULL,
	    	.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz,
		},
	},
};




void GPIO_ConfigInitAll(void)
{
	uint32_t i;
	uint32_t len = sizeof(GPIO_ConfigTable)/sizeof(GPIO_Config_t);

	for(i = 0;i < len;++i)
	{
		GPIO_Init(GPIO_ConfigTable[i].GPIOx, (GPIO_Config_t *)&GPIO_ConfigTable[i].GPIO_InitType);
		GPIO_PinAFConfig(GPIO_ConfigTable[i].GPIOx, GPIO_ConfigTable[i].GPIO_PinSource, GPIO_ConfigTable[i].GPIO_AF);
	}
	
}



