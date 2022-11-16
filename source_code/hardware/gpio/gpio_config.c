#include "gpio_config.h"
#include "at32f4xx.h"



typedef struct
{
	GPIO_Type * GPIOx;
	GPIO_InitType GPIO_InitType;
	
}GPIO_Config_t;



GPIO_Config_t GPIO_ConfigTable[] = 
{
	
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
		GPIO_Init(GPIO_ConfigTable[i].GPIOx, &GPIO_ConfigTable[i].GPIO_InitType);
	}
	
}



