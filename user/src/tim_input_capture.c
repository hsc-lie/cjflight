#include "tim_input_capture.h"


#include "misc.h"


uint32_t ppm_channel_value[PPM_Channel_MAX];

#if 0
void tim_input_capture_init()
{
    GPIO_InitType GPIO_InitStructure;
    TMR_TimerBaseInitType TimerBaseInitStruct;
    TMR_ICInitType TMR_ICInitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(TMR_INPUT_CAPTURE_RCC, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(GPIO_INPUT_CAPTURE_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pins =  TMR_INPUT_CAPTURE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;

    GPIO_Init(TMR_INPUT_CAPTURE_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(TMR_INPUT_CAPTURE_GPIO, GPIO_PinsSource2, GPIO_AF_0);

    TimerBaseInitStruct.TMR_DIV = 2400-1;
    TimerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    TimerBaseInitStruct.TMR_Period = 0xfff;
	TimerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
    TMR_TimeBaseInit(TMR_INPUT_CAPTURE, &TimerBaseInitStruct);

    TMR_ICInitStructure.TMR_Channel = TMR_Channel_1;
    TMR_ICInitStructure.TMR_ICPolarity = TMR_ICPolarity_Rising;      //TMR_ICPolarity_Rising  TMR_ICPolarity_BothEdge
    TMR_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI;    //TMR_ICSelection_DirectTI  TMR_ICSelection_IndirectTI
    TMR_ICInitStructure.TMR_ICDIV = TMR_ICDIV_DIV1;
    TMR_ICInitStructure.TMR_ICFilter = 0x0;
    TMR_ICInit(TMR_INPUT_CAPTURE, &TMR_ICInitStructure);
    //TMR_PWMIConfig(TMR15, &TMR_ICInitStructure);
	
    TMR_CCxCmd(TMR_INPUT_CAPTURE, TMR_Channel_1, TMR_CCx_Enable);
   
	//TMR_SelectInputTrigger(TMR_INPUT_CAPTURE, TMR_TRGSEL_TI1FP1);
	//TMR_SelectSlaveMode(TMR_INPUT_CAPTURE, TMR_SlaveMode_Reset);
	//TMR_SelectMasterSlaveMode(TMR_INPUT_CAPTURE, TMR_MasterSlaveMode_Enable);
   
   
    TMR_INTConfig(TMR_INPUT_CAPTURE, TMR_INPUT_CAPTURE_CC, ENABLE);
	//TMR_INTConfig(TMR_INPUT_CAPTURE, TMR_INT_Overflow, ENABLE);

    TMR_Cmd(TMR_INPUT_CAPTURE, ENABLE);

    
    //TMR15->CCM1 &= ~(3 << 0);
	//TMR15->CCM1 |= (1 << 0);	
    //TMR15->CCE |= 1 << 0;

 
    NVIC_InitStructure.NVIC_IRQChannel = TMR15_GLOBAL_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

#else
void tim_input_capture_init()
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStruct;
    TMR_TimerBaseInitType TimerBaseInitStruct;
    NVIC_InitType NVIC_InitStructure;
    
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_SYSCFGCOMP, ENABLE); 
    RCC_APB2PeriphClockCmd(TMR_INPUT_CAPTURE_RCC, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(GPIO_INPUT_CAPTURE_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_OD;

    GPIO_InitStructure.GPIO_Pins =  TMR_INPUT_CAPTURE_GPIO_PIN;
    GPIO_Init(TMR_INPUT_CAPTURE_GPIO, &GPIO_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineEnable = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    EXTI_GenerateSWInt(EXTI_Line0);

    TimerBaseInitStruct.TMR_DIV = 2400-1;
    TimerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    TimerBaseInitStruct.TMR_Period = 0xfff;
	TimerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
    TMR_TimeBaseInit(TMR_INPUT_CAPTURE, &TimerBaseInitStruct);


    TMR_Cmd(TMR_INPUT_CAPTURE, ENABLE);

    
    //TMR15->CCM1 &= ~(3 << 0);
	//TMR15->CCM1 |= (1 << 0);	
    //TMR15->CCE |= 1 << 0;

 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}


#endif



void tim_get_ppm()
{
    static uint8_t now_ppm_channel = PPM_Channel1;
    static uint32_t now_tim_count_value = 0;
    static uint32_t last_tim_count_value = 0;
    static uint32_t now_ppm_channel_value = 0;

    //now_tim_count_value = TMR_GetCapture2(TMR_INPUT_CAPTURE);
    now_tim_count_value = TMR_INPUT_CAPTURE->CNT;
    if(now_tim_count_value >= last_tim_count_value)
    {
        now_ppm_channel_value = now_tim_count_value - last_tim_count_value;
    }
    else
    {
        now_ppm_channel_value = ((0xFFFF - last_tim_count_value) + now_tim_count_value); 
    }
    last_tim_count_value = now_tim_count_value;

    if(now_ppm_channel_value > 150)
    {
        now_ppm_channel = PPM_Channel1;
    }
    else
    {
        ppm_channel_value[now_ppm_channel] = now_ppm_channel_value;
        now_ppm_channel++;
        if(now_ppm_channel >= PPM_Channel_MAX)
        {
            now_ppm_channel = PPM_Channel1;
        }
    }
	
  
}


uint32_t get_ppm_channel_value(PPM_Channel_Type channel_num)
{
    return ppm_channel_value[channel_num];
}


