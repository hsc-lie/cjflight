#include "bsp_timer.h"

#include "at32f4xx_tim.h"


#define TIMER3_PWM_HZ          (400)
#define TIMER3_PWM_DUTY_MAX    (10000)



void BSPTimer3Init()
{
	TMR_TimerBaseInitType timerBaseInitStruct;
    TMR_OCInitType ocInitStructure;
	
	timerBaseInitStruct.TMR_DIV = 120000000/(TIMER3_PWM_DUTY_MAX * TIMER3_PWM_HZ) - 1;
    timerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    timerBaseInitStruct.TMR_Period = TIMER3_PWM_DUTY_MAX - 1;
	timerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
    TMR_TimeBaseInit(TMR3, &timerBaseInitStruct);

    TMR_OCStructInit(&ocInitStructure);
	ocInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
    ocInitStructure.TMR_OutputState = TMR_OutputState_Enable;
    ocInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
    ocInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
    ocInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_Low;
    ocInitStructure.TMR_OCIdleState = TMR_OCIdleState_Set;
    ocInitStructure.TMR_OCNIdleState = TMR_OCIdleState_Reset;

    ocInitStructure.TMR_Pulse = 4000;
    TMR_OC1Init(TMR3, &ocInitStructure);

    ocInitStructure.TMR_Pulse = 4000;
    TMR_OC2Init(TMR3, &ocInitStructure);

    ocInitStructure.TMR_Pulse = 4000;
    TMR_OC3Init(TMR3, &ocInitStructure);

    ocInitStructure.TMR_Pulse = 4000;
    TMR_OC4Init(TMR3, &ocInitStructure);

    TMR_Cmd(TMR3, ENABLE);
}

void BSPTimer6Init()
{
	//uint32_t ms = 500;
	
	TMR_TimerBaseInitType timerBaseInitStruct;
    //NVIC_InitType NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);

    timerBaseInitStruct.TMR_DIV = 120-1;
    timerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    //timerBaseInitStruct.TMR_Period = 1000*ms-1;
    timerBaseInitStruct.TMR_Period = 0xFFFFFFFF;
    timerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;

    TMR_TimeBaseInit(TMR6, &timerBaseInitStruct);
    //TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);
	TMR_Cmd(TMR6, ENABLE);

    /*
    NVIC_InitStruct.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
    */
}

uint32_t TimerGetCount(TMR_Type* TMRx)
{
    return TMRx->CNT;
}
