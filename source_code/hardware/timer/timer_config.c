#include "timer_config.h"

#include "at32f4xx_tim.h"


#define TIMER3_PWM_HZ          (400)
#define TIMER3_PWM_DUTY_MAX    (10000)



static void Timer3_ConfigInit()
{
	TMR_TimerBaseInitType TimerBaseInitStruct;
    TMR_OCInitType TMR_OCInitStructure;
	
	TimerBaseInitStruct.TMR_DIV = 120000000/(TIMER3_PWM_DUTY_MAX * TIMER3_PWM_HZ) - 1;
    TimerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    TimerBaseInitStruct.TMR_Period = TIMER3_PWM_DUTY_MAX - 1;
	TimerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
    TMR_TimeBaseInit(TMR3, &TimerBaseInitStruct);

    TMR_OCStructInit(&TMR_OCInitStructure);
	TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
    TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
    TMR_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
    TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
    TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_Low;
    TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Set;
    TMR_OCInitStructure.TMR_OCNIdleState = TMR_OCIdleState_Reset;

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC1Init(TMR3, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC2Init(TMR3, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC3Init(TMR3, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC4Init(TMR3, &TMR_OCInitStructure);

    TMR_Cmd(TMR3, ENABLE);
}




void Timer_ConfigInitAll()
{
	Timer3_ConfigInit();
}

