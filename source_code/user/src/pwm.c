#include "pwm.h"



void pwm_init()
{
    GPIO_InitType GPIO_InitStructure;
    TMR_TimerBaseInitType TimerBaseInitStruct;
    TMR_OCInitType TMR_OCInitStructure;

    RCC_AHBPeriphClockCmd(PWM_GPIO_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(PWM_TMR_RCC, ENABLE);
    

    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;

    GPIO_InitStructure.GPIO_Pins =  PWM1_TMR_GPIO_PIN;
    GPIO_Init(PWM1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins =  PWM2_TMR_GPIO_PIN;
    GPIO_Init(PWM2_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins =  PWM3_TMR_GPIO_PIN;
    GPIO_Init(PWM3_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins =  PWM4_TMR_GPIO_PIN;
    GPIO_Init(PWM4_GPIO, &GPIO_InitStructure);


    GPIO_PinAFConfig(PWM1_GPIO, PWM1_GPIO_PinsSource, PWM1_GPIO_AF);
    GPIO_PinAFConfig(PWM2_GPIO, PWM2_GPIO_PinsSource, PWM2_GPIO_AF);
    GPIO_PinAFConfig(PWM3_GPIO, PWM3_GPIO_PinsSource, PWM3_GPIO_AF);
    GPIO_PinAFConfig(PWM4_GPIO, PWM4_GPIO_PinsSource, PWM4_GPIO_AF);
 
    TimerBaseInitStruct.TMR_DIV = 120000000/(PWM_DUTY * PWM_HZ) - 1;
    TimerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    TimerBaseInitStruct.TMR_Period = PWM_DUTY-1;
	TimerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
    TMR_TimeBaseInit(PWM_TMR, &TimerBaseInitStruct);

    TMR_OCStructInit(&TMR_OCInitStructure);
	TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
    TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
    TMR_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
    TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
    TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_Low;
    TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Set;
    TMR_OCInitStructure.TMR_OCNIdleState = TMR_OCIdleState_Reset;

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC1Init(PWM_TMR, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC2Init(PWM_TMR, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC3Init(PWM_TMR, &TMR_OCInitStructure);

    TMR_OCInitStructure.TMR_Pulse = 4000;
    TMR_OC4Init(PWM_TMR, &TMR_OCInitStructure);

    TMR_Cmd(PWM_TMR, ENABLE);

    //TMR_CtrlPWMOutputs(PWM_TMR, ENABLE);

}


void pwm_out(PWM_Channel_Type channel, uint32_t duty)
{
    switch(channel)
    {
        case PWM_Channel_1:
        {
            TMR_SetCompare1(PWM_TMR, duty);
            break;
        }
        case PWM_Channel_2:
        {
            TMR_SetCompare2(PWM_TMR, duty);
            break;
        }
        case PWM_Channel_3:
        {
            TMR_SetCompare3(PWM_TMR, duty);
            break;
        }
        case PWM_Channel_4:
        {
            TMR_SetCompare4(PWM_TMR, duty);
            break;
        }
        default:
            break;
    }  
}

