#include "time.h"

#include "at32f4xx_tim.h"
#include "misc.h"

#include "led.h"



void tim_init_ms(uint16_t ms)
{

    TMR_TimerBaseInitType TimerBaseInitStruct;
    NVIC_InitType NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);

    TimerBaseInitStruct.TMR_DIV = 120-1;
    TimerBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_Up;
    TimerBaseInitStruct.TMR_Period = 1000*ms-1;
    TimerBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;

    TMR_TimeBaseInit(TMR6, &TimerBaseInitStruct);
    //TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);
	TMR_Cmd(TMR6, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
    
}

void time_count_start_us()
{
	TMR6->CNT = 0;
}

uint32_t time_count_end_us()
{
    return TMR6->CNT;
}



void tim_500ms_task()
{
    static uint32_t tim_500ms_count = 0;
    uint32_t tim_500ms_count_max = 60000000 / ((TMR6->AR + 1) * (TMR6->DIV + 1));

    tim_500ms_count++;
    if(tim_500ms_count == tim_500ms_count_max)
    {
        tim_500ms_count = 0;


        led_water_lamp();   
    }
}
