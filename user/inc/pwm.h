#ifndef _PWM_H_
#define _PWM_H_


#include "at32f4xx.h"
#include "at32f4xx_gpio_ex.h"
#include "at32f4xx_tim.h"


#define PWM_HZ        400  
#define PWM_DUTY      10000     


#define PWM_TMR_RCC           RCC_APB1PERIPH_TMR3 
#define PWM_GPIO_RCC          RCC_AHBPERIPH_GPIOB      


#define PWM_TMR           TMR3

#define PWM1_GPIO         GPIOB
#define PWM2_GPIO         GPIOB
#define PWM3_GPIO         GPIOB
#define PWM4_GPIO         GPIOB


#define PWM1_TMR_GPIO_PIN    GPIO_Pins_4
#define PWM2_TMR_GPIO_PIN    GPIO_Pins_5
#define PWM3_TMR_GPIO_PIN    GPIO_Pins_0
#define PWM4_TMR_GPIO_PIN    GPIO_Pins_1

#define PWM1_GPIO_PinsSource     GPIO_PinsSource4
#define PWM2_GPIO_PinsSource     GPIO_PinsSource5
#define PWM3_GPIO_PinsSource     GPIO_PinsSource0
#define PWM4_GPIO_PinsSource     GPIO_PinsSource1

#define PWM1_GPIO_AF     GPIO_AF_1
#define PWM2_GPIO_AF     GPIO_AF_1
#define PWM3_GPIO_AF     GPIO_AF_1
#define PWM4_GPIO_AF     GPIO_AF_1


typedef enum
{
    PWM_Channel_1,
    PWM_Channel_2,
    PWM_Channel_3,
    PWM_Channel_4,
    PWM_Channel_MAX,
}PWM_Channel_Type;



void pwm_init(void);
void pwm_out(PWM_Channel_Type channel, uint32_t duty);



#endif /*_PWM_H_*/
