#ifndef _TIM_INPUT_CAPTURE_H_
#define _TIM_INPUT_CAPTURE_H_

#include "at32f4xx_gpio_ex.h"
#include "at32f4xx_tim.h"

#include "at32f4xx.h"

#define TMR_INPUT_CAPTURE_RCC       RCC_APB2PERIPH_TMR15
#define GPIO_INPUT_CAPTURE_RCC      RCC_AHBPERIPH_GPIOA

#define TMR_INPUT_CAPTURE_GPIO      GPIOA
#define TMR_INPUT_CAPTURE_GPIO_PIN  GPIO_Pins_0


#define TMR_INPUT_CAPTURE           TMR15          
#define TMR_INPUT_CAPTURE_CC        TMR_INT_CC1



typedef enum
{
    PPM_Channel1,
    PPM_Channel2,
    Throttle,               //油门
    PPM_Channel4,
    PPM_Channel5,
    PPM_Channel6,
    PPM_Channel_MAX
}PPM_Channel_Type;

void tim_input_capture_init(void);
void tim_get_ppm(void);
uint32_t get_ppm_channel_value(PPM_Channel_Type channel_num);

#endif /*_TIM_INPUT_CAPTURE_H_*/

