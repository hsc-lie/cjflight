#ifndef _MY_SYSTICK_H
#define _MY_SYSTICK_H

#include "at32f4xx.h"
#include "core_cm4.h"



void systick_delay_us(unsigned int us);
void systick_delay_ms(unsigned int ms);

#endif
