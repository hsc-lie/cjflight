#ifndef _TIME_H_
#define _TIME_H_


#include "at32f4xx.h"


void tim_init_ms(uint16_t ms);
void tim_500ms_task(void);

void time_count_start_us(void);
uint32_t time_count_end_us(void);


#endif /*_TIME_H_*/
