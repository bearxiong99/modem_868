#include "delay.h"

#include "stm32f4xx.h"

#define DELAY_TIMER        TIM1
#define DELAY_FREQ         168000000

static uint32_t g_tick_ms = 0;

static void init_timebase()
{
    TIM_TimeBaseInitTypeDef timebase;
    
    TIM_TimeBaseStructInit(&timebase);

    timebase.TIM_Prescaler = 0;
    timebase.TIM_CounterMode = TIM_CounterMode_Up;
    timebase.TIM_Period = 0xFFFF;
    timebase.TIM_ClockDivision = TIM_CKD_DIV1;
    timebase.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(DELAY_TIMER, &timebase);
    TIM_Cmd(DELAY_TIMER, ENABLE);
}

static void delay_ticks(uint16_t ticks)
{
	uint16_t start;
	volatile uint16_t now;
	
	if(ticks)
	{
		start = TIM_GetCounter(DELAY_TIMER);
		
		do
		{
			now = TIM_GetCounter(DELAY_TIMER);
		} while((uint16_t)(now - start) < ticks);
	}
}

void delay_init()
{
	uint32_t freq = DELAY_FREQ;
	
	init_timebase();
	
	g_tick_ms = (freq + 500) / 1000;
}

void delay_ms(uint32_t delay)
{
	uint64_t total = 0;
	
	total = (uint64_t)(g_tick_ms * delay);
	
	while(total > 60000)
	{
		delay_ticks(60000);
		total -= 60000;
	}
	
	delay_ticks((uint16_t)total);
}