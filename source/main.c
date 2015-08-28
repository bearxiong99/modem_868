#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"

#include "sx1276_cfg.h"
#include "delay.h"
#include "led.h"

static void hardware()
{
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC |
                           RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_SYSCFG, ENABLE);
}

int main()
{
    hardware();
    
    led_init();
    delay_init();
    sx1276_cfg_init();
    
    xTaskCreate(task_led, "led", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    vTaskStartScheduler();
}

void assert_failed(const char *f, int l)
{
    (void)f;
    (void)l;

    while(1) 
    {
    }
}

void vApplicationMallocFailedHook()
{
    while(1)
    {
    }
}

void vApplicationStackOverflowHook()
{
    while(1)
    {
    }
}
