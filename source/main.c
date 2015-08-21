#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"

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
}

int main()
{
    hardware();
    
    led_init();
    
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