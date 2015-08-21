#include "led.h"

#include "stm32f4xx.h"

#include "task.h"

#define LED_GPIO_LINE        GPIOB
#define LED_GPIO_PIN         GPIO_Pin_9

void led_init()
{
    GPIO_InitTypeDef  gpio;

    GPIO_StructInit(&gpio);

    gpio.GPIO_Pin = LED_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(LED_GPIO_LINE, &gpio);
}

void led_on()
{
    GPIO_SetBits(LED_GPIO_LINE, LED_GPIO_PIN);
}

void led_off()
{
    GPIO_ResetBits(LED_GPIO_LINE, LED_GPIO_PIN);
}

portTASK_FUNCTION(task_led, p)
{
    (void)p;
    
    while(1)
    {
        led_on();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        led_off();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}