#ifndef LED_H
#define LED_H

#include "FreeRTOS.h"

void led_init();

void led_on();

void led_off();

portTASK_FUNCTION_PROTO(task_led, p);

#endif