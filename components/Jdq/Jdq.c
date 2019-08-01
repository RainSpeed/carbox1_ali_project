#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "Jdq.h"


#define GPIO_Jdq1      (GPIO_NUM_27)



void Jdq_Init(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1<<GPIO_Jdq1);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);  

    Jdq_Status=0;


}


void Jdq_On(void)
{
    gpio_set_level(GPIO_Jdq1, 1);
    Jdq_Status=1;
}

void Jdq_Off(void)
{
    gpio_set_level(GPIO_Jdq1, 0);
    Jdq_Status=0;
}



