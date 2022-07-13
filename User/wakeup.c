/*
 * wakeup.c
 *
 *  Created on: 2022年5月3日
 *      Author: admin
 */
#include "ch32v30x_rcc.h"

void wakeup_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
}


void wakeup_enter(void)
{
    /*清除WU状态位*/
    PWR_ClearFlag (PWR_FLAG_WU);

    /* 使能WKUP引脚的唤醒功能 ，使能PA0*/
    PWR_WakeUpPinCmd (ENABLE);

    /* 进入待机模式 */
    PWR_EnterSTANDBYMode();
}
