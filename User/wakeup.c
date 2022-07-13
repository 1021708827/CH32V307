/*
 * wakeup.c
 *
 *  Created on: 2022��5��3��
 *      Author: admin
 */
#include "ch32v30x_rcc.h"

void wakeup_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
}


void wakeup_enter(void)
{
    /*���WU״̬λ*/
    PWR_ClearFlag (PWR_FLAG_WU);

    /* ʹ��WKUP���ŵĻ��ѹ��� ��ʹ��PA0*/
    PWR_WakeUpPinCmd (ENABLE);

    /* �������ģʽ */
    PWR_EnterSTANDBYMode();
}
