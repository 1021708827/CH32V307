/*
 * key.c
 *
 *  Created on: 2022年5月3日
 *      Author: admin
 */
#include "ch32v30x_rcc.h"
#include "key.h"
void GPIO_INIT(void){
    GPIO_InitTypeDef GPIO_InitTypdefStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

    GPIO_InitTypdefStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitTypdefStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitTypdefStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitTypdefStruct);

    GPIO_InitTypdefStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitTypdefStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitTypdefStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitTypdefStruct);
    GPIO_ResetBits(GPIOE,GPIO_Pin_11);    //输出0
}




