/*
 * rtc.h
 *
 *  Created on: 2022��5��4��
 *      Author: admin
 */

#ifndef USER_RTC_H_
#define USER_RTC_H_

//#define time1 10

void RTC_Init(u8 time);        //��ʼ��RTC,����0,ʧ��;1,�ɹ�;

void SysTickEnableOrDisable(u8 status);
void RTC_NVIC_Config(void);

#endif /* USER_RTC_H_ */
