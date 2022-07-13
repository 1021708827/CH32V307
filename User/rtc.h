/*
 * rtc.h
 *
 *  Created on: 2022年5月4日
 *      Author: admin
 */

#ifndef USER_RTC_H_
#define USER_RTC_H_

//#define time1 10

void RTC_Init(u8 time);        //初始化RTC,返回0,失败;1,成功;

void SysTickEnableOrDisable(u8 status);
void RTC_NVIC_Config(void);

#endif /* USER_RTC_H_ */
