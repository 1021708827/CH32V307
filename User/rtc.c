#include "ch32v30x.h"
#include "rtc.h"
#include "ch32v30x.h"
#include "debug.h"
//实时时钟配置
//初始化RTC时钟,同时检测时钟是否工作正常
//返回0:正常
//其他:错误代码
void RTC_Init(u8 time)
{
    //检查是不是第一次配置时钟
    u8 temp=0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);    //使能PWR和BKP外设时钟
    PWR_BackupAccessCmd(ENABLE);    //使能后备寄存器访问

    BKP_DeInit();   //复位备份区域
    RCC_LSEConfig(RCC_LSE_ON);  //设置外部低速晶振(LSE),使用外设低速晶振
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) //检查指定的RCC标志位设置与否,等待低速晶振就绪
    {
        temp++;
        Delay_Ms(10);
    }
//    if(temp>=250)return 1;//初始化时钟失败,晶振有问题

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);     //设置RTC时钟(RTCCLK),选择LSE作为RTC时钟
    RCC_RTCCLKCmd(ENABLE);  //使能RTC时钟
    RTC_WaitForSynchro();       //等待RTC寄存器同步
    RTC_WaitForLastTask();  //等待最近一次对RTC寄存器的写操作完成

    RTC_ITConfig(RTC_IT_ALR, ENABLE);       //使能RTC时钟中断

    RTC_WaitForLastTask();  //等待最近一次对RTC寄存器的写操作完成
    RTC_EnterConfigMode();/// 允许配置

    RTC_SetPrescaler(32767); //设置RTC预分频的值
    RTC_SetCounter(0);       // 这里的设置，每设置一次就要等待写操作完成 这是个大坑啊，坑了我将近一天的时间
    RTC_WaitForLastTask();   //等待最近一次对RTC寄存器的写操作完成
    RTC_SetAlarm(time-1);         // 设置闹钟的时间要加1哦,也就是说现在是3S
    RTC_WaitForLastTask();  //等待最近一次对RTC寄存器的写操作完成
    RTC_ExitConfigMode();   //退出配置模式
    RTC_NVIC_Config();      //RCT中断分组设置

//    return 0; //ok
}

void RTC_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //EXTI17配置
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;         // RTC闹钟为外部中断17
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 时钟中断配置
    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;     //RTC全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //先占优先级1位,从优先级3位
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //先占优先级0位,从优先级4位
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能该通道中断
    NVIC_Init(&NVIC_InitStructure);     //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

/*
    函数：滴答定时器开关
    输入参数：status 滴答定时器工作状态 EABLE打开  DISABLE关闭
    输出参数：无
*/
void SysTickEnableOrDisable(u8 status)
{
    SysTick->CTLR = status;    // 关闭滴答定时器
    SysTick->CNT = 0x00;     // 清空val，清空定时器
}


