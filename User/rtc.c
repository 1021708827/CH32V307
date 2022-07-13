#include "ch32v30x.h"
#include "rtc.h"
#include "ch32v30x.h"
#include "debug.h"
//ʵʱʱ������
//��ʼ��RTCʱ��,ͬʱ���ʱ���Ƿ�������
//����0:����
//����:�������
void RTC_Init(u8 time)
{
    //����ǲ��ǵ�һ������ʱ��
    u8 temp=0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);    //ʹ��PWR��BKP����ʱ��
    PWR_BackupAccessCmd(ENABLE);    //ʹ�ܺ󱸼Ĵ�������

    BKP_DeInit();   //��λ��������
    RCC_LSEConfig(RCC_LSE_ON);  //�����ⲿ���پ���(LSE),ʹ��������پ���
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) //���ָ����RCC��־λ�������,�ȴ����پ������
    {
        temp++;
        Delay_Ms(10);
    }
//    if(temp>=250)return 1;//��ʼ��ʱ��ʧ��,����������

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);     //����RTCʱ��(RTCCLK),ѡ��LSE��ΪRTCʱ��
    RCC_RTCCLKCmd(ENABLE);  //ʹ��RTCʱ��
    RTC_WaitForSynchro();       //�ȴ�RTC�Ĵ���ͬ��
    RTC_WaitForLastTask();  //�ȴ����һ�ζ�RTC�Ĵ�����д�������

    RTC_ITConfig(RTC_IT_ALR, ENABLE);       //ʹ��RTCʱ���ж�

    RTC_WaitForLastTask();  //�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_EnterConfigMode();/// ��������

    RTC_SetPrescaler(32767); //����RTCԤ��Ƶ��ֵ
    RTC_SetCounter(0);       // ��������ã�ÿ����һ�ξ�Ҫ�ȴ�д������� ���Ǹ���Ӱ��������ҽ���һ���ʱ��
    RTC_WaitForLastTask();   //�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_SetAlarm(time-1);         // �������ӵ�ʱ��Ҫ��1Ŷ,Ҳ����˵������3S
    RTC_WaitForLastTask();  //�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_ExitConfigMode();   //�˳�����ģʽ
    RTC_NVIC_Config();      //RCT�жϷ�������

//    return 0; //ok
}

void RTC_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //EXTI17����
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;         // RTC����Ϊ�ⲿ�ж�17
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // ʱ���ж�����
    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;     //RTCȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ�1λ,�����ȼ�3λ
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //��ռ���ȼ�0λ,�����ȼ�4λ
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //ʹ�ܸ�ͨ���ж�
    NVIC_Init(&NVIC_InitStructure);     //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

/*
    �������δ�ʱ������
    ���������status �δ�ʱ������״̬ EABLE��  DISABLE�ر�
    �����������
*/
void SysTickEnableOrDisable(u8 status)
{
    SysTick->CTLR = status;    // �رյδ�ʱ��
    SysTick->CNT = 0x00;     // ���val����ն�ʱ��
}


