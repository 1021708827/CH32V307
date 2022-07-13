#include "ch32v30x_conf.h"
#include "debug.h"
#include "AHT_10.h"
#include "lcd.h"
#include "AP3216C.h"
#include "MPU6050.h"
#include "graph.h"
#include "MahonyAHRS.h"
#include <math.h>
#include  "wakeup.h"
#include  "key.h"
#include "rtc.h"
#include "I2S.h"
#include "es8388.h"
#include "gps.h"
#include <stdio.h>
#include "F:\Mounriver_studio\MounRiver\MounRiver_Studio\toolchain\RISC-V Embedded GCC\riscv-none-embed\include\string.h"
#define USART2_RXBUFF_SIZE   80
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
u8 SendFlag=0;
GPS_INFO   GPS;  //GPS��Ϣ�ṹ��
char Usart2RecBuf[USART2_RXBUFF_SIZE];//����2�������ݻ���
unsigned int Rx2Counter = 0;   //����2�յ����ݱ�־λ
unsigned char rev_start = 0;     //���տ�ʼ��־
unsigned char rev_stop  = 0;     //����ֹͣ��־
unsigned char gps_flag = 0;
unsigned char num = 0;
float jingdu =0;
float weidu = 0;
u8 GPS_rx_flag = 0;
u8 GpsInitOkFlag=0;             //GPS��λ���ճɹ���־
u8 go_on = 0;
imu_data_t acc;
imu_data_t gyro;
imu_data_t offset_gyro;
float pitch,roll,yaw;
u8 set_time;
float alpha = 0.4;

static float acc_x = 0;
static float acc_y = 0;
static float acc_z = 0;
float temperature, humidity;
     u16 ir, als, ps;
     char error_num = 0;
     u16 count;

     /* Global define */
     #define RXBUF_SIZE 4096 // DMA buffer size
     #define size(a)   (sizeof(a) / sizeof(*(a)))
     /* Global Variable */
     u8 TxBuffer[] = " ";
     u8 RxBuffer[RXBUF_SIZE]={0};

     void USARTx_CFG(void)
     {
         GPIO_InitTypeDef  GPIO_InitStructure;
         USART_InitTypeDef USART_InitStructure;
         //����ʱ��
         RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

         /* USART6 TX-->C0  RX-->C1 */
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
         GPIO_Init(GPIOC, &GPIO_InitStructure);
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX����������
         GPIO_Init(GPIOC, &GPIO_InitStructure);

         USART_InitStructure.USART_BaudRate = 115200;                    // ������
         USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // ����λ 8
         USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ֹͣλ 1
         USART_InitStructure.USART_Parity = USART_Parity_No;             // ��У��
         USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������
         USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //ʹ�� RX �� TX

         USART_Init(UART6, &USART_InitStructure);
         DMA_Cmd(DMA2_Channel7, ENABLE);                                  //�������� DMA
         USART_Cmd(UART6, ENABLE);                                        //����UART
     }
     void DMA_INIT(void)
     {
         DMA_InitTypeDef DMA_InitStructure;
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

         // TX DMA ��ʼ��
         DMA_DeInit(DMA2_Channel6);
         DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART6->DATAR);        // DMA �����ַ����ָ���Ӧ������
         DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;                   // DMA �ڴ��ַ��ָ���ͻ��������׵�ַ
         DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // ���� : ���� ��Ϊ �յ㣬�� �ڴ� ->  ����
         DMA_InitStructure.DMA_BufferSize = 0;                                   // ��������С,��ҪDMA���͵����ݳ���,Ŀǰû�����ݿɷ�
         DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ����������
         DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ��ַ����������
         DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // ��������λ��8λ(Byte)
         DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ�����λ��8λ(Byte)
         DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ��ͨģʽ�������������ѭ������
         DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ����
         DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // M2P,����M2M
         DMA_Init(DMA2_Channel6, &DMA_InitStructure);

         // RX DMA ��ʼ�������λ������Զ�����
         DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // ���ջ�����
         DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // ���� : ���� ��Ϊ Դ���� �ڴ� <- ����
         DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;                          // ����������Ϊ RXBUF_SIZE
         DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // ѭ��ģʽ�����ɻ��λ�����
         DMA_Init(DMA2_Channel7, &DMA_InitStructure);
     }

     FlagStatus uartWriteWiFi(char * data , uint16_t num)
     {
         //���ϴη���δ��ɣ�����
         if(DMA_GetCurrDataCounter(DMA2_Channel6) != 0){
             return RESET;
         }

         DMA_ClearFlag(DMA2_FLAG_TC8);
         DMA_Cmd(DMA2_Channel6, DISABLE );           // �� DMA �����
         DMA2_Channel6->MADDR = (uint32_t)data;      // ���ͻ�����Ϊ data
         DMA_SetCurrDataCounter(DMA2_Channel6,num);  // ���û���������
         DMA_Cmd(DMA2_Channel6, ENABLE);             // �� DMA
         return SET;
     }
     FlagStatus uartWriteWiFiStr(char * str)
     {
         uint16_t num = 0;
         while(str[num])num++;           // �����ַ�������
         return uartWriteWiFi(str,num);
     }

     uint16_t rxBufferReadPos = 0;       //���ջ�������ָ��
     uint32_t uartReadWiFi(char * buffer , uint16_t num)
     {
         uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7); //���� DMA ����β��λ��
         uint16_t i = 0;
         if (rxBufferReadPos == rxBufferEnd){
             // �����ݣ�����
             return 0;
         }

         while (rxBufferReadPos!=rxBufferEnd && i < num){
             buffer[i] = RxBuffer[rxBufferReadPos];
             i++;
             rxBufferReadPos++;
             if(rxBufferReadPos >= RXBUF_SIZE){
                 // ����������������
                 rxBufferReadPos = 0;
             }
         }
         return i;
     }
     char uartReadByteWiFi()
     {
         char ret;
         uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7);
         if (rxBufferReadPos == rxBufferEnd){
             // �����ݣ�����
             return 0;
         }
         ret = RxBuffer[rxBufferReadPos];
         rxBufferReadPos++;
         if(rxBufferReadPos >= RXBUF_SIZE){
             // ����������������
             rxBufferReadPos = 0;
         }
         return ret;
     }


     uint16_t uartAvailableWiFi()
     {
         uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7);//���� DMA ����β��λ��
         // ����ɶ��ֽ�
         if (rxBufferReadPos <= rxBufferEnd){
             return rxBufferEnd - rxBufferReadPos;
         }else{
             return rxBufferEnd +RXBUF_SIZE -rxBufferReadPos;
         }
     }
     void USART2_Init(u32 baud)
      {
           USART_InitTypeDef USART_InitStructure;
           NVIC_InitTypeDef NVIC_InitStructure;
           GPIO_InitTypeDef GPIO_InitStructure;

           RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
           RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

           // Configure USART3 Rx (PB.11) as input floating
           GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
           GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
           GPIO_Init(GPIOA, &GPIO_InitStructure);

           // Configure USART3 Tx (PB.10) as alternate function push-pull
           GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
           GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
           GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
           GPIO_Init(GPIOA, &GPIO_InitStructure);

           USART_InitStructure.USART_BaudRate = baud;
           USART_InitStructure.USART_WordLength = USART_WordLength_8b;
           USART_InitStructure.USART_StopBits = USART_StopBits_1;
           USART_InitStructure.USART_Parity = USART_Parity_No;
           USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
           USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


           // Configure USART3
           USART_Init(USART2, &USART_InitStructure);
           // Enable USART3 Receive interrupts
           USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
           // Enable the USART3
           USART_Cmd(USART2, ENABLE);

           //Configure the NVIC Preemption Priority Bits
           NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

           // Enable the USART3 Interrupt
           NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
           NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
           NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
           NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
           NVIC_Init(&NVIC_InitStructure);
       }



void gyroOffsetInit(imu_data_t *gyro_p)
{
    imu_data_t gyro_data;

    for (uint16_t i = 0; i < 100; ++i)
    {
        MPU_Get_Gyroscope(&gyro_data.x_data,&gyro_data.y_data,&gyro_data.z_data);    // ��ȡ�����ǽ��ٶ�
        gyro_p->x_data += gyro_data.x_data;
        gyro_p->y_data += gyro_data.y_data;
        gyro_p->z_data += gyro_data.z_data;
        Delay_Ms(5);    // ��� 1Khz
    }
    gyro_p->x_data /=100;
    gyro_p->y_data /=100;
    gyro_p->z_data /=100;
}




void MPU_GET()
{
        MPU_Get_Accelerometer(&acc.x_data,&acc.y_data,&acc.z_data);   //��ü��ٶ�ԭʼ����
        MPU_Get_Gyroscope(&gyro.x_data,&gyro.y_data,&gyro.z_data);    //���������ԭʼ����

        acc_x = ((float)acc.x_data ) * alpha + acc_x *(1-alpha);
        acc_y = ((float)acc.y_data ) * alpha + acc_y *(1-alpha);
        acc_z = ((float)acc.z_data ) * alpha + acc_z *(1-alpha);

        gyro.x_data -= offset_gyro.x_data;
        gyro.y_data -= offset_gyro.y_data;
        gyro.z_data -= offset_gyro.z_data;

        //                              ת��Ϊ����,������Ϊ16.4
        MahonyAHRSupdateIMU(gyro.x_data/16.4/57.3 *(-1.0)  ,gyro.y_data/16.4/57.3 *(-1.0)  ,gyro.z_data/16.4/57.3,(acc_x/4096)*(-1.0),(acc_y/4096)*(-1.0),acc_z/4096);


        pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
        roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
//        yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;   //yaw
 //     printf("pitch=%d,roll=%d,yaw=%d\r\n",(int)(pitch),(int)(roll),(int)(yaw));

}

__attribute__((interrupt("WCH-Interrupt-fast")))
void RTCAlarm_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//�����ж�
    {
        RTC_SetCounter(0);      // ���RTC�����������¿�ʼ����
        RTC_WaitForLastTask();
        RTC_SetAlarm(0);        // �������� �������ӵ�ʱ��Ҫ��1Ŷ,,Ҳ����˵������3S
        RTC_WaitForLastTask();
    }

    set_time-=1;
    RTC_ClearITPendingBit(RTC_IT_ALR);      //�������ж�
    EXTI_ClearITPendingBit(EXTI_Line17);     //�����¼������������һ��EXTI_17�ⲿ�жϣ��˱�־λҪ����������´�ֹͣģʽ����ʧ��
    RTC_ClearITPendingBit(RTC_IT_OW);       //�������ж�
    RTC_WaitForLastTask();
}

void key_test(void)
{
    if( ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_5 ) || ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_4 ))
    {
        Delay_Ms(20);
        if( ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_5 ) || ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_4 ))
        {

            lcd_set_color(BLACK,RED);
            lcd_show_string(70, 200, 16,"                 ");

            Show_Chinese(70, 200, (u8*)jing, 16, RED, BLACK);
            Show_Chinese(90, 200, (u8*)gao, 16, RED, BLACK);//����

            Show_Chinese(110, 200, (u8*)you, 16, RED, BLACK);
            Show_Chinese(130, 200, (u8*)ren, 16, RED, BLACK);
            Show_Chinese(150, 200, (u8*)shang, 16, RED, BLACK);
            Show_Chinese(170, 200, (u8*)ta1, 16, RED, BLACK);//��������

        }
        while( ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_4 ) || ! GPIO_ReadInputDataBit( GPIOE, GPIO_Pin_5 ));
    }
}

void init()
{
    set_time = 10;
    wakeup_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(9600);
    GPS_rx_flag = 0;
    USART2_Init(9600);

    lcd_init();
    GPIO_INIT();
    AHT10_Init();
    AP3216C_Init();
    while(MPU_Init());
    gyroOffsetInit(&offset_gyro);

    GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_SET);
    GPS_rx_flag = 1;

    DMA_INIT();
    USARTx_CFG();
    USART_DMACmd(UART6,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);
}

void xianshi_init()
{    float temperature, humidity;
     u16 ir, als, ps;
    Show_Chinese(40, 10,(u8*)tie,24, RED, 0xFFFF);
    Show_Chinese(70, 10,(u8*)ta,24, RED, 0xFFFF);
    Show_Chinese(100, 10,(u8*)jian,24, RED, 0xFFFF);
    Show_Chinese(130, 10,(u8*)ce,24, RED, 0xFFFF);
    Show_Chinese(160, 10,(u8*)xi,24, RED, 0xFFFF);
    Show_Chinese(190, 10,(u8*)tong,24, RED, 0xFFFF);  //�������ϵͳ

    Show_Chinese(45, 50, (u8*)wen, 16, ORANGE, BLACK);
    Show_Chinese(65, 50, (u8*)du, 16, ORANGE, BLACK);//�¶�:

    Show_Chinese(145, 50, (u8*)shi, 16, ORANGE, BLACK);
    Show_Chinese(165, 50, (u8*)du, 16, ORANGE, BLACK);//ʪ��:


    Show_Chinese(5, 80, (u8*)guang, 16, ORANGE, BLACK);
    Show_Chinese(25, 80, (u8*)zhao, 16, ORANGE, BLACK);
    Show_Chinese(45, 80, (u8*)qiang, 16, ORANGE, BLACK);
    Show_Chinese(65, 80, (u8*)du, 16, ORANGE, BLACK);//����ǿ��:

    Show_Chinese(145, 80, (u8*)ju, 16, ORANGE, BLACK);
    Show_Chinese(165, 80, (u8*)li, 16, ORANGE, BLACK);//����:

    Show_Chinese(25, 110, (u8*)fu, 16, ORANGE, BLACK);
    Show_Chinese(45, 110, (u8*)yang, 16, ORANGE, BLACK);
    Show_Chinese(65, 110, (u8*)jiao, 16, ORANGE, BLACK);//������ pitch

    Show_Chinese(125, 110, (u8*)heng, 16, ORANGE, BLACK);
    Show_Chinese(145, 110, (u8*)gun, 16, ORANGE, BLACK);
    Show_Chinese(165, 110, (u8*)jiao, 16, ORANGE, BLACK);//����� roll

//    Show_Chinese(25, 170, (u8*)pian, 16, ORANGE, BLACK);
//    Show_Chinese(45, 170, (u8*)hang, 16, ORANGE, BLACK);
//    Show_Chinese(65, 170, (u8*)jiao, 16, ORANGE, BLACK);//ƫ���� yaw

    Show_Chinese(60, 140, (u8*)jin, 16, ORANGE, BLACK);
    Show_Chinese(80, 140, (u8*)du, 16, ORANGE, BLACK);//����


    Show_Chinese(60, 170, (u8*)wei, 16, ORANGE, BLACK);
    Show_Chinese(80, 170, (u8*)du, 16, ORANGE, BLACK);//����

    Show_Chinese(70, 200, (u8*)an, 16, GREEN, BLACK);
    Show_Chinese(90, 200, (u8*)quan, 16, GREEN, BLACK);//��ȫ

    Show_Chinese(110, 200, (u8*)zheng, 16, GREEN, BLACK);
    Show_Chinese(130, 200, (u8*)chang, 16, GREEN, BLACK);
    Show_Chinese(150, 200, (u8*)yun, 16, GREEN, BLACK);
    Show_Chinese(170, 200, (u8*)xing, 16, GREEN, BLACK);//��������


}

void shuju_show()
{
    temperature = AHT10_Read_Temperature();
    humidity = AHT10_Read_Humidity();
    lcd_set_color(BLACK,ORANGE);
    lcd_show_string(80, 50, 16,":%4d", (int)(temperature));
    lcd_show_string(180, 50, 16,":%4d", (int)(humidity));             //��ʾ��ʪ��

    AP3216C_ReadData(&ir, &ps, &als);
    lcd_set_color(BLACK,ORANGE);
    lcd_show_string(80, 80, 16,":%4d", als);  //����ǿ��
    lcd_show_string(180, 80, 16,":%4d", ps);   //����

    MPU_GET();
    lcd_show_string(80, 110, 16,":%3d", (int)(pitch));
    lcd_show_string(180, 110, 16,":%3d", (int)(roll));
//    lcd_show_string(80, 170, 16,":%4d", (int)(yaw));//��ʾ��̬��

    lcd_show_string(95, 140, 16,":  %.5f", jingdu);
    lcd_show_string(95, 170, 16,":  %.5f", weidu);
}


void shuimian()
{
    RTC_Init(20);
    SysTickEnableOrDisable(DISABLE);
    RTC_ClearITPendingBit(RTC_IT_OW | RTC_IT_ALR);      //�������ж�
    Delay_Ms(50);
    wakeup_enter();
}

void gps_get()
{
    if (rev_stop == 1 && count++>=1000 )   //���������һ��
    {

        printf("%s\n",Usart2RecBuf);
            if (GPS_RMC_Parse(Usart2RecBuf, &GPS)) //����GPRMC
            {

                    error_num = 0;

                    rev_stop  = 0;
                    GpsInitOkFlag=1;
                    GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_RESET);//GPSָʾ������
                    int wd_int,jd_int,wd_float,jd_float =0;
                    wd_int = GPS.latitude_Degree;
                    jd_int = GPS.longitude_Degree;
                    jingdu =GPS.longitude_Degree;
                    weidu = GPS.latitude_Degree;
                    wd_float = (int)(GPS.latitude_Degree*10000)%10000;
                    jd_float = (int)(GPS.longitude_Degree*10000)%10000;
                    printf("%d.%d\n",wd_int,wd_float);
                    printf("%d.%d\n",jd_int,jd_float);
                    go_on =1;
            }
            else
            {
                    if (error_num++ >= 3) //���������Ч����3��
                    {
                            error_num = 3;
                            GpsInitOkFlag = 0;
                            GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_SET);//�ƹر�

                    }
                    gps_flag = 0;
                    rev_stop  = 0;
            }
            count=0;

    }

}

int main(void)
{

    init();
    xianshi_init();

    while(1)
    {


        do{
            gps_get();
        }while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11));


        RTC_Init(20);
        SysTickEnableOrDisable(DISABLE);
        RTC_ClearITPendingBit(RTC_IT_OW | RTC_IT_ALR);      //�������ж�

        do{
          key_test();
          shuju_show();
        }while(set_time!=0);

//        while(uartWriteWiFi("AT+RST\r\n",8));
//            Delay_Ms(1000);
//            printf("%d\r\n",1);
//
//            // ��Ϊ Station ģʽ
//            while(uartWriteWiFiStr("AT+CWMODE=1\r\n"));
//            Delay_Ms(1000);
//            printf("%d\r\n",2);
//
//            // ����ʱ��� SNTP ������
//            while(uartWriteWiFiStr("AT+CIPSNTPCFG=1,8,\"ntp1.aliyun.com\"\r\n"));
//            Delay_Ms(1000);
//            printf("%d\r\n",3);
//
//            // ����һ����Ϊ SSID������Ϊ PASSWORD �� WiFi ���磬
//            while(uartWriteWiFiStr("AT+CWJAP=\"Gifty\",\"12345678\"\r\n"));
//            Delay_Ms(10000);
//            printf("%d\r\n",4);
//
//            while(uartWriteWiFiStr("AT+MQTTUSERCFG=0,1,\"NULL\",\"Gifty&hgsrmNPZvom\",\"982AF7FAC38E5F8562694B2E28EC7958D3CD003B\",0,0,\"\"\r\n"));
//            Delay_Ms(3000);
//            printf("%d\r\n",5);
//
//            while(uartWriteWiFiStr("AT+MQTTCLIENTID=0,\"135790|securemode=3\\,signmethod=hmacsha1|\"\r\n"));
//            Delay_Ms(3000);
//            printf("%d\r\n",6);
//
//            while(uartWriteWiFiStr("AT+MQTTCONN=0,\"hgsrmNPZvom.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883,1\r\n"));
//            Delay_Ms(3000);
//            printf("%d\r\n",7);
//
//
//            while(uartWriteWiFiStr("AT+MQTTSUB=0,\"/sys/hgsrmNPZvom/Gifty/thing/event/property/set\",1\r\n"));
//            Delay_Ms(3000);
//            printf("%d\r\n",8);
//
//            while(uartWriteWiFiStr("AT+MQTTPUB=0,\"/sys/hgsrmNPZvom/Gifty/thing/event/property/post\",\"{\"method\":\"thing.service.property.set\"\,\"id\":\"123\"\,\"params\":{\"temp\":50}\,\"version\":\"1.0.0\"}\",1,0\r\n"));
//            Delay_Ms(3000);
//            printf("%d\r\n",9);
//


        shuimian();
    }

}


void USART2_IRQHandler(void)
{
       u8 ch;
     if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
     {
              ch = USART_ReceiveData(USART2);

        if (GPS_rx_flag == 1)
                {

            if ((ch == '$') && (gps_flag == 0))  //����յ��ַ�'$'���㿪ʼ����
                        {
                                rev_start = 1;
                                rev_stop  = 0;

                        }

                        if (rev_start == 1)  //��־λΪ1����ʼ����
                        {
                                Usart2RecBuf[num++] = ch;  //�ַ��浽������
//                                strcpy(ch,Usart2RecBuf);
//                                printf("%s",Usart2RecBuf);
                                if (ch == '\n')     //������յ�����
                                {
                                        Usart2RecBuf[num] = '\0';
                                        rev_start = 0;
                                        rev_stop  = 1;
                                        gps_flag = 1;
                                        num = 0;

                                        printf("%s",Usart2RecBuf);
                                }

                        }
                }
     }

     if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) == SET)
     {
         USART_ClearFlag(USART2,USART_FLAG_ORE);
     }
     USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}


