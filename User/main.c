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
GPS_INFO   GPS;  //GPS信息结构体
char Usart2RecBuf[USART2_RXBUFF_SIZE];//串口2接收数据缓存
unsigned int Rx2Counter = 0;   //串口2收到数据标志位
unsigned char rev_start = 0;     //接收开始标志
unsigned char rev_stop  = 0;     //接收停止标志
unsigned char gps_flag = 0;
unsigned char num = 0;
float jingdu =0;
float weidu = 0;
u8 GPS_rx_flag = 0;
u8 GpsInitOkFlag=0;             //GPS定位接收成功标志
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
         //开启时钟
         RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

         /* USART6 TX-->C0  RX-->C1 */
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
         GPIO_Init(GPIOC, &GPIO_InitStructure);
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX，输入上拉
         GPIO_Init(GPIOC, &GPIO_InitStructure);

         USART_InitStructure.USART_BaudRate = 115200;                    // 波特率
         USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 数据位 8
         USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 停止位 1
         USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验
         USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
         USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //使能 RX 和 TX

         USART_Init(UART6, &USART_InitStructure);
         DMA_Cmd(DMA2_Channel7, ENABLE);                                  //开启接收 DMA
         USART_Cmd(UART6, ENABLE);                                        //开启UART
     }
     void DMA_INIT(void)
     {
         DMA_InitTypeDef DMA_InitStructure;
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

         // TX DMA 初始化
         DMA_DeInit(DMA2_Channel6);
         DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART6->DATAR);        // DMA 外设基址，需指向对应的外设
         DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;                   // DMA 内存基址，指向发送缓冲区的首地址
         DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 方向 : 外设 作为 终点，即 内存 ->  外设
         DMA_InitStructure.DMA_BufferSize = 0;                                   // 缓冲区大小,即要DMA发送的数据长度,目前没有数据可发
         DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址自增，禁用
         DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存地址自增，启用
         DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据位宽，8位(Byte)
         DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据位宽，8位(Byte)
         DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 普通模式，发完结束，不循环发送
         DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级最高
         DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // M2P,禁用M2M
         DMA_Init(DMA2_Channel6, &DMA_InitStructure);

         // RX DMA 初始化，环形缓冲区自动接收
         DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // 接收缓冲区
         DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 方向 : 外设 作为 源，即 内存 <- 外设
         DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;                          // 缓冲区长度为 RXBUF_SIZE
         DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // 循环模式，构成环形缓冲区
         DMA_Init(DMA2_Channel7, &DMA_InitStructure);
     }

     FlagStatus uartWriteWiFi(char * data , uint16_t num)
     {
         //如上次发送未完成，返回
         if(DMA_GetCurrDataCounter(DMA2_Channel6) != 0){
             return RESET;
         }

         DMA_ClearFlag(DMA2_FLAG_TC8);
         DMA_Cmd(DMA2_Channel6, DISABLE );           // 关 DMA 后操作
         DMA2_Channel6->MADDR = (uint32_t)data;      // 发送缓冲区为 data
         DMA_SetCurrDataCounter(DMA2_Channel6,num);  // 设置缓冲区长度
         DMA_Cmd(DMA2_Channel6, ENABLE);             // 开 DMA
         return SET;
     }
     FlagStatus uartWriteWiFiStr(char * str)
     {
         uint16_t num = 0;
         while(str[num])num++;           // 计算字符串长度
         return uartWriteWiFi(str,num);
     }

     uint16_t rxBufferReadPos = 0;       //接收缓冲区读指针
     uint32_t uartReadWiFi(char * buffer , uint16_t num)
     {
         uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7); //计算 DMA 数据尾的位置
         uint16_t i = 0;
         if (rxBufferReadPos == rxBufferEnd){
             // 无数据，返回
             return 0;
         }

         while (rxBufferReadPos!=rxBufferEnd && i < num){
             buffer[i] = RxBuffer[rxBufferReadPos];
             i++;
             rxBufferReadPos++;
             if(rxBufferReadPos >= RXBUF_SIZE){
                 // 超出缓冲区，回零
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
             // 无数据，返回
             return 0;
         }
         ret = RxBuffer[rxBufferReadPos];
         rxBufferReadPos++;
         if(rxBufferReadPos >= RXBUF_SIZE){
             // 超出缓冲区，回零
             rxBufferReadPos = 0;
         }
         return ret;
     }


     uint16_t uartAvailableWiFi()
     {
         uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7);//计算 DMA 数据尾的位置
         // 计算可读字节
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
        MPU_Get_Gyroscope(&gyro_data.x_data,&gyro_data.y_data,&gyro_data.z_data);    // 获取陀螺仪角速度
        gyro_p->x_data += gyro_data.x_data;
        gyro_p->y_data += gyro_data.y_data;
        gyro_p->z_data += gyro_data.z_data;
        Delay_Ms(5);    // 最大 1Khz
    }
    gyro_p->x_data /=100;
    gyro_p->y_data /=100;
    gyro_p->z_data /=100;
}




void MPU_GET()
{
        MPU_Get_Accelerometer(&acc.x_data,&acc.y_data,&acc.z_data);   //获得加速度原始数据
        MPU_Get_Gyroscope(&gyro.x_data,&gyro.y_data,&gyro.z_data);    //获得陀螺仪原始数据

        acc_x = ((float)acc.x_data ) * alpha + acc_x *(1-alpha);
        acc_y = ((float)acc.y_data ) * alpha + acc_y *(1-alpha);
        acc_z = ((float)acc.z_data ) * alpha + acc_z *(1-alpha);

        gyro.x_data -= offset_gyro.x_data;
        gyro.y_data -= offset_gyro.y_data;
        gyro.z_data -= offset_gyro.z_data;

        //                              转换为弧度,灵敏度为16.4
        MahonyAHRSupdateIMU(gyro.x_data/16.4/57.3 *(-1.0)  ,gyro.y_data/16.4/57.3 *(-1.0)  ,gyro.z_data/16.4/57.3,(acc_x/4096)*(-1.0),(acc_y/4096)*(-1.0),acc_z/4096);


        pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
        roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
//        yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;   //yaw
 //     printf("pitch=%d,roll=%d,yaw=%d\r\n",(int)(pitch),(int)(roll),(int)(yaw));

}

__attribute__((interrupt("WCH-Interrupt-fast")))
void RTCAlarm_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//闹钟中断
    {
        RTC_SetCounter(0);      // 清除RTC计数器，从新开始计数
        RTC_WaitForLastTask();
        RTC_SetAlarm(0);        // 设置闹钟 设置闹钟的时间要加1哦,,也就是说现在是3S
        RTC_WaitForLastTask();
    }

    set_time-=1;
    RTC_ClearITPendingBit(RTC_IT_ALR);      //清闹钟中断
    EXTI_ClearITPendingBit(EXTI_Line17);     //闹钟事件发生，会产生一个EXTI_17外部中断，此标志位要清除，否则下次停止模式进入失败
    RTC_ClearITPendingBit(RTC_IT_OW);       //清闹钟中断
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
            Show_Chinese(90, 200, (u8*)gao, 16, RED, BLACK);//警告

            Show_Chinese(110, 200, (u8*)you, 16, RED, BLACK);
            Show_Chinese(130, 200, (u8*)ren, 16, RED, BLACK);
            Show_Chinese(150, 200, (u8*)shang, 16, RED, BLACK);
            Show_Chinese(170, 200, (u8*)ta1, 16, RED, BLACK);//有人上塔

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
    Show_Chinese(190, 10,(u8*)tong,24, RED, 0xFFFF);  //铁塔监测系统

    Show_Chinese(45, 50, (u8*)wen, 16, ORANGE, BLACK);
    Show_Chinese(65, 50, (u8*)du, 16, ORANGE, BLACK);//温度:

    Show_Chinese(145, 50, (u8*)shi, 16, ORANGE, BLACK);
    Show_Chinese(165, 50, (u8*)du, 16, ORANGE, BLACK);//湿度:


    Show_Chinese(5, 80, (u8*)guang, 16, ORANGE, BLACK);
    Show_Chinese(25, 80, (u8*)zhao, 16, ORANGE, BLACK);
    Show_Chinese(45, 80, (u8*)qiang, 16, ORANGE, BLACK);
    Show_Chinese(65, 80, (u8*)du, 16, ORANGE, BLACK);//光照强度:

    Show_Chinese(145, 80, (u8*)ju, 16, ORANGE, BLACK);
    Show_Chinese(165, 80, (u8*)li, 16, ORANGE, BLACK);//距离:

    Show_Chinese(25, 110, (u8*)fu, 16, ORANGE, BLACK);
    Show_Chinese(45, 110, (u8*)yang, 16, ORANGE, BLACK);
    Show_Chinese(65, 110, (u8*)jiao, 16, ORANGE, BLACK);//俯仰角 pitch

    Show_Chinese(125, 110, (u8*)heng, 16, ORANGE, BLACK);
    Show_Chinese(145, 110, (u8*)gun, 16, ORANGE, BLACK);
    Show_Chinese(165, 110, (u8*)jiao, 16, ORANGE, BLACK);//横滚脚 roll

//    Show_Chinese(25, 170, (u8*)pian, 16, ORANGE, BLACK);
//    Show_Chinese(45, 170, (u8*)hang, 16, ORANGE, BLACK);
//    Show_Chinese(65, 170, (u8*)jiao, 16, ORANGE, BLACK);//偏航脚 yaw

    Show_Chinese(60, 140, (u8*)jin, 16, ORANGE, BLACK);
    Show_Chinese(80, 140, (u8*)du, 16, ORANGE, BLACK);//经度


    Show_Chinese(60, 170, (u8*)wei, 16, ORANGE, BLACK);
    Show_Chinese(80, 170, (u8*)du, 16, ORANGE, BLACK);//经度

    Show_Chinese(70, 200, (u8*)an, 16, GREEN, BLACK);
    Show_Chinese(90, 200, (u8*)quan, 16, GREEN, BLACK);//安全

    Show_Chinese(110, 200, (u8*)zheng, 16, GREEN, BLACK);
    Show_Chinese(130, 200, (u8*)chang, 16, GREEN, BLACK);
    Show_Chinese(150, 200, (u8*)yun, 16, GREEN, BLACK);
    Show_Chinese(170, 200, (u8*)xing, 16, GREEN, BLACK);//正常运行


}

void shuju_show()
{
    temperature = AHT10_Read_Temperature();
    humidity = AHT10_Read_Humidity();
    lcd_set_color(BLACK,ORANGE);
    lcd_show_string(80, 50, 16,":%4d", (int)(temperature));
    lcd_show_string(180, 50, 16,":%4d", (int)(humidity));             //显示温湿度

    AP3216C_ReadData(&ir, &ps, &als);
    lcd_set_color(BLACK,ORANGE);
    lcd_show_string(80, 80, 16,":%4d", als);  //光照强度
    lcd_show_string(180, 80, 16,":%4d", ps);   //距离

    MPU_GET();
    lcd_show_string(80, 110, 16,":%3d", (int)(pitch));
    lcd_show_string(180, 110, 16,":%3d", (int)(roll));
//    lcd_show_string(80, 170, 16,":%4d", (int)(yaw));//显示姿态角

    lcd_show_string(95, 140, 16,":  %.5f", jingdu);
    lcd_show_string(95, 170, 16,":  %.5f", weidu);
}


void shuimian()
{
    RTC_Init(20);
    SysTickEnableOrDisable(DISABLE);
    RTC_ClearITPendingBit(RTC_IT_OW | RTC_IT_ALR);      //清闹钟中断
    Delay_Ms(50);
    wakeup_enter();
}

void gps_get()
{
    if (rev_stop == 1 && count++>=1000 )   //如果接收完一行
    {

        printf("%s\n",Usart2RecBuf);
            if (GPS_RMC_Parse(Usart2RecBuf, &GPS)) //解析GPRMC
            {

                    error_num = 0;

                    rev_stop  = 0;
                    GpsInitOkFlag=1;
                    GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_RESET);//GPS指示灯亮起
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
                    if (error_num++ >= 3) //如果数据无效超过3次
                    {
                            error_num = 3;
                            GpsInitOkFlag = 0;
                            GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_SET);//灯关闭

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
        RTC_ClearITPendingBit(RTC_IT_OW | RTC_IT_ALR);      //清闹钟中断

        do{
          key_test();
          shuju_show();
        }while(set_time!=0);

//        while(uartWriteWiFi("AT+RST\r\n",8));
//            Delay_Ms(1000);
//            printf("%d\r\n",1);
//
//            // 设为 Station 模式
//            while(uartWriteWiFiStr("AT+CWMODE=1\r\n"));
//            Delay_Ms(1000);
//            printf("%d\r\n",2);
//
//            // 设置时域和 SNTP 服务器
//            while(uartWriteWiFiStr("AT+CIPSNTPCFG=1,8,\"ntp1.aliyun.com\"\r\n"));
//            Delay_Ms(1000);
//            printf("%d\r\n",3);
//
//            // 连接一个名为 SSID、密码为 PASSWORD 的 WiFi 网络，
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

            if ((ch == '$') && (gps_flag == 0))  //如果收到字符'$'，便开始接收
                        {
                                rev_start = 1;
                                rev_stop  = 0;

                        }

                        if (rev_start == 1)  //标志位为1，开始接收
                        {
                                Usart2RecBuf[num++] = ch;  //字符存到数组中
//                                strcpy(ch,Usart2RecBuf);
//                                printf("%s",Usart2RecBuf);
                                if (ch == '\n')     //如果接收到换行
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


