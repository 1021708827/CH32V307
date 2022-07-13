#include "debug.h"
#include "AHT_10.h"
#include "IIC.h"



/**
 * @brief	向ATH10写入数据
 *
 * @param   cmd		命令
 * @param   data	要写入的数据
 * @param   len		写入数据大小
 *
 * @return  u8		0,正常,其他,错误代码
 */
u8 AHT10_Write_Data(u8 cmd, u8 *data, u8 len)
{
    u8 i=0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,((AHT10_IIC_ADDR << 1) | 0),I2C_Direction_Transmitter); //发送器件地址+写命令

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    I2C_SendData(I2C2,cmd);         //写寄存器地址

    while(i < len)
    {
        while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET )

                I2C_SendData(I2C2,data[i]);     //发送数据
                i++;

    }
//    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET );
    I2C_GenerateSTOP( I2C2, ENABLE );

    return 0;
}


/**
 * @brief	读一个字节
 *
 * @param   void
 *
 * @return  u8		读到的数据
 */
u8 AHT10_ReadOneByte(void)
{
    u8 res = 0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,(AHT10_IIC_ADDR << 1) | 0X01,I2C_Direction_Receiver); //发送器件地址+读命令
    //等待应答
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
    //收一个字节数据
    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) ==  RESET );
	res = I2C_ReceiveData( I2C2 );
    I2C_GenerateSTOP( I2C2, ENABLE ); //产生一个停止条件
    return res;
}

/**
 * @brief	读数据
 *
 * @param   data	数据缓存
 * @param   len		读数据大小
 *
 * @return  u8		0,正常,其他,错误代码
 */
u8 AHT10_Read_Data(u8 *data, u8 len)
{
    u8 i=0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,(AHT10_IIC_ADDR << 1) | 0X01,I2C_Direction_Receiver); //发送器件地址+读命令
   
    //等待应答
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );

    while(i < len)
    {
        if( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) !=  RESET )
        {
            if(i == (len - 2))
            {
                I2C_AcknowledgeConfig( I2C2, DISABLE );
                data[i] = I2C_ReceiveData(I2C2);		//读数据,发送nACK
            }
            else
            {
                data[i] = I2C_ReceiveData(I2C2);		//读数据,发送ACK
            }
            i++;
        }

    }

    I2C_GenerateSTOP( I2C2, ENABLE ); //产生一个停止条件
    return 0;
}

/**
 * @brief	读取温度数据
 *
 * @param   void
 *
 * @return  float	温度数据（单位：摄氏度）
 */
float AHT10_Read_Temperature(void)
{
    u8 res = 0;
    u8 cmd[2] = {0x33, 0};
    u8 temp[6];

    float cur_temp;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //发送读取数据命令

    if(res) return 1;
    Delay_Ms(80);
    res = AHT10_Read_Data(temp, 6);             //读取数据

    if(res) return 1;
    cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;

    return cur_temp;
}

/**
 * @brief	读取湿度数据
 *
 * @param   void
 *
 * @return  float	湿度数据（单位：%RH）
 */
float AHT10_Read_Humidity(void)
{
    u8 res = 0;
    u8 cmd[2] = {0x33, 0};
    u8 humi[6];
    float cur_humi;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //发送读取数据命令

    if(res)	return 1;
    Delay_Ms(80);
    res = AHT10_Read_Data(humi, 6);				//读取数据

    if(res)	return 1;
    cur_humi = ((humi[1]) << 12 | humi[2] << 4 | (humi[3] & 0xF0)) * 100.0 / (1 << 20);

    return cur_humi;
}

/**
 * @brief	ATH10传感器初始化
 *
 * @param   void
 *
 * @return  u8		0,初始化成功，其他,失败
 */
u8 AHT10_Init(void)
{
    u8 res;
    u8 temp[2] = {0, 0};

    IIC_Init(200000,0x02);      //初始化IIC接口：注意这里的IIC总线为：SCL-PB10 SDA-PB11

    res = AHT10_Write_Data(AHT10_NORMAL_CMD, temp, 2);

    if(res != 0)    return 1;

    Delay_Ms(300);

    temp[0] = 0x08;
    temp[1] = 0x00;
    res = AHT10_Write_Data(AHT10_CALIBRATION_CMD, temp, 2);

    if(res != 0)    return 1;

    Delay_Ms(300);

    return 0;
}


