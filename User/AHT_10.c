#include "debug.h"
#include "AHT_10.h"
#include "IIC.h"



/**
 * @brief	��ATH10д������
 *
 * @param   cmd		����
 * @param   data	Ҫд�������
 * @param   len		д�����ݴ�С
 *
 * @return  u8		0,����,����,�������
 */
u8 AHT10_Write_Data(u8 cmd, u8 *data, u8 len)
{
    u8 i=0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,((AHT10_IIC_ADDR << 1) | 0),I2C_Direction_Transmitter); //����������ַ+д����

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    I2C_SendData(I2C2,cmd);         //д�Ĵ�����ַ

    while(i < len)
    {
        while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET )

                I2C_SendData(I2C2,data[i]);     //��������
                i++;

    }
//    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET );
    I2C_GenerateSTOP( I2C2, ENABLE );

    return 0;
}


/**
 * @brief	��һ���ֽ�
 *
 * @param   void
 *
 * @return  u8		����������
 */
u8 AHT10_ReadOneByte(void)
{
    u8 res = 0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,(AHT10_IIC_ADDR << 1) | 0X01,I2C_Direction_Receiver); //����������ַ+������
    //�ȴ�Ӧ��
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
    //��һ���ֽ�����
    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) ==  RESET );
	res = I2C_ReceiveData( I2C2 );
    I2C_GenerateSTOP( I2C2, ENABLE ); //����һ��ֹͣ����
    return res;
}

/**
 * @brief	������
 *
 * @param   data	���ݻ���
 * @param   len		�����ݴ�С
 *
 * @return  u8		0,����,����,�������
 */
u8 AHT10_Read_Data(u8 *data, u8 len)
{
    u8 i=0;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

//    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,(AHT10_IIC_ADDR << 1) | 0X01,I2C_Direction_Receiver); //����������ַ+������
   
    //�ȴ�Ӧ��
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );

    while(i < len)
    {
        if( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) !=  RESET )
        {
            if(i == (len - 2))
            {
                I2C_AcknowledgeConfig( I2C2, DISABLE );
                data[i] = I2C_ReceiveData(I2C2);		//������,����nACK
            }
            else
            {
                data[i] = I2C_ReceiveData(I2C2);		//������,����ACK
            }
            i++;
        }

    }

    I2C_GenerateSTOP( I2C2, ENABLE ); //����һ��ֹͣ����
    return 0;
}

/**
 * @brief	��ȡ�¶�����
 *
 * @param   void
 *
 * @return  float	�¶����ݣ���λ�����϶ȣ�
 */
float AHT10_Read_Temperature(void)
{
    u8 res = 0;
    u8 cmd[2] = {0x33, 0};
    u8 temp[6];

    float cur_temp;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //���Ͷ�ȡ��������

    if(res) return 1;
    Delay_Ms(80);
    res = AHT10_Read_Data(temp, 6);             //��ȡ����

    if(res) return 1;
    cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;

    return cur_temp;
}

/**
 * @brief	��ȡʪ������
 *
 * @param   void
 *
 * @return  float	ʪ�����ݣ���λ��%RH��
 */
float AHT10_Read_Humidity(void)
{
    u8 res = 0;
    u8 cmd[2] = {0x33, 0};
    u8 humi[6];
    float cur_humi;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //���Ͷ�ȡ��������

    if(res)	return 1;
    Delay_Ms(80);
    res = AHT10_Read_Data(humi, 6);				//��ȡ����

    if(res)	return 1;
    cur_humi = ((humi[1]) << 12 | humi[2] << 4 | (humi[3] & 0xF0)) * 100.0 / (1 << 20);

    return cur_humi;
}

/**
 * @brief	ATH10��������ʼ��
 *
 * @param   void
 *
 * @return  u8		0,��ʼ���ɹ�������,ʧ��
 */
u8 AHT10_Init(void)
{
    u8 res;
    u8 temp[2] = {0, 0};

    IIC_Init(200000,0x02);      //��ʼ��IIC�ӿڣ�ע�������IIC����Ϊ��SCL-PB10 SDA-PB11

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


