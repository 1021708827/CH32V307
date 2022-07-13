#include "AP3216C.h"
#include "debug.h"
#include "IIC.h"

/**
 * @brief	��ʼ��AP3216C
 *
 * @param   void
 *
 * @return  u8		0,��ʼ���ɹ�
 *					1,��ʼ��ʧ��
 */
u8 AP3216C_Init(void)
{
    u8 temp = 0;
    IIC_Init(200000,0x02);         				//��ʼ��IIC
    AP3216C_WriteOneByte(0x00, 0X04);	//��λAP3216C
    Delay_Ms(50);						//AP33216C��λ����10ms
    AP3216C_WriteOneByte(0x00, 0X03);	//����ALS��PS+IR

    temp = AP3216C_ReadOneByte(0X00);		//��ȡ�ո�д��ȥ��0X03

    if(temp == 0X03)return 0;				//AP3216C����

    else return 1;						//AP3216Cʧ��
}

/**
 * @brief	��ȡAP3216C������,��ȡԭʼ���ݣ�����ALS,PS��IR
 *			ע�⣡���ͬʱ��ALS,IR+PS�Ļ��������ݶ�ȡ��ʱ����Ҫ����112.5ms
 *
 * @param   ir	��������
 * @param   ps	��������
 * @param   als	��������
 *
 * @return  void
 */
void AP3216C_ReadData(u16* ir, u16* ps, u16* als)
{
    u8 buf[6];
    u8 i;

    for(i = 0; i < 6; i++)
    {
        buf[i] = AP3216C_ReadOneByte(0X0A + i);		//ѭ����ȡ���д���������
    }

    if(buf[0] & 0X80)*ir = 0;						//IR_OFλΪ1,��������Ч

    else *ir = ((u16)buf[1] << 2) | (buf[0] & 0X03); 	//��ȡIR������������

    *als = ((u16)buf[3] << 8) | buf[2];				//��ȡALS������������

    if(buf[4] & 0x40)*ps = 0;    					//IR_OFλΪ1,��������Ч

    else *ps = ((u16)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F); //��ȡPS������������
}

/**
 * @brief	IICдһ���ֽ�
 *
 * @param   reg		�Ĵ�����ַ
 * @param   data	Ҫд�������
 *
 * @return  u8		0,����,����,�������
 */
u8 AP3216C_WriteOneByte(u8 reg, u8 data)
{
    I2C_AcknowledgeConfig( I2C2, ENABLE );

    I2C_GenerateSTART( I2C2, ENABLE );

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,((AP3216C_ADDR << 1) | 0),I2C_Direction_Transmitter);//����������ַ+д����

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );  //�ȴ�Ӧ��

    I2C_SendData(I2C2,reg);     //д�Ĵ�����ַ

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ); //�ȴ�Ӧ��

    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET );
    I2C_SendData(I2C2,data);      //��������

    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_TXE ) ==  RESET );

    I2C_GenerateSTOP( I2C2, ENABLE );
    return 0;
}

/**
 * @brief	IIC��һ���ֽ�
 *
 * @param   reg		�Ĵ�����ַ
 *
 * @return  u8		����������
 */
u8 AP3216C_ReadOneByte(u8 reg)
{
    u8 res;

    I2C_AcknowledgeConfig( I2C2, ENABLE );

    I2C_GenerateSTART( I2C2, ENABLE );

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress(I2C2,(AP3216C_ADDR << 1) | 0X00,I2C_Direction_Transmitter); //����������ַ+д����

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ); //�ȴ�Ӧ��
    I2C_SendData(I2C2,reg);         //д�Ĵ�����ַ

    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress(I2C2,((AP3216C_ADDR << 1) | 0x01),I2C_Direction_Receiver);//����������ַ+������
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) ); //�ȴ�Ӧ��

    I2C_AcknowledgeConfig( I2C2, DISABLE );

    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) ==  RESET );
    res = I2C_ReceiveData( I2C2 ); //������,����nACK


    I2C_GenerateSTOP( I2C2, ENABLE );//����һ��ֹͣ����
    return res;
}

