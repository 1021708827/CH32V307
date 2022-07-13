#ifndef __AHT10_H
#define __AHT10_H

/* I2C Mode Definition */
#define HOST_MODE   0
#define SLAVE_MODE   1

/* I2C Communication Mode Selection */
#define I2C_MODE   HOST_MODE
//#define I2C_MODE   SLAVE_MODE

#define AHT10_IIC_ADDR	0x38			//AHT10 IIC��ַ

#define AHT10_CALIBRATION_CMD 	0xE1 	//У׼����(�ϵ��ֻ��Ҫ����һ��)
#define AHT10_NORMAL_CMD 		0xA8 	//��������ģʽ
#define AHT10_GET_DATA 			0xAC 	//��ȡ��������

u8 AHT10_Init(void);
float AHT10_Read_Temperature(void);
float AHT10_Read_Humidity(void);

#endif