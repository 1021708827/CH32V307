/*
 * I2S.h
 *
 *  Created on: 2022��5��6��
 *      Author: admin
 */

#ifndef USER_I2S_H_
#define USER_I2S_H_

#define  Len    64000  //��SRAM����16λ��ʽ������ݳ��ȡ�8K�����ʣ�˫ͨ����1s����32KB



void I2S2_Init_TX(void);
void I2S2_Init_RX(void);
void DMA_Rx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize );
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize);

#endif /* USER_I2S_H_ */
