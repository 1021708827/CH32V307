/*
 * I2S.h
 *
 *  Created on: 2022年5月6日
 *      Author: admin
 */

#ifndef USER_I2S_H_
#define USER_I2S_H_

#define  Len    64000  //在SRAM中以16位形式存放数据长度。8K采样率，双通道，1s数据32KB



void I2S2_Init_TX(void);
void I2S2_Init_RX(void);
void DMA_Rx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize );
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize);

#endif /* USER_I2S_H_ */
