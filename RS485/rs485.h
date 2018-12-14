#ifndef __RS485_H
#define __RS485_H			 
#include "common.h"	 								  

	
#define	Temperature  0
#define Humidity     1
#define OAF_JDRK		 2
#define OAF_FJL			 3	
#define DL_1		 4	
#define DL_2		5




#define REC_MASK	512  			//�����������ֽ��� 512  	
extern u8 RS485_receive_str[REC_MASK]; 	//���ջ���,���512���ֽ�
extern u8 uart2_byte_count;   			  //���յ������ݳ���
extern u16 USART2_RX_STA;    					//����״̬���	
extern unsigned char Res_Flag;				//����Э���־λ
extern unsigned char Query[6][8];			//��ѯ���
extern unsigned int GetVal[10];
extern unsigned char outval[2];
//ģʽ����
#define RS485_TX_EN		PGout(6)	    //485ģʽ����.0,����;1,����.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);

unsigned int Rec_proc(void);

#endif	   
















