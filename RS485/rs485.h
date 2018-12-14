#ifndef __RS485_H
#define __RS485_H			 
#include "common.h"	 								  

	
#define	Temperature  0
#define Humidity     1
#define OAF_JDRK		 2
#define OAF_FJL			 3	
#define DL_1		 4	
#define DL_2		5




#define REC_MASK	512  			//定义最大接收字节数 512  	
extern u8 RS485_receive_str[REC_MASK]; 	//接收缓冲,最大512个字节
extern u8 uart2_byte_count;   			  //接收到的数据长度
extern u16 USART2_RX_STA;    					//接收状态标记	
extern unsigned char Res_Flag;				//接收协议标志位
extern unsigned char Query[6][8];			//问询命令集
extern unsigned int GetVal[10];
extern unsigned char outval[2];
//模式控制
#define RS485_TX_EN		PGout(6)	    //485模式控制.0,接收;1,发送.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);

unsigned int Rec_proc(void);

#endif	   
















