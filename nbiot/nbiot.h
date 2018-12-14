#ifndef __USART6_H
#define __USART6_H
#include "stdio.h"	
#include "common.h" 

//////////////////////////////////////////////////////////////////////////////////	 

#define USART6_REC_NUM  			100  	//�����������ֽ��� 200
extern u8 uart_byte_count;          //uart_byte_countҪС��USART_REC_LEN
extern u8 receive_str[USART6_REC_NUM];  

void uart6_init(u32 bound);
void uart6SendChars(u8 *str, u16 strlen);
int NBiot_SendCmd(char* cmd, char* reply, int wait);
int NBiot_Init();

#endif


