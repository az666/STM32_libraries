#include "rs485.h"	
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h"
#include "usart3.h"
	 
/*********************************************************************************
*********************�������� STM32F407Ӧ�ÿ�����(�����)*************************
**********************************************************************************
* �ļ�����: rs485.c                                                              *
* �ļ�������rs485��ʼ��                                                          *
* �������ڣ�2017.08.30                                                           *
* ��    ����V1.0                                                                 *
* ��    �ߣ�Clever                                                               *
* ˵    ����                                                                     * 
* ��    ���������̴��������ѧϰ�ο�                                             *
**********************************************************************************
*********************************************************************************/



unsigned int GetVal[10];
unsigned char outval[2];
unsigned char Query[6][8] = 
{
	{ 0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB },//Temperature&Humidity  01 04 00 00 00 02 71 CB
	{ 0x01, 0x04, 0x00, 0x01, 0x00, 0x01, 0x60, 0x0A },//Humidity
	{ 0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38 },//OAF_JDRK
	{ 0x01, 0x04, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xC6 },//OAF_FJL
	{0xFE,0x04,0x00,0x00,0x00,0x01,0x25,0xC5},//DL_1
	{0xFE,0x04,0x00,0x01,0x00,0x01,0x74,0x05},//DL_2
};
//��ַ�洢��  	  
unsigned char Address[10] = 
{	
	0x01,	/*��ʪ�ȵ�ַ					Temperature&Humidity	*/	
	0x01, /*��ʪ�ȵ�ַ					Temperature&Humidity	*/	
	0x02,	/*�����ʿƵ�ַ				OAF_JDRK							*/
	0x03,	/*�����Ѽ��޵�ַ			OAF_FJL								*/
	0xFE,	/*DL_1*/
	0xFE,	/*���е�ַ*/
	0x06,	/*���е�ַ*/
	0x07,	/*���е�ַ*/
	0x08,	/*���е�ַ*/
	0x09, /*���е�ַ*/
};

//���ջ����� 	
u8 RS485_receive_str[512];   //���ջ���,���128���ֽ�.
u8 uart2_byte_count=0;        //���յ������ݳ���
u16 USART2_RX_STA;    					//����״̬���	

//����Э���־λ
unsigned char Res_Flag;

unsigned int Rec_proc(void);
										 
//��ʼ��IO ����2   bound:������	
void RS485_Init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
  //����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3
	
	//PG8���������485ģʽ����  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOG6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��PG8
	
	RS485_TX_EN=0;				//��ʼ��Ĭ��Ϊ����ģʽ	
	
   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ��� 2	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//���������ж�

	//Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����2�����жϷ�����
void USART2_IRQHandler(void)
{
	unsigned int Val = 0;
	static char Flag = 0;
	u8 rec_data;
int readcmd[7];	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���յ�����
	{	 	
		rec_data =(u8)USART_ReceiveData(USART2);         //(USART2->DR) ��ȡ���յ�������
		//printf("%X\n", rec_data);
		//uart3SendChar(rec_data);  //FE 04 02 0F A0 A8 AC
		//uart3SendChar(Flag);
		switch(Flag)
		{
			case 0:
				if(rec_data == Address[Res_Flag])
				{
					RS485_receive_str[Flag] = rec_data;
					Flag++;
				}
				break;
			case 1:
					RS485_receive_str[Flag] = rec_data;
					Flag++;
				break;
			case 2:
					RS485_receive_str[Flag] = rec_data;
					Flag++;
				break;
			case 3:
				RS485_receive_str[Flag] = rec_data;
				Flag++;
				break;
			case 4:
				RS485_receive_str[Flag] = rec_data;
				Flag++;
				break;
			case 5:
				RS485_receive_str[Flag] = rec_data;
				Flag++;				
				break;
			case 6:
				RS485_receive_str[Flag] = rec_data;
				Flag = 0;
				Val = (RS485_receive_str[3]*256) + (RS485_receive_str[4]*1);
			  //outval[0] = RS485_receive_str[3]; 
			 // outval[1] = RS485_receive_str[4];
				//uart3SendChar(RS485_receive_str[3]); 
			  GetVal[Res_Flag] = Val; 
				break;
		}	
	}  											 
} 
/****************************************************************************
* ��    ��: void RS485_Send_Data(u8 *buf,u8 len)
* ��    �ܣ�RS485����len���ֽ�
* ��ڲ�����buf:�������׵�ַ
            len:���͵��ֽ��� 
* ���ز�������
* ˵    ����(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�����ݳ��Ȳ�Ҫ����512���ֽ�)       
****************************************************************************/	
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			    //����Ϊ����ģʽ
  for(t=0;t<len;t++)			//ѭ����������
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET); //�ȴ����ͽ���		
    USART_SendData(USART2,buf[t]); //��������
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);   //�ȴ����ͽ���		
	uart2_byte_count=0;	  
	RS485_TX_EN=0;				//����������Ϊ����ģʽ	
}







