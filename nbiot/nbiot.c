#include "nbiot.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h" 

/*********************************************************************************
*********************�������� STM32F407Ӧ�ÿ�����(�����)*************************
**********************************************************************************
*��ԴԴ������ �����ɰ����޸�
*QQȺ��476840321 
*                               *
**********************************************************************************
*********************************************************************************/

u8 receive_str6[USART6_REC_NUM];     //���ջ�������,���USART_REC_LEN���ֽ� 
u8 uart_byte_count6=0;
//����ṹ�������洢��������
typedef struct {
    char USART_BUFF[100];
    int USART_Length;
    int flag;
}Usart_Struct;

Usart_Struct struct_usart6;
/****************************************************************************
* ��    ��: void uart1_init(u32 bound)
* ��    �ܣ�USART6��ʼ��
* ��ڲ�����bound��������
* ���ز�������
* ˵    ���� 
****************************************************************************/
void uart6_init(u32 bound)
{   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART1ʱ�� 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10����ΪUSART1
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);          //��ʼ��PA9��PA10
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); //��ʼ������1	
  USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);         //��������ж�
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;      //����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	  //����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

/*******************************************************************************  
* �����ֽ� 
*******************************************************************************/    
void uart6_send_char(u8 temp)      
{        
    USART_SendData(USART6,(u8)temp);        
    while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);         
}    

/*******************************************************************************  
* �����ַ���  
*******************************************************************************/    
void uart6_send_buff(u8 buf[],u32 len)     
{    
    u32 i;    
    for(i=0;i<len;i++)    
    uart6_send_char(buf[i]);
    // �����û��з�   NB����ATָ���Ҫ����
   // uart2_send_char('\r');
   // uart2_send_char('\n');

}
void USART6_IRQHandler(void)  
{
	    uint8_t ch;
#ifdef SYSTEM_SUPPORT_OS        
    OSIntEnter();    
#endif
    if(USART_GetITStatus(USART6,USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART6,USART_IT_RXNE);
        ch = USART_ReceiveData(USART6);
        struct_usart6.USART_BUFF[struct_usart6.USART_Length++] = ch;
        struct_usart6.flag = 1;
    }
    if( USART_GetITStatus( USART6, USART_IT_IDLE ) == SET )
    {
        USART_ClearITPendingBit(USART6,USART_IT_IDLE);
        struct_usart6.flag = 1;
        ch = USART_ReceiveData(USART6);
    }
#ifdef SYSTEM_SUPPORT_OS     
    OSIntExit();                                             
#endif
	
} 

/*******************************************************************************  
* �����ַ���  ����������ֵ�Ƿ���ȷ
* cmdΪ����ֵ  reply ΪУ�鷵��ֵ wait Ϊ��ʱ
*******************************************************************************/   
int NBiot_SendCmd(char* cmd, char* reply, int wait)
{
    struct_usart6.USART_Length = 0;
   // printf("[NBiot_SendCmd] %s\r\n", cmd);

    uart6_send_buff((u8*)cmd, strlen(cmd));

    delay_ms(wait);

    if (strcmp(reply, "") == 0) //����ֵΪ��
    {
        return 0;
    }

    if (struct_usart6.USART_Length != 0) //����ֵ��Ϊ��
    {
        struct_usart6.USART_BUFF[struct_usart6.USART_Length] = '\0';

        if (strstr((char*)struct_usart6.USART_BUFF, reply))
        {
            printf("\r\n%s+++YES\r\n", struct_usart6.USART_BUFF);

            return 1;
        }
        else if (strstr((char*)struct_usart6.USART_BUFF, "ERROR"))
        {
            printf("ERROR...\r\n");

            delay_ms(3000);

            return 0;
        }
        else
        {  
            printf("\r\n%s+++NO\r\n", struct_usart6.USART_BUFF);

            return 0;
        }  
    }  
}
int NBiot_Init()  
{
    int ret = 0;
    ret = NBiot_SendCmd("admin#AT+VER","OK", 1000);  //ѯ�ʰ汾����Ϊ��ʼ��
    if (!ret)
    {
        printf("Cannot initialize NBIOT module");
        return 0;
    }
    return ret;
}

