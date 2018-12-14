#include "nbiot.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h" 

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
*资源源于网络 代码由阿正修改
*QQ群：476840321 
*                               *
**********************************************************************************
*********************************************************************************/

u8 receive_str6[USART6_REC_NUM];     //接收缓存数组,最大USART_REC_LEN个字节 
u8 uart_byte_count6=0;
//定义结构体用来存储接收数据
typedef struct {
    char USART_BUFF[100];
    int USART_Length;
    int flag;
}Usart_Struct;

Usart_Struct struct_usart6;
/****************************************************************************
* 名    称: void uart1_init(u32 bound)
* 功    能：USART6初始化
* 入口参数：bound：波特率
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart6_init(u32 bound)
{   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART1时钟 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10复用为USART1
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);          //初始化PA9，PA10
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1	
  USART_Cmd(USART6, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);         //开启相关中断
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;      //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	  //根据指定的参数初始化VIC寄存器、
}

/*******************************************************************************  
* 发送字节 
*******************************************************************************/    
void uart6_send_char(u8 temp)      
{        
    USART_SendData(USART6,(u8)temp);        
    while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);         
}    

/*******************************************************************************  
* 发送字符串  
*******************************************************************************/    
void uart6_send_buff(u8 buf[],u32 len)     
{    
    u32 i;    
    for(i=0;i<len;i++)    
    uart6_send_char(buf[i]);
    // 可设置换行符   NB串口AT指令不需要换行
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
* 发送字符串  并解析返回值是否正确
* cmd为传入值  reply 为校验返回值 wait 为延时
*******************************************************************************/   
int NBiot_SendCmd(char* cmd, char* reply, int wait)
{
    struct_usart6.USART_Length = 0;
   // printf("[NBiot_SendCmd] %s\r\n", cmd);

    uart6_send_buff((u8*)cmd, strlen(cmd));

    delay_ms(wait);

    if (strcmp(reply, "") == 0) //返回值为空
    {
        return 0;
    }

    if (struct_usart6.USART_Length != 0) //返回值不为空
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
    ret = NBiot_SendCmd("admin#AT+VER","OK", 1000);  //询问版本号作为初始化
    if (!ret)
    {
        printf("Cannot initialize NBIOT module");
        return 0;
    }
    return ret;
}

