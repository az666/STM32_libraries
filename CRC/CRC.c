#include <CRC.h>


u16 crc16(u8 *ptr,u8 len)
{
	unsigned long wcrc=0XFFFF;//预置16位crc寄存器，初值全部为1
	int i=0,j=0;//定义计数
	for(i=0;i<len;i++)//循环计算每个数据
  {
		wcrc^=*ptr++;//将八位数据与crc寄存器亦或.指针地址增加，指向下个数据
		for(j=0;j<8;j++)//循环计算数据的
		{
			if(wcrc&0X0001)//判断右移出的是不是1，如果是1则与多项式进行异或。
			{
				wcrc=wcrc>>1^0XA001;//先将数据右移一位再与上面的多项式进行异或
			}
			else//如果不是1，则直接移出
			{
				wcrc>>=1;//直接移出
			}
		}
	}
	return wcrc<<8|wcrc>>8;//低八位在前，高八位在后
}
