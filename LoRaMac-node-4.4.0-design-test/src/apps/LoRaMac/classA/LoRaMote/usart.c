/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	

#include "usart.h"
#include "stm32l1xx.h"

UART_HandleTypeDef huart1;

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       //接收状态标记	  
uint16_t USART_RX_CNT=0;  //接收长度

uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

//初始化IO 串口1 
//bound:波特率
void uart_init(uint32_t bound)
{	
	//UART 初始化设置
	huart1.Instance=USART1;					    //USART1
	huart1.Init.BaudRate=bound;				    //波特率
	huart1.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	huart1.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	huart1.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	huart1.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&huart1);					    //HAL_UART_Init()会使能UART1  调用HAL_UART_MspInit
	
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  
	
	//串口中断接收，以中断方式接收指定长度数据。
	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄
void HAL_UART_MspInit(UART_HandleTypeDef *huart)//被HAL_UART_Init()调用
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
		//__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;//高速
		GPIO_Initure.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

//		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
//		GPIO_Initure.Mode=GPIO_MODE_INPUT;	//模式要设置为输入模式！	
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		HAL_NVIC_SetPriority(USART1_IRQn,8,0);			//抢占优先级3，子优先级3
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		
#endif	
	}
}




//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//HAL_UART_IRQHandler调用
//{
//	//printf("\r\n 进入HAL_UART_RxCpltCallback \r\n");
//	if(huart->Instance==USART1)//如果是串口1
//	{
//		if((USART_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
//				else USART_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}

//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//HAL_UART_IRQHandler调用
{
	//printf("\r\n 进入HAL_UART_RxCpltCallback \r\n");
	if(huart->Instance==USART1)//如果是串口1
	{
			USART_RX_BUF[USART_RX_CNT++]=aRxBuffer[0] ;
			if(USART_RX_CNT>(USART_REC_LEN-1))USART_RX_CNT=0;//接收数据错误,重新开始接收	  
	}
}


//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	uint32_t timeout=0;
	//printf("\r\n 进入USART1_IRQHandler \r\n");
	HAL_UART_IRQHandler(&huart1);	//调用HAL库中断处理公用函数
//	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);  
//   __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  
	
	timeout=0;
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//等待就绪
	{
	 timeout++;////超时处理
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置 RxXferCount 为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
} 

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFF);
    return ch;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
