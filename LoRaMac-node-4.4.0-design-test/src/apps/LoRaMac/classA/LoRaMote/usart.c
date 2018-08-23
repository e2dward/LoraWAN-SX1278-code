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

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA=0;       //����״̬���	  
uint16_t USART_RX_CNT=0;  //���ճ���

uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

//��ʼ��IO ����1 
//bound:������
void uart_init(uint32_t bound)
{	
	//UART ��ʼ������
	huart1.Instance=USART1;					    //USART1
	huart1.Init.BaudRate=bound;				    //������
	huart1.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	huart1.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	huart1.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	huart1.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&huart1);					    //HAL_UART_Init()��ʹ��UART1  ����HAL_UART_MspInit
	
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  
	
	//�����жϽ��գ����жϷ�ʽ����ָ���������ݡ�
	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��
void HAL_UART_MspInit(UART_HandleTypeDef *huart)//��HAL_UART_Init()����
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
		//__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;//����
		GPIO_Initure.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

//		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
//		GPIO_Initure.Mode=GPIO_MODE_INPUT;	//ģʽҪ����Ϊ����ģʽ��	
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		
#if EN_USART1_RX
		HAL_NVIC_SetPriority(USART1_IRQn,8,0);			//��ռ���ȼ�3�������ȼ�3
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		
#endif	
	}
}




//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//HAL_UART_IRQHandler����
//{
//	//printf("\r\n ����HAL_UART_RxCpltCallback \r\n");
//	if(huart->Instance==USART1)//����Ǵ���1
//	{
//		if((USART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//			{
//				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA|=0x8000;	//��������� 
//			}
//			else //��û�յ�0X0D
//			{	
//				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//				}		 
//			}
//		}

//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//HAL_UART_IRQHandler����
{
	//printf("\r\n ����HAL_UART_RxCpltCallback \r\n");
	if(huart->Instance==USART1)//����Ǵ���1
	{
			USART_RX_BUF[USART_RX_CNT++]=aRxBuffer[0] ;
			if(USART_RX_CNT>(USART_REC_LEN-1))USART_RX_CNT=0;//�������ݴ���,���¿�ʼ����	  
	}
}


//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 
	uint32_t timeout=0;
	//printf("\r\n ����USART1_IRQHandler \r\n");
	HAL_UART_IRQHandler(&huart1);	//����HAL���жϴ����ú���
//	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);  
//   __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  
	
	timeout=0;
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//�ȴ�����
	{
	 timeout++;////��ʱ����
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ����� RxXferCount Ϊ1
	{
	 timeout++; //��ʱ����
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
