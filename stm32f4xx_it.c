#include "stm32f4xx_it.h"
//#include "main.h"
#include <stdio.h>
//#include <string.h>//memcpy doesn't work

uint8_t aSrcBuffer[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
extern __IO uint8_t aDstBuffer[SIZE_BUF];
extern uint8_t aTxBuffer[SIZE_BUF];
extern __IO uint8_t BytesReceived;

uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s/startup_stm32f429x.s).    */
/******************************************************************************/
char cbuf[8]="\n\rHELP\n\r";
char rebuf[9]="\n\rRE %%\n\r";
uint8_t buf[SIZE_BUF];

void Send_nbyte(uint8_t* v,uint8_t n)
{//Hide
	//STM_EVAL_LEDToggle(LED3);
    u16 length;
    u8* ptx = aTxBuffer;//TX_BUF;
    //DMA_Cmd(DMA1_Stream1, DISABLE);//DMA_RX Channel disabled NOW//
    for(length=0;length<n; length++)
    	ptx[length]=v[length];
    DMA_SetCurrDataCounter(DMA1_Stream3,length);//DMA1_Channel4->CNDTR = length;
    DMA_Cmd(DMA1_Stream3, ENABLE);//DMA1_Channel4->CCR |= (u16)DMA_CCR4_EN;
	  /* Waiting the end of Data transfer */
	while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
	while (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)==RESET);

	  /* Clear DMA Transfer Complete Flags */
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
	  /* Clear USART Transfer Complete Flags */
	USART_ClearFlag(USART3,USART_FLAG_TC);
}

void UART_CMD_HANDLER(void)
{//Hide

    //CMD
#if 0
        /* Clear DMA Transfer Complete Flags */
  	    DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
#endif
#if 0
	if(Buffercmp8(aSrcBuffer, buf, 10)==1)
		STM_EVAL_LEDToggle(LED4);
	else
		STM_EVAL_LEDToggle(LED2);
#endif
	int i;
	for(i=0;i<BytesReceived;i++)
	{
		buf[i]=aDstBuffer[i];
	}

	//Send_nbyte(buf,BytesReceived);
	//__IO uint32_t nCount=1000;
	    //for(; nCount != 0; nCount--);
    if(buf[0]=='%' && buf[1]=='%')
    	Send_nbyte((uint8_t*)rebuf,9);
    else
    	Send_nbyte(buf,BytesReceived);

	//DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Stream1,SIZE_BUF);//DMA1_Channel5->CNDTR = (u16)0x00FF;     // Number of data to transfer = 255
    DMA_Cmd(DMA1_Stream1, ENABLE);
	USART3->CR1 |= (u16)USART_CR1_RE;       // USART3 receiver is enabled
	BytesReceived = 0; // Clear amount of bytes have been received
	//USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
}



void USART3_IRQHandler(void)
{
#if 0
	/* Waiting the end of Data transfer */
	  while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);

	  while (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF)==RESET);

	  /* Clear DMA Transfer Complete Flags */
	  DMA_ClearFlag(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF);
	  /* Clear USART Transfer Complete Flags */
	  USART_ClearFlag(USARTx,USART_FLAG_TC);

	  if (Buffercmp(aTxBuffer, aRxBuffer, BUFFERSIZE) != FAILED)
	  {
	    /* Turn ON LED2 */
	    STM_EVAL_LEDOn(LED2);
	  }
	  else
	  {
	    /* Turn ON LED3 */
	    STM_EVAL_LEDOn(LED3);
	  }
#endif
	uint16_t x,y;//,z;
    x = USART3->SR;
    x = USART3->DR; // Clear IDLE interrupt flag (and also others) by reading registers
	/* Clear DMA Transfer Complete Flags */
	//DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
	/* Clear USART Transfer Complete Flags */
	//USART_ClearFlag(USART3,USART_FLAG_TC);
	//USART_ClearFlag(USART3,USART_IT_IDLE);
    x = DMA_GetCurrDataCounter(DMA1_Stream1);//x = DMA1_Channel5->CNDTR; // Bytes remaining to receive
    //BytesReceived = SIZE_BUF-x;//SIZE_BUF - x; // Amount of bytes have been received
    //if (BytesReceived >0) // If nonzero amount of bytes have been received
    y=(SIZE_BUF-x);
    if ( y>0)
    {

        USART3->CR1 &= (u16)(~USART_CR1_RE); // USART3 receiver is disabled
        DMA_Cmd(DMA1_Stream1, DISABLE);//DMA_RX Channel disabled//DMA1_Channel5->CCR &= (u16)(~DMA_CCR5_EN)
        //USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);
#if 1
        /* Clear DMA Transfer Complete Flags */
  	    DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
#endif
  	    BytesReceived=y;
#if 0
        //CMD
#if 0
		if(Buffercmp8(aSrcBuffer, aDstBuffer, 10)==1)
			STM_EVAL_LEDToggle(LED4);
		else
			STM_EVAL_LEDToggle(LED2);
#endif
#if 1
		int i;
		for(i=0;i<y;i++)
		{
				buf[i]=aDstBuffer[i];
		}
#else
		memcpy(buf,aDstBuffer,y);
#endif
#if 1
	    if(buf[0]=='%' && buf[1]=='%')
	    	Send_nbyte((uint8_t*)rebuf,9);
#endif
	    Send_nbyte(buf,BytesReceived);
	    uint8_t len=sprintf((char *)buf,"\n\r%03u\n\r",BytesReceived);
	    //Send_nbyte(buf,5);
	    Send_nbyte(buf,len);
		DMA_SetCurrDataCounter(DMA1_Stream1,SIZE_BUF);//DMA1_Channel5->CNDTR = (u16)0x00FF;     // Number of data to transfer = 255
	    DMA_Cmd(DMA1_Stream1, ENABLE);
		USART3->CR1 |= (u16)USART_CR1_RE;       // USART3 receiver is enabled
		BytesReceived = 0; // Clear amount of bytes have been received
#endif

    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
