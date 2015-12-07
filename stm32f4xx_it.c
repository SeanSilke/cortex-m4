#include "stm32f4xx_it.h"
//#include "main.h"
#include <stdio.h>
//#include <string.h>//memcpy doesn't work

//#define USART_DR_ADDRESS   ((uint32_t)0x40004804)
   /* Definition for USARTx resources ******************************************/
#ifdef USE_STM324xG_EVAL
#define USARTx                           USART3
#define USARTx_CLK                       RCC_APB1Periph_USART3
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_10
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTx_TX_SOURCE                 GPIO_PinSource10
#define USARTx_TX_AF                     GPIO_AF_USART3

#define USARTx_RX_PIN                    GPIO_Pin_11
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTx_RX_SOURCE                 GPIO_PinSource11
#define USARTx_RX_AF                     GPIO_AF_USART3

/* Definition for DMAx resources ********************************************/
//#define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04)

#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
#define USARTx_DMAx_CLK_INIT             RCC_AHB1PeriphClockCmd

#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream3
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3

#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream1
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1
#else
#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_USART1

#define USARTx_RX_PIN                    GPIO_Pin_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_USART1

/* Definition for DMAx resources ********************************************/
//#define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04)

#define USARTx_DMA                       DMA2
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA2
#define USARTx_DMAx_CLK_INIT             RCC_AHB1PeriphClockCmd

#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF7
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF7
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF7
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF7
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF7

#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream2
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF2
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF2
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF2
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF2
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF2
#endif //USE_STM324xG_EVAL
#if 0
#define USARTx_DMA_TX_IRQn               DMA1_Stream3_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler
#endif

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

char cbuf[8]="\n\rHELP\n\r";
char rebuf[9]="\n\rRE %%\n\r";
uint8_t buf[SIZE_BUF];

void InitClock()
{
	  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK, ENABLE);
	  RCC_AHB1PeriphClockCmd(USARTx_RX_GPIO_CLK, ENABLE);
#if RTS_CTS
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
#endif

	  /* Enable USART clock */
	  USARTx_CLK_INIT(USARTx_CLK, ENABLE);

	  /* Enable the DMA clock */
	  USARTx_DMAx_CLK_INIT(USARTx_DMAx_CLK, ENABLE);
}

void USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /*INIT USARTx GPIO configuration -----------------------------------------------*/
  ///* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//GPIO_Mode_IN -does'n work;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

#if RTS_CTS
  GPIO_PinAFConfig(GPIOD, GPIO_Pin_11, GPIO_AF_USART3);               // CTS
  GPIO_PinAFConfig(GPIOD, GPIO_Pin_12, GPIO_AF_USART3);               // RTS


 //enable GPIOD
 //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//CTS_1_Pin ; // CTS pin
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
   GPIO_Init(GPIOD, &GPIO_InitStructure); /// GPIO D

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//RTS_1_Pin  ; // RTS pin
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

  USART_InitTypeDef  USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;//115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//;//USART_HardwareFlowControl_RTS_CTS;// ;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USARTx, &USART_InitStructure);

  //Note: The IDLE bit will not be set again until the RXNE bit has been set itself (i.e. a new idle
  //line occurs).
  //USART_ITConfig(USARTx, USART_IT_TC, DISABLE);
  //USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);

  USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);

  /* Enable DMA Rx/TX request */
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}


void DMA_Config(void)
{
  DMA_InitTypeDef  DMA_InitStructure;

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(USARTx->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_BufferSize = SIZE_BUF;//BUFFERSIZE ;
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer ;
  DMA_Init(USARTx_TX_DMA_STREAM,&DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_BufferSize = SIZE_BUF;//BUFFERSIZE ;
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aDstBuffer ;
  DMA_Init(USARTx_RX_DMA_STREAM,&DMA_InitStructure);

  /* Enable DMA_RX(DMA1_Stream1) ONLY! */
  DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);
 }

void Send_nbyte(uint8_t* v,uint8_t n)
{//Hide
	//STM_EVAL_LEDToggle(LED3);
    u16 length;
    u8* ptx = aTxBuffer;//TX_BUF;
    //DMA_Cmd(DMA1_Stream1, DISABLE);//DMA_RX Channel disabled NOW//
    for(length=0;length<n; length++)
    	ptx[length]=v[length];
    USARTx_TX_DMA_STREAM->NDTR=length;//DMA_SetCurrDataCounter(DMA1_Stream3,length);
    USARTx_TX_DMA_STREAM->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA1_Stream3, ENABLE);
	  /* Waiting the end of Data transfer */
    while((USARTx->SR & USART_FLAG_TC) == (uint16_t)RESET);//while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
    //if USARTx_TX_DMA_STREAM 1,2,3   then USARTx_DMA->LISR
    //if USARTx_TX_DMA_STREAM 4,5,6,7 then USARTx_DMA->HISR
#ifdef USE_STM324xG_EVAL
    while((USARTx_DMA->LISR & USARTx_TX_DMA_FLAG_TCIF) == (uint32_t)RESET);//while (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)==RESET);
	/* Clear DMA1 Transfer Complete Flags */
    //if USARTx_TX_DMA_STREAM 1,2,3   then USARTx_DMA->LISR
    //if USARTx_TX_DMA_STREAM 4,5,6,7 then USARTx_DMA->HISR
    //RESERVED_MASK (uint32_t)0x0F7D0F7D
    USARTx_DMA->LIFCR = (uint32_t)(USARTx_TX_DMA_FLAG_TCIF & (uint32_t)0x0F7D0F7D);//DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
	  /* Clear USART Transfer Complete Flags */
#else //USE_STM324xG_EVAL
    while((USARTx_DMA->HISR & USARTx_TX_DMA_FLAG_TCIF) == (uint32_t)RESET);//while (DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)==RESET);
    USARTx_DMA->HIFCR = (uint32_t)(USARTx_TX_DMA_FLAG_TCIF & (uint32_t)0x0F7D0F7D);// DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);
#endif //USE_STM324xG_EVAL
    USARTx->SR = (uint16_t)~USART_FLAG_TC;//USART_ClearFlag(USART3,USART_FLAG_TC);
}

void UART_CMD_HANDLER(void)
{//Hide
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

	// DMA_Cmd(DMA1_Stream3, DISABLE);
    // Number of data to transfer = 255
    USARTx_RX_DMA_STREAM->NDTR = (uint16_t)SIZE_BUF;//DMA_SetCurrDataCounter(DMA1_Stream1,SIZE_BUF);
    USARTx_RX_DMA_STREAM->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA1_Stream1, ENABLE);
    USARTx->CR1 |= (u16)USART_CR1_RE;       // USART3 receiver is enabled
	BytesReceived = 0; // Clear amount of bytes have been received
	//USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
}



void USART3_IRQHandler(void)
{
	uint16_t x,y;//,z;
    x = USARTx->SR;
    x = USARTx->DR; // Clear IDLE interrupt flag (and also others) by reading registers
    x = ((uint16_t)(USARTx_RX_DMA_STREAM->NDTR));//DMA_GetCurrDataCounter(DMA1_Stream1);// Bytes remaining to receive
    y=(SIZE_BUF-x);
    if ( y>0)
    {

        USARTx->CR1 &= (u16)(~USART_CR1_RE); // USARTx receiver is disabled
        USARTx_RX_DMA_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA1_Stream1, DISABLE);//DMA_RX Channel disabled
#if 1
        /* Clear DMA1 Transfer Complete Flags */
        //if USARTx_TX_DMA_STREAM 1,2,3   then USARTx_DMA->LISR
        //if USARTx_TX_DMA_STREAM 4,5,6,7 then USARTx_DMA->HISR
        USARTx_DMA->LIFCR = (uint32_t)(USARTx_RX_DMA_FLAG_TCIF & (uint32_t)0x0F7D0F7D);//DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
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
		USARTx->CR1 |= (u16)USART_CR1_RE;       // USARTx receiver is enabled
		BytesReceived = 0; // Clear amount of bytes have been received
#endif

    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
