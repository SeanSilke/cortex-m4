//#include "main.h"
#include "stm32f4xx_it.h"
#include "stm324xg_eval.h"

#define RCC_AHB1Periph_DMA RCC_AHB1Periph_DMA1
#define USART_DR_ADDRESS   ((uint32_t)0x40004804)
   /* Definition for USARTx resources ******************************************/
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
#define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04)

#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1

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

#define USARTx_DMA_TX_IRQn               DMA1_Stream3_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler

DMA_InitTypeDef  DMA_InitStructure;
__IO uint8_t BytesReceived=0;



__IO uint8_t      aDstBuffer[SIZE_BUF];//= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t      aTxBuffer[SIZE_BUF];//= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Private function prototypes -----------------------------------------------*/
static void DMA_Config(void);
static void USART_Config(void);
static void Delay(__IO uint32_t nCount);

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       files startup_stm32f40_41xxx.s
       before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

  /* Initialize Leds and Key Button mounted on EVAL board */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);

  //STM_EVAL_PBInit(BUTTON, BUTTON_MODE_EXTI);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
#if RTS_CTS
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
#endif

  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);

  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);

  /* Enable DMA1 clock ********************************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA, ENABLE);

  /* EVAL_COM1 (USART3) configuration --------------------------------------*/
  USART_Config();
  /* DMA1/2 Stream1/5 channel4 (connected to USART3_RX/USART1_RX) configuration */
  DMA_Config();


  //USART3->CR1 |= (u16)USART_CR1_RE;       // USART3 receiver is enabled RX ON

  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable Idle line detected USART */

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  while (1)
  {
    if(BytesReceived>0) 	UART_CMD_HANDLER();
    Delay(0xFFFFF);
    STM_EVAL_LEDToggle(LED1);

  }

}

static void USART_Config(void)
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
  USART_DMACmd(EVAL_COM1, USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(EVAL_COM1, USART_DMAReq_Tx, ENABLE);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}


static void DMA_Config(void)
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

static void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

void exit(int arg) {
    while (arg == arg);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
