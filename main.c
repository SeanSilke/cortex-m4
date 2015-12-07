//#include "main.h"
#include "stm32f4xx_it.h"
#include "stm324xg_eval.h"


DMA_InitTypeDef  DMA_InitStructure;
__IO uint8_t BytesReceived=0;



__IO uint8_t      aDstBuffer[SIZE_BUF];//= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t      aTxBuffer[SIZE_BUF];//= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Private function prototypes -----------------------------------------------*/
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
#ifdef USE_STM324xG_EVAL
  #error "asdf"
  STM_EVAL_LEDInit(LED4);
#endif
  //STM_EVAL_PBInit(BUTTON, BUTTON_MODE_EXTI);

  InitClock();
  /* EVAL_COM1 (USART3) configuration --------------------------------------*/
  USART_Config();
  /* DMA1/2 Stream1/5 channel4 (connected to USART3_RX/USART1_RX) configuration */
  DMA_Config();

  //USART3->CR1 |= (u16)USART_CR1_RE;       // USART3 receiver is enabled RX ON


  /* Enable the USARTx Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;//
#ifdef USE_STM324xG_EVAL
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
#else
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
#endif
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  while (1)
  {
    if(BytesReceived>0) 	UART_CMD_HANDLER();
    Delay(0xFFFFF);
    STM_EVAL_LEDToggle(LED1);
    Delay(0xFFFFF);
    Send_nbyte("+",1);

  }

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

void _init(void) {
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
