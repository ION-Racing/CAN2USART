#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"


void NVIC_Configuration(void);
void GPIO_Configuration(void);
void CAN_Config(void);
void USART_Configuration(void);
void USART1_IRQHandler(void);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);

int main(void)
{
	GPIO_Configuration();
	USART_Configuration();

	// NVIC
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable CAN1 RX0 interrupt IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	CAN_Config();


	CanTxMsg TxMessage;
	TxMessage.StdId = 0x321;
	TxMessage.ExtId = 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	while(1)
	{
	/* print welcome information */

	/*const unsigned char menu[] = "Hei\r";
	UARTSend(menu, sizeof(menu));*/


	/*TxMessage.Data[0] = 0x55;
	CAN_Transmit(CAN1, &TxMessage);*/

	volatile uint32_t i = 0xFFFF;
	while(i-- > 1);
	}
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
CanRxMsg RxMessage;

  RxMessage.StdId=0x00;
  RxMessage.ExtId=0x00;
  RxMessage.IDE=0;
  RxMessage.DLC=0;
  RxMessage.FMI=0;
  RxMessage.Data[0]=0x00;
  RxMessage.Data[1]=0x00;

  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

  uint8_t i;
  switch (RxMessage.StdId) {
	case 0x0FF:
		UART_SendByte('c');
		break;
	case 0x100:
		UART_SendByte('p');
		break;
	case 0x101:
		UART_SendByte('d');
		break;
	default:
		UART_SendByte('x');
		break;
  }
  for(i = 0; i<RxMessage.DLC; i++){
	  UART_SendByte(RxMessage.Data[i]);
  }
  UART_SendByte('\n');


}

void CAN_Config(void){

//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
//
//	 CAN_InitTypeDef        CAN_InitStructure;
//	  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//
//	/* CAN register init */
//	  CAN_DeInit(CAN1);
//
//
//	  CAN_StructInit(&CAN_InitStructure);
//
//	  /* CAN cell init */
//	  CAN_InitStructure.CAN_TTCM=DISABLE;
//	  CAN_InitStructure.CAN_ABOM=DISABLE;
//	  CAN_InitStructure.CAN_AWUM=DISABLE;
//	  CAN_InitStructure.CAN_NART=DISABLE;
//	  CAN_InitStructure.CAN_RFLM=DISABLE;
//	  CAN_InitStructure.CAN_TXFP=DISABLE;
//	  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
//	  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
//
//	  /* Baudrate = 500 Kbps */
//	  CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;
//	  CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
//	  CAN_InitStructure.CAN_Prescaler=12;
//	  CAN_Init(CAN1, &CAN_InitStructure);
//
//	  /* CAN filter init */
//	  CAN_FilterInitStructure.CAN_FilterNumber=1;
//	  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
//	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
//	  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
//	  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
//	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
//	  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
//	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
//	  CAN_FilterInitStructure.CAN_FilterActivation=DISABLE;
//	  CAN_FilterInit(&CAN_FilterInitStructure);
//
//	  /* CAN FIFO0 message pending interrupt enable */
//	  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN1 and CAN2 register init */
	CAN_DeInit(CAN1);

	/* Struct init*/
	CAN_StructInit(&CAN_InitStructure);

	/* CAN1 and CAN2  cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;

	CAN_InitStructure.CAN_Prescaler =12; // 500 kbps


	/*Initializes the CAN1  and CAN2 */
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN1 filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 1;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);


	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}


void GPIO_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// USART
	// PA9:  TX
	// PA10: RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	// CAN
	// PB8: RX
	// PB9: TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
}

void USART_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void UART_SendByte(char byte){
	USART_SendData(USART1, byte);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

