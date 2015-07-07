#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include "ION_CAN.h"
#include "RPI_MSG.h"

void NVIC_Configuration(void);
void GPIO_Configuration(void);
void CAN_Config(void);
void USART_Configuration(void);

int main(void)
{
	GPIO_Configuration();
	USART_Configuration();
	CAN_Config();

	while(1);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	/* ECU */
	// Speed
	if(RxMessage.StdId == CAN_MSG_SPEED){
		RPI_Send(RPI_TX_SPEED,  RxMessage.Data[0], RxMessage.Data[1]);
	}
	else if(RxMessage.StdId == CAN_ERR_STARTUP){
		RPI_Send(RPI_TX_ERR_STARTUP,  RxMessage.Data[0], 0x00);
	}

	/* BMS */
	// State of charge
	else if(RxMessage.StdId == CAN_MSG_BMS_CHARGE){
		RPI_Send(RPI_TX_SOC,  RxMessage.Data[0]);
	}

	// Voltage
	else if(RxMessage.StdId == CAN_MSG_BMS_VOLTAGE){
		RPI_Send(RPI_TX_VOLTAGE,  RxMessage.Data[0]);
	}

	/* Pedals */
	// Pedal values
	else if(RxMessage.StdId == CAN_MSG_PEDALS){
		RPI_Send(RPI_TX_TORQUE_PEDAL, RxMessage.Data[0], RxMessage.Data[1]);
		RPI_Send(RPI_TX_BRAKE_PEDAL, RxMessage.Data[2], RxMessage.Data[3]);
	}

	else if(RxMessage.StdId == CAN_ERR_PEDALS_IMPLAUSIBILITY){
		RPI_Send(RPI_TX_ERR_PEDAL, RxMessage.Data[0], RxMessage.Data[1]);
	}
}

void RPI_Send(uint8_t address, uint8_t data1, uint8_t data2){
	UART_SendByte(0xFF);
	UART_SendByte(0xFF);
	UART_SendByte(address);
	UART_SendByte(data2);
	UART_SendByte(data1);
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

void CAN_Config(void){

	// NVIC
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable CAN1 RX0 interrupt IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	// CAN1-module
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	CAN_DeInit(CAN1);

	CAN_StructInit(&CAN_InitStructure);

	// Cell Init
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

	CAN_InitStructure.CAN_Prescaler = 12; // 500 kbps

	CAN_Init(CAN1, &CAN_InitStructure);

	// Filter init
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

	// Enable CAN1 FIFO0 Message pending interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

uint8_t CANTx(uint32_t address, uint8_t length, uint8_t data[8]) {

	CanTxMsg msg;
	msg.StdId 	= address;
	msg.IDE 	= CAN_Id_Standard;
	msg.RTR		= CAN_RTR_Data;
	msg.DLC		= length;

	uint8_t i = 0;
	for(i=0; i<length; i++){
		msg.Data[i] = data[i];
	}

	return CAN_Transmit(CAN1, &msg);
}

void USART_Configuration(void)
{
    // NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

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

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void UART_SendByte(char byte){
	USART_SendData(USART1, byte);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

uint8_t buffer[200];
uint8_t readingAddress = 0;
uint8_t state = 0;
uint8_t current = 0;


void USART1_IRQHandler(void){
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
		uint8_t data = USART_ReceiveData(USART1);
		switch (state) {
			case 0:
				if(data == 0xFF) state++;
				break;
			case 1:
				if(data == 0xFF) state++;
				else state = 0;
				break;
			case 2:
				readingAddress = data;
				state++;
				break;
			case 3:
				buffer[current++] = data;
				if(current >= 2){
					state = 0;
					readingAddress = 0;
					current = 0;

					RxMessage(readingAddress, buffer[1], buffer[0]);
				}
				break;
			default:
				break;
		}
	}
}

void RxMessage(uint8_t address, uint8_t data1, uint8_t data2)
{
	if(address == RPI_RX_PEDALS_CALIBRATE){
		uint8_t data[8];
		data[0] = data2;

		CANTx(CAN_MSG_PEDALS_CALIBRATE, 1, data);
	}
}

