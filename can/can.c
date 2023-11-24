/*
  **********************************************************************************
  * @title		can.c
  * @platform	STM32F103
  * @author		Anton Novikov
  * @version	V1.0.0
  * @date		07.09.2017
  *
  * @brief		Initializes CAN communication, configures GPIOs, and sets up CAN filters.
  *         	Configures CAN interrupts for handling message reception.
  *
  **********************************************************************************
*/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_can.h"

#include "misc.h"
#include "can.h"
#include "ssd1306.h"

#include <stdio.h>
#include <stdlib.h>

// Macro for checking the value of a bit at a specified position (rightmost)
#define GETBIT(x,pos)( ((x) & ( 0x1 << (pos) )) !=0 )

// Ignition states, Std CAN ID
const uint16_t ignitionStdID = 0x050;			// CAN ID of the status ignition
int ignitionStd = 0;

// Coolant Temperature states, Std CAN ID
const uint16_t CoolantTempStdID = 0x420;		// CAN ID of the Instrument Cluster
int CoolantTemp = 0;
int CoolantTempCheck = 0;

/* This function is designed for configuring I/O ports for STM32F103 series microcontrollers. */
void init_CAN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* CAN GPIOs configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 		// Enable AFIO clock
	RCC_APB2PeriphClockCmd(CAN1_Periph, ENABLE); 				// Enable port clock

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);		// Enable CAN bus clock

	// Configure CAN RX pin
	GPIO_InitStructure.GPIO_Pin   = CAN1_RX_SOURCE;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

	// Configure CAN TX pin
	GPIO_InitStructure.GPIO_Pin   = CAN1_TX_SOURCE;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

	#ifdef CAN1_ReMap
		GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);			// Remap Can1 to PB8, PB9
	#endif

	// Initialize CAN bus
	CAN_InitTypeDef CAN_InitStructure;

	CAN_DeInit( CAN1);
	CAN_StructInit(&CAN_InitStructure);

	// CAN cell initialization
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;				// Normal mode of operation
//	CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;				// For testing without connected bus devices
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	//3tq - 500 Kb; 4tq - 1000 Kb; 4tq - 250 Kb; 3tq - 125 Kb; 4tq - 100 Kb; 4tq - 50 Kb;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	//4tq - 500 Kb; 4tq - 1000 Kb; 4tq - 250 Kb; 4tq - 125 Kb; 4tq - 100 Kb; 4tq - 50 Kb;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler = CAN1_SPEED_PRESCALE;		// Choose the required speed from can.h
	CAN_Init(CAN1, &CAN_InitStructure);

	// CAN filter initialization
	// When configuring filters, pay attention to the filter numbers, otherwise you may overwrite them
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

    // Two filter blocks
    // Filter that passes all messages according to the mask for a standard frame (scaling by 32 bits)
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                        	// Filter number, available from 0 to 13
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      	// Filter mode, Mask
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;    	// Scaling
    CAN_FilterInitStructure.CAN_FilterIdHigh = ignitionStdID<<5;       	 	// High part of the filter
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                   	// Low part of the filter
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ignitionStdID<<5;   		// High part of the mask
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               	// Low part of the mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;       	// FIFO buffer number (we have only two)
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              	// Filter activation
    CAN_FilterInit(&CAN_FilterInitStructure);

    // Filter that passes all messages according to the mask for a standard frame
    CAN_FilterInitStructure.CAN_FilterNumber = 1;                        	// Filter number, available from 0 to 13
    CAN_FilterInitStructure.CAN_FilterIdHigh = CoolantTempStdID<<5;     	// High part of the filter
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CoolantTempStdID<<5; 	// High part of the mask
    CAN_FilterInit(&CAN_FilterInitStructure);

    // NVIC Configuration
    // Setting up an interrupt for handling FIFO0 buffer
    // Enable CAN1 RX0 interrupt IRQ channel
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// Enable CAN FIFO0 message pending interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	// Setting up an interrupt for handling FIFO1 buffer
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable CAN FIFO1 message pending interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
}

/* CAN bus interrupt handler for buffer FIFO0 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	// Create a variable RxMessage of type CanRxMsg to store received data.
	CanRxMsg RxMessage;

	// Initialize the values of RxMessage fields before receiving data.
	RxMessage.DLC = 	0x00;
	RxMessage.ExtId = 	0x00;
	RxMessage.FMI = 	0x00;
	RxMessage.IDE = 	0x00;
	RxMessage.RTR = 	0x00;
	RxMessage.StdId = 	0x00;
	RxMessage.Data [0] = 0x00;
	RxMessage.Data [1] = 0x00;
	RxMessage.Data [2] = 0x00;
	RxMessage.Data [3] = 0x00;
	RxMessage.Data [4] = 0x00;
	RxMessage.Data [5] = 0x00;
	RxMessage.Data [6] = 0x00;
	RxMessage.Data [7] = 0x00;

	// Check for the presence of CAN1 FIFO0 interrupt.
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		// Get actual reception of data from FIFO0.
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		// Check if the received frame is an standard frame.
		// But there should not be any others, because we have configured the reception of a standard frame into the FIFO0 buffer.
		if (RxMessage.IDE == CAN_Id_Standard)
		{
			// Ignition status
			if (RxMessage.StdId == ignitionStdID) {
				// Check if the second bit in the first byte is set
				//if (GETBIT(RxMessage.Data[0], 1)) {
				// The state of this byte does not change when the ignition is on, so there is no need to check bit by bit.
				if (RxMessage.Data[0] == 0x02) {
				ignitionStd = 1;
				} else {
				ignitionStd = 0;
				}
			}
			// Coolant temp status
			if (RxMessage.StdId == CoolantTempStdID)
			{
				CoolantTemp = RxMessage.Data[0];
				ulong CoolantTemp = CoolantTemp;
				CoolantTempCheck = 1;
			} else {
				CoolantTempCheck = 0;
			}
		}
	}
}

/* CAN bus interrupt handler for buffer FIFO1 */
void CAN1_RX1_IRQHandler(void)
{
	// Create a variable RxMessage of type CanRxMsg to store received data.
	CanRxMsg RxMessage;

	// Initialize the values of RxMessage fields before receiving data.
	RxMessage.DLC = 	0x00;
	RxMessage.ExtId = 	0x00;
	RxMessage.FMI = 	0x00;
	RxMessage.IDE = 	0x00;
	RxMessage.RTR = 	0x00;
	RxMessage.StdId = 	0x00;
	RxMessage.Data [0] = 0x00;
	RxMessage.Data [1] = 0x00;
	RxMessage.Data [2] = 0x00;
	RxMessage.Data [3] = 0x00;
	RxMessage.Data [4] = 0x00;
	RxMessage.Data [5] = 0x00;
	RxMessage.Data [6] = 0x00;
	RxMessage.Data [7] = 0x00;

	// Check for the presence of CAN1 FIFO1 interrupt.
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP1) != RESET)
	{
		// Get actual reception of data from FIFO1.
		CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);

		// Check if the received frame is an extended frame.
		if (RxMessage.IDE == CAN_Id_Extended)
		{
			// Additional actions that can be performed if the frame is extended.
		}
	}
}
