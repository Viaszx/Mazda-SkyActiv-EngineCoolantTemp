/*
  **********************************************************************************
  * @title		main.c
  * @platform	STM32F103
  * @author		Anton Novikov
  * @version	V1.0.0
  * @date		07.09.2017
  *
  * @brief		Manages display updates based on the ignition signal and shows Engine Coolant Temperature (ECT),
  * 			and includes timers for future use (timer_1000ms and timer_10000x100ms are currently inactive).
  *
  **********************************************************************************
*/

#include "stm32f10x.h"  		// Include header file for STM32F10x microcontroller.
#include "stm32f10x_gpio.h"  	// Include header file for GPIO operations.
#include "stm32f10x_can.h"  	// Include header file for CAN operations.
#include "stm32f10x_rcc.h"  	// Include header file for clock control operations.
#include "stm32f10x_tim.h"  	// Include header file for timer operations.

#include "misc.h"  		// Include miscellaneous header file.
#include "can.h"  		// Include header file for CAN operations.
#include "ssd1306.h"  	// Include header file for OLED display operations.

#include <stdbool.h>  	// Include header file for boolean values.
#include <stdio.h>  	// Include standard input/output header file.
#include <stdlib.h>  	// Include header file for general-purpose functions.

void init_timer();  	// Function prototype for timer initialization.
void res_timer(int timer_num, int timer_type);  	// Function prototype for timer reset.

// Definition of tpTimer structure
typedef struct
  {
	bool run;  	// Timer running flag.
	bool res;	// Timer reset flag.
	int val;	// Current timer value.
  } tpTimer;

// Array of timers with 1000 ms interval
tpTimer timer_1000ms[10] = {{false, false, 0}};			// Array of timers with a 1000 ms interval.
int time_togle_1ms = 0;  								// Flag toggled every millisecond.
int time_togle_1ms_old = 0;								// Previous value of the toggle flag.

// Array of timers with 10000 x 100 ms interval
tpTimer timer_10000x100ms[10] = {{false, false, 0}};	// Array of timers with a 10000 x 100 ms interval.
int time_togle_100ms = 0;  								// Flag toggled every 100 milliseconds.
int time_togle_100ms_old = 0;							// Previous value of the toggle flag.

void SSD1306_ON(void);
uint8_t ssd1306_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address);

// Program execution delay.
// The function uses a variable of volatile type to prevent compiler optimization.
void Delay(void) {
  volatile uint32_t i;
  for (i=0; i != 0x800000; i++);
}

int main(void)
{
	// Create a structure of type GPIO_InitTypeDef to configure GPIO parameters.
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable the clock for GPIO port C to make it usable.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Configure parameters for GPIO_Pin_13 in the GPIO_InitStructure structure.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				// Select pin 13 on GPIO port C.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		// Set the pin to operate in output mode with push-pull configuration.
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// Set the operating speed of the port.

	// Apply the configured settings to GPIO port C using the specified structure
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Enable clock for AFIO (Alternative Function I/O)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	// Remap SWJ_JTAG, in this case, disable JTAG on PB3.
	// After this, PB3 can be used as a GPIO pin
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// Current menu item. Default is 1
	int MenuPosition = 1;

	int NeedUpdate = 1;			// Flag indicating if LCD update is needed.
	int DelayUpdate = 0;		// Delay counter for LCD update.

	SystemInit();				// Initialize system settings.
	void SSD1306_ON();			// Turn on SSD1306 display.
	SSD1306_Init();				// Initialize SSD1306 display.
	init_timer();				// Initialize timers.
	init_CAN();					// Initialize CAN communication.

	SSD1306_GotoXY(12, 33);		// Set cursor position on SSD1306 display.
	SSD1306_Puts("MAZDA CX-5", &Font_11x18, SSD1306_COLOR_WHITE);		// Display text on SSD1306.
	SSD1306_GotoXY(40, 53);
	SSD1306_Puts("LOADING", &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();												// Update SSD1306 display.
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);									// Reset GPIO pin 13.
	Delay();
	GPIO_SetBits(GPIOC, GPIO_Pin_13);									// Set GPIO pin 13.

	while (1) {
	if (ignitionStd != 0) {
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			MenuPosition = 1;
		} else {
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			MenuPosition = 2;
		}

	// Check if ignition signal is present and DelayUpdate is more than 8000.
	if ((ignitionStd !=0) && (DelayUpdate < 8000)) {
		DelayUpdate ++;
	} else {
		NeedUpdate = 1;
		DelayUpdate = 0;
	}
		/* Timer management */
		// time_togle_1ms flag is toggled each 1ms
		// Each timer counts up to 10 seconds
		if (time_togle_1ms != time_togle_1ms_old) {
			for (int i = 0; i < 10; i++) {
				if (timer_1000ms[i].run && timer_1000ms[i].val < 10000) {
					timer_1000ms[i].val++;
				}
			}
			time_togle_1ms_old = time_togle_1ms;
		}
		// time_togle_100ms flag is toggled each 100ms
		// Each timer counts up to 100 seconds
		if (time_togle_100ms != time_togle_100ms_old) {
			for (int i = 0; i < 10; i++) {
				if (timer_10000x100ms[i].run
						&& timer_10000x100ms[i].val < 10000) {
					timer_10000x100ms[i].val++;
				}
			}
			time_togle_100ms_old = time_togle_100ms;
		}
		// Buffer to store characters, used for converting numbers to strings.
		char buf[32];

		// Constant representing a temperature value
		ulong constTemp = 40;

		switch (MenuPosition) {
		case 1:
			if (NeedUpdate) {
				SSD1306_Fill(SSD1306_COLOR_BLACK);							// Clear the OLED display.
				SSD1306_GotoXY(0, 36);										// Set the cursor position and display "ECT:" on the OLED display.
				SSD1306_Puts("ECT: ", &Font_16x26, SSD1306_COLOR_WHITE);
				itoa((CoolantTemp - constTemp), buf, 10);					// Convert the difference between CoolantTemp and constTemp to a string and display it.
				SSD1306_GotoXY(70, 37);
				SSD1306_Puts(buf, &Font_16x26, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen();
				// Do not redraw if the menu item has not changed
				NeedUpdate = 0;
			}
			break;

		case 2:
			if (NeedUpdate) {
				SSD1306_Fill(SSD1306_COLOR_BLACK);
				SSD1306_GotoXY(12, 33);
				SSD1306_Puts("MAZDA CX-5", &Font_11x18, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(40, 53);
				SSD1306_Puts("DIESEL", &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen();
				// Do not redraw if the menu item has not changed
				NeedUpdate = 0;
			}
			break;
		}
    }
}

void res_timer(int timer_num, int timer_type) {
	// Reset the specified timer based on the timer type
	if (timer_type == 1) {
		// Reset 1000ms timer
		if (timer_num >= 0 && timer_num < 10) {
			timer_1000ms[timer_num].res = false;
			timer_1000ms[timer_num].val = 0;
		}
	} else if (timer_type == 100) {
		// Reset 10000x100ms timer
		if (timer_num >= 0 && timer_num < 10) {
			timer_10000x100ms[timer_num].res = false;
			timer_10000x100ms[timer_num].val = 0;
		}
	}
}

void init_timer() {
	// Initialize timers TIM3 and TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);

	base_timer.TIM_Prescaler = (SystemCoreClock / 10000) - 1;

	// Configure TIM3 with a period of 10
	base_timer.TIM_Period = 10;
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &base_timer);

	// Configure TIM4 with a period of 1000
	base_timer.TIM_Period = 1000;
	base_timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &base_timer);

	// Enable TIM3 and TIM4 update interrupts
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	// Enable TIM3 and TIM4
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	// Configure NVIC for TIM3
	NVIC_InitTypeDef NVIC_InitStructure3;
	NVIC_InitStructure3.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure3.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure3.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure3.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure3);

	// Configure NVIC for TIM4
	NVIC_InitTypeDef NVIC_InitStructure4;
	NVIC_InitStructure4.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure4.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure4.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure4.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure4);
}

void TIM3_IRQHandler() {
	// Handle TIM3 interrupt
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		// Toggle the 1ms flag
		if (time_togle_1ms == 0) {
			time_togle_1ms = 1;
		} else {
			time_togle_1ms = 0;
		}
	}
}

void TIM4_IRQHandler() {
	// Handle TIM4 interrupt
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		// Toggle the 100ms flag
		if (time_togle_100ms == 0) {
			time_togle_100ms = 1;
		} else {
			time_togle_100ms = 0;
		}
	}
}
