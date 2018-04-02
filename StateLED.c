/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    StateLED.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "DataTypeDefinitions.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
typedef enum {AMARILLO,/*!< Definition to configure a pin as input */
			  ROJO,
			  MORADO,
			  AZUL,
			  VERDE/*!< Definition to configure a pin as output */
			 }ColorLED;

typedef enum {ON,/*!< Definition to configure a pin as input */
			  OFF
			 }ColorWhite;

typedef struct
{
	uint8 white[2];
	uint8 next[5];
}StateType;

const StateType FSM_Moore[2]=
		{
				{{ON, OFF},{AMARILLO,ROJO, MORADO, AZUL, VERDE}}, /**Even*/
				{{ON, OFF},{VERDE,AZUL, MORADO, ROJO, AMARILLO}}  /**Odd*/
		};
int main(void) {
	uint8 On;
	uint8 Move=0;
	uint8 currentState;
	uint8 flag;
	uint8 position=0;
	uint8 inputValue1;
	uint8 inputValue2;
	/**********************************************************************************/
		SIM->SCGC5 = 0x2E00;
		/**Pin control configuration of GPIOB pin22 and pin21 and GPIOA pin4 as GPIO*/
		PORTB->PCR[21] = 0x00000100;
		PORTB->PCR[22] = 0x00000100;

		/**Pin control configuration of GPIOC pin6 as GPIO with is pull-up resistor enabled*/
		PORTC->PCR[6] = 0x00000103;
		PORTA->PCR[4]  =0X000000103;
		/**Pin control configuration of GPIOE pin26 as GPIO*/
		PORTE->PCR[26] = 0x00000100;
		/**Assigns a safe value to the output pin21 of the GPIOB*/
		GPIOB->PDOR = 0x00200000;
		/**Assigns a safe value to the output pin22 of the GPIOB*/
		GPIOB->PDOR |= 0x00400000;
		/**Assigns a safe value to the output pin26 of the GPIOE*/
		GPIOE->PDOR |= 0x04000000;
		GPIOC->PDDR &=~(0x40);
		GPIOA->PDDR &=~(0x10);
		/**Configures GPIOB pin21 as output*/
		GPIOB->PDDR = 0x00200000;
		/**Configures GPIOB pin22 as output*/
		GPIOB->PDDR |= 0x00400000;
		/**Configures GPIOE pin26 as output*/
		GPIOE->PDDR |= 0x04000000;
		/**********************************************************************************/
		while(TRUE){
	    /**Reads all the GPIOC*/
		inputValue1 = GPIOC->PDIR;
		inputValue2 = GPIOA->PDIR;
		/**Masks the GPIOC in the bit of interest*/
		inputValue1 = inputValue1 & 0x40;
		inputValue2 = inputValue2 & 0X10;

		while(FALSE == inputValue1 && FALSE == inputValue2)
				{
					On=0;
					currentState = FSM_Moore[Move].white[On];
					GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
					GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
					GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
					flag = 1;
					inputValue1 = GPIOC->PDIR;
					inputValue2 = GPIOA->PDIR;
					inputValue1 = inputValue1 & 0x40;
					inputValue2 = inputValue2 & 0X10;
				}

				if(FALSE == inputValue1)
				{
					Move=0;
					if(0 == flag)
						position +=1;
					currentState = FSM_Moore[Move].next[position];
					if (5 == position)
						position=0;
					currentState = FSM_Moore[Move].next[position];
					flag = 1;
				}

				else if(FALSE == inputValue2)
				{
					if(0 == flag)
						position -=1;
					currentState = FSM_Moore[Move].next[position];
					if(255 == position)
						position=4;
					currentState = FSM_Moore[Move].next[position];
					flag = 1;
				}
				else{
				flag = 0;
			}

				switch (position)
						{
						case 0:							/**Green*/

							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
							break;
						case 1:							/**Blue*/
							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
							break;
						case 2:
							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
							GPIOB->PDOR &= ~(0x00400000);/**Read led on*/
							break;
						case 3:
							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOB->PDOR &= ~(0x00400000);/**Read led on*/
							break;
						case 4:
							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
							GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
							break;
						default:
							GPIOB->PDOR |= 0x00400000;/**Red led off*/
							GPIOE->PDOR |= 0x4000000;/**Green led off*/
							GPIOB->PDOR |= 0x00200000;/**Blue led off*/
							GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
							break;
						}

		/*
				else if(FALSE == inputValue1)
				{
					position= position+1;
					Move=0;
					currentState = FSM_MOORE[Move].next[position];
					if(0 == flag)
						contador +=1;
					if (5 == contador)
						contador=0;
					flag = 1;
				}
				else if(FALSE == inputValue2)
				{
					if(0 == flag)
						contador -=1;
					if(255 == contador)
						contador=4;
					flag = 1;
				}
				else{
				flag = 0;
			}*/
		}
    return 0 ;
}
