/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include <stm32f4xx.h>
//#include <core_cm4.h>
/*SysTick Control/ Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos_1		16U											/*!< SysTick CTRL: COUNTFLAG Position*/
#define SysTick_CTRL_COUNTFLAG_Msk_1		(1UL << SysTick_CTRL_COUNTFLAG_Pos_1)		/*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos_1		2U											/*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk_1		(1UL << SysTick_CTRL_CLKSOURCE_Pos_1)		/*!< SysTick CTRL: CLKSOURCE Mask*/

#define SysTick_CTRL_TICKINT_Pos_1			1U											/*!< SysTick CTRL: TICKINT Position*/
#define SysTick_CTRL_TICKINT_Msk_1			(1UL << SysTick_CTRL_TICKINT_Pos_1)				/*!< SysTick CTRL: TICKINT Mask  */

#define SysTick_CTRL_ENABLE_Pos_1			  0U											/*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk_1			  (1UL << SysTick_CTRL_ENABLE_Pos_1)			/*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions*/
#define SysTick_LOAD_RELOAD_Pos_1			  0U
#define SysTick_LOAD_RELOAD_Msk_1			  (0xFFFFFFUL)

#define SysTick_VAL_CURRENT_Pos_1			  0U
#define SysTick_VAL_CURRENT_Msk_1			  (0xFFFFFFUL)
#define HAL_MAX_DELAY_1						      (0xFFFFFFFFUL)

uint32_t SystemCoreClock_1 = 16000000;
uint32_t SysTick_Config1(uint32_t ticks);
volatile uint32_t uwTick_1;
volatile uint32_t uwTickFreq_1 = 1U; // 1KHZ
static int test;
void SysTick_Handler1(void);

void SysTick_Handler1(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  //HAL_IncTick();
	uwTick_1 += uwTickFreq_1;
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
uint32_t SysTick_Config1(uint32_t ticks)
{
	if((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk_1)
	{
		return (1UL);
	}
	SysTick->LOAD =(uint32_t)(ticks - 1UL); 						/* Set Reload Register*/
	NVIC_SetPriority(SysTick_IRQn,(1UL << __NVIC_PRIO_BITS) - 1UL); /* Set Prioty of Systick */
	SysTick->VAL = 0UL;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk_1 |
					SysTick_CTRL_TICKINT_Msk_1   |
					SysTick_CTRL_ENABLE_Msk_1;
	return (0UL);

}
void delay_1ms(uint32_t Delay)
{
	uint32_t tickStart = uwTick_1;
	uint32_t wait = Delay;
	if(wait < HAL_MAX_DELAY_1)
	{
		wait += (uint32_t)(uwTickFreq);
	}
	while((uwTick_1 - tickStart) < wait)
	{

	}
}
int main(void)
{

	SysTick_Config1(SystemCoreClock_1/1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 test = 0;
	// HAL_Delay(Delay);
	  delay_1ms(1000);
	  test = 1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
