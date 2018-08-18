/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
extern uint8_t catcher_status;
extern uint32_t duration;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
	
	if(!catcher_status)
	{
		HAL_TIM_Base_Start(&htim6);
		catcher_status=1;
		//HAL_NVIC_ClearPendingIRQ
		EXTI->RTSR &= ~EXTI_RTSR_TR0;
		EXTI->FTSR |= EXTI_FTSR_TR0;
	}
		//Если поймали спадающий фронт
			else
				{
					HAL_TIM_Base_Stop(&htim6);  				//		TIM6->CR1 &= ~TIM_CR1_CEN;         // Останавливаем таймер
					duration=__HAL_TIM_GET_COUNTER(&htim6);		//duration = TIM6->CNT;              // Считываем значение длительности в мкс
					__HAL_TIM_SET_COUNTER(&htim6, 0);			//TIM6->CNT = 0;                     // Обнуляем регистр-счётчик

					// Переключаемся на отлов нарастающего фронта
					catcher_status = 0;
					EXTI->FTSR &= ~EXTI_FTSR_TR0;
					EXTI->RTSR |= EXTI_RTSR_TR0;
					// Запускаем таймер 6 на отсчёт 50 мс
					HAL_TIM_Base_Start_IT(&htim6);				//    TIM6->DIER |= TIM_DIER_UIE;        // Разрешаем прерывание от таймера
					HAL_TIM_Base_Start(&htim6);					//    TIM6->CR1 |= TIM_CR1_CEN;          // Запускаем таймер
					EXTI->PR |= 0x01;
					//HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);//__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_10);	//  EXTI->PR |= 0x01;                    //Очищаем флаг
					
				}
	
	
	
	

/**
	* @brief Пример функции прерывания от кнопки
	static int i = 0;
 
void EXTI0_IRQHandler(void)
{
	if(i%2 != 1){
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		i++;
	}
	else{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		i++;
	}
	if(i >= 10) i = 0;
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
	
	*/

										// описание работы с прерыванием :http://www.mcu.by/%D1%81%D1%82%D0%B0%D1%80%D1%82-arm-%D0%BF%D1%80%D0%B5%D1%80%D1%8B%D0%B2%D0%B0%D0%BD%D0%B8%D1%8F-%D0%BD%D0%B0-stm32f429i-disco-hal/
										//HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn) – функция сбрасывает бит в регистре прерываний, для EXTI0_IRQn
										//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0) – функция выхода прерывания с возвратом к тому месту, где прервался исполняемый код при возникновении прерывания.



	/* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles TIM6 global and DAC channel underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
//==============================================================================TIM6_IT
	TIM6->SR &= ~TIM_SR_UIF;             // Сбрасываем флаг UIF
  HAL_GPIO_WritePin(Trigger1_GPIO_Port, Trigger1_Pin, GPIO_PIN_SET);			//GPIOC->ODR |= GPIO_Pin_3;            // Включаем сигнальный импульс
  // Запускаем таймер 7 на отсчёт 10 мс
  TIM7->DIER |= TIM_DIER_UIE;          // Разрешаем прерывание от таймера 7
  TIM7->CR1 |= TIM_CR1_CEN;            // Запускаем таймер

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
//==============================================================================TIM7_IT
	  TIM7->SR &= ~TIM_SR_UIF;             // Сбрасываем флаг UIF
	  HAL_GPIO_WritePin(Trigger1_GPIO_Port, Trigger1_Pin, GPIO_PIN_RESET);		//GPIOA->ODR &= ~GPIO_Pin_9;           // Останавливаем сигнальный импульс
	  TIM7->DIER &= ~TIM_DIER_UIE;         // Запрещаем прерывание от таймера 7
		
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
