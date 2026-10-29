/**
  ******************************************************************************
  * @file    led.c
  * @brief   板载 LED（PB9）驱动：GPIO 初始化 + FreeRTOS 闪烁任务
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "../inc/led.h"

/* Private define ------------------------------------------------------------*/
#define LED_GPIO_PORT       GPIOB
#define LED_GPIO_PIN        GPIO_PIN_9
#define LED_BLINK_PERIOD    500    /* ms */

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread(void const *argument);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  配置 LED 引脚（PB9，推挽输出）。
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = LED_GPIO_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  创建 FreeRTOS 闪烁任务。
  * @param  None
  * @retval None
  */
void LED_TaskCreate(void)
{
	osThreadDef(LED, LED_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate(osThread(LED), NULL);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  每 LED_BLINK_PERIOD 毫秒翻转一次 LED。用 TogglePin 而不是写电平，
  *         所以无论 LED 是高有效还是低有效，看起来都一样。
  * @param  argument 未使用
  * @retval None
  */
static void LED_Thread(void const *argument)
{
	(void) argument;

	for (;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
		osDelay(LED_BLINK_PERIOD);
	}
}
