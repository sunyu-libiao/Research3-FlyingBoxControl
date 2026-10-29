/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "inc/can.h"
#include "inc/led.h"
#include "inc/motor.h"
#include "inc/tof.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 控制任务让全部电机同步往复：每程相对当前位置走 MOTOR_CTRL_TRAVEL_001MM，
   到位后翻转方向再发，如此循环。相对命令的 +T 与 -T 严格相消，不需要坐标系。
   速度与加速度不在这里给，它们属于电机自身的配置，已由 Motor_InitAll()
   按 MotorInfo.speed001mms / accel001mms2 下发。 */
#define MOTOR_CTRL_TRAVEL_001MM 10000   /* 0.1 mm 单位 -> 1000.0 mm */

#define MOTOR_CTRL_STACK_SIZE   128     /* 字                        */
#define MOTOR_CTRL_POLL_MS      10      /* 等初始化/等到位的轮询间隔 */
#define MOTOR_CTRL_IDLE_MS      100     /* 初始化失败后的空闲周期     */
#define MOTOR_CTRL_FRAME_GAP_MS 5       /* 每台电机命令之间的间隔     */
/* 单程等待上限。0x0420 只在电机在线时上报，电机没使能或断线时剩余位移会一直
   停在 0，没上限就会把“没动”当成“已到位”，一路空发指令。
   取值按 行程/速度 + 速度/加速度 估出的理论耗时留数倍余量，
   改 MOTOR_CTRL_TRAVEL_001MM 或电机的 speed001mms / accel001mms2 后要复核。 */
#define MOTOR_CTRL_ARRIVE_TIMEOUT_MS 20000u

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int SystemClock_Config(void);
static int MotorControl_WaitArrived(uint32_t timeoutMs);
static void MotorControl_Task(void const *argument);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  系统时钟：HSE 8 MHz 经 PLL x9 得 SYSCLK 72 MHz。
  *         HCLK = 72 MHz，PCLK1 = 36 MHz，PCLK2 = 72 MHz。
  *         APB1 上限就是 36 MHz；CAN 位定时按 PCLK1 = 36 MHz 推算，改这里
  *         必须同步改 can.c 的 Prescaler/TimeSeg。
  * @param  None
  * @retval 成功返回 0，HSE 起振或 PLL 锁定失败返回 -1
  */
static int SystemClock_Config(void)
{
	RCC_OscInitTypeDef oscInit = { 0 };
	RCC_ClkInitTypeDef clkInit = { 0 };

	oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscInit.HSEState = RCC_HSE_ON;
	oscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	oscInit.PLL.PLLState = RCC_PLL_ON;
	oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	oscInit.PLL.PLLMUL = RCC_PLL_MUL9;          /* 8 MHz x 9 = 72 MHz */
	if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
	{
		return -1;
	}

	clkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	clkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	/* 48 MHz < SYSCLK <= 72 MHz 需要 2 个 Flash 等待周期 */
	if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_2) != HAL_OK)
	{
		return -1;
	}

	return 0;
}

/**
  * @brief  等全部电机走完当前这一程。每台都必须先出现非 0 剩余位移，
  *         确认指令被受理、真的动了起来；之后剩余位移全部归零才算到位。
  *         先看“动起来”不能省：剩余位移在上一程结束时停在 0，若直接判归零，
  *         拿到的是上一程的旧值，会在电机还没动身时就翻转方向、一路空发。
  * @param  timeoutMs 等待上限，0 表示不等、直接判超时
  * @retval 0 全部到位，-1 超时（有电机一直没动，或没走完）
  */
static int MotorControl_WaitArrived(uint32_t timeoutMs)
{
	const uint8_t fullMask = (uint8_t)((1u << MOTOR_COUNT) - 1u);
	uint8_t movedMask = 0;
	uint8_t allZero;
	uint8_t i;
	uint32_t t0 = HAL_GetTick();

	for (;;)
	{
		allZero = 1;
		for (i = 0; i < MOTOR_COUNT; i++)
		{
			if (Motor_GetInfo(i)->remainMain != 0)
			{
				movedMask |= (uint8_t)(1u << i);
				allZero = 0;
			}
		}

		if ((movedMask == fullMask) && (allZero != 0u))
		{
			return 0;
		}

		if ((HAL_GetTick() - t0) >= timeoutMs)
		{
			return -1;
		}

		osDelay(MOTOR_CTRL_POLL_MS);
	}
}

/**
  * @brief  电机控制任务。等初始化任务把参数/轮径/速度/加速度/使能都发完之后，
  *         让全部电机同步往复：每程相对位移 MOTOR_CTRL_TRAVEL_001MM，
  *         到位后翻转方向。
  *         只有初始化整体成功才下发运动命令：初始化失败可能意味某轮没使能、
  *         轮径未配置，此时运动命令会被静默忽略或按错的比例执行。
  * @param  argument 未使用
  * @retval None
  */
static void MotorControl_Task(void const *argument)
{
	int32_t travel;
	uint8_t i;

	(void) argument;

	/* 初始化任务在调度器启动后才跑，这里阻塞轮等到它出结果 */
	while (Motor_GetInitResult() == MOTOR_INIT_PENDING)
	{
		osDelay(MOTOR_CTRL_POLL_MS);
	}

	if (Motor_GetInitResult() == 0)
	{
		travel = MOTOR_CTRL_TRAVEL_001MM;
		for (;;)
		{
			for (i = 0; i < MOTOR_COUNT; i++)
			{
				(void)Motor_SetTargetPositionMain(i, travel, MOTOR_POS_RELATIVE);
				osDelay(MOTOR_CTRL_FRAME_GAP_MS);
			}

			/* 有电机没走完就停手：继续翻转方向会让各台行程错开、越跑越偏 */
			if (MotorControl_WaitArrived(MOTOR_CTRL_ARRIVE_TIMEOUT_MS) != 0)
			{
				break;
			}

			travel = -travel;
		}
	}

	for (;;)
	{
		osDelay(MOTOR_CTRL_IDLE_MS);
	}
}

/**
  * @brief  创建电机控制任务。必须在 Motor_InitTaskCreate() 之后调用，
  *         让初始化任务排在前面（任务内部还会再等初始化结果）。
  * @param  None
  * @retval None
  */
static void MotorControl_TaskCreate(void)
{
	osThreadDef(MotorCtrl, MotorControl_Task, osPriorityNormal, 0, MOTOR_CTRL_STACK_SIZE);
	osThreadCreate(osThread(MotorCtrl), NULL);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
	HAL_Init();  
	
	/* 时钟起不来就不能继续：CAN 位定时基于 PCLK1 = 36 MHz，掉到 HSI 上
	   总线速率会错，而外设也不该带着错的时序启动 */
	if (SystemClock_Config() != 0)
	{
		for (;;)
			;
	}

	LED_Init();
	LED_TaskCreate();

	/* CAN 没起来就不建接收任务，避免任务在未就绪的外设上取队列 */
	if (CAN_Init() == 0)
	{
		Motor_Init();          /* 先把帧处理器挂好，再让接收任务开始取帧 */
		Tof_Init();            /* 两个协议共存：can 依次交给所有已注册回调 */
		CAN_RxTaskCreate();    /* 先建接收任务：初始化帧的 ACK 才有任务去消化 */
		Motor_InitTaskCreate();/* 参数 → 轮径 → 使能 放到调度器启动后跑 */
		Tof_InitTaskCreate();  /* TOF 配置帧同样放到调度器启动后 */
		MotorControl_TaskCreate();  /* 等初始化完成后让 5 台电机各相对走 500 mm */
	}

	/* Start scheduler */
	osKernelStart();

	  /* We should never get here as control is now taken by the scheduler */
	for (;;)
		;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
