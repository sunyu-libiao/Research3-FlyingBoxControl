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
/* 电机任务：进程内唯一的电机控制发起方，上电后一次性跑完整个启动流程
   （逐台归位 → 全部初始化 → 全部使能 → 全部置零），之后待命。
   集中到一个任务的原因：帧的顺序有要求（先设速度再给位置、归位期间只能有
   一台在出力），多任务同时发指令会互相打乱。 */
#define MOTOR_CTRL_STACK_SIZE   224     /* 字：Motor_BringUp 内部要发帧 + osDelay */
#define MOTOR_CTRL_IDLE_MS      500     /* 启动完成后待命周期                    */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int SystemClock_Config(void);
static int MoveUntilTof(uint8_t motorIndex, uint8_t tofIndex, int32_t travel,
                        uint16_t stopDistance, uint8_t decreasing,
                        uint32_t speed, uint32_t timeoutMs);
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
  * @brief  让一台电机持续运动，直到 TOF 测得的距离满足条件时停住。
  *
  *         协议里没有“停止”指令，也没有速度模式，所以做法是：先发一段
  *         足够长的相对位移让它自己跑，条件满足后把目标改成“当前位置”
  *         （相对 0）令其减速停车。
  *
  *         判据只看两件事：最近一帧读数 valid，且它确实是一帧新的（seq 变化）。
  *         只看 distance 会被两种情况骗过：无效读数下 distance 是 0（看起来
  *         “距离已很小”），以及同一帧被反复读到。
  *
  *         ★ 不要在这里加“样本不超过 N ms 才算数”这类新鲜度窗口：上报周期由
  *           模块自己掌控，且会变；窗口比实际周期短时判据永远不过，表现就是
  *           距离早就够了却一直不停。新鲜度不是停车判据应该关心的事。
  *
  *         不要求连续多帧满足：上报周期不确定，要求几帧就至少多等几个周期，
  *         速度快时可能整段错过窗口，而漏停的代价是撞机。
  *
  *         反向失效保护：曾经看到过读数、之后又连续看不到，就当作目标已经
  *         不可见（例如已经近过模块量程下限），立即停车。一直盲走只会更近。
  *
  *         无论怎么退出都会发出停车命令。返回后这台电机内部的速度是 speed，
  *         结构体里的 speed001mms 并未改变，要恢复运行参数得重新 Motor_InitAll()。
  * @param  motorIndex: 电机下标
  * @param  tofIndex: TOF 下标
  * @param  travel: 相对位移，0.1 mm，符号决定方向
  * @param  stopDistance: 距离阈值，0.1 mm
  * @param  decreasing: 1 = 距离缩到阈值以下时停（正在接近目标），0 = 反之
  * @param  speed: 接近速度，0.1 mm/s
  * @param  timeoutMs: 最长等待时间，ms
  * @retval 0 条件满足已停，-1 超时，-2 始终无有效数据，-3 参数或发帧错误，
  *         -4 读数丢失（曾有过、后来没了）
  */
static int MoveUntilTof(uint8_t motorIndex, uint8_t tofIndex, int32_t travel,
                        uint16_t stopDistance, uint8_t decreasing,
                        uint32_t speed, uint32_t timeoutMs)
{
	enum
	{
		POLL_MS = 10,
		/* 曾经看到过有效读数、之后又断了这么久，就当目标已不可见，停车。
		   取“多久算断”而不是“多久算新”：断流持续存在时，只有这种方向
		   的判据能保证机构最终会停下来。 */
		SAMPLE_LOST_MS = 2000
	};

	const TofInfo *tof;
	uint32_t elapsed = 0;
	uint32_t lastSampleTick = 0;
	uint8_t lastSeq = 0;
	uint8_t haveSeq = 0;
	uint8_t seen = 0;
	int result = -1;

	if ((motorIndex >= MOTOR_MAX_COUNT) || (tofIndex >= TOF_COUNT))
	{
		return -3;
	}

	tof = Tof_GetInfo(tofIndex);
	if (tof == NULL)
	{
		return -3;
	}

	if (Motor_SetSpeedMain(motorIndex, speed) != 0)
	{
		return -3;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	if (Motor_SetTargetPositionMain(motorIndex, travel, MOTOR_POS_RELATIVE) != 0)
	{
		return -3;
	}

	while (elapsed < timeoutMs)
	{
		if (tof->valid != 0)
		{
			seen = 1;
			lastSampleTick = HAL_GetTick();

			/* 同一帧只判一次：seq 不变说明模块还没测出新值 */
			if ((haveSeq == 0) || (tof->seq != lastSeq))
			{
				uint8_t reached;

				haveSeq = 1;
				lastSeq = tof->seq;

				if (decreasing != 0)
				{
					reached = (tof->distance <= stopDistance) ? 1u : 0u;
				}
				else
				{
					reached = (tof->distance >= stopDistance) ? 1u : 0u;
				}

				if (reached != 0)
				{
					result = 0;
					break;
				}
			}
		}
		else if ((seen != 0) &&
		         ((HAL_GetTick() - lastSampleTick) >= SAMPLE_LOST_MS))
		{
			result = -4;
			break;
		}

		osDelay(POLL_MS);
		elapsed += POLL_MS;
	}

	/* 区分“始终没数据”和“有数据但没满足条件” */
	if ((result != 0) && (haveSeq == 0))
	{
		result = -2;
	}

	/* 所有退出路径都要停：不改目标的话电机会一直跑到行程尽头 */
	(void)Motor_SetTargetPositionMain(motorIndex, 0, MOTOR_POS_RELATIVE);
	osDelay(MOTOR_HOME_SETTLE_MS);

	return result;
}

/**
  * @brief  电机任务。上电后一次性跑完启动流程，然后做测试动作并待命：
  *         等电机复位 → 9 号单独初始化并归位 → 11 号单独初始化并归位 →
  *         全部重新初始化 → 全部使能 → 把此刻定义为绝对 0 →
  *         10 号绝对定位（把装置转到正前方）→
  *         9 号持续前进，直到 TOF 距离满足条件时停车。
  *         整个流程是阻塞的（逐台等归位延时），期间本任务一直占着。
  *         不重试：失败只计数，由上层检查机械与参数。
  * @param  argument 未使用
  * @retval None
  */
static void MotorControl_Task(void const *argument)
{
	(void) argument;

	/* 内部含上电等待与逐台归位，会阻塞几十秒 */
	(void)Motor_BringUp();

	/* 10 号（下标 3，不参与归位）绝对定位，把装置转到正前方。运行参数已由
	   Motor_BringUp() 末尾的 Motor_InitAll() 下发且已使能，这里只发目标位置。
	   它不参与归位，绝对 0 是启动末尾置零那一刻的位置。
	   必须先转到位再前进，否则测距方向不对。 */
	(void)Motor_SetTargetPositionMain(3, 80000, MOTOR_POS_ABSOLUTE);
	(void)Motor_WaitArrive(3, 5000);

	/* 测试：9 号（下标 2）持续前进，直到 25 号 TOF（下标 0）测得的距离缩到阈值。
	   前进时距离变小，所以 decreasing 传 1；方向反了就把 travel 的符号翻掉。 */
	(void)MoveUntilTof(2, 0, 1000000, 3000, 1, 25000, 30000);

	for (;;)
	{
		osDelay(MOTOR_CTRL_IDLE_MS);
	}
}

/**
  * @brief  创建电机任务。必须在 CAN_RxTaskCreate() 之后创建，
  *         否则启动流程发出的配置帧没有任务去消化应答。
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
		Tof_InitTaskCreate();  /* TOF 配置帧放到调度器启动后 */
		Tof_PollTaskCreate();  /* 配置完成后按固定周期请求距离 */
		MotorControl_TaskCreate();  /* 电机启动流程：归位 → 初始化 → 使能 → 置零 */
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
