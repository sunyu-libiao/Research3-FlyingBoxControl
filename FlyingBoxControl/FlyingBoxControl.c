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

/* 停车用的相对位移增量，协议单位（轮缘弧长，对已标定的轮径就是 0.1 mm 量级）。
   ★ 实测发 0 停不住：电机不认零位移，命令被丢掉，机构继续跑到行程尽头。
     doc/电机控制流程.md 里也有旁证——非零相对目标都会回 0x0100 受理帧，
     相对 0 位移不会回。所以用一个"几乎为零但不为零"的增量，让目标落到
     当前位置旁边，电机随即减速停住。
   ★ 不能大：增量本身就是新的剩余位移，越大停得越晚、超调越多。
   试 1 不行就试 5，再不行说明相对命令这条路走不通，需要改用绝对定位。 */
#define MOTOR_STOP_DELTA        1

/* 停车帧的发送次数与间隔。相对命令每帧都重新取"当前位置 + Δ"作为目标，
   不累加，重复发最多让机构多走 Δ 那一点点，所以可以用多帧冗余提高送达
   概率：bxCAN 是单次发送（NART = 1），丢一帧就是真丢。 */
#define MOTOR_STOP_FRAME_COUNT  3u
#define MOTOR_STOP_FRAME_GAP_MS 50u

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 电机任务的最后一次运动结果，调试时在调试器里直接看这个变量：
   0 = 距离到阈值且停车帧已发出，-1 = 超时，-2 = 始终无有效读数，
   -5 = 停车帧全部发送失败。 */
static volatile int8_t s_lastMoveResult;

/* Private function prototypes -----------------------------------------------*/
static int SystemClock_Config(void);
static int stopMotor(uint8_t motorIndex);
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
  * @brief  发停车命令：相对一个极小增量，把目标拉到当前位置旁边。
  *
  *         相对命令的语义是「目标 = 收到帧那一刻的当前位置 + Δ」。
  *
  *         ★ 不能用 Δ = 0。实测三帧 00 00 00 00 09 00 确实上线了，电机不停；
  *           doc/电机控制流程.md 也记着非零相对目标才回 0x0100 受理帧，
  *           零位移根本不进运动状态机。所以取一个非零但极小的 Δ。
  *
  *         Δ 的符号按剩余位移的方向取：remain = 目标 − 当前位置，为正说明
  *         目标在前方、机构正在向正向运动，此时 Δ 也取正，新目标刚好落在
  *         当前位置前方一点。符号反了会让电机先朝反方向走一小段。
  *
  *         ★ 不要改成发「−剩余」。那样新目标会落到「当前位置 − 剩余」，
  *           电机反而朝反方向走一整段，实测表现为到点后前后震一下、之后
  *           接着跑。
  * @param  motorIndex: 电机下标
  * @retval 0 至少一帧已发出，-1 参数错，-5 全部发送失败
  */
static int stopMotor(uint8_t motorIndex)
{
	const MotorInfo *info;
	int32_t delta;
	uint8_t attempt;
	uint8_t sent = 0;

	info = Motor_GetInfo(motorIndex);
	if (info == NULL)
	{
		return -1;
	}

	/* 运动方向由剩余位移的符号给出：负剩余说明目标在后方 */
	delta = (info->remainMain < 0) ? -MOTOR_STOP_DELTA : MOTOR_STOP_DELTA;

	for (attempt = 0; attempt < MOTOR_STOP_FRAME_COUNT; attempt++)
	{
		if (Motor_SetTargetPositionMain(motorIndex, delta, MOTOR_POS_RELATIVE) == 0)
		{
			sent = 1;
		}

		osDelay(MOTOR_STOP_FRAME_GAP_MS);
	}

	return (sent != 0u) ? 0 : -5;
}

/**
  * @brief  让一台电机持续运动，直到 TOF 测得的距离到达阈值时停住。
  *
  *         协议里没有“停止”指令，也没有速度模式，所以做法是：先发一段
  *         足够长的相对位移让它自己跑，条件满足后再让它的目标回到当前位置
  *         （相对 0 位移），做法见 stopMotor() 的说明。
  *
  *         判据只有一条：读数 valid 且距离已经达到阈值。到了就发停车指令。
  *         valid 必须查——无效读数下 distance 被置 0，会被当成“已经贴到目标”，
  *         一启动就停。
  *
  *         不要求“必须是新的一帧”：距离已经到了就该停，同一帧重复读到并不
  *         改变这个事实；而漏停的代价是撞机。
  *
  *         不再限制读数新鲜度：上报周期由模块自己掌控且会变，拿一个固定窗口
  *         去卡它，窗口比实际周期短时判据永远不过，表现就是距离早就够了却
  *         一直不停。循环由 timeoutMs 兜底，不会永久卡死。
  *
  *         停车由 stopMotor() 负责；它的帧没能发出时返回 -5。本函数不确认
  *         电机是否真的停住——帧发出去了不等于电机执行了。
  *         返回后这台电机内部的速度是 speed，结构体里的 speed001mms 并未改变，
  *         要恢复运行参数得重新 Motor_InitAll()。
  * @param  motorIndex: 电机下标
  * @param  tofIndex: TOF 下标
  * @param  travel: 相对位移，0.1 mm，符号决定方向
  * @param  stopDistance: 距离阈值，0.1 mm
  * @param  decreasing: 1 = 距离缩到阈值以下时停（正在接近目标），0 = 反之
  * @param  speed: 接近速度，0.1 mm/s
  * @param  timeoutMs: 最长等待时间，ms
  * @retval 0 条件满足且停车帧已发出，-1 超时，-2 始终无有效数据，
  *         -3 参数或发帧错误，-5 停车帧发送失败（机构可能还在动）
  */
static int MoveUntilTof(uint8_t motorIndex, uint8_t tofIndex, int32_t travel,
                        uint16_t stopDistance, uint8_t decreasing,
                        uint32_t speed, uint32_t timeoutMs)
{
	enum { POLL_MS = 10 };

	const TofInfo *tof;
	uint32_t elapsed = 0;
	uint8_t everValid = 0;
	int stopResult;
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
		/* valid 必须查：无效读数下 distance 被置 0，会被当成“已经贴到目标” */
		if (tof->valid != 0)
		{
			uint8_t reached;

			everValid = 1;

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

		osDelay(POLL_MS);
		elapsed += POLL_MS;
	}

	/* 区分“始终没数据”和“有数据但没到阈值” */
	if ((result != 0) && (everValid == 0))
	{
		result = -2;
	}

	/* 停车是安全路径：这里返回非 0 表示机构可能还在动，优先级高于上面的结果 */
	stopResult = stopMotor(motorIndex);
	if (stopResult != 0)
	{
		osDelay(MOTOR_HOME_SETTLE_MS);
		return stopResult;
	}

	osDelay(MOTOR_HOME_SETTLE_MS);

	return result;
}

/**
  * @brief  电机任务。上电后一次性跑完启动流程，然后做测试动作并待命：
  *         等电机复位 → 9 号单独初始化并归位 → 11 号单独初始化并归位 →
  *         全部重新初始化 → 全部使能 → 把此刻定义为绝对 0 →
  *         10 号绝对定位（把装置转到正前方）→
  *         9 号持续前进，直到 TOF 距离满足条件时停车 →
  *         11 号绝对定位到 11000。
  *         整个流程是阻塞的（逐台等归位延时与到位），期间本任务一直占着。
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
	(void)Motor_SetTargetPositionMain(3, 75000, MOTOR_POS_ABSOLUTE);
	(void)Motor_WaitArrive(3, 5000);

	/* 测试：9 号（下标 2）持续前进，直到 25 号 TOF（下标 0）测得的距离缩到阈值。
	   前进时距离变小，所以 decreasing 传 1；方向反了就把 travel 的符号翻掉。
	   travel 与 speed 是轮缘弧长量，必须与 9 号的轮径同量级；阈值来自 TOF，
	   是真实的毫米量，不随轮径变化（协议单位 0.1 mm，mm × 10）。
	   返回值留在 s_lastMoveResult 里，含义见该变量的声明处。 */
	s_lastMoveResult = (int8_t)MoveUntilTof(2, 0, 10000, 1000, 1, 500, 30000);

	/* 9 号停定之后，11 号（下标 4）绝对定位到 11000。
	   绝对 0 是 Motor_BringUp() 末尾置零那一刻的位置，与 9 号的停车位置无关，
	   所以这里给的是坐标值而不是相对量。11 号驱动的是钩爪类机构，行程没有
	   可标定的直线关系，11000 按它自己的配置单位走，不代表真实毫米。

	   ★ 只有 9 号确实到位并停住（0）才动 11 号。超时（-1）、始终无读数（-2）、
	     停车帧全丢（-5）时 9 号可能还在动，此时再让 11 号动作会让两台机构
	     同时运动，出问题无法归因。 */
	if (s_lastMoveResult == 0)
	{
		(void)Motor_SetTargetPositionMain(4, 110000, MOTOR_POS_ABSOLUTE);
		(void)Motor_WaitArrive(4, 5000);
	}

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
