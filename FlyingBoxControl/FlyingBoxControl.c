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

/* 位置查询（0x0408 子码 0x08）的应答时限。应答本身很快，留几倍余量吸收
   总线忙与接收任务排队。 */
#define MOTOR_POS_QUERY_TIMEOUT_MS 200u

/* 回原点等待时看剩余位移的轮询周期。0x0420 约 50 Hz，取得比它还快没有意义。 */
#define MOTOR_RETURN_POLL_MS       20u

/* 「主轴回原点 + 两根皮带同步跟随」的等待时限。按最坏行程与当前运行速度
   估算，并留出加速段与反馈延迟的余量。 */
#define MOTOR_RETURN_TIMEOUT_MS    15000u

/* 回原点动作没执行（前一步没成功）时的记录值 */
#define MOTOR_RETURN_SKIPPED       (-9)

/* 一次能同时驱动的电机数上限（组式接近用）。只给"两根皮带一起走"用，
   留 2 就够；需要更多就同步改这里。 */
#define MOTOR_MOVE_GROUP_MAX       2u

/* 皮带组向 26 号 TOF 接近：行程预算、速度与时限。
   行程只要"够长"，真正的停车靠 TOF 判据，多余的行程不会走完；
   但不能超过皮带机械行程，否则会顶到末端触发保护释放。
   ★ 速度不能跟着运行参数一起翻倍。停车是减速过程，刹距约 v^2/(2a)，
     当前配置下已经占掉阈值的一半：再翻倍会让刹距超过 5 cm 阈值，
     必然冲过目标点。要提速得先加大加速度或放宽阈值。 */
#define BELT_APPROACH_TRAVEL       (-10000)     /* 负值 = 物理向后，与 homeTravel001mm 的符号惯例一致 */
#define BELT_APPROACH_SPEED        800u
#define BELT_APPROACH_TIMEOUT_MS   30000u

/* 皮带组从 26 号 TOF 前的停点退回到 11 cm 处：正位移 = 物理向前。
   判据变成"距离变大"，刹车滞后只会让停点更远，不会撞上机械末端，
   所以行程与速度都可以沿用接近那一组。 */
#define BELT_FORWARD_TRAVEL        10000
#define BELT_FORWARD_DISTANCE      1100u        /* 11 cm × 10，与 TOF 的 0.1 mm 单位一致 */
#define BELT_FORWARD_TIMEOUT_MS    30000u

/* 上一段动作停稳到下一段开始之间的静止间隔 */
#define MOTOR_SEQ_PAUSE_MS         3000u

/* 9 号第二段的绝对目标：50 cm = 500 mm，协议单位 0.1 mm */
#define AXIS_FORWARD_TARGET        5000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 电机任务的最后一次运动结果，调试时在调试器里直接看这个变量：
   0 = 距离到阈值且停车帧已发出，-1 = 超时，-2 = 始终无有效读数，
   -5 = 停车帧全部发送失败。 */
static volatile int8_t s_lastMoveResult;

/* 紧接着的「主轴回原点 + 皮带跟随」的结果：MOTOR_RETURN_SKIPPED = 没执行，
   0 = 三台都已到位，其它负值见 ReturnAxisToOrigin() 的返回码。 */
static volatile int8_t s_lastReturnResult = MOTOR_RETURN_SKIPPED;

/* 皮带组接近 TOF 的结果。MOTOR_RETURN_SKIPPED = 前面某步失败没执行，
   其余见 MoveGroupUntilTof() 的返回码。 */
static volatile int8_t s_lastBeltResult = MOTOR_RETURN_SKIPPED;

/* 7/8 号向前退回（TOF 距离超过 11 cm）的结果，取值同上。 */
static volatile int8_t s_lastBeltForwardResult = MOTOR_RETURN_SKIPPED;

/* 9 号绝对定位到 50 cm（7/8 号跟随）的结果，取值见 MoveAxisAbsoluteWithBelts()。 */
static volatile int8_t s_lastAxisForwardResult = MOTOR_RETURN_SKIPPED;

/* 最后 9 号单独回绝对 0 的结果，取值见 Motor_WaitArrive()。 */
static volatile int8_t s_lastAxisZeroResult = MOTOR_RETURN_SKIPPED;

/* Private function prototypes -----------------------------------------------*/
static int SystemClock_Config(void);
static int stopMotor(uint8_t motorIndex);
static int readPosition(uint8_t motorIndex, int32_t *position, uint32_t timeoutMs);
static uint8_t isNearTarget(int32_t remain);
static int ReturnAxisToOrigin(uint8_t motorIndex, uint8_t beltIndexA, uint8_t beltIndexB,
                              uint32_t timeoutMs);
static int MoveAxisAbsoluteWithBelts(uint8_t motorIndex, uint8_t beltIndexA, uint8_t beltIndexB,
                                     int32_t target, uint32_t arriveTimeoutMs);
static int MoveGroupUntilTof(uint8_t count, const uint8_t *motorIndices, uint8_t tofIndex,
                             int32_t travel, uint16_t stopDistance, uint8_t decreasing,
                             uint32_t speed, uint32_t timeoutMs);
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
  * @brief  让一组电机同时持续运动，直到 TOF 测得的距离到达阈值时全部停住。
  *
  *         协议里没有“停止”指令，也没有速度模式，所以做法是：先发一段
  *         足够长的相对位移让它们自己跑，条件满足后再让目标回到当前位置，
  *         做法见 stopMotor() 的说明。
  *
  *         ★ 同时启动靠帧序：先把所有台的速度都发完，再连着发位移，
  *           让位移帧之间只隔 CAN 发送时间。若写成“设一台速度就发一台位移”，
  *           后一台会比前一台晚十几毫秒，同步就散了。
  *
  *         判据只有一条：读数 valid 且距离已经达到阈值。到了就全部停车。
  *         valid 必须查——无效读数下 distance 被置 0，会被当成“已经贴到目标”，
  *         一启动就停。
  *
  *         不要求“必须是新的一帧”，也不限制读数新鲜度：距离已经到了就该停，
  *         `0x0793` 的上报周期由模块自己掌控且会变，拿固定窗口去卡它会让判据
  *         永远不过。循环由 timeoutMs 兜底。
  *
  *         停车由 stopMotor() 逐台负责；任意一台的停车帧发不出去都返回 -5。
  *         返回后这些电机内部的速度是 speed，结构体里的 speed001mms 并未改变，
  *         要恢复运行参数得重新 Motor_InitAll()。
  * @param  count: 电机台数，1~MOTOR_MOVE_GROUP_MAX
  * @param  motorIndices: 电机下标数组
  * @param  tofIndex: TOF 下标
  * @param  travel: 每台的相对位移，0.1 mm，符号决定方向（组内同向同量）
  * @param  stopDistance: 距离阈值，0.1 mm
  * @param  decreasing: 1 = 距离缩到阈值以下时停（正在接近目标），0 = 反之
  * @param  speed: 接近速度，0.1 mm/s
  * @param  timeoutMs: 最长等待时间，ms
  * @retval 0 条件满足且停车帧已发出，-1 超时，-2 始终无有效数据，
  *         -3 参数或发帧错误，-5 至少一台停车帧发送失败
  */
static int MoveGroupUntilTof(uint8_t count, const uint8_t *motorIndices, uint8_t tofIndex,
                             int32_t travel, uint16_t stopDistance, uint8_t decreasing,
                             uint32_t speed, uint32_t timeoutMs)
{
	enum { POLL_MS = 10 };

	const TofInfo *tof;
	uint32_t elapsed = 0;
	uint8_t everValid = 0;
	uint8_t stopFailed = 0;
	uint8_t i;
	int result = -1;

	if ((count == 0u) || (count > MOTOR_MOVE_GROUP_MAX) || (tofIndex >= TOF_COUNT))
	{
		return -3;
	}

	for (i = 0; i < count; i++)
	{
		if (motorIndices[i] >= MOTOR_MAX_COUNT)
		{
			return -3;
		}
	}

	tof = Tof_GetInfo(tofIndex);
	if (tof == NULL)
	{
		return -3;
	}

	/* 先把速度全部下发完，再发位移：顺序见函数说明里的“同时启动” */
	for (i = 0; i < count; i++)
	{
		if (Motor_SetSpeedMain(motorIndices[i], speed) != 0)
		{
			return -3;
		}
		osDelay(MOTOR_INIT_FRAME_GAP_MS);
	}

	for (i = 0; i < count; i++)
	{
		if (Motor_SetTargetPositionMain(motorIndices[i], travel, MOTOR_POS_RELATIVE) != 0)
		{
			return -3;
		}
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

	/* 停车是安全路径：每台都要停，任意一台失败都要报出来 */
	for (i = 0; i < count; i++)
	{
		if (stopMotor(motorIndices[i]) != 0)
		{
			stopFailed = 1;
		}
	}

	osDelay(MOTOR_HOME_SETTLE_MS);

	return (stopFailed != 0u) ? -5 : result;
}

/**
  * @brief  单台版本，参数与返回值同 MoveGroupUntilTof()。
  * @param  motorIndex: 电机下标
  * @param  tofIndex: TOF 下标
  * @param  travel: 相对位移，0.1 mm，符号决定方向
  * @param  stopDistance: 距离阈值，0.1 mm
  * @param  decreasing: 1 = 距离缩到阈值以下时停（正在接近目标），0 = 反之
  * @param  speed: 接近速度，0.1 mm/s
  * @param  timeoutMs: 最长等待时间，ms
  * @retval 0 条件满足且停车帧已发出，其余负值见 MoveGroupUntilTof()
  */
static int MoveUntilTof(uint8_t motorIndex, uint8_t tofIndex, int32_t travel,
                        uint16_t stopDistance, uint8_t decreasing,
                        uint32_t speed, uint32_t timeoutMs)
{
	return MoveGroupUntilTof(1u, &motorIndex, tofIndex, travel, stopDistance,
	                         decreasing, speed, timeoutMs);
}

/**
  * @brief  剩余位移是否已经落进到位死区。
  * @param  remain: 剩余位移，0.1 mm
  * @retval 1 在死区内，0 还有余量
  */
static uint8_t isNearTarget(int32_t remain)
{
	return ((remain <= MOTOR_ARRIVE_DEADBAND) &&
	        (remain >= -MOTOR_ARRIVE_DEADBAND)) ? 1u : 0u;
}

/**
  * @brief  读一台电机的绝对位置：发 0x0408 子码 0x08，等 0x0409 应答。
  *
  *         ★ 不能用 lastRxTick 判新鲜。那个时间戳被任何一帧刷新（心跳、
  *           剩余位移、力矩都是几十到上百 Hz），拿到的是"最近有帧"，不是
  *           "最近有位置应答"。所以靠 positionRxCount 前进来判断：它只在
  *           子码 0x08 的应答里递增。
  * @param  motorIndex: 电机下标
  * @param  position: 输出，0.1 mm，同一坐标系下带符号
  * @param  timeoutMs: 等应答的时限
  * @retval 0 读到新应答，-1 参数错/发帧失败/超时
  */
static int readPosition(uint8_t motorIndex, int32_t *position, uint32_t timeoutMs)
{
	const MotorInfo *info;
	uint32_t elapsed = 0;
	uint32_t before;

	info = Motor_GetInfo(motorIndex);
	if (info == NULL)
	{
		return -1;
	}

	before = info->positionRxCount;
	if (Motor_QueryPosition(motorIndex) != 0)
	{
		return -1;
	}

	while (elapsed < timeoutMs)
	{
		if (info->positionRxCount != before)
		{
			*position = info->position;
			return 0;
		}

		osDelay(MOTOR_RETURN_POLL_MS);
		elapsed += MOTOR_RETURN_POLL_MS;
	}

	return -1;
}

/**
  * @brief  主轴回绝对 0，同时两根皮带按同一物理方向走同一距离。
  *
  *         距离不是外部给的，而是由主轴当前绝对位置反推：它要回到 0，
  *         走的就是「0 − 当前位置」，三台都走这一段，机构才是刚性一起动。
  *         所以必须先读位置——0x0420 报的是"到目标的剩余量"，给不出绝对值。
  *
  *         三台的位移取同一个数：三台的轮径都已按实测标定，协议单位统一是
  *         真实行程的 0.1 mm，同一个数值就是同样的物理距离，与各自轮径无关。
  *
  *         ★ 符号：本机三台的 reverse 与安装方向已调到"正数 = 同一物理方向"，
  *           因此主轴朝 0 走的方向（−当前位置）直接就是皮带该走的相对量。
  *           若以后改了三台的 reverse 组合，这里的符号要跟着重算。
  *
  *         ★ 主轴的速度要先恢复。MoveUntilTof() 把它的速度降成接近速度后
  *           没有还原，不还原的话它比两根皮带慢，三者不会同步。
  *
  *         三帧背靠背发出，让它们几乎同时启动；到位判定在一个循环里同时看
  *         三台的剩余位移。不能串行等：串行等完第一台时另外两台早已就位，
  *           再调 Motor_WaitArrive() 会因为没有观察到"它在动"而白等到超时。
  * @param  motorIndex: 主轴下标
  * @param  beltIndexA: 皮带 A 下标
  * @param  beltIndexB: 皮带 B 下标
  * @param  timeoutMs: 等待三台到位的最长时间
  * @retval 0 三台都已到位，-1 参数错，-2 读不到主轴位置，-3 发帧失败，
  *         -4 超时（至少一台没到位）
  */
static int ReturnAxisToOrigin(uint8_t motorIndex, uint8_t beltIndexA, uint8_t beltIndexB,
                              uint32_t timeoutMs)
{
	const MotorInfo *axis;
	const MotorInfo *beltA;
	const MotorInfo *beltB;
	uint32_t elapsed = 0;
	int32_t position;
	int32_t travel;
	uint8_t moving = 0;

	axis = Motor_GetInfo(motorIndex);
	beltA = Motor_GetInfo(beltIndexA);
	beltB = Motor_GetInfo(beltIndexB);
	if ((axis == NULL) || (beltA == NULL) || (beltB == NULL))
	{
		return -1;
	}

	if (readPosition(motorIndex, &position, MOTOR_POS_QUERY_TIMEOUT_MS) != 0)
	{
		return -2;
	}

	travel = -position;
	if ((travel <= MOTOR_ARRIVE_DEADBAND) && (travel >= -MOTOR_ARRIVE_DEADBAND))
	{
		/* 已经在原点附近，三台都不用动 */
		return 0;
	}

	/* 恢复主轴运行速度，否则它比两根皮带慢，同步就无从谈起 */
	if (Motor_SetSpeedMain(motorIndex, axis->speed001mms) != 0)
	{
		return -3;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	/* 三帧连续发出：主轴绝对 0，两根皮带同一相对位移 */
	if ((Motor_SetTargetPositionMain(motorIndex, 0, MOTOR_POS_ABSOLUTE) != 0) ||
	    (Motor_SetTargetPositionMain(beltIndexA, travel, MOTOR_POS_RELATIVE) != 0) ||
	    (Motor_SetTargetPositionMain(beltIndexB, travel, MOTOR_POS_RELATIVE) != 0))
	{
		return -3;
	}

	/* 等三台同时到位。必须见过至少一台"确实在动"之后，才承认全部进入死区，
	   否则命令刚发出时读到的旧剩余值（0）会被当成"已经到位"。 */
	while (elapsed < timeoutMs)
	{
		if ((isNearTarget(axis->remainMain) == 0u) ||
		    (isNearTarget(beltA->remainMain) == 0u) ||
		    (isNearTarget(beltB->remainMain) == 0u))
		{
			moving = 1;
		}
		else if (moving != 0u)
		{
			return 0;
		}

		osDelay(MOTOR_RETURN_POLL_MS);
		elapsed += MOTOR_RETURN_POLL_MS;
	}

	return -4;
}

/**
  * @brief  主轴绝对定位，同时两根皮带按同一物理方向走同一距离；只等主轴到位。
  *
  *         与 ReturnAxisToOrigin() 的差别只在等待判据：这里皮带纯跟随，整段是否
  *         走完由主轴说了算。等皮带一起进死区会让主轴到位后再白等一段——两台
  *         皮带的速度与主轴不一定完全相同，收尾时刻对不齐。
  *
  *         距离同样由主轴坐标反推（目标 − 当前位置），三台走同一段，机构才是
  *         刚性一起动；皮带不单独给参数。
  *
  *         ★ 三台的速度都要先恢复配置值。前面的接近动作把速度降到接近速度后
  *           没有还原，不恢复的话主轴或皮带会明显落后，同一步里就不同步了。
  * @param  motorIndex: 主轴下标
  * @param  beltIndexA: 皮带 A 下标
  * @param  beltIndexB: 皮带 B 下标
  * @param  target: 主轴的绝对目标，0.1 mm
  * @param  arriveTimeoutMs: 等主轴到位的最长时间
  * @retval 0 主轴已到位，-1 参数错，-2 读不到主轴位置，-3 发帧失败，
  *         其余负值见 Motor_WaitArrive()
  */
static int MoveAxisAbsoluteWithBelts(uint8_t motorIndex, uint8_t beltIndexA, uint8_t beltIndexB,
                                     int32_t target, uint32_t arriveTimeoutMs)
{
	const MotorInfo *axis;
	const MotorInfo *beltA;
	const MotorInfo *beltB;
	int32_t position;
	int32_t travel;

	axis = Motor_GetInfo(motorIndex);
	beltA = Motor_GetInfo(beltIndexA);
	beltB = Motor_GetInfo(beltIndexB);
	if ((axis == NULL) || (beltA == NULL) || (beltB == NULL))
	{
		return -1;
	}

	if (readPosition(motorIndex, &position, MOTOR_POS_QUERY_TIMEOUT_MS) != 0)
	{
		return -2;
	}

	travel = target - position;
	if ((travel <= MOTOR_ARRIVE_DEADBAND) && (travel >= -MOTOR_ARRIVE_DEADBAND))
	{
		/* 已经在目标附近，三台都不用动 */
		return 0;
	}

	if ((Motor_SetSpeedMain(motorIndex, axis->speed001mms) != 0) ||
	    (Motor_SetSpeedMain(beltIndexA, beltA->speed001mms) != 0) ||
	    (Motor_SetSpeedMain(beltIndexB, beltB->speed001mms) != 0))
	{
		return -3;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	/* 三帧连续发出：主轴绝对目标，两根皮带同一相对位移 */
	if ((Motor_SetTargetPositionMain(motorIndex, target, MOTOR_POS_ABSOLUTE) != 0) ||
	    (Motor_SetTargetPositionMain(beltIndexA, travel, MOTOR_POS_RELATIVE) != 0) ||
	    (Motor_SetTargetPositionMain(beltIndexB, travel, MOTOR_POS_RELATIVE) != 0))
	{
		return -3;
	}

	return Motor_WaitArrive(motorIndex, arriveTimeoutMs);
}

/**
  * @brief  电机任务。上电后一次性跑完启动流程，然后做测试动作并待命：
  *         等电机复位 → 9 号单独初始化并归位 → 11 号单独初始化并归位 →
  *         全部重新初始化 → 全部使能 → 把此刻定义为绝对 0 →
  *         10 号绝对定位（把装置转到正前方）→
  *         9 号持续前进，直到 TOF 距离满足条件时停车 →
  *         11 号绝对定位到 110000 →
  *         9 号回绝对 0，同时 7/8 号按相同方向相同距离跟随 →
  *         11 号绝对定位到 10000 →
  *         10 号绝对定位到 225000 →
  *         7/8 号同时向后走，直到 26 号 TOF 的距离小于 5 cm →
  *         停 3 s → 7/8 号向前退回，直到 26 号 TOF 的距离超过 11 cm →
  *         10 号绝对定位到 75000 →
  *         9 号绝对定位到 50 cm，7/8 号同向同距跟随 →
  *         9 号单独回绝对 0。
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
	s_lastMoveResult = (int8_t)MoveUntilTof(2, 0, 10000, 1050, 1, 2000, 30000);

	/* 9 号停定之后，11 号（下标 4）绝对定位。
	   绝对 0 是 Motor_BringUp() 末尾置零那一刻的位置，与 9 号的停车位置无关，
	   所以这里给的是坐标值而不是相对量。11 号驱动的是钩爪类机构，行程没有
	   可标定的直线关系，数值按它自己的配置单位走，不代表真实毫米。

	   ★ 只有 9 号确实到位并停住（0）才动 11 号。超时（-1）、始终无读数（-2）、
	     停车帧全丢（-5）时 9 号可能还在动，此时再让 11 号动作会让两台机构
	     同时运动，出问题无法归因。 */
	if (s_lastMoveResult == 0)
	{
		(void)Motor_SetTargetPositionMain(4, 110000, MOTOR_POS_ABSOLUTE);
		(void)Motor_WaitArrive(4, 5000);
		/* 9 号回绝对 0，同时 7/8 号按同一物理方向走同一距离。
		   距离由 9 号的当前坐标反推，不另外给参数；三台的符号与速度关系
		   见 ReturnAxisToOrigin() 的说明。 */
		s_lastReturnResult = (int8_t)ReturnAxisToOrigin(2, 0, 1, MOTOR_RETURN_TIMEOUT_MS);
	}

	/* 回原点成功后才做后面三步：那时 9 号与 7/8 都刚回到绝对 0，
	   坐标系起点一致，后面的绝对定位才有确定含义。回原点失败就不做，
	   避免在位置未知的情况下让别的轴动起来。 */
	if (s_lastReturnResult == 0)
	{
		/* 11 号（下标 4）绝对定位到 10000 */
		(void)Motor_SetTargetPositionMain(4, 10000, MOTOR_POS_ABSOLUTE);
		(void)Motor_WaitArrive(4, 5000);

		/* 10 号（下标 3）绝对定位到 225000 */
		(void)Motor_SetTargetPositionMain(3, 240000, MOTOR_POS_ABSOLUTE);
		(void)Motor_WaitArrive(3, 10000);

		/* 7/8 号同时向后走，直到 26 号 TOF（下标 1）的距离小于 5 cm。
		   阈值 500 = 50 mm × 10；decreasing = 1 表示距离缩小时停。
		   ★ 向后（负位移）与“距离要变小”必须同时成立，这依赖 TOF 装在
		     跟随皮带靠近目标的那一侧。若实际是反的，把 BELT_APPROACH_TRAVEL
		     的符号翻掉即可。 */
		{
			static const uint8_t belts[2] = { 0u, 1u };

			s_lastBeltResult = (int8_t)MoveGroupUntilTof(2u, belts, 1u,
			                                             BELT_APPROACH_TRAVEL, 600u, 1u,
			                                             BELT_APPROACH_SPEED,
			                                             BELT_APPROACH_TIMEOUT_MS);
		}
	}

	/* 上一段停稳 3 s 之后再退回：接近结束时机构还有残余速度与回弹，
	   立刻反向会让 TOF 读数带着振动，停下点不可重复。 */
	if (s_lastBeltResult == 0)
	{
		osDelay(MOTOR_SEQ_PAUSE_MS);

		/* 7/8 号同时向前（正位移）退出，直到 26 号 TOF（下标 1）的距离超过
		   11 cm。距离变大，所以 decreasing 传 0。 */
		{
			static const uint8_t belts[2] = { 0u, 1u };

			s_lastBeltForwardResult = (int8_t)MoveGroupUntilTof(2u, belts, 1u,
			                                                    BELT_FORWARD_TRAVEL,
			                                                    BELT_FORWARD_DISTANCE, 0u,
			                                                    BELT_APPROACH_SPEED,
			                                                    BELT_FORWARD_TIMEOUT_MS);
		}
	}

	if (s_lastBeltForwardResult == 0)
	{
		/* 10 号（下标 3）从 240000 转回绝对 75000 */
		(void)Motor_SetTargetPositionMain(3, 75000, MOTOR_POS_ABSOLUTE);
		(void)Motor_WaitArrive(3, 10000);

		/* 9 号（下标 2）绝对定位到 50 cm，7/8 号同向同距跟随；
		   到位判据只看 9 号的剩余位移。 */
		s_lastAxisForwardResult = (int8_t)MoveAxisAbsoluteWithBelts(2, 0, 1,
		                                                           AXIS_FORWARD_TARGET,
		                                                           MOTOR_RETURN_TIMEOUT_MS);

		if (s_lastAxisForwardResult == 0)
		{
			/* 最后 9 号单独回绝对 0，7/8 号不动 */
			(void)Motor_SetTargetPositionMain(2, 0, MOTOR_POS_ABSOLUTE);
			s_lastAxisZeroResult = (int8_t)Motor_WaitArrive(2, MOTOR_RETURN_TIMEOUT_MS);
		}
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
