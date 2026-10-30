/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机 CAN 协议（V1.1）驱动
  *
  * 发送：协议里每个帧 ID 一个函数，只负责拼载荷、交给 CAN_Send()。
  * 接收：CAN 接收任务每取到一帧就调用 Motor_ParseFrame()，按帧 ID 分派到
  *       对应电机的 MotorInfo。
  *
  * 载荷一律按小端解释。
  *
  * 协议里两个地址字段都被写成“设备号”，实测含义不同（见 motor.h 的 MotorInfo）：
  *   运动帧 0x0413~0x0416 的 Data[4]、以及 0x0420/0x0440 的帧 ID 跟马达号；
  *   0x0380 心跳、0x0408/0x0409 的 Data[0]、0x0313 Data[7] 跟设备号。
  *
  * 与 xlsx 不一致、以实测为准的几处（doc/电机CAN控制流程.md）：
  *   1. 0x041A 的 ACK 是 3 字节：Data[0~1] 原始帧 ID，Data[2] 马达号；
  *      协议表写的“2 字节”漏掉了末字节，且末字节不是设备号。
  *   2. 0x0409 应答是 Data[0]=设备号、Data[1]=查询子码、Data[2..]=载荷；
  *      所以不需要记住在途查询就能归属。Data[6~7] 含义未确认，不读。
  *   3. 错误上报的帧 ID 本身就是错误码；0x0100 [34 12] 是“运动指令受理”标记，
  *      不是错误，别据此判失败。
  *   4. 0x0420 是“到目标点的剩余量”（减到 0 即到位），不是绝对位置；
  *      要绝对位置只能发 0x0408 子码 0x08 查询。
  *   5. 周期反馈帧由 0x0316 的 Byte6 bit5/bit4 = 0 开启，与 0x0413 使能无关；
  *      0x0413 只决定电机能否被驱动，发 [4A] 释放后反馈流仍在。
  *   6. 0x0408 配轮径（子码 0x24）实测会回 0x0409，xlsx 写“无应答”是错的；
  *      而且不配轮径运动指令不会执行。
  *   7. 0x060D 查 SN 实测无应答；0x0312 由电机上电主动广播，不带地址，未解析。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "../inc/motor.h"

/* 帧 ID ---------------------------------------------------------------------*/
#define MOTOR_ID_ARM            0x0403
#define MOTOR_ID_MULTI          0x0408  /* 轮径 / 蜂鸣 / 查询共用这一个 ID */
#define MOTOR_ID_CONFIG_PARAM   0x0316
#define MOTOR_ID_BIND_SN        0x0313
#define MOTOR_ID_QUERY_SN       0x060D
#define MOTOR_ID_ARM_MOVE       0x0700
#define MOTOR_ID_LOCK           0x0413
#define MOTOR_ID_SET_ACCEL_MAIN 0x0414
#define MOTOR_ID_SET_SPEED_MAIN 0x0415
#define MOTOR_ID_SET_POS_MAIN   0x0416
#define MOTOR_ID_SET_ACCEL_SUB  0x0417
#define MOTOR_ID_SET_SPEED_SUB  0x0418
#define MOTOR_ID_SET_POS_SUB    0x0419
#define MOTOR_ID_HEARTBEAT      0x0380  /* + 设备号 */
#define MOTOR_ID_REMAIN_POS     0x0420  /* + 马达号 */
#define MOTOR_ID_TORQUE         0x0440  /* + 马达号 */
#define MOTOR_ID_STATUS         0x0409
#define MOTOR_ID_ACK            0x041A
#define MOTOR_ID_ERROR_MIN      0x0100
#define MOTOR_ID_ERROR_MAX      0x017F

/* 0x0408 的 Data[1] 子功能 */
#define MOTOR_SUB_WHEEL         0x24
#define MOTOR_SUB_BEEP          0x07

/* 0x0403 的 Data[1] 子功能 */
#define MOTOR_SUB_ARM_MEMBERS   0x40
#define MOTOR_SUB_ARM_ZERO      0x41

#define MOTOR_ARM_SLOT_MAX      6       /* Data[2~7] 能放几个设备号 */
#define MOTOR_SN_LEN            7

/* Private variables ---------------------------------------------------------*/
static MotorInfo s_motors[MOTOR_MAX_COUNT];
static uint16_t s_lastErrorCode;
static uint32_t s_errorFrameCount;

/* 每台电机一份默认配置模板（Flash）。下标与状态表一致：0 -> 设备号 MOTOR_DEV_BASE。
   只填配置区，状态区隐式为 0；Motor_Init() 整份拷入 s_motors 后成为可读的
   单一数据源，收发与配置都从这里取值，不再有第二份副本需要同步。
   地址（devId/motorId）不在这里写死：仍由 MOTOR_DEV_BASE 推导，
   避免同一个基准在两处维护。 */
#define MOTOR_PARAMS_DEFAULT \
	.reserved0 = 0x0B,               \
	.polePairs = 4,                  \
	.speedP = 0x0B,                  \
	.speedI = 0x0F,                  \
	.torqueP = 0x0C,                 \
	.torqueI = 0x0F,                 \
	.currentCw = 0x0A,               \
	.currentCcw = 0x0A,              \
	.brake = 0,                      \
	.damping = 0,                    \
	.speedD = 0x05,                  \
	.torqueD = 0x0F,                 \
	.autoUnlock = 0,                 \
	.disableRemainPos = 0,           \
	.disableRemainTorque = 0

static const MotorInfo s_defaultMotors[MOTOR_COUNT] =
{
	/* 下标 0 / 设备号 7，控制一根皮带滑动，与 9 号同在一条水平轴上，
	   无绝对位置需求。轮径不是机械直径：按皮带实测行程反算出的
	   "电机每转对应的皮带行程"。
	   速度与加速度跟 9 号取同一个值：协议这两个字段是轮缘弧长量，轮径
	   已经标定成真实行程，所以相同数值就是相同的物理速度，与轮径大小无关。 */
	{
		.reverse = 0,
		.wheelDiameter001mm = 3180u,
		.speed001mms = 3200u,
		.accel001mms2 = 1600u,
		.params = { MOTOR_PARAMS_DEFAULT },
		.homeTravel001mm = 0,
		.homeSpeed001mms = 3200u,
		.homeAccel001mms2 = 1600u
	},
	/* 下标 1 / 设备号 8，控制另一根皮带滑动，安装方向相反，与 9 号同在
	   一条水平轴上，无绝对位置需求。标定与下标 0 相同（同一套传动）。 */
	{
		.reverse = 1,
		.wheelDiameter001mm = 3180u,
		.speed001mms = 3200u,
		.accel001mms2 = 1600u,
		.params = { MOTOR_PARAMS_DEFAULT },
		.homeTravel001mm = 0,
		.homeSpeed001mms = 3200u,
		.homeAccel001mms2 = 1600u
	},
	/* 下标 2 / 设备号 9，安装方向相反，向后找末端。
	   这个值就是命令电机走的距离，要够从最远端走到头；等待时长也按它算，
	   所以不能多填。符号表示“朝哪边找”，反向位会翻转内部正转方向，
	   符号要不要跟着反向位走必须实测确认。
	   这里的轮径不是机械直径：按实测行程反算出的“电机每转对应的前进量”，
	   与其余四台的默认值不是一个量级。位移/速度/加速度都是轮缘弧长量，
	   必须与这个轮径同步标定，改一个就要改全部。 */
	{
		.reverse = 1,
		.wheelDiameter001mm = 622u,
		.speed001mms = 3200u,
		.accel001mms2 = 1600u,
		.params = { MOTOR_PARAMS_DEFAULT },
		.homeTravel001mm = -7257,
		.homeSpeed001mms = 3200u,
		.homeAccel001mms2 = 1600u
	},
	/* 下标 3 / 设备号 10，暂不处理 */
	{
		.reverse = 0,
		.wheelDiameter001mm = 60000u,
		.speed001mms = 160000u,
		.accel001mms2 = 80000u,
		.params = { MOTOR_PARAMS_DEFAULT },
		.homeTravel001mm = -700000,
		.homeSpeed001mms = MOTOR_HOME_SPEED_001MMS,
		.homeAccel001mms2 = MOTOR_HOME_ACCEL_001MMS2
	},
	/* 下标 4 / 设备号 11，安装方向相反，向后找末端 */
	{
		.reverse = 1,
		.wheelDiameter001mm = 60000u,
		.speed001mms = 160000u,
		.accel001mms2 = 80000u,
		.params = { MOTOR_PARAMS_DEFAULT },
		.homeTravel001mm = -400000,
		.homeSpeed001mms = MOTOR_HOME_SPEED_001MMS,
		.homeAccel001mms2 = MOTOR_HOME_ACCEL_001MMS2
	}
};

/* Private function prototypes -----------------------------------------------*/
static uint32_t getU32(const uint8_t *data);
static void putU32(uint8_t *data, uint32_t value);
static uint32_t getLeValue(const uint8_t *data, uint8_t len);
static uint8_t findIndexByDev(uint8_t devId);
static uint8_t findIndexByMotor(uint8_t motorId);
static void touchMotor(uint8_t motorIndex);
static int sendQuery(uint8_t motorIndex, uint8_t queryKind);
static void parseQueryResponse(const CAN_Frame *frame);
static uint32_t homeWaitMsFor(int32_t travel, uint32_t speed001mms, uint32_t accel001mms2);
static int configOneMotor(uint8_t motorIndex, uint32_t speed001mms, uint32_t accel001mms2);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  小端读 32 位。
  */
static uint32_t getU32(const uint8_t *data)
{
	return (uint32_t)data[0] |
	       ((uint32_t)data[1] << 8) |
	       ((uint32_t)data[2] << 16) |
	       ((uint32_t)data[3] << 24);
}

/**
  * @brief  小端写 32 位。
  */
static void putU32(uint8_t *data, uint32_t value)
{
	data[0] = (uint8_t)(value & 0xFFu);
	data[1] = (uint8_t)((value >> 8) & 0xFFu);
	data[2] = (uint8_t)((value >> 16) & 0xFFu);
	data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

/**
  * @brief  按实际长度取最多 4 字节的小端值。查询响应长度随查询类型变化，
  *         用定长读越界的风险由这里的长度参数挡住。
  */
static uint32_t getLeValue(const uint8_t *data, uint8_t len)
{
	uint32_t value = 0;
	uint8_t n = (len > 4u) ? 4u : len;
	uint8_t i;

	for (i = 0; i < n; i++)
	{
		value |= (uint32_t)data[i] << (8u * i);
	}

	return value;
}

/**
  * @brief  按设备号查下标。
  * @param  devId: 设备号
  * @retval 下标，找不到返回 MOTOR_MAX_COUNT
  */
static uint8_t findIndexByDev(uint8_t devId)
{
	uint8_t i;

	for (i = 0; i < MOTOR_MAX_COUNT; i++)
	{
		if (s_motors[i].devId == devId)
		{
			return i;
		}
	}

	return MOTOR_MAX_COUNT;
}

/**
  * @brief  按马达号查下标。
  * @param  motorId: 马达号
  * @retval 下标，找不到返回 MOTOR_MAX_COUNT
  */
static uint8_t findIndexByMotor(uint8_t motorId)
{
	uint8_t i;

	for (i = 0; i < MOTOR_MAX_COUNT; i++)
	{
		if (s_motors[i].motorId == motorId)
		{
			return i;
		}
	}

	return MOTOR_MAX_COUNT;
}

/**
  * @brief  记录一次收帧：刷新时间戳并累计帧数。
  * @param  motorIndex: 下标
  * @retval None
  */
static void touchMotor(uint8_t motorIndex)
{
	s_motors[motorIndex].rxCount++;
	s_motors[motorIndex].lastRxTick = HAL_GetTick();
}

/**
  * @brief  发一条 0x0408 查询。
  * @param  motorIndex: 下标
  * @param  queryKind: MOTOR_QUERY_*
  * @retval 0 已发出，-1 失败
  */
static int sendQuery(uint8_t motorIndex, uint8_t queryKind)
{
	uint8_t data[2];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	data[0] = s_motors[motorIndex].devId;
	data[1] = queryKind;
	return CAN_Send(MOTOR_ID_MULTI, data, 2);
}

/**
  * @brief  处理 0x0409 查询应答：Data[0]=设备号, Data[1]=子码, Data[2..]=载荷。
  * @param  frame: 收到的帧
  * @retval None
  */
static void parseQueryResponse(const CAN_Frame *frame)
{
	uint8_t motorIndex;
	uint32_t value;
	const uint8_t *payload;

	if (frame->len < 2u)
	{
		return;
	}

	motorIndex = findIndexByDev(frame->data[0]);
	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return;
	}

	/* 载荷长度随子码变化，用 getLeValue 的长度参数挡住越界读 */
	payload = (frame->len > 2u) ? &frame->data[2] : frame->data;
	value = getLeValue(payload, (uint8_t)((frame->len > 2u) ? (frame->len - 2u) : 0u));

	switch (frame->data[1])
	{
	case MOTOR_QUERY_POSITION:
		s_motors[motorIndex].position = (int32_t)value;
		s_motors[motorIndex].positionRxCount++;
		break;
	case MOTOR_QUERY_STATUS:
		s_motors[motorIndex].status = (uint8_t)value;
		break;
	case MOTOR_QUERY_TEMPERATURE:
		s_motors[motorIndex].temperature = (int16_t)value;
		break;
	case MOTOR_QUERY_FIRMWARE:
		s_motors[motorIndex].firmware = value;
		break;
	default:
		/* 子码 0x24（配轮径回显）等不影响状态 */
		break;
	}

	touchMotor(motorIndex);
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  清空全部电机状态，把 Flash 默认模板整份拷入配置区，按下标分配地址，
  *         并把本文件的解析函数注册成 CAN 的帧处理器。
  * @param  None
  * @retval None
  */
void Motor_Init(void)
{
	uint8_t i;

	memset(s_motors, 0, sizeof(s_motors));
	for (i = 0; i < MOTOR_MAX_COUNT; i++)
	{
		if (i < MOTOR_COUNT)
		{
			s_motors[i] = s_defaultMotors[i];          /* 配置区 + 清零的状态区 */
			s_motors[i].devId = (uint8_t)(MOTOR_DEV_BASE + i);
			s_motors[i].motorId = (uint8_t)(MOTOR_DEV_BASE + i);
		}
		else
		{
			/* 0xFF = 未分配：若沿用下标，设备号 7 会同时属于下标 0 和 7 */
			s_motors[i].devId = MOTOR_ADDR_UNASSIGNED;
			s_motors[i].motorId = MOTOR_ADDR_UNASSIGNED;
		}
	}

	s_lastErrorCode = 0;
	s_errorFrameCount = 0;

	CAN_AddFrameHandler(Motor_ParseFrame);
}

/**
  * @brief  按归位行程 / 归位速度 / 归位加速度估算走完该行程的耗时，单位 ms。
  *
  *         梯形速度曲线：
  *           d_acc = v^2 / (2a)      加速段走的距离
  *           t_acc = v / a           加速段耗时
  *           d <= d_acc → 全程都在加速，实际耗时 sqrt(2d/a) <= t_acc，
  *                        取 t_acc 作为上界；
  *           d >  d_acc → t_acc + (d - d_acc) / v。
  *
  *         距离取 homeTravel001mm 的绝对值——把它当作电机实际要走的距离，
  *         不需要知道机械尺寸。全部用 0.1 mm 系列单位，比值无量纲，乘 1000
  *         换算成毫秒。这里不加额外余量：余量靠 homeTravel001mm 多填。
  *
  *         ★ 加速段距离含 v^2，速度量级放大后积会超出 32 位，必须用 64 位算，
  *           否则静默溢出，等待时长会被算短。
  * @param  travel: homeTravel001mm，0.1 mm，符号无关
  * @param  speed001mms: 这一台的归位速度，0.1 mm/s
  * @param  accel001mms2: 这一台的归位加速度，0.1 mm/s^2
  * @retval 估算耗时，ms
  */
static uint32_t homeWaitMsFor(int32_t travel, uint32_t speed001mms, uint32_t accel001mms2)
{
	uint32_t distance = (uint32_t)((travel < 0) ? -travel : travel);
	uint32_t accelMs = (speed001mms * 1000u) / accel001mms2;
	uint32_t accelDist = (uint32_t)(((uint64_t)speed001mms * speed001mms) /
	                                (2u * accel001mms2));

	if (distance <= accelDist)
	{
		return accelMs;
	}

	return accelMs +
	       (((distance - accelDist) * 1000u) / speed001mms);
}

/**
  * @brief  给单台电机下发完整配置：参数 → 轮径 → 加速度 → 速度。
  *         速度与加速度由调用方给：归位阶段要用比运行值低得多的参数，
  *         否则小轮径下高速启动的电流峰值会直接触发过流保护。
  *         每帧之间插 MOTOR_INIT_FRAME_GAP_MS，会 osDelay。
  * @param  motorIndex: 下标
  * @param  speed001mms: 0x0415 速度，0.1 mm/s
  * @param  accel001mms2: 0x0414 加速度，0.1 mm/s^2
  * @retval 0 所有帧已发出，-1 至少一帧发送失败
  */
static int configOneMotor(uint8_t motorIndex, uint32_t speed001mms, uint32_t accel001mms2)
{
	int result = 0;

	if (Motor_ConfigParams(motorIndex) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	if (Motor_ConfigWheelDiameter(motorIndex, s_motors[motorIndex].wheelDiameter001mm) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	if (Motor_SetAccelerationMain(motorIndex, accel001mms2) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	if (Motor_SetSpeedMain(motorIndex, speed001mms) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	return result;
}

/**
  * @brief  按下标配置全部电机（运行参数）：参数 → 轮径 → 加速度 → 速度。
  *         载荷全部取自对应 MotorInfo 的配置区。SN 已在电机侧固化，不再绑定。
  *
  *         ★ 这里不发使能帧（0x0413）。使能由 Motor_BringUp() 统一发，
  *           这样才能控制某一时刻到底有哪几台在出力。
  *
  *         每帧之间插 MOTOR_INIT_FRAME_GAP_MS：电机处理参数/轮径写入需要时间，
  *         连续帧贴在一起时后面的帧会被忽略，表现为部分电机收不到配置。
  *         本函数会 osDelay，只能在任务上下文调用。
  * @param  None
  * @retval 0 所有帧已发出，-1 至少一帧发送失败
  */
int Motor_InitAll(void)
{
	uint8_t i;
	int result = 0;

	for (i = 0; i < MOTOR_COUNT; i++)
	{
		if (configOneMotor(i, s_motors[i].speed001mms, s_motors[i].accel001mms2) != 0)
		{
			result = -1;
		}
	}

	return result;
}


/**
  * @brief  单台归位：初始化这一台 → 使能 → 按 homeTravel001mm 发相对位移 →
  *         按同一段距离推算出的时间定时等待 → 清除残留目标 → 把位置定义成 0。
  *
  *         先重新初始化而不是沿用已有参数：电机上一次归位顶到末端触发过
  *         保护释放，内部的参数与输出级状态已不可信，必须重发 0x0316/0x0408
  *         才能正常驱动。这里用的是归位专用的低速低加速，跑正常运动前必须
  *         由 Motor_BringUp() 重新下发运行参数。
  *
  *         不做任何闭环判据。实测电机顶到机械末端会触发电机自身保护并断开
  *         输出，之后连 0x0420 剩余位移反馈都不再上报，任何"读到变化才动作"
  *         的判据都会永远等不到。改用定时后靠两件事保证正确：
  *           ① homeTravel001mm 就是电机实际要走的距离，时间按它算
  *              （见 homeWaitMsFor），到期时这段行程正好走完；
  *           ② 这个距离要够从最远端走到头（让电机自己触到保护），
  *              且正负号对应“向后”；余量也靠这里多填的距离给。
  *         宁可多等，不能早于到末端就置零。
  * @param  motorIndex: 下标
  * @retval 0 已置零，1 该电机不参与归位，-1 有帧发送失败
  */
int Motor_HomeByRelease(uint8_t motorIndex)
{
	int32_t travel;
	int result = 0;

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	travel = s_motors[motorIndex].homeTravel001mm;
	if (travel == 0)
	{
		return 1;
	}

	/* 用这一台的归位参数重新配置：归位速度通常低于运行值，小轮径下高速
	   启动的电流峰值很容易撞上过流保护。归位参数必须和轮径同量级，
	   否则同样的数值在标定过的轮径上会快出两个数量级。 */
	if (configOneMotor(motorIndex, s_motors[motorIndex].homeSpeed001mms,
	                   s_motors[motorIndex].homeAccel001mms2) != 0)
	{
		return -1;
	}

	/* 只使能这一台：其它电机保持释放，不会跟着动 */
	if (Motor_SetEnabled(motorIndex, 1) != 0)
	{
		return -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	if (Motor_SetTargetPositionMain(motorIndex, travel, MOTOR_POS_RELATIVE) != 0)
	{
		return -1;
	}

	/* 定时等待：时间按这段行程本身算，不是固定常数——改行程就会改上电耗时。
	   余量就来自多填的行程：多出的距离 / 速度 = 多等的时间。 */
	osDelay(homeWaitMsFor(travel, s_motors[motorIndex].homeSpeed001mms,
	                      s_motors[motorIndex].homeAccel001mms2));

	/* 清掉未完成的目标。0x0416 相对命令的目标 = 当前位置 + Δ，此时剩余位移
	   还很大，若直接置零并重新使能，电机会接着往后顶——必须先把目标改成
	   当前位置（相对 0 位移），它才会停在原地。 */
	if (Motor_SetTargetPositionMain(motorIndex, 0, MOTOR_POS_RELATIVE) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_HOME_SETTLE_MS);

	/* 把这个位置定义成坐标零点。0x0B 只改坐标系，电机不动。
	   若电机此时仍处于保护释放状态，这一帧可能不被受理——由 Motor_BringUp()
	   末尾统一使能后的置零步骤兜底。 */
	if (Motor_SetTargetPositionMain(motorIndex, 0, MOTOR_POS_SET_ZERO) != 0)
	{
		result = -1;
	}
	osDelay(MOTOR_INIT_FRAME_GAP_MS);

	return result;
}

/**
  * @brief  完整上电流程，按顺序串行完成：
  *           ① 等 MOTOR_INIT_SETTLE_MS，让电机完成上电复位；
  *           ② 逐台（homeTravel001mm 非 0 的）独立走一遍
  *              Motor_HomeByRelease：初始化 → 使能 → 向后跑到末端 → 置零；
  *           ③ 重新初始化全部电机（归位用的是低速低加速，必须改回运行参数）；
  *           ④ 使能全部电机；
  *           ⑤ 把此刻全部电机的当前位置设为绝对 0。
  *
  *         逐台做而不是同时做：同时顶到末端时 CAN 上只有帧 ID 的错误上报
  *         无法归属，出了问题查不出是哪一台；而且同时大电流更容易触发保护。
  *         第 ⑤ 步对已置零的那几台是幂等的，对不参与归位的电机则是把当前
  *         位置直接定为原点，同时兜住"归位阶段因电机仍在保护释放而丢失"的情况。
  *
  *         ★ 总耗时 = 上电稳定 + 各台(配置 + 使能 + 归位等待 + 稳定 + 帧间隔)
  *           + 整组重新初始化 + 全部使能 + 全部置零。
  *           其中归位等待由 homeTravel001mm 与归位速度/加速度推算（见
  *           homeWaitMsFor），所以改这几个参数或增删参与归位的台数都会改变
  *           上电耗时，必须重算，不要沿用旧结论。
  *
  *         本函数是阻塞的（逐台按行程等待），只能在任务上下文调用。
  * @param  None
  * @retval 累计失败帧次数
  */
int Motor_BringUp(void)
{
	uint8_t i;
	int failures = 0;

	/* 电机上电复位需要时间，早于它发出的 0x0316 会被静默丢弃 */
	osDelay(MOTOR_INIT_SETTLE_MS);

	for (i = 0; i < MOTOR_COUNT; i++)
	{
		if (Motor_HomeByRelease(i) < 0)
		{
			failures++;
		}
	}

	/* 归位阶段下发的是低速低加速，这里整组改回运行参数 */
	if (Motor_InitAll() != 0)
	{
		failures++;
	}

	for (i = 0; i < MOTOR_COUNT; i++)
	{
		if (Motor_SetEnabled(i, 1) != 0)
		{
			failures++;
		}
		osDelay(MOTOR_INIT_FRAME_GAP_MS);
	}

	for (i = 0; i < MOTOR_COUNT; i++)
	{
		if (Motor_SetTargetPositionMain(i, 0, MOTOR_POS_SET_ZERO) != 0)
		{
			failures++;
		}
		osDelay(MOTOR_INIT_FRAME_GAP_MS);
	}

	return failures;
}

/**
  * @brief  指定下标对应的设备号与马达号。两者相同时不用调用。
  * @param  motorIndex: 下标
  * @param  devId: 设备号（心跳、查询、反馈寻址）
  * @param  motorId: 马达号（运动帧与 0x0420/0x0440 寻址）
  * @retval None
  */
void Motor_SetAddress(uint8_t motorIndex, uint8_t devId, uint8_t motorId)
{
	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return;
	}

	s_motors[motorIndex].devId = devId;
	s_motors[motorIndex].motorId = motorId;
}

/**
  * @brief  取某个下标的只读状态。
  * @param  motorIndex: 下标
  * @retval 状态指针，越界返回 NULL
  */
const MotorInfo *Motor_GetInfo(uint8_t motorIndex)
{
	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return NULL;
	}

	return &s_motors[motorIndex];
}

/**
  * @brief  解析一帧并更新对应电机状态。在 CAN 接收任务上下文运行。
  * @param  frame: 收到的帧
  * @retval None
  */
void Motor_ParseFrame(const CAN_Frame *frame)
{
	uint32_t id;
	uint8_t index;

	if (frame == NULL)
	{
		return;
	}

	id = frame->id;

	/* ACK 的末字节是马达号，不是设备号 */
	if (id == MOTOR_ID_ACK)
	{
		if (frame->len < 3u)
		{
			return;
		}
		index = findIndexByMotor(frame->data[2]);
		if (index >= MOTOR_MAX_COUNT)
		{
			return;
		}
		s_motors[index].lastAckId = (uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8));
		s_motors[index].ackCount++;
		touchMotor(index);
		return;
	}

	/* 查询应答自带设备号与子码 */
	if (id == MOTOR_ID_STATUS)
	{
		parseQueryResponse(frame);
		return;
	}

	/* 错误上报：帧 ID 本身就是错误码，且不带地址，无法归属到具体电机。
	   0x0100 是运动指令受理标记，会伴随每次非零相对运动出现，不算错误。 */
	if ((id >= MOTOR_ID_ERROR_MIN) && (id <= MOTOR_ID_ERROR_MAX))
	{
		if (id != MOTOR_ID_ERROR_MIN)
		{
			s_lastErrorCode = (uint16_t)id;
			s_errorFrameCount++;
		}
		return;
	}

	if ((id >= MOTOR_ID_HEARTBEAT) && (id < MOTOR_ID_HEARTBEAT + MOTOR_ID_SPAN))
	{
		index = findIndexByDev((uint8_t)(id - MOTOR_ID_HEARTBEAT));
		if (index >= MOTOR_MAX_COUNT)
		{
			return;
		}
		s_motors[index].heartbeatCount++;
	}
	else if ((id >= MOTOR_ID_REMAIN_POS) && (id < MOTOR_ID_REMAIN_POS + MOTOR_ID_SPAN))
	{
		index = findIndexByMotor((uint8_t)(id - MOTOR_ID_REMAIN_POS));
		if (index >= MOTOR_MAX_COUNT)
		{
			return;
		}
		if (frame->len >= 8u)
		{
			s_motors[index].remainMain = (int32_t)getU32(&frame->data[0]);
			s_motors[index].remainSub = (int32_t)getU32(&frame->data[4]);
		}
	}
	else if ((id >= MOTOR_ID_TORQUE) && (id < MOTOR_ID_TORQUE + MOTOR_ID_SPAN))
	{
		index = findIndexByMotor((uint8_t)(id - MOTOR_ID_TORQUE));
		if (index >= MOTOR_MAX_COUNT)
		{
			return;
		}
		if (frame->len >= 4u)
		{
			s_motors[index].torque = (int16_t)(frame->data[2] | ((uint16_t)frame->data[3] << 8));
		}
	}
	else
	{
		return;    /* 不认识的帧：放掉 */
	}

	touchMotor(index);
}

/**
  * @brief  最近一次错误上报的错误码（帧 ID 本身）。0x0100 的受理标记不计入。
  * @param  None
  * @retval 错误码，见 MOTOR_ERR_*
  */
uint16_t Motor_GetLastErrorCode(void)
{
	return s_lastErrorCode;
}

/**
  * @brief  错误上报帧的累计数量。
  * @param  None
  * @retval 帧数
  */
uint32_t Motor_GetErrorFrameCount(void)
{
	return s_errorFrameCount;
}

/**
  * @brief  0x0416 设置主通道目标位置。
  * @param  motorIndex: 下标
  * @param  position: 目标位置，0.1 mm
  * @param  positionType: MOTOR_POS_*
  * @retval 0 已发出，-1 失败
  */
int Motor_SetTargetPositionMain(uint8_t motorIndex, int32_t position, uint8_t positionType)
{
	uint8_t data[6];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], (uint32_t)position);
	data[4] = s_motors[motorIndex].motorId;
	data[5] = positionType;
	return CAN_Send(MOTOR_ID_SET_POS_MAIN, data, 6);
}

/**
  * @brief  等这台走到目标点：轮询 0x0420 剩余位移，绝对值落入死区即认为到位。
  *
  *         只适用于初始化完成之后的常规运动。那时绝对 0 已经建立，运动不会
  *         去顶机械末端，电机不会触发保护释放，剩余位移会一路更新；归位阶段
  *         不能用这条判据——顶到末端触发保护后连 0x0420 都不再上报。
  *
  *         用死区而不是等于 0：位置量有量化，实测到位后剩余位移停在 0 附近
  *         一个到两个单位上且符号会抖，等“精确归零”会永远等不到。
  *         判据要求先看到超出死区的值、再看到它落入死区，否则会把命令之前
  *         留下的旧值当成就位；代价是距离短到两次反馈之间就走完时会报超时。
  * @param  motorIndex: 下标
  * @param  timeoutMs: 最长等待时间，ms
  * @retval 0 已到位，-1 超时，-2 下标越界
  */
int Motor_WaitArrive(uint8_t motorIndex, uint32_t timeoutMs)
{
	const MotorInfo *info;
	uint32_t elapsed = 0;
	uint8_t moving = 0;

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -2;
	}

	info = &s_motors[motorIndex];

	while (elapsed < timeoutMs)
	{
		if ((info->remainMain > MOTOR_ARRIVE_DEADBAND) ||
		    (info->remainMain < -MOTOR_ARRIVE_DEADBAND))
		{
			moving = 1;
		}
		else if (moving != 0)
		{
			return 0;
		}

		osDelay(MOTOR_ARRIVE_POLL_MS);
		elapsed += MOTOR_ARRIVE_POLL_MS;
	}

	return -1;
}

/**
  * @brief  0x0419 设置从通道目标位置（无目标类型，沿用电机侧设置）。
  * @param  motorIndex: 下标
  * @param  position: 目标位置，0.1 mm
  * @retval 0 已发出，-1 失败
  */
int Motor_SetTargetPositionSub(uint8_t motorIndex, int32_t position)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], (uint32_t)position);
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_SET_POS_SUB, data, 5);
}

/**
  * @brief  0x0415 设置主通道速度。
  * @param  motorIndex: 下标
  * @param  speed: 速度，0.1 mm/s
  * @retval 0 已发出，-1 失败
  */
int Motor_SetSpeedMain(uint8_t motorIndex, uint32_t speed)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], speed);
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_SET_SPEED_MAIN, data, 5);
}

/**
  * @brief  0x0418 设置从通道速度。
  * @param  motorIndex: 下标
  * @param  speed: 速度，0.1 mm/s
  * @retval 0 已发出，-1 失败
  */
int Motor_SetSpeedSub(uint8_t motorIndex, uint32_t speed)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], speed);
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_SET_SPEED_SUB, data, 5);
}

/**
  * @brief  0x0414 设置主通道加速度。
  * @param  motorIndex: 下标
  * @param  acceleration: 加速度，0.1 mm/s^2
  * @retval 0 已发出，-1 失败
  */
int Motor_SetAccelerationMain(uint8_t motorIndex, uint32_t acceleration)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], acceleration);
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_SET_ACCEL_MAIN, data, 5);
}

/**
  * @brief  0x0417 设置从通道加速度。
  * @param  motorIndex: 下标
  * @param  acceleration: 加速度，0.1 mm/s^2
  * @retval 0 已发出，-1 失败
  */
int Motor_SetAccelerationSub(uint8_t motorIndex, uint32_t acceleration)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	putU32(&data[0], acceleration);
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_SET_ACCEL_SUB, data, 5);
}

/**
  * @brief  0x0413 上电使能 / 下电释放。只决定电机能否被驱动，不影响反馈流；
  *         Data[4] 填马达号。
  * @param  motorIndex: 下标
  * @param  enable: 非 0 使能，0 释放
  * @retval 0 已发出，-1 失败
  */
int Motor_SetEnabled(uint8_t motorIndex, uint8_t enable)
{
	uint8_t data[5];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	memset(data, 0, sizeof(data));
	data[0] = (enable != 0u) ? MOTOR_CMD_ENABLE : MOTOR_CMD_RELEASE;
	data[4] = s_motors[motorIndex].motorId;
	return CAN_Send(MOTOR_ID_LOCK, data, 5);
}

/**
  * @brief  0x0316 写入电机参数（初始化）。载荷全部取自该下标的配置区：
  *         Byte6 bit7 = reverse、bit3..0 = motorId，Byte7 = devId，其余取自 params。
  *         无应答，写入是否生效只能靠运动行为验证，所以这里不回写状态表。
  * @param  motorIndex: 下标
  * @retval 0 已发出，-1 失败
  */
int Motor_ConfigParams(uint8_t motorIndex)
{
	const MotorInfo *m;
	uint8_t data[8];

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	m = &s_motors[motorIndex];

	data[0] = (uint8_t)(((m->params.reserved0 & 0x0Fu) << 4) | (m->params.polePairs & 0x0Fu));
	data[1] = (uint8_t)(((m->params.speedP & 0x0Fu) << 4) | (m->params.speedI & 0x0Fu));
	data[2] = (uint8_t)(((m->params.torqueP & 0x0Fu) << 4) | (m->params.torqueI & 0x0Fu));
	data[3] = (uint8_t)(((m->params.currentCw & 0x0Fu) << 4) | (m->params.currentCcw & 0x0Fu));
	data[4] = (uint8_t)(((m->params.brake & 0x01u) << 1) | (m->params.damping & 0x01u));
	data[5] = (uint8_t)(((m->params.speedD & 0x0Fu) << 4) | (m->params.torqueD & 0x0Fu));
	data[6] = (uint8_t)(((m->reverse & 0x01u) << 7) |
	                    ((m->params.autoUnlock & 0x01u) << 6) |
	                    ((m->params.disableRemainPos & 0x01u) << 5) |
	                    ((m->params.disableRemainTorque & 0x01u) << 4) |
	                    (m->motorId & 0x0Fu));
	data[7] = m->devId;

	return CAN_Send(MOTOR_ID_CONFIG_PARAM, data, 8);
}

/**
  * @brief  0x0403 配置机械臂成员。
  * @param  motorIndex: 发起控制的电机下标（报文 Data[0] 填它的设备号）
  * @param  motorIds: 参与机械臂控制的电机编号数组
  * @param  count: 个数，1~6
  * @retval 0 已发出，-1 失败
  */
int Motor_SetArmMembers(uint8_t motorIndex, const uint8_t *motorIds, uint8_t count)
{
	uint8_t data[8];
	uint8_t i;

	if ((motorIndex >= MOTOR_MAX_COUNT) || (motorIds == NULL) ||
	    (count == 0u) || (count > MOTOR_ARM_SLOT_MAX))
	{
		return -1;
	}

	data[0] = s_motors[motorIndex].devId;
	data[1] = MOTOR_SUB_ARM_MEMBERS;
	for (i = 0; i < count; i++)
	{
		data[2 + i] = motorIds[i];
	}

	return CAN_Send(MOTOR_ID_ARM, data, (uint8_t)(2u + count));
}

/**
  * @brief  0x0403 设置机械臂零点。
  * @param  motorIndex: 发起控制的电机下标（报文 Data[0] 填它的设备号）
  * @param  motorIds: 参与机械臂控制的电机编号数组
  * @param  count: 个数，1~6
  * @retval 0 已发出，-1 失败
  */
int Motor_SetArmZero(uint8_t motorIndex, const uint8_t *motorIds, uint8_t count)
{
	uint8_t data[8];
	uint8_t i;

	if ((motorIndex >= MOTOR_MAX_COUNT) || (motorIds == NULL) ||
	    (count == 0u) || (count > MOTOR_ARM_SLOT_MAX))
	{
		return -1;
	}

	data[0] = s_motors[motorIndex].devId;
	data[1] = MOTOR_SUB_ARM_ZERO;
	for (i = 0; i < count; i++)
	{
		data[2 + i] = motorIds[i];
	}

	return CAN_Send(MOTOR_ID_ARM, data, (uint8_t)(2u + count));
}

/**
  * @brief  0x0700 控制机械臂动作。
  * @param  delta1~delta4: 电机 1~4 的相对位移，0.1 mm
  * @retval 0 已发出，-1 失败
  */
int Motor_MoveArm(int16_t delta1, int16_t delta2, int16_t delta3, int16_t delta4)
{
	uint8_t data[8];
	int16_t delta[4];
	uint8_t i;

	delta[0] = delta1;
	delta[1] = delta2;
	delta[2] = delta3;
	delta[3] = delta4;

	for (i = 0; i < 4u; i++)
	{
		data[2u * i] = (uint8_t)((uint16_t)delta[i] & 0xFFu);
		data[(2u * i) + 1u] = (uint8_t)(((uint16_t)delta[i] >> 8) & 0xFFu);
	}

	return CAN_Send(MOTOR_ID_ARM_MOVE, data, 8);
}

/**
  * @brief  0x0408 配置轮径。实测不配轮径运动指令不会执行，协议要求重复写三次。
  *         三次之间插 MOTOR_INIT_FRAME_GAP_MS，本函数会 osDelay。
  * @param  motorIndex: 下标
  * @param  diameter: 轮径，0.01 mm
  * @retval 三次都发出返回 0，任一次失败返回 -1
  */
int Motor_ConfigWheelDiameter(uint8_t motorIndex, uint16_t diameter)
{
	uint8_t data[4];
	int result = 0;
	uint8_t i;

	if (motorIndex >= MOTOR_MAX_COUNT)
	{
		return -1;
	}

	data[0] = s_motors[motorIndex].devId;
	data[1] = MOTOR_SUB_WHEEL;
	data[2] = (uint8_t)(diameter & 0xFFu);
	data[3] = (uint8_t)((diameter >> 8) & 0xFFu);

	for (i = 0; i < 3u; i++)
	{
		if (CAN_Send(MOTOR_ID_MULTI, data, 4) != 0)
		{
			result = -1;
		}
		osDelay(MOTOR_INIT_FRAME_GAP_MS);
	}

	return result;
}

/**
  * @brief  0x0408 蜂鸣提示。
  * @param  motorIndex: 下标
  * @param  volume: 音量，取值由电机侧定义
  * @param  tone: 频率信息，Data[3~5] 原样透传
  * @retval 0 已发出，-1 失败
  */
int Motor_Beep(uint8_t motorIndex, uint8_t volume, const uint8_t tone[3])
{
	uint8_t data[6];

	if ((motorIndex >= MOTOR_MAX_COUNT) || (tone == NULL))
	{
		return -1;
	}

	data[0] = s_motors[motorIndex].devId;
	data[1] = MOTOR_SUB_BEEP;
	data[2] = volume;
	data[3] = tone[0];
	data[4] = tone[1];
	data[5] = tone[2];

	return CAN_Send(MOTOR_ID_MULTI, data, 6);
}

/**
  * @brief  0x0408 查询温度，应答写入 MotorInfo.temperature。
  * @param  motorIndex: 下标
  * @retval 0 已发出，-1 失败
  */
int Motor_QueryTemperature(uint8_t motorIndex)
{
	return sendQuery(motorIndex, MOTOR_QUERY_TEMPERATURE);
}

/**
  * @brief  0x0408 查询当前位置，应答写入 MotorInfo.position。
  *         `0x0420` 传来的是“到目标的剩余量”，绝对位置只能靠这条查询。
  * @param  motorIndex: 下标
  * @retval 0 已发出，-1 失败
  */
int Motor_QueryPosition(uint8_t motorIndex)
{
	return sendQuery(motorIndex, MOTOR_QUERY_POSITION);
}

/**
  * @brief  0x0408 查询状态，应答写入 MotorInfo.status。
  * @param  motorIndex: 下标
  * @retval 0 已发出，-1 失败
  */
int Motor_QueryStatus(uint8_t motorIndex)
{
	return sendQuery(motorIndex, MOTOR_QUERY_STATUS);
}

/**
  * @brief  0x0408 查询固件版本，应答写入 MotorInfo.firmware。
  * @param  motorIndex: 下标
  * @retval 0 已发出，-1 失败
  */
int Motor_QueryFirmware(uint8_t motorIndex)
{
	return sendQuery(motorIndex, MOTOR_QUERY_FIRMWARE);
}

/**
  * @brief  0x060D 获取设备 SN。实测这颗固件无应答，SN 只能靠上电 0x0312 抓取。
  * @param  deviceType: 设备类型
  * @retval 0 已发出，-1 失败
  */
int Motor_RequestSn(uint8_t deviceType)
{
	uint8_t data[2];

	data[0] = deviceType;
	data[1] = 0;
	return CAN_Send(MOTOR_ID_QUERY_SN, data, 2);
}

/**
  * @brief  0x0313 绑定设备 SN。
  * @param  motorIndex: 下标（报文 Data[7] 填它的设备号）
  * @param  sn: 7 字节 SN
  * @retval 0 已发出，-1 失败
  */
int Motor_BindSn(uint8_t motorIndex, const uint8_t *sn)
{
	uint8_t data[8];

	if ((motorIndex >= MOTOR_MAX_COUNT) || (sn == NULL))
	{
		return -1;
	}

	memcpy(data, sn, MOTOR_SN_LEN);
	data[7] = s_motors[motorIndex].devId;
	return CAN_Send(MOTOR_ID_BIND_SN, data, 8);
}
