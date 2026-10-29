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
#define MOTOR_INIT_STACK_SIZE   128     /* 字：Motor_InitAll + CAN_Send */

/* Private variables ---------------------------------------------------------*/
static MotorInfo s_motors[MOTOR_MAX_COUNT];
static uint16_t s_lastErrorCode;
static uint32_t s_errorFrameCount;
static int s_initResult = MOTOR_INIT_PENDING;

/* 0x0316 参数预设：协议表的参考值 B4 BF CF AA 00 5F 拆成各字段。
   设备号与马达号由 Motor_InitAll() 逐台填写，不在这里固定。 */
static const MotorConfig s_defaultParams =
{
	.reserved0 = 0x0B,
	.polePairs = 4,
	.speedP = 0x0B,
	.speedI = 0x0F,
	.torqueP = 0x0C,
	.torqueI = 0x0F,
	.currentCw = 0x0A,
	.currentCcw = 0x0A,
	.brake = 0,
	.damping = 0,
	.speedD = 0x05,
	.torqueD = 0x0F,
	.reverse = 0,
	.autoUnlock = 0,
	.disableRemainPos = 0,
	.disableRemainTorque = 0,
	.motorIndex = 0,
	.deviceId = 0
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
static void Motor_InitTask(void const *argument);

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
  * @brief  清空全部电机状态、按 MOTOR_DEV_BASE 给已配置电机分配地址
  *         （设备号 = 马达号 = 基准 + 下标，其余表项标记为未分配），
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
		/* 0xFF = 未分配：若沿用下标，设备号 7 会同时属于下标 0 和 7 */
		if (i < MOTOR_COUNT)
		{
			s_motors[i].devId = (uint8_t)(MOTOR_DEV_BASE + i);
			s_motors[i].motorId = (uint8_t)(MOTOR_DEV_BASE + i);
		}
		else
		{
			s_motors[i].devId = MOTOR_ADDR_UNASSIGNED;
			s_motors[i].motorId = MOTOR_ADDR_UNASSIGNED;
		}
	}

	s_lastErrorCode = 0;
	s_errorFrameCount = 0;

	CAN_SetFrameHandler(Motor_ParseFrame);
}

/**
  * @brief  按 MOTOR_COUNT / MOTOR_DEV_BASE 逐台初始化：写参数 → 配轮径 → 使能。
  *         地址取 Motor_Init() 登记的值（可用 Motor_SetAddress 覆盖），
  *         SN 已在电机侧固化，不再绑定。
  *         整个过程不插入延时：发送是全阻塞的，每条帧离开邮箱后才返回。
  * @param  None
  * @retval 0 所有帧已发出，-1 至少一帧发送失败
  */
int Motor_InitAll(void)
{
	MotorConfig params;
	uint8_t i;
	int result = 0;

	for (i = 0; i < MOTOR_COUNT; i++)
	{
		params = s_defaultParams;
		params.deviceId = s_motors[i].devId;
		params.motorIndex = (uint8_t)(s_motors[i].motorId & 0x0Fu);

		if (Motor_ConfigParams(i, &params) != 0)
		{
			result = -1;
		}
		if (Motor_ConfigWheelDiameter(i, MOTOR_WHEEL_DIAMETER_001MM) != 0)
		{
			result = -1;
		}
		if (Motor_SetEnabled(i, 1) != 0)
		{
			result = -1;
		}
	}

	return result;
}

/**
  * @brief  创建一次性初始化任务。调度器启动后才会真正跑，因此不在
  *         osKernelStart() 之前占用主线程，也不阻塞 LED 与接收任务。
  * @param  None
  * @retval None
  */
void Motor_InitTaskCreate(void)
{
	osThreadDef(MotorInit, Motor_InitTask, osPriorityNormal, 0, MOTOR_INIT_STACK_SIZE);
	osThreadCreate(osThread(MotorInit), NULL);
}

/**
  * @brief  初始化结果。
  * @param  None
  * @retval 0 成功，-1 至少一帧发送失败，MOTOR_INIT_PENDING 还没跑完
  */
int Motor_GetInitResult(void)
{
	return s_initResult;
}

/* FreeRTOS tasks -------------------------------------------------------------*/

/**
  * @brief  初始化任务：跑一遍 Motor_InitAll() 就删除自己。
  *         失败结果留在 s_initResult 里，由 Motor_GetInitResult() 读取。
  * @param  argument 未使用
  * @retval None
  */
static void Motor_InitTask(void const *argument)
{
	(void) argument;

	s_initResult = Motor_InitAll();
	vTaskDelete(NULL);
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
  * @brief  0x0316 写入电机参数（初始化）。Byte6 的 bit5/bit4 = 0 才开启周期反馈，
  *         bit3..0 是马达号；Byte7 是设备号。无应答，只能靠运动行为验证。
  * @param  motorIndex: 下标
  * @param  config: 参数字节；报文里的设备号/马达号取自 config
  * @retval 0 已发出，-1 失败
  */
int Motor_ConfigParams(uint8_t motorIndex, const MotorConfig *config)
{
	uint8_t data[8];

	if ((motorIndex >= MOTOR_MAX_COUNT) || (config == NULL))
	{
		return -1;
	}

	data[0] = (uint8_t)(((config->reserved0 & 0x0Fu) << 4) | (config->polePairs & 0x0Fu));
	data[1] = (uint8_t)(((config->speedP & 0x0Fu) << 4) | (config->speedI & 0x0Fu));
	data[2] = (uint8_t)(((config->torqueP & 0x0Fu) << 4) | (config->torqueI & 0x0Fu));
	data[3] = (uint8_t)(((config->currentCw & 0x0Fu) << 4) | (config->currentCcw & 0x0Fu));
	data[4] = (uint8_t)(((config->brake & 0x01u) << 1) | (config->damping & 0x01u));
	data[5] = (uint8_t)(((config->speedD & 0x0Fu) << 4) | (config->torqueD & 0x0Fu));
	data[6] = (uint8_t)(((config->reverse & 0x01u) << 7) |
	                    ((config->autoUnlock & 0x01u) << 6) |
	                    ((config->disableRemainPos & 0x01u) << 5) |
	                    ((config->disableRemainTorque & 0x01u) << 4) |
	                    (config->motorIndex & 0x0Fu));
	data[7] = config->deviceId;

	if (CAN_Send(MOTOR_ID_CONFIG_PARAM, data, 8) != 0)
	{
		return -1;
	}

	/* 电机从此按这两个地址应答，状态表跟着改，否则后续收发会找错电机 */
	s_motors[motorIndex].devId = config->deviceId;
	s_motors[motorIndex].motorId = config->motorIndex & 0x0Fu;
	return 0;
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
