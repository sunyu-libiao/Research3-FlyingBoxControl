/**
  ******************************************************************************
  * @file    tof.c
  * @brief   TOF8 激光测距模块驱动
  *
  * 发送：每个通道一个函数，只负责拼 8 字节请求、交给 CAN_Send()。
  * 接收：CAN 接收任务每取到一帧就调用 Tof_ParseFrame()，按设备号归到 TofInfo。
  *
  * 与 motor 共用 0x0408 / 0x0409 两个帧 ID，靠设备号区分：TOF 用 25 / 26，
  * 电机用 7~11，两个解析器各自只认自己表里的设备号，互不干扰。
  *
  * 协议文档（doc/tof8通讯协议.docx）与实际报文有出入，以 2026-10-29 实测为准：
  *   1. 发通道 10 开连续测距（B2=1）后，模块自动上报 0x0793，帧长 3 字节：
  *      B0 设备号 / B1~B2 距离 u16 LE。文档写的“B2~B3”是错的。
  *   2. 自动上报大约 10 秒后自行停止，需要重新发一次通道 10 才会再开。
  *   3. 0x0794（距离+温度）实测从未上报，不做解析。
  *   4. 通道 12 读数应答 0x0409 是 5 字节：
  *      B0 设备号 / B1 通道 0x0C / B2~B3 距离 u16 LE / B4 序号。
  *      B4 不是文档说的“新数据标志”，而是每次应答递增的计数（第二次读到 2）。
  *   5. 无效判据文档一处写“<=10”、一处写“<=1”，这里取更严格的 <=10，
  *      并同时认 8888 / 9999 两个哨兵值；若实测发现近距离被误判，改
  *      TOF_RAW_MIN_VALID 即可。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "../inc/tof.h"

/* 帧 ID ---------------------------------------------------------------------*/
#define TOF_ID_CONFIG           0x0316
#define TOF_ID_REQUEST          0x0408
#define TOF_ID_RESPONSE         0x0409

/* 请求帧固定 8 字节 */
#define TOF_FRAME_LEN           8

/* 响应帧里载荷的位置 */
#define TOF_OFFSET_DEV          0
#define TOF_OFFSET_CHANNEL      1
#define TOF_OFFSET_VALUE        2
#define TOF_OFFSET_SEQ          4       /* 0x0409 末字节：每次应答递增 */

/* 0x0793 自动上报：B0 设备号，B1~B2 距离（帧长 3） */
#define TOF_REPORT_OFFSET_DIST  1
#define TOF_REPORT_MIN_LEN      3

#define TOF_INIT_STACK_SIZE     128     /* 字                          */

/* Private variables ---------------------------------------------------------*/
static TofInfo s_tofs[TOF_MAX_COUNT];
static int s_initResult = TOF_INIT_PENDING;

/* 0x0316 默认配置：ROI 0x11~0x22、阈值 0xFF、不旋转、4x4 持续测量 */
static const TofConfig s_defaultConfig =
{
	.roiStart = 0x11,
	.roiEnd = 0x22,
	.reserved2 = 0x00,
	.threshold = 0xFF,
	.rotation = 0x00,
	.mode = (uint8_t)(TOF_MODE_4X4 | TOF_MODE_ALWAYS_MEASURE),
	.reserved6 = 0x00
};

/* Private function prototypes -----------------------------------------------*/
static uint8_t findIndexByDev(uint8_t devId);
static void touchTof(uint8_t index);
static int sendChannel(uint8_t index, uint8_t channel, uint8_t value);
static void storeDistance(uint8_t index, uint16_t raw);
static void parseResponse(const CAN_Frame *frame);
static void parseReport(const CAN_Frame *frame);
static void Tof_InitTask(void const *argument);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  按设备号查下标。
  * @param  devId: 设备号
  * @retval 下标，找不到返回 TOF_MAX_COUNT
  */
static uint8_t findIndexByDev(uint8_t devId)
{
	uint8_t i;

	for (i = 0; i < TOF_MAX_COUNT; i++)
	{
		if (s_tofs[i].devId == devId)
		{
			return i;
		}
	}

	return TOF_MAX_COUNT;
}

/**
  * @brief  记录一次收帧：刷新时间戳并累计帧数。
  * @param  index: 下标
  * @retval None
  */
static void touchTof(uint8_t index)
{
	s_tofs[index].rxCount++;
	s_tofs[index].lastRxTick = HAL_GetTick();
}

/**
  * @brief  发一条 0x0408 通道请求（B2 填 value，其余补 0）。
  * @param  index: 下标
  * @param  channel: 通道号，TOF_CH_*
  * @param  value: B2，用于通道 10 的开关；无意义时传 0
  * @retval 0 已发出，-1 失败
  */
static int sendChannel(uint8_t index, uint8_t channel, uint8_t value)
{
	uint8_t data[TOF_FRAME_LEN];

	if (index >= TOF_MAX_COUNT)
	{
		return -1;
	}

	memset(data, 0, sizeof(data));
	data[TOF_OFFSET_DEV] = s_tofs[index].devId;
	data[TOF_OFFSET_CHANNEL] = channel;
	data[TOF_OFFSET_VALUE] = value;

	return CAN_Send(TOF_ID_REQUEST, data, TOF_FRAME_LEN);
}

/**
  * @brief  保存一次测距结果，同时给出有效性判定。
  * @param  index: 下标
  * @param  raw: 原始 u16，0.1 mm
  * @retval None
  */
static void storeDistance(uint8_t index, uint16_t raw)
{
	s_tofs[index].rawDistance = raw;

	if ((raw <= TOF_RAW_MIN_VALID) ||
	    (raw == TOF_RAW_INVALID_LONG) ||
	    (raw == TOF_RAW_INVALID_SHORT))
	{
		s_tofs[index].valid = 0;
		s_tofs[index].distance = 0;
	}
	else
	{
		s_tofs[index].valid = 1;
		s_tofs[index].distance = raw;
	}
}

/**
  * @brief  处理 0x0409 应答：B0 设备号、B1 通道号，其余随通道变化。
  * @param  frame: 收到的帧
  * @retval None
  */
static void parseResponse(const CAN_Frame *frame)
{
	uint8_t index;
	uint8_t channel;

	if (frame->len < 2u)
	{
		return;
	}

	index = findIndexByDev(frame->data[TOF_OFFSET_DEV]);
	if (index >= TOF_MAX_COUNT)
	{
		return;         /* 不是 TOF 的应答 */
	}

	channel = frame->data[TOF_OFFSET_CHANNEL];
	s_tofs[index].lastChannel = channel;

	switch (channel)
	{
	case TOF_CH_DISTANCE:
		if (frame->len >= 4u)
		{
			storeDistance(index,
			              (uint16_t)(frame->data[2] | ((uint16_t)frame->data[3] << 8)));
		}
		/* 末字节实测是递增序号，不是“新数据标志” */
		if (frame->len >= 5u)
		{
			s_tofs[index].seq = frame->data[TOF_OFFSET_SEQ];
		}
		break;
	case TOF_CH_RESOLUTION:
		if (frame->len >= 3u)
		{
			s_tofs[index].resolution = frame->data[TOF_OFFSET_VALUE];
		}
		break;
	case TOF_CH_FREQUENCY:
		if (frame->len >= 3u)
		{
			s_tofs[index].frequency = frame->data[TOF_OFFSET_VALUE];
		}
		break;
	default:
		/* 通道 8/9/10/11：只把 B2 记下来，含义随通道而定 */
		if (frame->len >= 3u)
		{
			s_tofs[index].lastCommand = frame->data[TOF_OFFSET_VALUE];
		}
		break;
	}

	touchTof(index);
}

/**
  * @brief  处理 0x0793 自动上报（帧长 3）：B0 设备号、B1~B2 距离。
  * @param  frame: 收到的帧
  * @retval None
  */
static void parseReport(const CAN_Frame *frame)
{
	uint8_t index;

	if (frame->len < TOF_REPORT_MIN_LEN)
	{
		return;
	}

	index = findIndexByDev(frame->data[TOF_OFFSET_DEV]);
	if (index >= TOF_MAX_COUNT)
	{
		return;
	}

	storeDistance(index, (uint16_t)(frame->data[TOF_REPORT_OFFSET_DIST] |
	                                ((uint16_t)frame->data[TOF_REPORT_OFFSET_DIST + 1u] << 8)));
	s_tofs[index].reportCount++;
	touchTof(index);
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  清空状态、按下标分配设备号，并把本文件的解析函数注册到 CAN。
  * @param  None
  * @retval None
  */
void Tof_Init(void)
{
	uint8_t i;

	memset(s_tofs, 0, sizeof(s_tofs));
	for (i = 0; i < TOF_MAX_COUNT; i++)
	{
		/* 0xFF = 未分配：沿用下标会让设备号 25 同时属于下标 0 和 25 */
		s_tofs[i].devId = (i < TOF_COUNT) ? (uint8_t)(TOF_DEV_BASE + i) : 0xFFu;
	}

	s_initResult = TOF_INIT_PENDING;

	CAN_AddFrameHandler(Tof_ParseFrame);
}

/**
  * @brief  按下标分配设备号，用于覆盖默认的 TOF_DEV_BASE + 下标。
  * @param  index: 下标
  * @param  devId: 设备号
  * @retval None
  */
void Tof_SetAddress(uint8_t index, uint8_t devId)
{
	if (index >= TOF_MAX_COUNT)
	{
		return;
	}

	s_tofs[index].devId = devId;
}

/**
  * @brief  取某个下标的只读状态。
  * @param  index: 下标
  * @retval 状态指针，越界返回 NULL
  */
const TofInfo *Tof_GetInfo(uint8_t index)
{
	if (index >= TOF_MAX_COUNT)
	{
		return NULL;
	}

	return &s_tofs[index];
}

/**
  * @brief  解析一帧并更新对应模块状态。在 CAN 接收任务上下文运行。
  * @param  frame: 收到的帧
  * @retval None
  */
void Tof_ParseFrame(const CAN_Frame *frame)
{
	if (frame == NULL)
	{
		return;
	}

	if (frame->id == TOF_ID_RESPONSE)
	{
		parseResponse(frame);
	}
	else if (frame->id == TOF_ID_REPORT_DISTANCE)
	{
		parseReport(frame);
	}
	/* 0x0794 实测未出现且载荷布局与 0x0793 不同，不解析；
	   其它帧 ID 不是 TOF 的，直接放掉 */
}

/**
  * @brief  0x0316 下发配置帧。
  * @param  index: 下标
  * @param  config: 配置内容，B7 由驱动填设备号
  * @retval 0 已发出，-1 失败
  */
int Tof_Config(uint8_t index, const TofConfig *config)
{
	uint8_t data[TOF_FRAME_LEN];

	if ((index >= TOF_MAX_COUNT) || (config == NULL))
	{
		return -1;
	}

	data[0] = config->roiStart;
	data[1] = config->roiEnd;
	data[2] = config->reserved2;
	data[3] = config->threshold;
	data[4] = config->rotation;
	data[5] = config->mode;
	data[6] = config->reserved6;
	data[7] = s_tofs[index].devId;

	return CAN_Send(TOF_ID_CONFIG, data, TOF_FRAME_LEN);
}

/**
  * @brief  按默认配置初始化全部模块。
  * @param  None
  * @retval 0 所有帧已发出，-1 至少一帧发送失败
  */
int Tof_ConfigAll(void)
{
	uint8_t i;
	int result = 0;

	for (i = 0; i < TOF_COUNT; i++)
	{
		if (Tof_Config(i, &s_defaultConfig) != 0)
		{
			result = -1;
		}
	}

	return result;
}

/**
  * @brief  创建一次性初始化任务。调度器启动后才会真正跑。
  * @param  None
  * @retval None
  */
void Tof_InitTaskCreate(void)
{
	osThreadDef(TofInit, Tof_InitTask, osPriorityNormal, 0, TOF_INIT_STACK_SIZE);
	osThreadCreate(osThread(TofInit), NULL);
}

/**
  * @brief  初始化结果。
  * @param  None
  * @retval 0 成功，-1 至少一帧发送失败，TOF_INIT_PENDING 还没跑完
  */
int Tof_GetInitResult(void)
{
	return s_initResult;
}

/**
  * @brief  0x0408 通道 10：连续测距开关。
  * @param  index: 下标
  * @param  enable: 非 0 开，0 关
  * @retval 0 已发出，-1 失败
  */
int Tof_SetContinuous(uint8_t index, uint8_t enable)
{
	return sendChannel(index, TOF_CH_CONTINUOUS, (enable != 0u) ? 1u : 0u);
}

/**
  * @brief  0x0408 通道 11：启动单次测距。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_StartSingle(uint8_t index)
{
	return sendChannel(index, TOF_CH_SINGLE, 0);
}

/**
  * @brief  0x0408 通道 12：请求读数。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_RequestDistance(uint8_t index)
{
	return sendChannel(index, TOF_CH_DISTANCE, 0);
}

/**
  * @brief  0x0408 通道 8：复位 VL53Lx。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_ResetSensor(uint8_t index)
{
	return sendChannel(index, TOF_CH_RESET_SENSOR, 0);
}

/**
  * @brief  0x0408 通道 9：复位 MCU。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_ResetMcu(uint8_t index)
{
	return sendChannel(index, TOF_CH_RESET_MCU, 0);
}

/**
  * @brief  0x0408 通道 15：查询分辨率，应答写入 TofInfo.resolution。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_QueryResolution(uint8_t index)
{
	return sendChannel(index, TOF_CH_RESOLUTION, 0);
}

/**
  * @brief  0x0408 通道 16：查询频率，应答写入 TofInfo.frequency。
  * @param  index: 下标
  * @retval 0 已发出，-1 失败
  */
int Tof_QueryFrequency(uint8_t index)
{
	return sendChannel(index, TOF_CH_FREQUENCY, 0);
}

/* FreeRTOS tasks -------------------------------------------------------------*/

/**
  * @brief  初始化任务：跑一遍 Tof_ConfigAll() 就删除自己。
  * @param  argument 未使用
  * @retval None
  */
static void Tof_InitTask(void const *argument)
{
	(void) argument;

	s_initResult = Tof_ConfigAll();
	vTaskDelete(NULL);
}
