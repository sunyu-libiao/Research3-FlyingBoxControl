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
  *   2. 自动上报大约 10 秒后自行停止。实测：停掉之后单独重发“开启”不能把流
  *      拉回来，必须先关再开——Tof_PollTask 周期性做一次关→开脉冲来重启上报流。
  *      这个“必须先关再开”的结论来自现场调试，未经单变量对照实验验证。
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
#define TOF_POLL_STACK_SIZE     128     /* 字：每轮只拼两帧就睡，很浅   */

/* 配置帧之后到发出第一条测量命令之间的间隔。模块要时间把 0x0316 的参数写进去，
   紧跟的通道命令会被丢掉，表现为“配置了但始终无读数”。 */
#define TOF_CMD_GAP_MS          100u

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
static int sendContinuous(uint8_t index, uint8_t enable);
static void storeDistance(uint8_t index, uint16_t raw);
static void parseResponse(const CAN_Frame *frame);
static void parseReport(const CAN_Frame *frame);
static void Tof_InitTask(void const *argument);
static void Tof_PollTask(void const *argument);

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
  *         通道 10 的结构不一样，它有 ROI 字段，必须用 sendContinuous()。
  * @param  index: 下标
  * @param  channel: 通道号，TOF_CH_*
  * @param  value: B2，无意义时传 0
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
  * @brief  发通道 10（连续测距开关）。
  *
  *         这个通道的帧结构和其它通道不同：除了 B2 的开关，B3/B4 还必须
  *         带 ROI 起止。留 0 时模块会在不存在的区域里测距，读数恒为无效哨兵值，
  *         表现为“命令开了但永远测不到距离”。
  *         ROI 取值与 0x0316 配置帧保持一致，不另外写一套。
  * @param  index: 下标
  * @param  enable: 非 0 开，0 关
  * @retval 0 已发出，-1 失败
  */
static int sendContinuous(uint8_t index, uint8_t enable)
{
	uint8_t data[TOF_FRAME_LEN];

	if (index >= TOF_MAX_COUNT)
	{
		return -1;
	}

	memset(data, 0, sizeof(data));
	data[TOF_OFFSET_DEV] = s_tofs[index].devId;
	data[TOF_OFFSET_CHANNEL] = TOF_CH_CONTINUOUS;
	data[TOF_OFFSET_VALUE] = (enable != 0u) ? 1u : 0u;
	data[3] = s_defaultConfig.roiStart;
	data[4] = s_defaultConfig.roiEnd;

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
	/* 上报帧没有序号字段，本地自增；调用方靠 seq 变化判“是不是新样本” */
	s_tofs[index].seq++;
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
  * @brief  按默认配置初始化全部模块：先下发 0x0316 参数，再开启连续测距。
  *
  *         ★ 连续测距（通道 10）必须发。只配 0x0316 时模块不会 startRanging，
  *           通道 12 读到的永远是无效值 —— 这是“配置看起来成功但距离一直无效”
  *           的直接原因。
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

	/* 等参数真正写进模块，再开连续测距 */
	osDelay(TOF_CMD_GAP_MS);

	for (i = 0; i < TOF_COUNT; i++)
	{
		if (Tof_SetContinuous(i, 1) != 0)
		{
			result = -1;
		}
		osDelay(TOF_CMD_GAP_MS);
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
  *
  *         距离靠模块的 0x0793 自动上报获取，而上报流过一段时间会自行停止
  *         （连续测距本身不停）。重新把流拉起来需要先关再开，所以本函数
  *         在重启路径上会被连着调用两次，由 Tof_PollTask 周期执行。
  * @param  index: 下标
  * @param  enable: 非 0 开，0 关
  * @retval 0 已发出，-1 失败
  */
int Tof_SetContinuous(uint8_t index, uint8_t enable)
{
	return sendContinuous(index, enable);
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
  * @brief  初始化任务：等上电稳定后跑一遍 Tof_ConfigAll()，然后删除自己。
  * @param  argument 未使用
  * @retval None
  */
static void Tof_InitTask(void const *argument)
{
	(void) argument;

	/* TOF 模块上电复位需要时间，早于它发出的 0x0316 会被静默丢弃 */
	osDelay(TOF_INIT_SETTLE_MS);

	s_initResult = Tof_ConfigAll();
	vTaskDelete(NULL);
}

/**
  * @brief  上报流重启任务：按 TOF_RESTART_PERIOD_MS 对每个模块做一次“关→开”脉冲。
  *
  *         距离完全依赖模块的 0x0793 自动上报，而那个上报流过一段时间会
  *         自行停止，停下来之后 TofInfo 就冻在最后一次读数上（序号不再变）。
  *
  *         为什么不周期重发“开”：重复的开启命令不会让模块把上报流重新拉起来，
  *         必须先关再开才能从干净的起点重新进流。先关后开是顺手把停掉的和
  *         残留的状态一起清掉，比“开”再“开”更确定。
  *         两帧背靠背发，中间不插入间隔：CAN 保证模块按序收到，模块会把
  *         关闭先执行完再接开启。
  *
  *         不再周期发通道 12 请求读数：上报帧本身就是完整的距离数据，
  *         没必要再查一次。
  *
  *         用 osDelayUntil 而不是 osDelay：前者的下一个唤醒时刻是“上上次 +
  *         周期”，发送耗时与调度抖动不会累积成周期漂移；后者会让实际周期
  *         变成“周期 + 每轮开销”。
  * @param  argument 未使用
  * @retval None
  */
static void Tof_PollTask(void const *argument)
{
	uint8_t i;
	uint32_t wakeTick = osKernelSysTick();

	(void) argument;

	/* 模块收到 0x0316 之前不接受通道命令，先用短间隔等初始化任务跑完。
	   首次开启由 Tof_ConfigAll() 发，本任务只负责后续重启。 */
	while (Tof_GetInitResult() == TOF_INIT_PENDING)
	{
		osDelay(TOF_CMD_GAP_MS);
	}

	for (;;)
	{
		osDelayUntil(&wakeTick, TOF_RESTART_PERIOD_MS);

		/* 关→开重启上报流；逐模块处理，让每个模块的重启尽量紧凑 */
		for (i = 0; i < TOF_COUNT; i++)
		{
			(void)Tof_SetContinuous(i, 0);
			(void)Tof_SetContinuous(i, 1);
		}
	}
}

/**
  * @brief  创建上报流重启任务。调度器启动后才会真正跑。
  * @param  None
  * @retval None
  */
void Tof_PollTaskCreate(void)
{
	osThreadDef(TofPoll, Tof_PollTask, osPriorityNormal, 0, TOF_POLL_STACK_SIZE);
	osThreadCreate(osThread(TofPoll), NULL);
}
