/**
  ******************************************************************************
  * @file    tof.h
  * @brief   TOF8 激光测距模块驱动：测距状态 + 每个通道一个发送接口
  *
  * 依赖方向：tof -> can。can 不认识 TOF；与 motor 同一帧 ID 段也能共存，
  * 因为各模块只认领自己的设备号。
  * 协议来源：doc/tof8通讯协议.docx。
  ******************************************************************************
  */

#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can.h"

/* 状态表容量。设备号只在报文里出现（不是帧 ID 偏移），所以取值不受 4 bit 限制。 */
#define TOF_MAX_COUNT           8

/* ---- 本机 TOF 配置 ------------------------------------------------------ */

/* 两个模块，设备号 25 / 26（0x19 / 0x1A） */
#define TOF_COUNT               2
#define TOF_DEV_BASE            25

#if (TOF_COUNT) > (TOF_MAX_COUNT)
#error "TOF_COUNT 超过状态表容量 TOF_MAX_COUNT"
#endif

/* 上电后到发出第一条配置帧之前的等待。TOF 模块上电复位与收发器稳定都需要
   时间，过早下发 0x0316 会被静默丢弃（该帧无应答）。
   与 MOTOR_INIT_SETTLE_MS 相互独立，两个模块各自配置。 */
#define TOF_INIT_SETTLE_MS      3000u

/* 0x0408 请求里的通道号（B1），响应 0x0409 的 B1 回显同一个值 */
#define TOF_CH_RESET_SENSOR     8       /* 复位 VL53Lx */
#define TOF_CH_RESET_MCU        9       /* 复位 MCU */
#define TOF_CH_CONTINUOUS       10      /* 连续测距开关，B2 = 1 开 / 0 关 */
#define TOF_CH_SINGLE           11      /* 单次测距开始 */
#define TOF_CH_DISTANCE         12      /* 读数：B2~B3 距离、B4 新数据标志 */
#define TOF_CH_RESOLUTION       15      /* 响应 B2 = 16(4x4) / 64(8x8) */
#define TOF_CH_FREQUENCY        16      /* 响应 B2 = 频率 Hz */

/* 0x0316 配置帧里的模式位，可叠加 */
#define TOF_MODE_4X4            0x01
#define TOF_MODE_8X8            0x02
#define TOF_MODE_ALWAYS_MEASURE 0x04

/* 距离原始值（u16，0.1 mm）的无效判据：<= 10 或等于 8888/9999。
   0.1 mm 量程下 9999 换算过来是 999.9 mm。 */
#define TOF_RAW_MIN_VALID       10
#define TOF_RAW_INVALID_LONG    9999
#define TOF_RAW_INVALID_SHORT   8888

/* 自动上报帧（2026-10-29 实测）
   0x0793：帧长 3，B0 设备号，B1~B2 距离 u16 LE（0.1 mm）。
          只在通道 10 开连续测距后才会来，而且**停的是这个上报帧，
          不是连续测距本身**：上报停下后模块仍在持续测量，通道 12 主动
          查询依旧能拿到新读数。要维持上报流才需要重发通道 10。
   0x0794：文档里有（距离+温度），实测从未上报，未做解析。 */
#define TOF_ID_REPORT_DISTANCE  0x0793
#define TOF_ID_REPORT_DIST      0x0794  /* 实测未出现 */

/**
  * @brief  0x0316 配置帧（8 字节）。B7 由驱动填设备号。
  */
typedef struct
{
	uint8_t roiStart;           /* B0 ROI 起始，默认 0x11 */
	uint8_t roiEnd;             /* B1 ROI 结束，默认 0x22 */
	uint8_t reserved2;          /* B2 预留 */
	uint8_t threshold;          /* B3 光强阈值，默认 0xFF */
	uint8_t rotation;           /* B4 旋转角，默认 0x00 不旋转 */
	uint8_t mode;               /* B5 模式，TOF_MODE_* 叠加，默认 0x05 */
	uint8_t reserved6;          /* B6 预留 */
} TofConfig;

/**
  * @brief  单个模块的运行状态。以下标（0..TOF_COUNT-1）索引，Tof_GetInfo() 取用。
  *         下标 0 对应设备号 TOF_DEV_BASE，依次递增。
  */
typedef struct
{
	uint8_t  devId;

	/* 0x0409 通道 12 与 0x0793 共用的测距结果 */
	uint16_t rawDistance;       /* 原始 u16，0.1 mm */
	uint16_t distance;          /* 有效时为 rawDistance，无效时为 0 */
	uint8_t  valid;             /* 最近一次读数是否有效 */
	uint8_t  seq;               /* 每采到一帧读数就前进一次，用于判断是否为新样本。
	                               通道 12 用模块自带的序号，0x0793 上报帧无此字段，由驱动自增 */

	/* 0x0409 其它通道的应答 */
	uint8_t  lastChannel;       /* 最近一次应答的通道号，0 = 还没收到过 */
	uint8_t  lastCommand;       /* 通道 10 的开关回显 / 复位应答的 B2 */
	uint8_t  resolution;        /* 通道 15：16(4x4) / 64(8x8) */
	uint8_t  frequency;         /* 通道 16：Hz */

	/* 统计 */
	uint32_t rxCount;           /* 任何可识别帧的累计数 */
	uint32_t reportCount;       /* 0x0793 自动上报数 */
	uint32_t lastRxTick;        /* 最后一帧的 HAL_GetTick()，用于判离线 */
} TofInfo;

/* 初始化：清空状态、按下标分配设备号、把自己的解析函数注册到 CAN。
   必须在 CAN_Init() 之后、CAN_RxTaskCreate() 之前调用。 */
void Tof_Init(void);

/* 按下标分配设备号，默认就是 TOF_DEV_BASE + 下标。 */
void Tof_SetAddress(uint8_t index, uint8_t devId);

/* 取某个下标的只读状态，越界返回 NULL。 */
const TofInfo *Tof_GetInfo(uint8_t index);

/* CAN 接收回调本体，由 CAN 接收任务调用；Tof_Init() 已注册，通常无需手动调。 */
void Tof_ParseFrame(const CAN_Frame *frame);

/* ---- 配置与测量 -------------------------------------------------------- */

/* 0x0316 下发配置帧，设备号由下标决定。 */
int Tof_Config(uint8_t index, const TofConfig *config);

/* 按默认配置初始化全部模块：ROI 0x11~0x22、阈值 0xFF、不旋转、4x4 持续测量，
   随后开启连续测距（通道 10）。
   ★ 只下发 0x0316 而不开连续测距，模块不会开始测距，通道 12 读到的永远是
     无效值。这个函数会 osDelay，只能在任务上下文调用。
   返回 0 表示所有帧都已发出，-1 表示至少一帧发送失败。 */
int Tof_ConfigAll(void);

/* 创建一次性初始化任务：调度器启动后执行 Tof_ConfigAll()，完成后自删。 */
void Tof_InitTaskCreate(void);

/* Tof_ConfigAll() 还没跑完时的返回值 */
#define TOF_INIT_PENDING        (-2)

/* 初始化结果：0 成功，-1 至少一帧发送失败，TOF_INIT_PENDING 还没跑完。 */
int Tof_GetInitResult(void);

/* 重启上报流的周期，ms。距离完全依赖模块的 0x0793 自动上报，而上报流会自行
   停止，且重复的“开启”命令不能把流拉回来，必须先关再开，所以到期时发一对
   关闭 + 开启。
   ★ 周期要短于上报流的存活时间，否则两次重启之间会漏掉一段数据，
     表现为距离阶段性地停止更新。 */
#define TOF_RESTART_PERIOD_MS   5000u

/* 创建一个常驻任务，每 TOF_RESTART_PERIOD_MS 对每个模块发一对关闭 + 开启，
   重启 0x0793 上报流。首次开启由 Tof_ConfigAll() 负责，本任务不管首启。
   距离由 Tof_ParseFrame() 从上报帧写进 TofInfo，本任务不发读数请求。
   ★ 必须在 Tof_InitTaskCreate() 之后创建：模块收到 0x0316 之前不接受通道命令，
     本任务会先等初始化结果再开始发。 */
void Tof_PollTaskCreate(void);

/* 0x0408 通道 10：连续测距开关，enable 非 0 开。
   ★ 必须在 0x0316 之后发一次，否则模块不开始测距，既没有上报也没有读数。
   距离靠模块的 0x0793 自动上报获取，而那个上报流过一段时间会自行停止
   （连续测距本身不停）；重新把流拉起来需要先关再开，由 Tof_PollTask 周期执行。 */
int Tof_SetContinuous(uint8_t index, uint8_t enable);

/* 0x0408 通道 11：启动单次测距 */
int Tof_StartSingle(uint8_t index);

/* 0x0408 通道 12：请求读数，应答 0x0409 写入 TofInfo。
   应答实测为 5 字节：B0 设备号、B1=0x0C、B2~B3 距离、B4 递增序号。 */
int Tof_RequestDistance(uint8_t index);

/* 0x0408 通道 8 / 9：复位 VL53Lx / 复位 MCU */
int Tof_ResetSensor(uint8_t index);
int Tof_ResetMcu(uint8_t index);

/* 0x0408 通道 15 / 16：查询分辨率与频率 */
int Tof_QueryResolution(uint8_t index);
int Tof_QueryFrequency(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
