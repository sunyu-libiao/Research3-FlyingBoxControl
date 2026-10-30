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
          只在通道 10 开连续测距后才会来，且大约 10 秒后自行停止，
          需要重新发一次通道 10 才能继续。定位不得超过这个间隔。
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
	uint8_t  seq;               /* 0x0409 末字节：实测是递增序号，不是新旧标志 */

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

/* 按默认配置初始化全部模块：ROI 0x11~0x22、阈值 0xFF、不旋转、4x4 持续测量。 */
int Tof_ConfigAll(void);

/* 创建一次性初始化任务：调度器启动后执行 Tof_ConfigAll()，完成后自删。 */
void Tof_InitTaskCreate(void);

/* Tof_ConfigAll() 还没跑完时的返回值 */
#define TOF_INIT_PENDING        (-2)

/* 初始化结果：0 成功，-1 至少一帧发送失败，TOF_INIT_PENDING 还没跑完。 */
int Tof_GetInitResult(void);

/* 距离轮询任务的周期，ms。模块只在被请求时回一帧距离，要持续拿到数据
   就得周期发通道 12 请求。这条路径不受“自动上报会自行停止”的影响。 */
#define TOF_POLL_PERIOD_MS      10u

/* 创建一个常驻任务，按 TOF_POLL_PERIOD_MS 循环向每个模块请求一次距离，
   应答由 Tof_ParseFrame() 写进 TofInfo。
   ★ 必须在 Tof_InitTaskCreate() 之后创建：模块收到 0x0316 之前不应答请求，
     本任务会先等初始化结果再开始发。 */
void Tof_PollTaskCreate(void);

/* 0x0408 通道 10：连续测距开关，enable 非 0 开。
   实测：开之后模块以 0x0793 自动上报，但**大约 10 秒后自行停止**，
   要维持数据流必须周期重发（建议 ≤5 s），或改用通道 12 主动查询。 */
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
