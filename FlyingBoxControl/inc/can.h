/**
  ******************************************************************************
  * @file    can.h
  * @brief   PCA82C251 收发器（PA11/PA12）后面的 bxCAN 驱动（1 Mbps）
  ******************************************************************************
  */

#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAN_MAX_DLC     8

/* 一帧数据。id 已按帧类型展开：标准帧为 11 位，扩展帧为 29 位。 */
typedef struct
{
	uint32_t id;
	uint8_t  len;
	uint8_t  data[CAN_MAX_DLC];
} CAN_Frame;

/* 帧处理回调，在 CAN 接收任务上下文运行。同一帧会依次交给所有已注册回调。 */
typedef void (*CAN_FrameHandler_t)(const CAN_Frame *frame);

/* 可同时注册的回调个数。每多一个上层协议就占一个槽位。 */
#define CAN_MAX_FRAME_HANDLERS  4

int      CAN_Init(void);
void     CAN_RxTaskCreate(void);

/* 注册/注销帧处理回调。重复注册同一函数不会报错，也不会重复调用。
   注册成功返回 0，传入 NULL 或槽位已满返回 -1。 */
int      CAN_AddFrameHandler(CAN_FrameHandler_t handler);
void     CAN_RemoveFrameHandler(CAN_FrameHandler_t handler);
int      CAN_Send(uint32_t stdId, const uint8_t *data, uint8_t len);
uint32_t CAN_GetTxErrorCount(void);
uint32_t CAN_GetTxNoAckCount(void);
uint32_t CAN_GetTxSentCount(void);
/* 因“帧确定没上线”而重试的次数。持续增长说明总线在丢帧，但重试已把帧救回 */
uint32_t CAN_GetTxRetryCount(void);
uint32_t CAN_GetRxFrameCount(void);
uint32_t CAN_GetLastRxId(void);
uint32_t CAN_GetLastErrorCode(void);
uint32_t CAN_GetRxLostCount(void);
uint8_t  CAN_IsBusOff(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
