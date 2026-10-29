/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机 CAN 协议（V1.1）驱动：电机状态 + 每个帧 ID 一个发送接口
  *
  * 依赖方向：motor -> can。can 不认识电机，换掉上层协议不用动 can.c。
  * 协议表（xlsx）有多处与实测不符，本文件以 doc/电机CAN控制流程.md 为准。
  ******************************************************************************
  */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can.h"

/* 可管理的电机数量，同时也是状态表的下标范围。 */
#define MOTOR_MAX_COUNT         8

/* 反馈帧 ID = 基址 + 编号，编号的取值跨度：马达号只有 4 bit（0..15），
   0x0420 与 0x0440 两段之间也只隔 0x20。超出这个跨度的 ID 不是电机帧。 */
#define MOTOR_ID_SPAN           16

/* ---- 本机电机配置 ------------------------------------------------------- */

/* 挂在这条总线上的电机数量，以及第一个电机的设备号。
   设备号 = MOTOR_DEV_BASE + 下标，马达号与设备号相同，即 7..11。
   SN 已在电机侧固化，上电不再走 0x0313 绑定。 */
#define MOTOR_COUNT             5
#define MOTOR_DEV_BASE          7

#if (MOTOR_COUNT) > (MOTOR_MAX_COUNT)
#error "MOTOR_COUNT 超过状态表容量 MOTOR_MAX_COUNT"
#endif

/* 设备号与马达号都只有 4 bit，超过 15 会与相邻帧 ID 段相撞 */
#if ((MOTOR_DEV_BASE) + (MOTOR_COUNT) - 1) > 15
#error "设备号/马达号只有 4 bit，最大到 15"
#endif

/* 每台电机的配置（0x0316 参数、方向位、轮径、速度、加速度）都放在
   MotorInfo 的配置区里，母版是 motor.c 的 s_defaultMotors[]（Flash，
   下标 0 -> 设备号 MOTOR_DEV_BASE）。Motor_Init() 整份拷入状态表后，
   初始化与收发都从这里取值，改配置只改这一处。
   轮径必须填实测的滚轮或齿轮节圆直径，填错不会报错，
   只会让该电机的位移/速度/加速度按同一比例缩放。 */

/* 状态表中“该下标未分配电机”的地址标记，不参与收发路由 */
#define MOTOR_ADDR_UNASSIGNED   0xFF

/* 上电后到发出第一条初始化帧之前的等待。电机需要时间完成自身上电复位，
   过早下发 0x0316 会被静默丢弃（该帧无应答，丢了也看不出来）。
   与 TOF_INIT_SETTLE_MS 相互独立，两个模块各自配置。 */
#define MOTOR_INIT_SETTLE_MS    3000u

/* 初始化帧之间的间隔。CAN_Send() 只阻塞到帧离开邮箱（1 Mbps 下约 0.1 ms），
   而 0x0316 要写参数、0x0408 要写轮径，电机处理跟不上连续帧；帧贴得太紧
   会让后面的 0x0413 静默落空，表现为“只有部分电机使能成功”。
   仅在初始化流程（任务上下文）里使用。 */
#define MOTOR_INIT_FRAME_GAP_MS 5u

/* 目标位置类型（0x0416 Data[5]） */
#define MOTOR_POS_RELATIVE      0x00    /* 相对当前位置 */
#define MOTOR_POS_ABSOLUTE      0x05    /* 绝对位置 */
#define MOTOR_POS_HOME          0x0A    /* 归零 */
#define MOTOR_POS_SET_ZERO      0x0B    /* 设当前位置为给定值 */

/* 上电使能 / 下电释放（0x0413 Data[0]） */
#define MOTOR_CMD_ENABLE        0x5A
#define MOTOR_CMD_RELEASE       0x4A

/* 查询子命令（0x0408 Data[1]） */
#define MOTOR_QUERY_TEMPERATURE 0x06
#define MOTOR_QUERY_POSITION    0x08
#define MOTOR_QUERY_STATUS      0x25
#define MOTOR_QUERY_FIRMWARE    0xFF

/* 状态位（0x0409 回答 MOTOR_QUERY_STATUS） */
#define MOTOR_STATUS_LOCKED     0x01
#define MOTOR_STATUS_HOMED      0x02

/* 错误上报：帧 ID 本身就是错误码（0x0100~0x017F）。0x0100 是例外——实测那是
   “运动指令受理”标记，每次非零相对运动都会出现且电机照常运动，不能当错误。 */
#define MOTOR_ERR_INCOMPLETE    0x0102  /* 未到达目标点 */
#define MOTOR_ERR_STALL         0x0104  /* 堵转 */
#define MOTOR_ERR_OVERTEMP      0x0105  /* 超温 */
#define MOTOR_ERR_UNDERVOLT     0x0107  /* 欠压 */
#define MOTOR_ERR_DISCONNECT    0x0108  /* 断线 */

/**
  * @brief  0x0316 电机参数。按协议表的 Bit 位域拼装，一个字段对应表里的一格。
  *         地址（设备号/马达号）与方向位不在这里：它们属于单台电机的安装
  *         配置，统一放在 MotorInfo 里，避免同一件事记两遍。
  */
typedef struct
{
	uint8_t reserved0;          /* Byte0 bit7..4 文档留空，参考值 B4 这里是 0xB */
	uint8_t polePairs;          /* Byte0 bit3..0 极对数 */
	uint8_t speedP;             /* Byte1 bit7..4 速度 P */
	uint8_t speedI;             /* Byte1 bit3..0 速度 I */
	uint8_t torqueP;            /* Byte2 bit7..4 转矩 P */
	uint8_t torqueI;            /* Byte2 bit3..0 转矩 I */
	uint8_t currentCw;          /* Byte3 bit7..4 顺时针最大电流 */
	uint8_t currentCcw;         /* Byte3 bit3..0 逆时针最大电流 */
	uint8_t brake;              /* Byte4 bit1 刹车 */
	uint8_t damping;            /* Byte4 bit0 阻尼 */
	uint8_t speedD;             /* Byte5 bit7..4 速度 D */
	uint8_t torqueD;            /* Byte5 bit3..0 转矩 D */
	uint8_t autoUnlock;         /* Byte6 bit6 往返模式自动解锁 */
	uint8_t disableRemainPos;   /* Byte6 bit5 1=关闭自动返回剩余位移 */
	uint8_t disableRemainTorque;/* Byte6 bit4 1=关闭自动返回剩余力矩 */
} MotorConfig;

/**
  * @brief  单台电机的全部信息：配置区 + 运行状态。
  *         以下标索引，下标 0 对应设备号 MOTOR_DEV_BASE。
  *
  * 配置区由 Motor_Init() 从 motor.c 里的 Flash 模板整份拷入，之后只有
  * Motor_SetAddress() 会改地址两个字段，其余字段视为只读；
  * 状态区只由 Motor_ParseFrame()（CAN 接收任务）写，其它任务只读。
  *
  * 协议里有两个地址字段，都被写成“设备号”，实测含义不同：
  *   devId   由 0x0313 Data[7] 写进电机，决定反馈与查询往哪回：
  *           0x0380+devId 心跳、 0x0408/0x0409 的 Data[0]。
  *   motorId 由 0x0316 Byte6 bit3..0 写进电机，决定运动帧听谁的：
  *           0x0413~0x0416 的 Data[4]，以及 0x0420+motorId / 0x0440+motorId。
  * 两者取相同值时最省事，Motor_Init() 就默认按下标取同一值。
  */
typedef struct
{
	/* ---- 配置区 ---- */
	uint8_t  devId;             /* 设备号，建议 0..15，否则 0x0380+dev 会撞上别的 ID 段 */
	uint8_t  motorId;           /* 马达号，0..15，同一条总线必须互不相同 */
	uint8_t  reverse;           /* 0x0316 Byte6 bit7，0=顺时针 1=逆时针（正对电机轴） */
	uint16_t wheelDiameter001mm;/* 0x0408 轮径，0.01 mm（100 mm -> 10000） */
	uint32_t speed001mms;       /* 0x0415 速度，0.1 mm/s */
	uint32_t accel001mms2;      /* 0x0414 加速度，0.1 mm/s^2 */
	MotorConfig params;         /* 0x0316 其余参数字节 */

	/* ---- 状态区 ---- */
	/* 0x0420 + motorId：到目标点的剩余位移，减到 0 即到位，不是绝对位置 */
	int32_t  remainMain;        /* 主通道，0.1 mm */
	int32_t  remainSub;         /* 从通道，0.1 mm */

	/* 0x0440 + motorId：力矩反馈 */
	int16_t  torque;            /* Data[2~3]，原始 s16 */

	/* 0x0409 查询应答：Data[0]=devId, Data[1]=子码, Data[2..]=载荷 */
	int32_t  position;          /* 子码 0x08 当前位置，0.1 mm；0x0420 给不出绝对值 */
	uint8_t  status;            /* 子码 0x25 状态位，MOTOR_STATUS_* */
	int16_t  temperature;       /* 子码 0x06，0.1 °C */
	uint32_t firmware;          /* 子码 0xFF */

	/* 0x041A：参数设置 ACK */
	uint16_t lastAckId;         /* 被应答的原始帧 ID，Data[0~1] 小端 */
	uint32_t ackCount;

	/* 统计 */
	uint32_t heartbeatCount;    /* 0x0380 + devId */
	uint32_t rxCount;           /* 任何可识别帧的累计数 */
	uint32_t lastRxTick;        /* 最后一帧的 HAL_GetTick()，用于判离线 */
} MotorInfo;

/* 初始化：清空状态，把 motor.c 里的 Flash 默认模板整份拷入配置区
   （模板不含地址，地址按设备号 = 马达号 = MOTOR_DEV_BASE + 下标填入；
   其余表项标为 MOTOR_ADDR_UNASSIGNED），再挂成 CAN 的帧处理器。
   必须在 CAN_Init() 之后、CAN_RxTaskCreate() 之前调用。 */
void Motor_Init(void);

/* 按下标初始化全部电机，每台按 MotorInfo 里的配置依次发：
  0x0316 参数 → 0x0408 轮径（×3）→ 0x0414 加速度 → 0x0415 速度 → 0x0413 使能。
   每帧之间插 MOTOR_INIT_FRAME_GAP_MS，只能在调度器启动后的任务上下文调用；
   在 main 里请改用 Motor_InitTaskCreate()。
   返回 0 表示所有帧都已发出，-1 表示至少一帧发送失败。 */
int Motor_InitAll(void);

/* Motor_InitAll() 还没跑完时的返回值 */
#define MOTOR_INIT_PENDING      (-2)

/* 创建一次性初始化任务：调度器启动后执行 Motor_InitAll()，完成后自删。
   与 CAN_RxTaskCreate() 同时使用，且必须在其之后创建，
   这样接收任务会先挂到队列上，配置期间的 ACK 不会堆积。 */
void Motor_InitTaskCreate(void);

/* 初始化结果：0 成功，-1 至少一帧发送失败，MOTOR_INIT_PENDING 还没跑完。 */
int Motor_GetInitResult(void);

/* 指定下标对应的两个地址，用于覆盖 Motor_Init() 按 MOTOR_DEV_BASE 给出的默认值。 */
void Motor_SetAddress(uint8_t motorIndex, uint8_t devId, uint8_t motorId);

/* 取某个下标的只读状态，越界返回 NULL。
   下标 0..MOTOR_COUNT-1 对应设备号 MOTOR_DEV_BASE..MOTOR_DEV_BASE+MOTOR_COUNT-1，
   不是直接拿设备号来索引。 */
const MotorInfo *Motor_GetInfo(uint8_t motorIndex);

/* CAN 接收回调本体，由 CAN 接收任务调用；Motor_Init() 已注册，通常无需手动调。 */
void Motor_ParseFrame(const CAN_Frame *frame);

/* 最近一次 0x0100~0x017F 上报的错误码与累计帧数（该帧不带地址，无法归属到电机）。
   实测 0x0100 带载荷 [34 12] 是“运动指令受理”标记，不能当错误。 */
uint16_t Motor_GetLastErrorCode(void);
uint32_t Motor_GetErrorFrameCount(void);

/* ---- 控制指令。motorIndex 是状态表下标，不是报文里的地址字段。 ---------- */

/* 0x0416 设置目标位置（主通道），position 单位 0.1 mm，positionType 见 MOTOR_POS_* */
int Motor_SetTargetPositionMain(uint8_t motorIndex, int32_t position, uint8_t positionType);

/* 0x0419 设置目标位置（从通道） */
int Motor_SetTargetPositionSub(uint8_t motorIndex, int32_t position);

/* 0x0415 设置主通道速度，速度单位 0.1 mm/s */
int Motor_SetSpeedMain(uint8_t motorIndex, uint32_t speed);

/* 0x0418 设置从通道速度 */
int Motor_SetSpeedSub(uint8_t motorIndex, uint32_t speed);

/* 0x0414 设置主通道加速度，加速度单位 0.1 mm/s^2 */
int Motor_SetAccelerationMain(uint8_t motorIndex, uint32_t acceleration);

/* 0x0417 设置从通道加速度 */
int Motor_SetAccelerationSub(uint8_t motorIndex, uint32_t acceleration);

/* 0x0413 上电使能 / 下电释放，enable 非 0 使能。
   只决定电机能否被驱动：周期反馈由 0x0316 参数开启，释放后反馈流不会停。 */
int Motor_SetEnabled(uint8_t motorIndex, uint8_t enable);

/* 0x0316 写入电机参数（初始化）。载荷全部取自该下标 MotorInfo 的配置区：
   Byte6 bit7 = reverse、bit3..0 = motorId，Byte7 = devId，其余取自 params。
   Byte6 的 bit5/bit4 留着 0 才会开启周期反馈（0x0420/0x0440/0x0380）。
   0x0316 无应答，配置是否真的生效只能靠运动行为验证。 */
int Motor_ConfigParams(uint8_t motorIndex);

/* 0x0403 Data[1]=0x40 配置机械臂成员，motorIds 为参与控制的设备号，1~6 个 */
int Motor_SetArmMembers(uint8_t motorIndex, const uint8_t *motorIds, uint8_t count);

/* 0x0403 Data[1]=0x41 设置机械臂零点，每次机械臂控制前都要发 */
int Motor_SetArmZero(uint8_t motorIndex, const uint8_t *motorIds, uint8_t count);

/* 0x0700 控制机械臂动作，四个电机各一个相对位移（0.1 mm） */
int Motor_MoveArm(int16_t delta1, int16_t delta2, int16_t delta3, int16_t delta4);

/* 0x0408 配置轮径，直径单位 0.01 mm。实测不配轮径运动指令不会执行，协议要求重复写 3 次。 */
int Motor_ConfigWheelDiameter(uint8_t motorIndex, uint16_t diameter);

/* 0x0408 蜂鸣提示，tone 为 Data[3~5] 的频率信息，按协议原样透传 */
int Motor_Beep(uint8_t motorIndex, uint8_t volume, const uint8_t tone[3]);

/* 0x0408 查询指令，应答由 0x0409 送回并写入 MotorInfo。
   需先完成 0x0313 绑定 + 0x0316 参数配置，否则电机不应答。 */
int Motor_QueryTemperature(uint8_t motorIndex);
int Motor_QueryPosition(uint8_t motorIndex);
int Motor_QueryStatus(uint8_t motorIndex);
int Motor_QueryFirmware(uint8_t motorIndex);

/* 0x060D 获取设备 SN。实测这颗固件对本查询无应答，SN 只能靠上电瞬间的 0x0312
   抓取（该帧不带地址，未做解析）。 */
int Motor_RequestSn(uint8_t deviceType);

/* 0x0313 绑定设备 SN，sn 指向 7 字节 */
int Motor_BindSn(uint8_t motorIndex, const uint8_t *sn);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
