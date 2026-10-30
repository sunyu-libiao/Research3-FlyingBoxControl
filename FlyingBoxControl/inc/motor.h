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
   实测上电后的复位窗口内电机一帧都不发，这里的取值要在该窗口之上留余量；
   往下压容易踩到未复位的窗口，表现为刚开始就保护释放。
   与 TOF_INIT_SETTLE_MS 相互独立，两个模块各自配置。 */
#define MOTOR_INIT_SETTLE_MS    3000u

/* 初始化帧之间的间隔。CAN_Send() 只阻塞到帧离开邮箱（1 Mbps 下约 0.1 ms），
   而 0x0316 要写参数、0x0408 要写轮径，电机处理跟不上连续帧；帧贴得太紧
   会让后面的 0x0413 静默落空，表现为“只有部分电机使能成功”。
   仅在初始化流程（任务上下文）里使用。 */
#define MOTOR_INIT_FRAME_GAP_MS 5u

/* 轮询剩余位移的间隔。0x0420 是电机主动上报的帧，间隔太密只是白占 CPU；
   太稀则可能整段短行程都落在两次采样之间，既看不到非零、也就判不出到位。 */
#define MOTOR_ARRIVE_POLL_MS    10u

/* 到位死区，单位同剩余位移（0.1 mm）。电机不会精细停在 0：协议的位置量有
   量化，实测到位后会停在 0 附近一个到两个单位上，且符号会抖。拿“等于 0”
   当判据会永远等不到，必须用绝对值小于死区来判。 */
#define MOTOR_ARRIVE_DEADBAND   5

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

/* ---- 归位（靠电机自保护释放停在末端）----------------------------------*/

/* 归位速度的默认值，0.1 mm/s。协议没有“速度模式”，只能发一个大行程
   相对命令让电机一直往一个方向跑。
   ★ 单位是轮缘弧长，数值随 wheelDiameter001mm 一起缩放：轮径放大 n 倍，
     要发出同样的转角就得把这里也放大 n 倍。
   这个宏只作为 MotorInfo.homeSpeed001mms 的默认值；轮径单独标定过的
   电机必须在自己的配置里给对应量级的值。
   归位结束后由 Motor_InitAll() 恢复 speed001mms。 */
#define MOTOR_HOME_SPEED_001MMS 80000u

/* 归位加速度的默认值，0.1 mm/s^2，同样随轮径缩放。
   实测依据（当时轮径 20 mm）：运行加速度档位会在加速段就触发过流保护释放，
   把加速度降到原值的十分之一就能平稳走完。边界是角加速度而不是协议数值，
   所以轮径或速度一变就必须重新换算，不能沿用旧数字。
   ★ 一出现启动就掉电，先把这一台的 homeAccel001mms2 减半，再降 homeSpeed001mms。
   与上面的速度一样，它只是默认值，逐台值在 MotorInfo 里。 */
#define MOTOR_HOME_ACCEL_001MMS2 40000u

/* 归位等待时长不是一个宏，而是由 homeTravel001mm / 该台的归位速度 / 该台的
   归位加速度在运行时算出来（motor.c 的 homeWaitMsFor）：
     加速段距离 d_acc = v^2 / (2a)，加速段耗时 t_acc = v / a；
     行程 d <= d_acc：全程都在加速，取 t_acc（上界）；
     行程 d >  d_acc：t_acc + (d - d_acc) / v。
   homeTravel001mm 就当成电机实际要走的距离，固件不需要知道任何机械尺寸。

   ★ 余量靠 homeTravel001mm 多填来给：多出的距离 / 速度就是多等的时间，
     代码不再额加固定余量。所以它也直接决定上电耗时，多填一倍就是多等
     一倍。改行程 / 速度 / 加速度都会改变上电总时长，要重新核实预算。 */

/* 停下后到置零之间的等待，让电机把剩余速度耗完、位置读数稳定 */
#define MOTOR_HOME_SETTLE_MS    200u

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
	uint16_t wheelDiameter001mm;/* 0x0408 轮径，0.01 mm。字段是 u16，上限 65535 = 655.35 mm，
	                               超出这个量级的"轮径"不是轮径（大概率是里程）；
	                               填错不会报错，只会让位移/速度/加速度按同一比例缩放。
	                               改这个值后，位移/速度/加速度（含 homeTravel001mm）
	                               必须按同样倍数改，否则实际行程会整体缩放 */
	uint32_t speed001mms;       /* 0x0415 速度，0.1 mm/s */
	uint32_t accel001mms2;      /* 0x0414 加速度，0.1 mm/s^2 */
	MotorConfig params;         /* 0x0316 其余参数字节 */
	/* 归位用的相对位移，0.1 mm。0 = 这台不需要原点，Motor_BringUp() 跳过。
	   就把它当成电机实际要走的距离：够从最远端走到头、让电机自己触发保护
	   释放即可。等待时长同样按它算（见 motor.c 的 homeWaitMsFor），所以也
	   不该多填。符号表示“朝哪边找”，负值即物理向后。reverse 位会翻转电机
	   内部正转方向，所以符号要不要跟着 reverse 走必须实测确认。 */
	int32_t  homeTravel001mm;
	/* 归位速度与加速度，0.1 mm/s 与 0.1 mm/s^2。逐台而不是全局：
	   轮径标定过的电机，同一个数值对应的真实速度与其它台差着量级，
	   共用一个全局值会让它归位时冲出去。默认取下面的 MOTOR_HOME_* 宏。 */
	uint32_t homeSpeed001mms;
	uint32_t homeAccel001mms2;

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

/* 按下标配置全部电机，每台按 MotorInfo 里的配置依次发：
  0x0316 参数 → 0x0408 轮径（×3）→ 0x0414 加速度 → 0x0415 速度。
  ★ 不发使能帧（0x0413）：使能由 Motor_BringUp() 在合适的时机统一发。
  每帧之间插 MOTOR_INIT_FRAME_GAP_MS，会 osDelay，只能在任务上下文调用。
  返回 0 表示所有帧都已发出，-1 表示至少一帧发送失败。 */
int Motor_InitAll(void);

/* 完整上电流程，从头到尾串行跑完（总时长十几到几十秒）：
    ① 等 MOTOR_INIT_SETTLE_MS，让电机完成上电复位；
    ② 逐台（homeTravel001mm 非 0 的）单独初始化 → 使能 → 向后跑长行程 →
       按行程推算的时间定时等待 → 清除残留目标 → 把停住的位置置零；
    ③ 重新初始化全部电机（归位时用的是低速低加速，必须改回运行参数）；
    ④ 使能全部电机；
    ⑤ 把此刻全部电机的当前位置设为绝对 0。

  为什么每台归位前都要重新初始化：电机顶到末端触发保护释放后，内部的
  参数与输出级状态已不可信，必须重新下发 0x0316/0x0408 才能正常驱动。
  为什么最后还要整组再初始化一遍：归位阶段下发的是低速低加速，直接拿来
  跑正常运动会很慢；而且归位期间被保护过的电机需要重新使能。

  会大量 osDelay，只能在调度器启动后的任务上下文中调用。
  返回累计失败帧次数。 */
int Motor_BringUp(void);

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

/* 0x0416 设置目标位置（主通道），position 单位 0.1 mm，positionType 见 MOTOR_POS_*
   发完不会自动等它到位，需要的话接着调 Motor_WaitArrive()。 */
int Motor_SetTargetPositionMain(uint8_t motorIndex, int32_t position, uint8_t positionType);

/* 等这台走到目标点：轮询 0x0420 剩余位移，绝对值落在 MOTOR_ARRIVE_DEADBAND
   以内即认为到位。
   ★ 只适用于初始化完成之后的常规运动：那时绝对 0 已建立，运动不会再顶到
     机械末端，电机不会触发保护释放，剩余位移会一路更新下去。归位阶段
     不能用这条判据（保护后连反馈都没了）。
   判据要求先看到超出死区的值、再看到它落入死区，只看“已经到位”会把命令
   之前的旧值当成就位。代价是距离短到两次反馈之间就走完时会报超时。
   会 osDelay，只能在任务上下文调用。
   返回 0 已到位，-1 超时，-2 下标越界。 */
int Motor_WaitArrive(uint8_t motorIndex, uint32_t timeoutMs);

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

/* ---- 归位（靠电机自保护释放停在末端）----------------------------------
   用法：直接调 Motor_BringUp()，它已经把"逐台归位 → 全部初始化 → 全部使能
   → 全部置零"串成一条完整流程。单台的 Motor_HomeByRelease() 一般不用直接调。
   两者都会 osDelay，只能在调度器启动后的任务上下文中调用。            */

/* 单台归位：初始化这一台（参数/轮径/归位用的速度与加速度）→ 使能 →
   按 homeTravel001mm 发一段相对位移 → 按同一段距离推算出的时间定时等待
   （电机顶到末端后自身保护会断开输出）→ 清除残留目标 → 把停住的位置定义成 0。
   不做任何闭环判据：实测保护动作后连剩余位移反馈都不再上报，读不到变化。
   homeTravel001mm 为 0 时立即返回 1（该电机不参与归位）。
   ★ homeTravel001mm 就是电机实际走的距离：太短会行程走完还没到末端，把半途
     当原点；太长则只是白等（时间也按它算）。填到"从最远端一定走得到头"即可。
   ★ 只使能这一台，其它电机保持释放，不会跟着动。
   ★ 返回后该台用的是归位参数（低速低加速），要跑正常运动必须重新
     Motor_InitAll()。
   返回 0 已置零，1 跳过，-1 有帧发送失败。 */
int Motor_HomeByRelease(uint8_t motorIndex);

/* 0x060D 获取设备 SN。实测这颗固件对本查询无应答，SN 只能靠上电瞬间的 0x0312
   抓取（该帧不带地址，未做解析）。 */
int Motor_RequestSn(uint8_t deviceType);

/* 0x0313 绑定设备 SN，sn 指向 7 字节 */
int Motor_BindSn(uint8_t motorIndex, const uint8_t *sn);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
