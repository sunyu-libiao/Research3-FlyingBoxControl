/**
  ******************************************************************************
  * @file    can.c
  * @brief   bxCAN 驱动：1 Mbps，标准帧，FIFO0 接收中断
  *
  * 接线（板卡 LB_RackRail_GyroACC）：
  *   PA12 = CAN_TX -> PCA82C251 TXD
  *   PA11 = CAN_RX <- PCA82C251 RXD
  *   （CAN 默认引脚映射，不需要 AFIO 重映射）
  *
  * 位定时（PCLK1 = 36 MHz，1 Mbps）：
  *   预分频 3 -> 12 MHz 时间元，1 + 8 + 3 = 12 tq 一位，采样点在 75%。
  *
  * 接收路径：FIFO0 中断把帧拷进 FreeRTOS 队列（ISR 安全发送），接收任务
  * 阻塞等待队列，再把帧交给注册的回调。中断里不做任何协议处理，回调运行在
  * 任务上下文，可以放心做解析与阻塞操作。
  *
  * 发送是一次性的（MCR.NART = 1，即关闭自动重传）。电机能解码并执行帧，但
  * 不驱动 ACK 槽，所以开着自动重传时每一帧都会一直挂着，100 ms 超时后中止，
  * CAN_Send() 报失败——尽管帧已经被接收并执行。一次性发送让邮箱能持续流转，
  * 缺失的应答改成计数而不是当成错误。因此 CAN_Send() 表示的是“帧已经发出”，
  * 而不是“有人确认收到了”——本板之前的 ACK 伙伴是上位机工具。
  *
  * 只在任务上下文使用。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "../inc/can.h"

/* Private define ------------------------------------------------------------*/
#define CAN_TX_TIMEOUT_MS       100
/* 队列深度：接收任务可能被回调阻塞几十毫秒，这段时间的突发流量要能装下。
   每帧占 sizeof(CAN_Frame)，队列从 configTOTAL_HEAP_SIZE 里分配。 */
#define CAN_RX_QUEUE_LEN        64
#define CAN_RX_STACK_SIZE       128     /* 字：队列等待 + 回调            */

/* Private variables ---------------------------------------------------------*/
static CAN_HandleTypeDef hcan1;
static QueueHandle_t canRxQueue;            /* 中断 -> 接收任务传递帧        */
static CAN_FrameHandler_t frameHandlers[CAN_MAX_FRAME_HANDLERS];  /* NULL = 空槽 */
static uint32_t rxLostCount;                /* 队列满而丢弃的帧数           */
static uint32_t txErrorCount;
static uint32_t txNoAckCount;               /* 发出去但无人应答的帧数       */
static uint32_t txSentCount;                /* 交给邮箱的帧数              */
static uint32_t rxFrameCount;               /* 收到的帧数，不论标识符      */
static uint32_t lastRxId;                   /* 最后一帧的标识符            */

static osMutexId canTxMutexId;          /* 串行化任务级的 CAN_Send 调用者 */

osMutexDef(canTxMutex);

/* Private function prototypes -----------------------------------------------*/
static void CAN_ReceiveTask(void const *argument);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  初始化 bxCAN：先建好接收队列与发送互斥量，再配置外设并启动。
  *         所有标识符都收进 FIFO0。
  * @param  None
  * @retval 成功返回 0，失败返回 -1
  */
int CAN_Init(void)
{
	CAN_FilterTypeDef filterConfig;

	/* 队列先于中断使能存在，避免中断拿到 NULL 句柄时无从入队 */
	if (canRxQueue == NULL)
	{
		canRxQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_Frame));
		if (canRxQueue == NULL)
		{
			return -1;
		}
	}

	if (canTxMutexId == NULL)
	{
		canTxMutexId = osMutexCreate(osMutex(canTxMutex));
		if (canTxMutexId == NULL)
		{
			return -1;
		}
	}

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	/* AutoBusOff：发生 bus-off 后自动恢复。没有它的话，只要有一次上电时
	   总线上没有 ACK 伙伴（上位机工具没开），CAN 控制器就会永久死掉：
	   重传风暴在几毫秒内把 TEC 推到 256，之后再也不回到总线上。 */
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	/* 一次性发送，见文件头：电机不驱动 ACK 槽，等应答会让每一帧都挂到超时 */
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		return -1;
	}

	/* 所有标识符都收进 FIFO0 */
	filterConfig.FilterBank = 0;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterIdHigh = 0x0000;
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000;    /* 掩码 0 = 全部匹配 */
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.SlaveStartFilterBank = 14;    /* 未使用，单路 CAN */
	if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
	{
		return -1;
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		return -1;
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		return -1;
	}

	return 0;
}

/**
  * @brief  发送一帧标准数据帧，阻塞到它离开邮箱为止。
  *         发送是一次性的，所以只要帧上一次总线就返回 0——不管有没有别的
  *         节点应答它。原因见文件头。
  * @param  stdId: 11 位标识符
  * @param  data: 载荷（len = 0 时传 NULL）
  * @param  len: 载荷长度 0-8
  * @retval 帧已发出返回 0，失败或超时返回 -1
  */
int CAN_Send(uint32_t stdId, const uint8_t *data, uint8_t len)
{
	CAN_TxHeaderTypeDef txHeader;
	static const uint8_t emptyPayload[CAN_MAX_DLC] = { 0 };
	uint32_t mailbox;
	uint32_t start;
	int result = -1;

	if ((len > CAN_MAX_DLC) || ((len > 0u) && (data == NULL)))
	{
		return -1;
	}

	/* 外设与互斥量都由 CAN_Init 建立，没初始化就别去碰寄存器 */
	if ((hcan1.Instance == NULL) || (canTxMutexId == NULL))
	{
		return -1;
	}

	/* 串行化并发的任务级发送者（多个任务 + 接收回调里的回显） */
	if (osMutexWait(canTxMutexId, CAN_TX_TIMEOUT_MS) != osOK)
	{
		txErrorCount++;
		return -1;
	}

	txHeader.StdId = stdId & 0x7FF;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = len;
	txHeader.TransmitGlobalTime = DISABLE;

	/* HAL 把邮箱以位掩码形式返回（CAN_TX_MAILBOX0/1/2 = 1/2/4），这正是
	   IsTxMessagePending/AbortTxRequest 需要的：不能再位移一次 */
	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader,
	                         (data != NULL) ? (uint8_t *)data : (uint8_t *)emptyPayload,
	                         &mailbox) == HAL_OK)
	{
		txSentCount++;
		start = HAL_GetTick();
		while (HAL_GetTick() - start < CAN_TX_TIMEOUT_MS)
		{
			if (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) == 0)
			{
				result = 0;    /* 一次性发送完成：帧已经出去了 */
				break;
			}
		}
		if (result != 0)
		{
			HAL_CAN_AbortTxRequest(&hcan1, mailbox);
		}
		else if (((hcan1.Instance->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos) == 3U)
		{
			/* LEC 3 = ACK 错误：发出去了，但没有任何节点应答。硬件在无错传输
			   时会清 LEC，所以这属于刚刚发出的那一帧。本总线上属于正常，
			   只做计数。 */
			txNoAckCount++;
		}
	}
	/* else：没有空闲邮箱 */

	if (result != 0)
	{
		txErrorCount++;        /* 没拿到邮箱或帧根本没出去 */
	}

	osMutexRelease(canTxMutexId);

	return result;
}

/**
  * @brief  注册帧处理回调。回调在 CAN 接收任务上下文运行，可以阻塞、可以做
  *         较重的解析；同一帧会依次交给所有已注册回调，由各自按帧 ID 认领。
  * @param  handler: 回调函数
  * @retval 注册成功返回 0，handler 为 NULL 或槽位已满返回 -1
  */
int CAN_AddFrameHandler(CAN_FrameHandler_t handler)
{
	uint8_t i;

	if (handler == NULL)
	{
		return -1;
	}

	for (i = 0; i < CAN_MAX_FRAME_HANDLERS; i++)
	{
		if (frameHandlers[i] == handler)
		{
			return 0;         /* 已注册，幂等 */
		}
	}

	for (i = 0; i < CAN_MAX_FRAME_HANDLERS; i++)
	{
		if (frameHandlers[i] == NULL)
		{
			frameHandlers[i] = handler;
			return 0;
		}
	}

	return -1;                /* 槽位已满 */
}

/**
  * @brief  注销帧处理回调。
  * @param  handler: 回调函数，NULL 无效
  * @retval None
  */
void CAN_RemoveFrameHandler(CAN_FrameHandler_t handler)
{
	uint8_t i;

	for (i = 0; i < CAN_MAX_FRAME_HANDLERS; i++)
	{
		if (frameHandlers[i] == handler)
		{
			frameHandlers[i] = NULL;
		}
	}
}

/**
  * @brief  bxCAN 发送错误计数器（总线受扰时非零）。
  * @param  None
  * @retval TEC 值，未初始化时为 0
  */
uint32_t CAN_GetTxErrorCount(void)
{
	return (hcan1.Instance != NULL) ? (hcan1.Instance->ESR & CAN_ESR_TEC) : 0u;
}

/**
  * @brief  发出去了但没有任何节点应答的帧数。板上单独挂在总线上时，
  *         它等于发送计数。
  * @param  None
  * @retval 无应答的发送计数
  */
uint32_t CAN_GetTxNoAckCount(void)
{
	return txNoAckCount;
}

/**
  * @brief  交给发送邮箱的帧数。
  * @param  None
  * @retval 发送计数
  */
uint32_t CAN_GetTxSentCount(void)
{
	return txSentCount;
}

/**
  * @brief  收到的帧数，不论标识符是什么。配合 CAN_GetLastRxId()，这是
  *         区分“对端不在总线上”（0）和“它在说话但不理我们”最快的手段。
  * @param  None
  * @retval 接收帧数
  */
uint32_t CAN_GetRxFrameCount(void)
{
	return rxFrameCount;
}

/**
  * @brief  最近收到的一帧的标识符。
  * @param  None
  * @retval 11 位标识符，还没收到任何帧时为 0
  */
uint32_t CAN_GetLastRxId(void)
{
	return lastRxId;
}

/**
  * @brief  ESR.LEC 里的上一个 CAN 错误码：0 无，1 位填充，2 格式，3 ACK，
  *         4 位隐性，5 位显性，6 CRC，7 软件设置。
  * @param  None
  * @retval LEC 值
  */
uint32_t CAN_GetLastErrorCode(void)
{
	return (hcan1.Instance != NULL) ?
	       ((hcan1.Instance->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos) : 0u;
}

/**
  * @brief  因为接收队列满而被丢弃的帧数。
  * @param  None
  * @retval 丢失帧数
  */
uint32_t CAN_GetRxLostCount(void)
{
	return rxLostCount;
}

/**
  * @brief  bus-off 状态（CAN_ESR 的 BOFF 标志）。
  * @param  None
  * @retval bus-off 返回 1，否则返回 0
  */
uint8_t CAN_IsBusOff(void)
{
	return (hcan1.Instance != NULL) && ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0);
}

/**
  * @brief  创建接收任务。在 main 里 CAN_Init() 之后、启动调度器之前调用一次。
  * @param  None
  * @retval None
  */
void CAN_RxTaskCreate(void)
{
	if (canRxQueue == NULL)
	{
		return;        /* 未调用 CAN_Init */
	}

	osThreadDef(CanRx, CAN_ReceiveTask, osPriorityNormal, 0, CAN_RX_STACK_SIZE);
	osThreadCreate(osThread(CanRx), NULL);
}

/* FreeRTOS tasks -------------------------------------------------------------*/

/**
  * @brief  接收任务：阻塞等待队列，把帧交给注册的回调。没有注册回调时帧在
  *         这里被消费掉，队列不会因为无消费者而溢满，而诊断计数已在中断里
  *         更新，依然能看出总线上有没有流量。
  *         调度器启动前收到的帧会留在队列里，不会因任务还没跑起来而丢帧。
  * @param  argument 未使用
  * @retval None
  */
static void CAN_ReceiveTask(void const *argument)
{
	CAN_Frame frame;
	uint8_t i;

	(void) argument;

	for (;;)
	{
		if (xQueueReceive(canRxQueue, &frame, portMAX_DELAY) != pdTRUE)
		{
			continue;
		}

		for (i = 0; i < CAN_MAX_FRAME_HANDLERS; i++)
		{
			if (frameHandlers[i] != NULL)
			{
				frameHandlers[i](&frame);
			}
		}
	}
}

/* Interrupt / HAL callbacks -------------------------------------------------*/

/**
  * @brief  CAN FIFO0 中断：把硬件 FIFO 里的帧搬进 FreeRTOS 队列。
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief  FIFO0 有新帧时由 HAL_CAN_IRQHandler 调用。这里只搬数据，不做
  *         任何协议处理；队列满时丢帧并计数。
  * @param  hcan: CAN 句柄
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	CAN_Frame frame;
	uint8_t data[CAN_MAX_DLC];
	BaseType_t woken = pdFALSE;

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
		{
			break;
		}

		frame.id = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
		frame.len = (uint8_t)(rxHeader.DLC > CAN_MAX_DLC ? CAN_MAX_DLC : rxHeader.DLC);
		for (uint8_t i = 0; i < frame.len; i++)
		{
			frame.data[i] = data[i];
		}

		/* 计数先于入队，队列满时也能看出总线上有没有流量 */
		lastRxId = frame.id;
		rxFrameCount++;

		if ((canRxQueue == NULL) ||
		    (xQueueSendFromISR(canRxQueue, &frame, &woken) != pdTRUE))
		{
			rxLostCount++;
		}
	}

	portYIELD_FROM_ISR(woken);
}

/**
  * @brief  CAN 底层初始化（由 HAL_CAN_Init 调用）：时钟、GPIO、NVIC。
  * @param  hcan: CAN 句柄
  * @retval None
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if (hcan->Instance != CAN1)
	{
		return;
	}

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_CAN1_CLK_ENABLE();

	/* CAN_TX（PA12）：复用推挽 */
	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* CAN_RX（PA11）：输入 */
	GPIO_InitStructure.Pin = GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 数值大于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（优先级更低），
	   因此可以安全调用 FreeRTOS 的 FromISR API */
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}
