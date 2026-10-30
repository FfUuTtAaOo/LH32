#include "lh32_uart.h"

DMA_HandleTypeDef hDMA1, hDMA2;

/* ---- 串口引脚(改线只改这几行) ----
 * 默认: P1.5 = SIN0(RX), P1.6 = SOUT0(TX) */
#define UART_RX_PORT        pGPIO1
#define UART_RX_PIN         GPIO_PIN_5
#define UART_RX_AF          GPIO1_5_AF_SIN0

#define UART_TX_PORT        pGPIO1
#define UART_TX_PIN         GPIO_PIN_6
#define UART_TX_AF          GPIO1_6_AF_SOUT0

#define UART_BAUD_RATE      115200

#define IDLE_FRAME_INTERVAL 3               /* 空闲帧长度 */
#define TCD_COUNT 2

uint8_t commBuffer[MAX_COMM_SIZE];

void uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    BTIM_Base_InitTypeDef BTIM_InitStructure;
    DMAMUX_InitTypeDef DMAMUX_InitStructure;
    DMA_InitTypeDef DMA_InitStructure[TCD_COUNT];               /* 分散聚合模式的多个初始化结构体参数 */
    static DMA_DES_N_TypeDef TCD_Quene[TCD_COUNT] __ALIGN(32);  /* DMA TCD描述符必须32位对齐*/
    uint16_t IdleFrameValue;                                    /* 空闲帧对应定时长度 */
    uint32_t BTimerCTRLValue[2];                                /* 定时器配置参数 */

    /* Configure UART GPIO multiplexing pins */
    GPIO_InitStructure.Pin = UART_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = ENABLE;
    GPIO_InitStructure.Alternate = UART_RX_AF;
    LHL_GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = UART_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = UART_TX_AF;
    LHL_GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);

    /* Set UART parameters */
    UART_InitStructure.UART_BaudRate = UART_BAUD_RATE;                          // The maximum baud rate must be less than the clock frequency / 16
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;                    // data: 8
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                         // stop: 1
    UART_InitStructure.UART_Parity = UART_Parity_None;                          // parity: None
    LHL_UART_Init(pUART0, &UART_InitStructure);

    /* 3.1 BTIM TImer Base Init */
    BTIM_InitStructure.MDIS = ENABLE;
    BTIM_InitStructure.FreezingMode = DISABLE;
    BTIM_InitStructure.ChainedMode = DISABLE;
    LHL_BTIM_Base_Init(pBTIM, &BTIM_InitStructure);

    /* 3.2 Calculate idle interval: timeout is IDEL_FRAME_INTERVAL data bits */
    IdleFrameValue = (uint16_t) (SystemCoreClock * IDLE_FRAME_INTERVAL * 10 / UART_BAUD_RATE) - 1;
    LHL_BTIM_SetPeriod(BTIMER_1, IdleFrameValue);

    /* 3.3 Configure the parameters of the BTIMER CTRL register, and DMA1 is responsible for writing. */
    BTimerCTRLValue[0] = 0;
    BTimerCTRLValue[1] = BTIM_TCTRL_TEN_Msk | BTIM_TCTRL_TIE_Msk;
    
    /* 3.4 开启BTIM1中断：产生中断表示串口接收到一帧数据，且空闲时间超过空闲间隔设定 */
    LHL_BTIM_ITConfig(BTIMER_1, ENABLE);
    NVIC_EnableIRQ(TIM6_IRQn);                                  // BTIMER1对应TIM6_IRQ

    /* 4.1 配置DMAUX 请求源UART0 RX，触发DMA1 */
    DMAMUX_InitStructure.DMA_Channel = DMA_CHANNEL_1;
    DMAMUX_InitStructure.Request_Source = REQUEST_SOURCE_UART0_RX;
    DMAMUX_InitStructure.Periodic_Trigger = DISABLE;
    DMAMUX_InitStructure.Cmd = ENABLE;
    LHL_DMAMUX_Init(&DMAMUX_InitStructure);

    /* 4.2 配置DMA1，请求源来自DMAMUX UART_RX，用于搬运UART RX数据 */
    hDMA1.Channel = DMA_CHANNEL_1;
    hDMA1.Request = DMA_HARDWARE_REQUEST;
    hDMA1.Mode = DMA_CHAINING_MODE;
    hDMA1.Init.Direction = DMA_PERIPH_TO_MEMORY;                // 外设->内存
    hDMA1.Init.Src_Address = (uint32_t)(&pUART0->RBR_THR_DLL);
    hDMA1.Init.Dest_Address = (uint32_t)commBuffer;             // 串口数据环形缓存区
    hDMA1.Init.Data_Width = DMA_DATA_WIDTH_1B;
    hDMA1.Init.Data_Size = 1;
    hDMA1.Init.Repetition = MAX_COMM_SIZE;
    hDMA1.Init.Trans_Mode = DMA_CYCLIC_TRANSMISSION;
    hDMA1.Init.Chaining = TO_DMA_CHANNEL_2;                     // 链式触发DMA2
    if (LHL_DMA_Init(&hDMA1) != LHL_OK) {
        while(1);
    }    

    /* 4.3 配置DMA通道2，由DMA1触发，用于搬运BTIM设置参数 */    
    hDMA2.Channel = DMA_CHANNEL_2;                              // 句柄->Handle1
    hDMA2.Request = DMA_HARDWARE_REQUEST;
    hDMA2.Mode = DMA_SCATTER_GATHER_MODE;                       // 分散-聚合模式
    hDMA2.TCD_Count = TCD_COUNT;                                // TCD描述符数量
    hDMA2.TCD_List = TCD_Quene;                                 // TCD描述符链表
    hDMA2.TCD_Init = DMA_InitStructure;                         // TCD初始化队列

    DMA_InitStructure[0].Direction = DMA_MEMORY_TO_PERIPH;      // TCD0用于复位BTIM寄存器
    DMA_InitStructure[0].Src_Address = (uint32_t)&BTimerCTRLValue[0];
    DMA_InitStructure[0].Dest_Address = (uint32_t)(&pBTIM->TCTRL_1);
    DMA_InitStructure[0].Data_Width = DMA_DATA_WIDTH_4B;
    DMA_InitStructure[0].Data_Size = 1;
    DMA_InitStructure[0].Repetition = 1;
    DMA_InitStructure[0].Trans_Mode = DMA_CYCLIC_TRANSMISSION;
    DMA_InitStructure[0].TCD_Address = (int32_t)&TCD_Quene[1];
    DMA_InitStructure[0].Auto_Start = DISABLE;
    DMA_InitStructure[0].INT_Major = DISABLE;
    DMA_InitStructure[0].INT_Half = DISABLE;
    DMA_InitStructure[0].Chaining = TO_DMA_CHANNEL_NONE;

    DMA_InitStructure[1].Direction = DMA_MEMORY_TO_PERIPH;      // TCD1用于重启BTIM寄存器
    DMA_InitStructure[1].Src_Address = (uint32_t)&BTimerCTRLValue[1];
    DMA_InitStructure[1].Dest_Address = (uint32_t)(&pBTIM->TCTRL_1);
    DMA_InitStructure[1].Data_Width = DMA_DATA_WIDTH_4B;
    DMA_InitStructure[1].Data_Size = 1;
    DMA_InitStructure[1].Repetition = 1;
    DMA_InitStructure[1].Trans_Mode = DMA_CYCLIC_TRANSMISSION;
    DMA_InitStructure[1].TCD_Address = (int32_t)&TCD_Quene[0];
    DMA_InitStructure[1].Auto_Start = ENABLE;
    DMA_InitStructure[1].INT_Major = DISABLE;
    DMA_InitStructure[1].INT_Half = DISABLE;
    DMA_InitStructure[1].Chaining = TO_DMA_CHANNEL_NONE;
    if (LHL_DMA_Init(&hDMA2) != LHL_OK) {
        while(1);
    }

    LHL_DMA_Start(&hDMA1);
}
