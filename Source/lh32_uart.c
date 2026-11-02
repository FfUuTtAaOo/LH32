#include "lh32_uart.h"

/* ---- 串口引脚(改线只改这几行) ----
 * P1.5 = SIN0(RX), P1.6 = SOUT0(TX), 115200 8N1 */
#define UART_RX_PORT        pGPIO1
#define UART_RX_PIN         GPIO_PIN_5
#define UART_RX_AF          GPIO1_5_AF_SIN0

#define UART_TX_PORT        pGPIO1
#define UART_TX_PIN         GPIO_PIN_6
#define UART_TX_AF          GPIO1_6_AF_SOUT0

#define UART_BAUD_RATE      115200

uint8_t uart_buffer[UART_SIZE] = {0};       // 串口收发缓存
uint8_t uart_rx_count = 0;
uint8_t uart_tx_count = 0;

void uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    /* Configure UART GPIO multiplexing pins */
    GPIO_InitStructure.Pin = UART_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = ENABLE;
    GPIO_InitStructure.Alternate = UART_RX_AF;
    LHL_GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);                           // UART_RX

    GPIO_InitStructure.Pin = UART_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = UART_TX_AF;
    LHL_GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);                           // UART_TX

    /* Set UART parameters */
    UART_InitStructure.UART_BaudRate = UART_BAUD_RATE;                          // The maximum baud rate must be less than the clock frequency / 16
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;                    // data: 8
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                         // stop: 1
    UART_InitStructure.UART_Parity = UART_Parity_None;                          // parity: None
    LHL_UART_Init(pUART0, &UART_InitStructure);

    /* 3. 开启UART中断并设置优先级 */
    LHL_UART_ITConfig(pUART0, UART_IT_ERBFI, ENABLE);                           // 使能UART0接收中断    
    NVIC_EnableIRQ(UART0_IRQn);                                                 // 开启NVIC中断
    NVIC_SetPriority(UART0_IRQn, 1);                                            // 设置NVIC优先级为1
}

/* UART0 接收中断。
 * 先读 IIR 判断中断源, 只有"收到数据/线路状态/FIFO 超时"才去取数据;
 * IIR 里查不到原因说明是 IRQ 线被拉住的空转中断, 这时什么也不做(不能读 RBR,
 * 否则会把真实数据吃掉), 只累计计数, 达到保险丝值就把 RX 中断关掉。 */
void UART0_IRQHandler(void)
{
    uint8_t it = LHL_UART_GetITStatus(pUART0);                                  // 获取中断标志位

    switch (it) {
    case UART_IT_RXNE:
        uart_buffer[uart_rx_count++] = LHL_UART_ReadData(pUART0);                 // 接收数据
        break;

    default:
        break;
    }
}
