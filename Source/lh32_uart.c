#include "lh32_uart.h"

/* ---- DW_apb_uart 的两个"隐藏"寄存器 ----
 * 本芯片 UART 的寄存器分布与 Synopsys DW_apb_uart 完全一致
 * (FAR@0x70, USR@0x7C, HTX@0xA4, DMASA@0xA8, DLF@0xC0, RAR@0xC4, TAR@0xC8)，
 * 但 SDK 的 UART_TypeDef 把 0x20~0x6F 整段留成了 RESERVED0，其中就包括:
 *   0x30 = UER (UART Enable Register): bit0=RE 接收使能, bit1=TE 发送使能, 复位值 0
 *   0x34 = UCV (Component Version):   固定非 0, 可用来确认 0x30 确实是 UER
 * SDK 的 LHL_UART_Init 只配波特率和 LCR，**从未打开 TE/RE**。
 * TE=0 时写 THR 不会移出任何数据，而 LSR.TEMT 仍为 1 —— 表现正好是
 * "printf 立刻返回、程序照常跑、串口一个字节都收不到"。
 * 这里先读、再置位、再回读，结果放进 g_uart_check 便于判断该寄存器是否存在。 */
#define UART0_UER_ADDR      (UART0_BASE + 0x30u)
#define UART0_UCV_ADDR      (UART0_BASE + 0x34u)
#define UART_UER_RE         (1u << 0)
#define UART_UER_TE         (1u << 1)

/* ---- 串口引脚(改线只改这几行) ----
 * 默认: P1.5 = SIN0(RX), P1.6 = SOUT0(TX)。
 * 如果确认 P1.6 上怎么都出不来波形, 可整组换成 P0.6/P0.7:
 *     UART_RX_PORT pGPIO0  UART_RX_PIN GPIO_PIN_6  UART_RX_AF GPIO0_6_AF_SIN0
 *     UART_TX_PORT pGPIO0  UART_TX_PIN GPIO_PIN_7  UART_TX_AF GPIO0_7_AF_SOUT0
 * (P0.0~P0.3 已被 SPI1/DAC 占用; P0.6/P0.7 目前空闲) */
#define UART_RX_PORT        pGPIO1
#define UART_RX_PIN         GPIO_PIN_5
#define UART_RX_AF          GPIO1_5_AF_SIN0

#define UART_TX_PORT        pGPIO1
#define UART_TX_PIN         GPIO_PIN_6
#define UART_TX_AF          GPIO1_6_AF_SOUT0

UART_Check_t g_uart_check;

static void uart_selfcheck(void)
{
    uint32_t lcr;

    g_uart_check.con0     = UART_TX_PORT->CON0;
    g_uart_check.oe       = UART_TX_PORT->OE;
    g_uart_check.data_out = UART_TX_PORT->DATA_OUT;
    g_uart_check.ier      = pUART0->IER_DLH;
    g_uart_check.dlf      = pUART0->DLF;
    g_uart_check.lsr      = pUART0->LSR;
    g_uart_check.usr      = pUART0->USR;

    /* DLL/DLH 与 RBR/IER 共用地址, 必须先把 DLAB 置 1 才能读到分频值 */
    lcr = pUART0->LCR;
    g_uart_check.lcr = lcr;
    pUART0->LCR = lcr | UART_LCR_DLAB_Msk;
    g_uart_check.lcr_dlab = pUART0->LCR;
    g_uart_check.dll = pUART0->RBR_THR_DLL & 0xFFu;
    g_uart_check.dlh = pUART0->IER_DLH & 0xFFu;
    pUART0->LCR = lcr;                              /* 恢复 DLAB=0 */
}

/* 内部回环自检: MCR.LOOPBACK 把发送器的串行输出在芯片内部直接接回接收器,
 * 不经过任何引脚。能收到自己发的字节 = 波特率/帧格式/发送器/接收器全部正常,
 * 于是"收不到数据"就只可能出在引脚、板级走线或 PC 侧。 */
static void uart_loopback_test(void)
{
    uint32_t mcr = pUART0->MCR;
    uint32_t t = 2000000u;                          /* 远超一帧时间(115200 下约 87us) */

    pUART0->MCR = mcr | UART_MCR_LOOPBACK_Msk;      /* 进入内部回环 */
    pUART0->RBR_THR_DLL = 0x5Au;                    /* 发送 0x5A */

    while (((pUART0->LSR & UART_FLAG_DR) == 0u) && (t-- != 0u))
    {
    }

    g_uart_check.loop_lsr = pUART0->LSR;
    g_uart_check.loop_dr  = (g_uart_check.loop_lsr & UART_FLAG_DR) ? 1u : 0u;
    g_uart_check.loop_rx  = pUART0->RBR_THR_DLL & 0xFFu;   /* 读 RBR 会清 DR */

    pUART0->MCR = mcr;                              /* 退出回环 */
}

/* 连续发送 0x55(0b01010101): 串行帧里高低电平各占一半, 平均电平 ≈ VDD/2。
 * 用万用表(DC)量 P1.6:
 *     ≈ VDD/2 且读数跳动  -> SOUT0 确实路由到了 P1.6, 芯片在发 -> 问题在适配器/接线/PC
 *     稳定 VDD 不动       -> 这个 AF 值没路由到 SOUT0, 或 UART 根本没在移位
 * 这是唯一能用万用表区分"芯片到底发没发"的办法(单次 printf 的占空比太低, 看不出来)。 */
void uart_tx_test(void)
{
    while (1)
    {
        if (pUART0->LSR & UART_FLAG_THRE)
        {
            pUART0->RBR_THR_DLL = 0x55u;
        }
    }
}

void uart_pin_test(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* P1.6 从 UART 复用切回普通 GPIO 推挽输出 */
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIOx_AF_GPIO;
    LHL_GPIO_Init(pGPIO1, &GPIO_InitStructure);

    while (1)
    {
        LHL_GPIO_SetPins(pGPIO1, GPIO_PIN_6);
        LHL_Delay_uS(200000);
        LHL_GPIO_ResetPins(pGPIO1, GPIO_PIN_6);
        LHL_Delay_uS(200000);
    }
}

/* ---- AF 暴力扫描 ----
 * 板上只有 P1.5 / P1.6 两根线, 但"SOUT0 到底挂在哪个 AF 值上"无法从现象反推,
 * 于是干脆把 CON0 中该脚的 4 位复用字段 0~7 全试一遍, 每试一个就连续发一段 0x55,
 * 逻辑分析仪哪一段出 50% 占空比方波, 那个 AF 值就是答案。
 *
 * 关键设计: 第 af 段的字节数 = (af+1)*1000, 所以**波形长度就代表 AF 值**
 * (AF0 ≈ 87ms, AF7 ≈ 696ms), 不用盯着调试器看 g_uart_check.scan_af 也能认出来。
 * 每段之间静默 0.5s, 两轮(先 P1.6, 再 P1.5)之间静默 2s。
 *
 * ⚠️ 第二轮会把 P1.5 也配成输出: 若 USB-TTL 的 TX 还接在 P1.5 上, 会与本芯片
 *    对顶。因此 P1.5 用开漏+内部上拉, 只会拉低不会推高, 避免损坏器件。
 *    (P1.6 是本芯片 TX -> 适配器 RX, 无冲突, 用推挽以便波形干净。) */
static void uart_tx_burst(uint32_t count)
{
    while (count-- != 0u)
    {
        while ((pUART0->LSR & UART_FLAG_THRE) == 0u)
        {
        }
        pUART0->RBR_THR_DLL = 0x55u;
    }
}

static void uart_scan_pin(GPIO_TypeDef *pGPIO, uint16_t pin, uint32_t shift,
                          GPIOMode_TypeDef mode, GPIOPuPd_TypeDef pull)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint32_t af;

    GPIO_InitStructure.Pin = pin;
    GPIO_InitStructure.Mode = mode;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = pull;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIOx_AF_GPIO;
    LHL_GPIO_Init(pGPIO, &GPIO_InitStructure);

    for (af = 0u; af <= 7u; af++)
    {
        g_uart_check.scan_pin = (uint32_t)pin;
        g_uart_check.scan_af  = af;

        pGPIO->CON0 = (pGPIO->CON0 & ~(0xFu << shift)) | (af << shift);
        uart_tx_burst((af + 1u) * 1000u);           /* 段长 ∝ AF 值 */
        LHL_Delay_uS(500000);
    }
}

void uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    uint32_t uer;

    /* Configure UART GPIO multiplexing pins */
    GPIO_InitStructure.Pin = GPIO_PIN_5;               
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = ENABLE;
    GPIO_InitStructure.Alternate = GPIO1_5_AF_SIN0;                             // AFIO设置
    LHL_GPIO_Init(pGPIO1, &GPIO_InitStructure);                                 // P0.1 -> SIN0(UART_RX)

    GPIO_InitStructure.Pin = GPIO_PIN_6;               
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIO1_6_AF_SOUT0;                            // AFIO设置
    LHL_GPIO_Init(pGPIO1, &GPIO_InitStructure);                                 // P0.2 -> SOUT0(UART_TX)

    /* Set UART parameters */
    UART_InitStructure.UART_BaudRate = 115200;                                  // The maximum baud rate must be less than the clock frequency / 16
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;                    // data: 8
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                         // stop: 1
    UART_InitStructure.UART_Parity = UART_Parity_None;                          // parity: None
    g_uart_check.init_ret = (uint32_t)LHL_UART_Init(pUART0, &UART_InitStructure);

//    /* 打开发送/接收使能(SDK 漏掉的一步)。若本芯片没有 UER, 写入会被忽略,
//     * 回读值 uer_after 仍为 0, 不影响其它逻辑。 */
//    uer = *(volatile uint32_t *)UART0_UER_ADDR;
//    g_uart_check.uer_before = uer;
//    *(volatile uint32_t *)UART0_UER_ADDR = (uer & ~(UART_UER_RE | UART_UER_TE))
//                                         | UART_UER_RE | UART_UER_TE;
//    g_uart_check.uer_after = *(volatile uint32_t *)UART0_UER_ADDR;

//    g_uart_check.ucv = *(volatile uint32_t *)UART0_UCV_ADDR;

//    uart_selfcheck();
//    uart_loopback_test();
}
