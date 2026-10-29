#ifndef _LH32_UART_H
#define _LH32_UART_H

#include "lh32m0xx_lhl.h"

/* UART 自检结果: uart_init() 结束时自动填充, 供 Debug watch 直接看。
 * 每一项都是"寄存器实读值", 对照注释里的期望值就能定位故障点。 */
typedef struct
{
    uint32_t init_ret;    /* LHL_UART_Init 返回值  期望 0(LHL_OK); 3 = LHL_TIMEOUT */
    uint32_t con0;        /* UART_TX_PORT->CON0      TX 脚对应的 4 位复用字段应为 3 */
    uint32_t oe;          /* UART_TX_PORT->OE       TX 脚必须=1(输出使能) */
    uint32_t data_out;    /* UART_TX_PORT->DATA_OUT TX 脚空闲时应=1(空闲高电平) */
    uint32_t lcr;         /* LCR                  期望 0x03 (8 数据位/1 停止/无校验, DLAB=0) */
    uint32_t lcr_dlab;    /* LCR(DLAB=1 时)       期望 0x83 */
    uint32_t dll;         /* 波特率分频低字节     期望 8   (16MHz/16/115200) */
    uint32_t dlh;         /* 波特率分频高字节     期望 0 */
    uint32_t dlf;         /* 小数分频 DLF         期望 11  (11/16 = 0.6875) */
    uint32_t ier;         /* IER                  期望 0 */
    uint32_t lsr;         /* LSR                  空闲期望 0x60 (THRE|TEMT) */
    uint32_t usr;         /* USR                  bit0 = BUSY, 空闲期望 0 */
    uint32_t uer_before;  /* *(0x40013830) 写入前 UER: bit0=RE bit1=TE */
    uint32_t uer_after;   /* 同上, 写入 TE|RE 之后。读回 0x3 = 寄存器存在且已使能;
                           * 仍为 0 = 该地址无寄存器(说明本芯片把 TE/RE 硬接常开) */
    uint32_t ucv;         /* *(0x40013834) 组件版本。非 0 说明确实是 DW_apb_uart 的寄存器区 */

    /* ---- 内部回环自检(MCR.LOOPBACK): 把发送器输出直接在芯片内部接回接收器 ----
     * 这一步能证明"波特率 + 帧格式 + 发送器"整条芯内链路真的在工作,
     * 并且与外部引脚/接线完全无关。 */
    uint32_t loop_lsr;    /* 回环发送后读到的 LSR, 期望 bit0(DR)=1 */
    uint32_t loop_dr;     /* 1 = 收到了自己发的字节 */
    uint32_t loop_rx;     /* 收到的字节, 期望 0x5A */

    /* ---- AF 扫描(当前生效的复用编号) ---- */
    uint32_t scan_pin;    /* 正在扫描的引脚: 1 = P1.6, 0 = P0.7 */
    uint32_t scan_af;     /* uart_af_scan() 正在测试的 AF 值 0~7 */
} UART_Check_t;

extern UART_Check_t g_uart_check;

void uart_init(void);

/* P1.6 引脚通路自检(不会返回): 把 P1.6 切成普通 GPIO 输出 1Hz 方波。
 * 用万用表(DC)量你准备接串口的那根排针:
 *     读到 ≈ VDD/2  -> 该脚确实是 P1.6 且引脚通路正常
 *     读到 0V 或 VDD 不动 -> 引脚没通到排针, 或你量的是别的脚
 * 该函数会占用 P1.6, 测完把 main.c 里的 UART_PIN_TEST 改回 0。 */
void uart_pin_test(void);

/* 连续发送 0x55(不会返回): 平均电平 ≈ VDD/2。
 * 用万用表(DC)量 P1.6, 能区分"芯片到底有没有在移位发送":
 *     ≈ VDD/2 且跳动 -> 在发, 问题在适配器/接线
 *     稳定 VDD        -> 没在发(或 AF 值没路由到 P1.6)
 * 测完把 main.c 里的 UART_TX_TEST 改回 0。 */
void uart_tx_test(void);

#endif
