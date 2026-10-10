/******************************************************************************
 * DAC8562 驱动 (TI DAC8562S, 16-bit 双通道电压输出 DAC)
 ******************************************************************************
 * 硬件连接 (LH32M0G3x -> DAC8562):
 *   P0.3  ->  SYNC (片选, 低有效, 由软件控制)
 *   P0.1  ->  SCLK
 *   P0.2  ->  DIN  (MOSI)
 *   P1.5  ->  LDAC (低有效, 异步方式用于同步更新双通道)
 *   P1.6  ->  CLR  (低有效, 异步清零, 清零到零刻度)
 *
 * 说明:
 *   - DAC8562 只写器件, 无 MISO, 数据在 SCLK 下降沿被锁存, 使用 SPI Mode 1。
 *   - 一帧为 24 位: [X X | C2 C1 C0 | A2 A1 A0 | D15 ... D0]
 *   - 使用内部 2.5V 基准, 增益 1 -> 输出范围 0 ~ 2.5V。
 ******************************************************************************/
#ifndef _LH32_DAC8562_H
#define _LH32_DAC8562_H

#include "lh32m0xx_lhl.h"

/* 通道地址 (DB18-DB16) */
#define DAC8562_CH_A            0x00u       /* DAC-A */
#define DAC8562_CH_B            0x01u       /* DAC-B */
#define DAC8562_ADDR_GAIN       0x02u       /* 增益配置 (仅配合命令 000) */
#define DAC8562_ADDR_ALL        0x07u       /* DAC-A 和 DAC-B */

/* 内部基准电压 / 输出增益 (增益1 -> 0~2.5V; 增益2 -> 0~5V) */
#define DAC8562_INTERNAL_REF_V  2.5f
#define DAC8562_GAIN            1u

/* 初始化 (上电配置: 复位 + 使能内部基准 + 设置增益 + 上电) */
void     dac8562_init(void);

/* 向指定通道写入 16 位码并立即更新输出 */
void     dac8562_write(uint8_t channel, uint16_t code);

/* 按电压值写入 (自动换算为 DAC 码, 超出范围自动钳位) */
void     dac8562_write_voltage(uint8_t channel, float voltage);

/* 双通道同时更新 (异步方式: 先写输入寄存器, 再以 LDAC 下降沿同步更新) */
void     dac8562_write_both(uint16_t code_a, uint16_t code_b);

/* 直接控制 LDAC 引脚电平 (1:高, 0:低) */
void     dac8562_set_ldac(uint8_t level);

/* 硬件清零: CLR 下降沿, 两个通道输出复位到零刻度 */
void     dac8562_clear(void);

/* 电压值 -> DAC 码 */
uint16_t dac8562_voltage_to_code(float voltage);

#endif
