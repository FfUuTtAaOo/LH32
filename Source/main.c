/******************************************************************************
 * 版权所有：苏州领慧立芯科技有限公司
 * Copyright (c) 2020-2025 Suzhou Legendsemi Technology Co., Ltd.
 ******************************************************************************
 * All rights reserved. Distributed under MIT license.
 * The file is encoded in UTF-8 without signature.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lh32m0xx_lhl.h"
#include "lh32_clock.h"
#include "lh32_gpio.h"
#include "lh32_spi.h"
#include "lh32_i2c.h"
#include "lh32_adc.h"
#include "lh32_uart.h"
#include "lh32_dac8562.h"

/* Private define ------------------------------------------------------------*/ 
uint8_t I2C_RX_Data[10], I2C_TX_Data[10];

/* ---------- ADC 原始码 -> 输入电压 ----------
 * 基准电压 / PGA 增益 / 负端接法 统一定义在 Include/lh32_adc.h
 * (ADC_VREF_MV / SENSOR_PGA / SENSOR_AINM_MODE), 这里不再各写一份 ——
 * 之前 main.c 写 3300、lh32_adc.c 写 4096 的"两处基准不一致"就是这么来的。
 *   ⚠️ 另外记牢: lh32_adc.c 里那个 ADC_InitStructure.Reference 是**死字段**,
 *      改它不影响任何硬件, 只有这里的数学参与"码 -> 电压"换算。
 *
 * 码制: ADC_CODE_BIPOLAR = 24 位**补码**(不是偏移二进制!)
 *     0(相对负端) -> 0x000000,  +满量程 -> 0x7FFFFF,  负 -> 0x800000~0xFFFFFF
 *   正满量程(相对负端) = ADC_VREF_MV / PGA增益 */
#define ADC_PGA_GAIN        1.0f
#define ADC_FULLSCALE_MV    (ADC_VREF_MV / ADC_PGA_GAIN)

/* ---------- 传感器输入 -> DAC 输出 映射 (按需求) ----------
 *   传感器(经外部调理)双极性输出:  -2.5V(下限) -- 0V(中点) -- +2.5V(上限)
 *   线性映射到 DAC 输出:             0V           -- 1.25V     -- 2.5V
 *       Vout = (Vin + 2.5V) / 5V × 2.5V , 超出范围自动钳位
 *   校验: -2.5V -> 0V     0V -> 1.25V     +2.5V -> 2.5V
 *   (DAC8562 = 内部 2.5V 基准 + 增益 1 -> 输出满量程 0~2.5V)
 *   单位: 入参是 **V**, 与主循环里 g_fz_mv / g_fx_mv 的实际单位一致
 *         (第 1 步是 raw/2^23 × 2.5, 结果就是 ±2.5 的伏特值, 不是 mV)。 */
#define SENSOR_VIN_MIN_V    (-2500.0f)     /* 输入下限, 对应输出 0V     */
#define SENSOR_VIN_MAX_V    2500.0f        /* 输入上限, 对应输出满量程  */
/* DAC 输出满量程(单位: V), 直接由 DAC 驱动参数推导, 改增益时会自动跟随 */
#define DAC_VOUT_MAX_V      (DAC8562_INTERNAL_REF_V * (float)DAC8562_GAIN)

/* ---------- DAC 自检开关 ----------
 * 1 = 自检模式: 两个通道输出"固定电压"(A=1.25V, B=3.75V), 输出保持不动,
 *     便于用万用表直接在 DAC 引脚上验证 "软件 -> SPI -> DAC" 这一段是否正常;
 * 0 = 正常工作: 传感器输入电压 -> 0~5V 线性映射后输出。
 * 排查完请改回 0。 */
#define DAC_SELFTEST        0

/* ---------- 串口调试打印开关 ----------
 * 1 = 每个新 ADC 样本(50Hz)向 UART0 打印一行原始码/电压/目标输出;
 * 0 = 关闭, 主循环只做 ADC -> DAC, 不占用 CPU 在串口上。
 * UART0: P1.5 = SIN0(RX), P1.6 = SOUT0(TX), 115200 8N1, 见 lh32_uart.c。
 * 注意: 打印在 115200 下每行约 5ms, 会占用主循环时间, 但 ADC 只有 50SPS,
 *       所以不会丢样本(靠 g_adc_seq 判断新样本, 不会重复打印同一组)。 */
#define UART_DEBUG          0

/* ---------- P1.6 引脚通路自检 ----------
 * 1 = uart_init() 之后不再跑业务, 把 P1.6 切成普通 GPIO 输出约 1Hz 方波。
 *     用万用表(DC)量准备接串口的那根排针: 读到 ≈VDD/2 说明该脚确实是 P1.6 且
 *     引脚通路正常; 读到 0V 或 VDD 不动 说明脚位/走线不对。
 * 0 = 正常工作。
 * ⚠️ 打开时串口不会有任何输出(该脚已改成 GPIO)。测完改回 0。 */
#define UART_PIN_TEST       0

/* ---------- UART 连续发送自检 (AF 扫描已取代, 保留 0) ----------
 * 1 = uart_init() 之后连续发送 0x55; 用万用表(DC)量 P1.6 区分"芯片到底发没发"。
 * 0 = 关闭。要定位 AF 值请改用下面的 UART_AF_SCAN。 */
#define UART_TX_TEST        0

/* 供 Debug 的 watch 窗口直接观察。volatile 防止被优化掉。
 * g_fz_raw / g_fx_raw = 原始 24 位 **补码**(有符号数!):
 *   十进制显示范围 -8388608 ~ +8388607; 按 16 进制显示时, 负数一律是 0xFFxxxxxx
 *   (高 8 位 FF 是符号扩展出来的, 不是数据、也不是溢出标记):
 *       0x000000 = 0       0x7FFFFF = +满量程
 *       0xFFFFFF = -1(约等于 0)  0xFF800000 = -满量程(负向削平)
 *   "FF 开头"只有两种含义: ① 差分模式下输入低于 2.5V(正常负偏差);
 *                          ② 单端模式下 AIN 引脚在 0V 附近或更低(通常是没接信号)。
 *   码 -> 输入电压的公式随 SENSOR_AINM_MODE 自动切换, 见主循环。 */
volatile int32_t g_fz_raw, g_fx_raw;
volatile float g_fz_mv, g_fx_mv;                /* 传感器输入电压(FZ / FX) */
volatile float g_fz_vout_mv, g_fx_vout_mv;      /* 映射后的目标输出电压    */
float    vout_fz, vout_fx;
int32_t  fz_raw, fx_raw;

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  传感器输入电压 -> DAC 目标输出电压
  * @param  vin_v  : 传感器输入电压, 单位 **V** (极值 -2.5V ~ +2.5V)
  * @retval DAC 目标输出电压, 单位 **V** (0 ~ DAC_VOUT_MAX_V)
  * @note   -2.5V 及以下 -> 0V; +2.5V 及以上 -> 满量程; 之间线性插值
  *         返回单位是 V, 以便直接喂给 dac8562_voltage_to_code()
  */
static float sensor_to_dac_v(float vin_v)
{
    if (vin_v <= SENSOR_VIN_MIN_V)
    {
        return 0.0f;
    }
    if (vin_v >= SENSOR_VIN_MAX_V)
    {
        return DAC_VOUT_MAX_V;
    }
    return (vin_v - SENSOR_VIN_MIN_V) * DAC_VOUT_MAX_V
         / (SENSOR_VIN_MAX_V - SENSOR_VIN_MIN_V);
}

int main(void)
{
    /* 上电后IWDG默认开启，若无处理约20秒后复位 */
    LHL_IWDG_Stop();

    /* 复位所有GPIO默认状态 */
    LHL_GPIO_DeInit();

    /* 系统时钟: 配置为内部 HSI 二分频 = 16MHz -> SYSCLK=HCLK=PCLK1=PCLK2=16MHz */
    clock_init();

    /* Init Config */
    gpio_init();                    /* P1.1=LDAC, P1.4=CLR */
    uart_init();

#if (UART_PIN_TEST)
    uart_pin_test();                /* 不会返回: P1.6 输出方波, 供万用表/示波器验证走线 */
#endif

#if (UART_TX_TEST)
    uart_tx_test();                 /* 不会返回: 连续发 0x55, 万用表量 P1.6 应 ≈VDD/2 */
#endif
    printf("\r\nHello World! boot ok\r\n");

    spi_init();                     /* SPI1 主机(P0.1/P0.2, AF1), 连接 DAC8562 */
    i2c_init();
    adc_init();                     /* ADC0/ADC1 同步采样 -> DMA 搬运到 dADCBuffer(内部已启动) */

    /* DAC8562 上电配置: 软件复位 + 双通道上电 + 使能内部基准 + 设置增益 */
    dac8562_init();

#if (DAC_SELFTEST)
    /* ---- 自检模式: 输出两个固定电压, 并保持不变 ----*/
    dac8562_write(DAC8562_CH_A, dac8562_voltage_to_code(1.25f));
    dac8562_write(DAC8562_CH_B, dac8562_voltage_to_code(2.3f));
    while (1) {
        LHL_Delay_uS(100000);       /* 输出保持不动, 方便测量 */
    }
#else
    while(1)
    {
        /* ---- 1) 24位补码 -> 有符号, 换算成传感器输入电压(mV) ---- */
        g_fz_raw = fz_raw = ((int32_t)dADCBuffer.ADC0_Data << 8) >> 8;   /* AIN0 = FZ */
        g_fx_raw = fx_raw = ((int32_t)dADCBuffer.ADC1_Data << 8) >> 8;   /* AIN1 = FX */

        /* 负端 = AVSS 的经典单端测量: Vin(mV) = 码/2^23 × Vref(量程 0 ~ +Vref) */
        g_fz_mv = (float)fz_raw / 8388608.0f * ADC_FULLSCALE_MV;
        g_fx_mv = (float)fx_raw / 8388608.0f * ADC_FULLSCALE_MV;
        g_fz_mv = (-g_fz_mv);

        /* ---- 2) 输入电压 -> 目标输出电压: -2.5~+2.5V 线性映射到 0~2.5V ----
         *      sensor_to_dac_v() 返回单位是 V, 直接喂给 DAC 驱动(其入参也是 V) */
        vout_fz = sensor_to_dac_v(g_fz_mv);
        vout_fx = sensor_to_dac_v(g_fx_mv);
        g_fz_vout_mv = vout_fz * 1000.0f;   /* 供 watch 观察(换算成 mV) */
        g_fx_vout_mv = vout_fx * 1000.0f;

        /* ---- 3) 双通道同步写入 DAC (电压->码, LDAC 同步更新) ---- */
        dac8562_write_both(dac8562_voltage_to_code(vout_fx),
                           dac8562_voltage_to_code(vout_fz));

#if (UART_DEBUG)
        /* ---- 4) 串口打印: 仅在新样本到达时打印, 不重复刷同一组 ----
         * 全部用整数打印(%d), 不用 %f: MicroLIB 的 printf 浮点支持要额外配置,
         * 而这里 mV 精度到 1mV 已经足够观察。 */
        if (g_adc_seq != last_seq)
        {
            last_seq = g_adc_seq;
            printf("FZ raw=%d mV=%d out=%dmV | FX raw=%d mV=%d out=%dmV\r\n",
                   (int)g_fz_raw, (int)g_fz_mv, (int)g_fz_vout_mv,
                   (int)g_fx_raw, (int)g_fx_mv, (int)g_fx_vout_mv);
        }
#endif
    }
#endif
}
/*********************************End of File**********************************/
