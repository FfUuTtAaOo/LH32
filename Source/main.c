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
 *   传感器经外部调理后的输出:  1.7V(下限) -- 2.5V(中点) -- 3.3V(上限)
 *   线性映射到 DAC 输出:        0V         -- 2.5V        -- 5V
 *       Vout = (Vin - 1.7V) / (3.3V - 1.7V) × 5V , 超出范围自动钳位
 *   校验: 2.1V -> 1.25V    2.5V -> 2.5V    3.3V -> 5V
 *   (DAC8562 = 内部2.5V基准 + 增益2 -> 输出满量程 0~5V) */
#define SENSOR_VIN_MIN_MV   1700.0f     /* 输入下限, 对应输出 0V     */
#define SENSOR_VIN_MAX_MV   3300.0f     /* 输入上限, 对应输出满量程  */
/* DAC 输出满量程(单位: V), 直接由 DAC 驱动参数推导, 改增益时会自动跟随 */
#define DAC_VOUT_MAX_V      (DAC8562_INTERNAL_REF_V * (float)DAC8562_GAIN)

/* ---------- DAC 自检开关 ----------
 * 1 = 自检模式: 两个通道输出"固定电压"(A=1.25V, B=3.75V), 输出保持不动,
 *     便于用万用表直接在 DAC 引脚上验证 "软件 -> SPI -> DAC" 这一段是否正常;
 * 0 = 正常工作: 传感器输入电压按 1.7~3.3V -> 0~5V 线性映射后输出。
 * 排查完请改回 0。 */
#define DAC_SELFTEST        0

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

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  传感器输入电压 -> DAC 目标输出电压
  * @param  vin_mv : 传感器输入电压, 单位 mV
  * @retval DAC 目标输出电压, 单位 **V** (0 ~ DAC_VOUT_MAX_V)
  * @note   1.7V 及以下 -> 0V; 3.3V 及以上 -> 满量程; 之间线性插值
  *         返回单位是 V, 以便直接喂给 dac8562_voltage_to_code()
  */
static float sensor_to_dac_v(float vin_mv)
{
    if (vin_mv <= SENSOR_VIN_MIN_MV)
    {
        return 0.0f;
    }
    if (vin_mv >= SENSOR_VIN_MAX_MV)
    {
        return DAC_VOUT_MAX_V;
    }
    return (vin_mv - SENSOR_VIN_MIN_MV) * DAC_VOUT_MAX_V
         / (SENSOR_VIN_MAX_MV - SENSOR_VIN_MIN_MV);
}

int main(void)
{
    int32_t fz_raw, fx_raw;
    float   vout_fz, vout_fx;
    
    /* 上电后IWDG默认开启，若无处理约20秒后复位 */
    LHL_IWDG_Stop();
    
    /* 复位所有GPIO默认状态 */
    LHL_GPIO_DeInit();

    /* 系统时钟: 配置为内部 HSI 二分频 = 16MHz -> SYSCLK=HCLK=PCLK1=PCLK2=16MHz */
    clock_init();

    /* Init Config */
    gpio_init();                    /* P1.5=LDAC, P1.6=CLR */
    spi_init();                     /* SPI1 主机(P0.1/P0.2, AF1), 连接 DAC8562 */
    i2c_init();
    adc_init();                     /* ADC0/ADC1 同步采样 -> DMA 搬运到 dADCBuffer(内部已启动) */

    /* DAC8562 上电配置: 软件复位 + 双通道上电 + 使能内部基准 + 设置增益 */
    dac8562_init();

#if (DAC_SELFTEST)
    /* ---- 自检模式: 输出两个固定电压, 并保持不变 ----
     * 用万用表(黑表笔接 DAC8562 自己的 GND 脚)量 VOUTA / VOUTB 引脚:
     *     应量到  DAC-A ≈ 1.25V ,  DAC-B ≈ 3.75V
     *   - 能量到   -> "软件 -> SPI -> DAC" 整条链路正常, 之前的异常值来自下游电路/测量点;
     *   - 量不到   -> 0V/负值/乱跳, 问题在 SPI 接线、DAC 供电(AVDD)或基准电容。
     *
     * Debug 里可 watch 全局变量 g_dac_spi_status:
     *     0 = LHL_OK      -> SPI 24 个时钟正常发完;
     *     3 = LHL_TIMEOUT -> SPI 没产生时钟(引脚错配/外设未使能), 需检查 spi_init()。
     * 验证完把 DAC_SELFTEST 改回 0 即恢复正常的传感器映射逻辑。 */
    dac8562_write(DAC8562_CH_A, dac8562_voltage_to_code(2.5f));
    dac8562_write(DAC8562_CH_B, dac8562_voltage_to_code(3.0f));
    while (1)
    {
        LHL_Delay_uS(100000);       /* 输出保持不动, 方便测量 */
    }
#else
    while(1)
    {
        /* 六维力传感器接线(外部已做信号调理电路, 输出 1.7~3.3V 到 ADC):
         *   FZ -> AIN0 -> ADC0 -> dADCBuffer.ADC0_Data -> DAC-A
         *   FX -> AIN1 -> ADC1 -> dADCBuffer.ADC1_Data -> DAC-B
         *
         * 两路 ADC 同步采样(同一时刻的一对样本), 用 write_both 由 LDAC 下降沿
         * 同步更新, 保证两路输出同时生效。
         * 若想把 FX 送到 DAC-A, 把下面两句的变量对调即可。
         *
         * 负端接法与基准见 Include/lh32_adc.h 的 SENSOR_AINM_MODE / ADC_VREF_MV
         * (唯一来源, 不要在别处写死):
         *   0 = 单端(AVSS): 码 0x000000~0x7FFFFF 只表示 0 ~ +VREF; 输入到 0V 附近
         *       时因零点微负, 会看到 0xFFFFFF(=-1) 这种"FF 开头"的值。
         *   1 = 差分(REFP): 以基准节点为码 0 点, 低于它的输入全是**负码**,
         *       所以在 watch 里按 16 进制看必然是 0xFF 开头 —— 这是预期现象。
         *   16 进制里负数必然以 FF 开头, 是符号扩展的结果, 不是错误值。
         * 饱和特征: 码被削平在 +0x7FFFFF / -0x800000, 读数卡住不动。 */

        /* ---- 1) 24位补码 -> 有符号, 换算成传感器输入电压(mV) ---- */
        g_fz_raw = fz_raw = ((int32_t)dADCBuffer.ADC0_Data << 8) >> 8;   /* AIN0 = FZ */
        g_fx_raw = fx_raw = ((int32_t)dADCBuffer.ADC1_Data << 8) >> 8;   /* AIN1 = FX */
#if (SENSOR_AINM_MODE == 1)
        /* 负端 = REFP(内部基准节点) 的差分测量:
         *   码表示的是"输入相对基准的偏差", 所以
         *      Vin(mV) = Vref + 码/2^23 × Vref = Vref × (1 + 码/2^23)
         *   以 Vref=2.5V 为例:
         *      码 0           -> 2.500V  (中点)
         *      码 +0x28F5C3   -> 3.300V  (只占 +32% 量程, 远离饱和)
         *      码 -0x28F5C3   -> 1.700V
         *   好处: 传感器的 1.7~3.3V 整段只占 ±0.32 满量程, 再也不会被削平。 */
        g_fz_mv = ADC_FULLSCALE_MV * (1.0f + (float)fz_raw / 8388608.0f);
        g_fx_mv = ADC_FULLSCALE_MV * (1.0f + (float)fx_raw / 8388608.0f);
#else
        /* 负端 = AVSS 的经典单端测量: Vin(mV) = 码/2^23 × Vref(量程 0 ~ +Vref) */
        g_fz_mv = (float)fz_raw / 8388608.0f * ADC_FULLSCALE_MV;
        g_fx_mv = (float)fx_raw / 8388608.0f * ADC_FULLSCALE_MV;
#endif

        /* ---- 2) 输入电压 -> 目标输出电压: 1.7~3.3V 线性映射到 0~5V ----
         *      sensor_to_dac_v() 返回单位是 V, 直接喂给 DAC 驱动(其入参也是 V) */
        vout_fz = sensor_to_dac_v(g_fz_mv);
        vout_fx = sensor_to_dac_v(g_fx_mv);
        g_fz_vout_mv = vout_fz * 1000.0f;   /* 供 watch 观察(换算成 mV) */
        g_fx_vout_mv = vout_fx * 1000.0f;

        /* ---- 3) 双通道同步写入 DAC (电压->码, LDAC 同步更新) ---- */
        dac8562_write_both(dac8562_voltage_to_code(vout_fz / 2.0f),
                           dac8562_voltage_to_code(vout_fx / 2.0f));

        /* 主循环以"当前配置下可达的最高速率"刷新 DAC 双通道。
         * 当前 SYSCLK = HSI/2 = 16MHz (见 clock_init), PCLK1 = 16MHz,
         * SPI1(挂 APB1) SCLK = PCLK1/32 = 500kHz (为排线抗干扰故意降速):
         *   一帧 24bit @500kHz ≈ 48us -> write_both(两帧) ≈ 100us -> 约 10kHz
         *
         * 注意: 这里只是把"同一组数据"重复写进 DAC, 新样本仍然由 ADC 速率决定
         * (adc_init 中 FS = ADC_SPS_50, 即 20ms 才出一对新数据), 所以整链的有效
         * 吞吐仍是 50 对/秒。若要做到"每个新样本只处理一次", 应改为等待 DMA 完成
         * 标志(在 DMA1_CH0_IRQHandler 中置位)后再读取 dADCBuffer。 */
    }
#endif
}
/*********************************End of File**********************************/
