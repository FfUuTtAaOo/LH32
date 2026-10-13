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

/* ADC 原始码 -> 电压换算参数(必须与 lh32_adc.c 的配置保持一致)
 *   ADC_REF_MV: 基准电压 mV (对应 REF_INTERNAL_2P5V)
 *   ADC_PGA_GAIN: 与 SENSOR_PGA 对应的实际倍数(×1 -> 1)
 *   电压(mV) = 有符号24位码 × (ADC_REF_MV / ADC_PGA_GAIN) / 2^23 */
#define ADC_REF_MV      2500.0f
#define ADC_PGA_GAIN    1.0f
#define ADC_FULLSCALE_MV (ADC_REF_MV / ADC_PGA_GAIN)   /* 正满量程电压, 对应码 0x7FFFFF */

/* 供 Debug 的 watch 窗口直接观察的电压值(mV)。volatile 防止被优化掉。 */
volatile float g_fz_mv, g_fx_mv;

/* Functions -----------------------------------------------------------------*/
int main(void)
{
    uint16_t fz_code, fx_code;
    int32_t  fz_raw, fx_raw;
    
    /* 上电后IWDG默认开启，若无处理约20秒后复位 */
    LHL_IWDG_Stop();
    
    /* 复位所有GPIO默认状态 */
    LHL_GPIO_DeInit();

    /* 系统时钟: 配置为内部 HSI 二分频 = 16MHz -> SYSCLK=HCLK=PCLK1=PCLK2=16MHz */
    clock_init();

    /* Init Config */
    gpio_init();                    /* P1.5=LDAC, P1.6=CLR */
    spi_init();                     /* SPI0 主机, 连接 DAC8562 */
    i2c_init();
    adc_init();                     /* ADC0/ADC1 同步采样 -> DMA 搬运到 dADCBuffer(内部已启动) */

    /* DAC8562 上电配置: 软件复位 + 使能内部基准 + 设置增益 + 上电 */
    dac8562_init();

    while(1)
    {
        /* 六维力传感器接线(外部已做信号调理电路, 单端输出到 ADC):
         *   FZ -> AIN0 -> ADC0 -> dADCBuffer.ADC0_Data -> DAC-A
         *   FX -> AIN1 -> ADC1 -> dADCBuffer.ADC1_Data -> DAC-B
         *
         * 两路 ADC 同步采样(同一时刻的一对样本), 用 write_both 由 LDAC 下降沿
         * 同步更新, 保证两路输出同时生效。
         * 若想把 FX 送到 DAC-A, 把下面两行的变量对调即可。
         *
         * 码制: Code = ADC_CODE_BIPOLAR 是 24 位"补码"(注意: 不是偏移二进制!):
         *     0V -> 0x000000,  +满量程 -> 0x7FFFFF,  负电压 -> 0x800000~0xFFFFFF
         *   (库函数 LHL_ADC_GetVoltage 用 ((int32_t)raw << 8) >> 8 做符号扩展可佐证)
         *   正满量程 = 基准2500mV / PGA增益(见 lh32_adc.c 的 SENSOR_PGA)
         *   一旦达到满码就是"饱和", 读数会卡住不动 -> 说明 SENSOR_PGA 太大或输入超量程。 */
        fz_raw = ((int32_t)dADCBuffer.ADC0_Data << 8) >> 8;   /* AIN0=FZ: 24位补码 -> 符号扩展 */
        fx_raw = ((int32_t)dADCBuffer.ADC1_Data << 8) >> 8;   /* AIN1=FX */

        /* 换算为 mV, 方便在 Debug 里直接看电压是否合理 */
        g_fz_mv = (float)fz_raw * ADC_FULLSCALE_MV / 8388608.0f;
        g_fx_mv = (float)fx_raw * ADC_FULLSCALE_MV / 8388608.0f;

        fz_code = (uint16_t)((dADCBuffer.ADC0_Data >> 8) & 0xFFFFu);   /* AIN0=FZ */
        fx_code = (uint16_t)((dADCBuffer.ADC1_Data >> 8) & 0xFFFFu);   /* AIN1=FX */
        dac8562_write_both(fz_code, fx_code);

        /* 已去掉延时: 主循环以"当前配置下可达的最高速率"刷新 DAC 双通道。
         * 当前 SYSCLK = HSI/2 = 16MHz (见 clock_init), PCLK2 同样 16MHz,
         * SPI0 SCLK = PCLK2/4 = 4MHz:
         *   一帧 24bit @4MHz ≈ 6us -> write_both(两帧) ≈ 12us (+GPIO/函数开销) -> 约 60kHz
         *
         * 注意: 这里只是把"同一组数据"重复写进 DAC, 新样本仍然由 ADC 速率决定
         * (adc_init 中 FS = ADC_SPS_50, 即 20ms 才出一对新数据), 所以整链的有效
         * 吞吐仍是 50 对/秒。若要做到"每个新样本只处理一次", 应改为等待 DMA 完成
         * 标志(在 DMA1_CH0_IRQHandler 中置位)后再读取 dADCBuffer。 */
    }
}
/*********************************End of File**********************************/
