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

/* Functions -----------------------------------------------------------------*/
int main(void)
{
    uint16_t code_a, code_b;
    
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
        /* 两路 ADC 同步采样的数据都已在 dADCBuffer 中(DMA 持续更新), 分别送 DAC 双通道输出:
         *   AIN0 -> ADC0 -> dADCBuffer.ADC0_Data -> DAC-A
         *   AIN1 -> ADC1 -> dADCBuffer.ADC1_Data -> DAC-B
         * 用 write_both 由 LDAC 下降沿同步更新, 保证两路输出同时生效。 */
        code_a = (uint16_t)((dADCBuffer.ADC0_Data >> 8) & 0xFFFFu);   /* 24位右对齐双极性码取高16位 */
        code_b = (uint16_t)((dADCBuffer.ADC1_Data >> 8) & 0xFFFFu);
        dac8562_write_both(code_a, code_b);

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
