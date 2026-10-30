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
 * (ADC_VREF_MV / SENSOR_PGA / SENSOR_AINM_MODE), 这里不再各写一份
 * 正满量程(相对负端) = ADC_VREF_MV / PGA增益 */
#define ADC_PGA_GAIN        1.0f
#define ADC_FULLSCALE_MV    (ADC_VREF_MV / ADC_PGA_GAIN)

/* ---------- 传感器输入 -> DAC 输出 映射 (按需求) ----------
 *   传感器(经外部调理)双极性输出:  -2.5V(下限) -- 0V(中点) -- +2.5V(上限)
 *   线性映射到 DAC 输出:             0V           -- 1.25V     -- 2.5V
 *   (DAC8562 = 内部 2.5V 基准 + 增益 1 -> 输出满量程 0~2.5V) */
#define SENSOR_VIN_MIN_V    (-2500.0f)     /* 输入下限, 对应输出 0V     */
#define SENSOR_VIN_MAX_V    2500.0f        /* 输入上限, 对应输出满量程  */

/* DAC 输出满量程(单位: V), 直接由 DAC 驱动参数推导, 改增益时会自动跟随 */
#define DAC_VOUT_MAX_V      (DAC8562_INTERNAL_REF_V * (float)DAC8562_GAIN)

/* ---------- DAC 自检开关 ----------
 * 1 = 自检模式: 两个通道输出"固定电压", 输出保持不动,
 *     便于用万用表直接在 DAC 引脚上验证 "软件 -> SPI -> DAC" 这一段是否正常;
 * 0 = 正常工作: 传感器输入电压 -> 0~5V 线性映射后输出。
 * 排查完请改回 0。 */
#define DAC_SELFTEST        0

/* ---------- 串口调试打印开关 ---------- */
#define UART_DEBUG          0

/* 供 Debug 的 watch 窗口直接观察。volatile 防止被优化掉。 */
volatile int32_t g_fz_raw, g_fx_raw;
volatile float g_fz_mv, g_fx_mv;                /* 传感器输入电压(FZ / FX) */
volatile float g_fz_vout_mv, g_fx_vout_mv;      /* 映射后的目标输出电压    */
float    vout_fz, vout_fx;
int32_t  fz_raw, fx_raw;

/* Global variable -----------------------------------------------------------*/
bool bCommFlag;                 /* 串口空闲帧标志 */
uint32_t lastBufferIndex, currentBufferIndex, currentCommLength;

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

static void uart_process(void)
{
    if(bCommFlag)
    {
        lastBufferIndex = currentBufferIndex;
        currentBufferIndex = LHL_DMA_GetDestAddress(DMA_CHANNEL_1);
        /* 环形缓冲区数据长度计算 */
        if (currentBufferIndex > lastBufferIndex)
        {
            currentCommLength = currentBufferIndex - lastBufferIndex; 
        }
        else
        {
            currentCommLength = currentBufferIndex + MAX_COMM_SIZE - lastBufferIndex; 
        }            
        /* 依次打印上个通信帧数据 */        
        for (uint32_t i=0;i<currentCommLength;i++)
        {
            printf("%c",*((uint8_t *)lastBufferIndex++));
            if (lastBufferIndex >= (uint32_t)&commBuffer + MAX_COMM_SIZE)
            {
                lastBufferIndex = (uint32_t)&commBuffer;
            }
        }
        bCommFlag = false;           
    }    
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
    spi_init();                     /* SPI1 主机(P0.1/P0.2, AF1), 连接 DAC8562 */
    i2c_init();
    adc_init();                     /* ADC0/ADC1 同步采样 -> DMA 搬运到 dADCBuffer(内部已启动) */

    /* DAC8562 上电配置: 软件复位 + 双通道上电 + 使能内部基准 + 设置增益 */
    dac8562_init();
    
    lastBufferIndex = (uint32_t)&commBuffer;
    currentBufferIndex = (uint32_t)&commBuffer;
    bCommFlag = false;
    
    printf("hello world\r\n");

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

        uart_process();
    }
#endif
}
/*********************************End of File**********************************/
