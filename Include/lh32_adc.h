#ifndef _LH32_ADC_H
#define _LH32_ADC_H

#include "lh32m0xx_lhl.h"

/* 双ADC同步采样结果, 由 DMA 通道0 以分散聚合模式搬运至此 */
typedef struct {
    uint32_t    ADC0_Data;
    uint32_t    ADC1_Data;
} ADC_DoubleData_t;

/* DMA 搬运目标缓冲区
 * 必须定义为全局(或 static 全局), 不能放在 adc_init() 的栈上:
 * adc_init() 返回后栈空间会被其他函数复用, DMA 持续写入会破坏调用栈。 */
extern ADC_DoubleData_t dADCBuffer;

void adc_init(void);        /* 配置 ADC0/ADC1 + DMAMUX + DMA TCD, 并使能并启动采样 */
void adc_start(void);       /* 启动 ADC0/ADC1 转换(软件触发, 连续转换模式) */

#endif
