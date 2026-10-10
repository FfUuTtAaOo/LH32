/******************************************************************************
 * 版权所有：苏州领慧立芯科技有限公司
 * Copyright (c) 2020-2025 Suzhou Legendsemi Technology Co., Ltd.
 ******************************************************************************
 * All rights reserved. Distributed under MIT license.
 * The file is encoded in UTF-8 without signature.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lh32m0xx_lhl.h"
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
    /* 上电后IWDG默认开启，若无处理约20秒后复位 */
    LHL_IWDG_Stop();
    
    /* 复位所有GPIO默认状态 */
    LHL_GPIO_DeInit();

    /* Init Config */
    gpio_init();                    /* P1.5=LDAC, P1.6=CLR */
    spi_init();                     /* SPI0 主机, 连接 DAC8562 */
    i2c_init();
    adc_init();

    /* DAC8562 上电配置: 软件复位 + 使能内部基准 + 设置增益 + 上电 */
    dac8562_init();

    /* 通道B固定输出 1.25V (内部基准2.5V, 增益1 -> 半量程) */
    dac8562_write_voltage(DAC8562_CH_B, 1.25f);

    while(1)
    {
        /* 通道A输出 0 ~ 2.5V 锯齿波, 便于用万用表/示波器验证 */
        for (uint32_t code = 0; code <= 0xFFFFu; code += 256u)
        {
            dac8562_write(DAC8562_CH_A, (uint16_t)code);
            LHL_Delay_uS(2000);
        }
    }
}
/*********************************End of File**********************************/
