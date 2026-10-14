/******************************************************************************
 * 版权所有：苏州领慧立芯科技有限公司
 * Copyright (c) 2020-2025 Suzhou Legendsemi Technology Co., Ltd.
 ******************************************************************************
 * All rights reserved. Distributed under MIT license.
 * The file is encoded in UTF-8 without signature.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lh32m0xx_lhl.h"

/* Private define ------------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1);
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1);
}


/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}


/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	LHL_IncTick();
}

/******************************************************************************/
/*                 LH32M0xx  Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup.s).                                                         */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  * @note   本工程 DAC8562 使用 SPI1(pSPI1) 的阻塞式发送, 未使能 SPI 中断,
  *         此函数仅作保留备用(注意: 使用的是 pSPI1, 不是 pSPI0)。
  */
void SPI1_IRQHandler(void)
{
    /* SPI全双工通信，进入中断说明收到数据，且上一个填充的数据已同步发送 */
    SPI_RX_Buffer[index++] = (uint8_t)LHL_SPI_ReadData(pSPI1);                  // 收到从机发送的数据
    
    if (index >= MAX_SIZE)                                                      // 全部数据发送完毕，则指针复位，标志置位
    {
        index = 0;
        flag = 1;
    }
    else
    {
        LHL_SPI_WriteData(pSPI1, SPI_TX_Buffer[index]);                         // 还有数据，则填充下一个待发出的数据
    }
}

/**
  * @brief  This function handles DMA CHANNEL0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_CH0_IRQHandler(void)
{
    LHL_DMA_ClearPending(DMA_CHANNEL_0);                // 清中断标志

    /* 通道0 的句柄是 hDMA0(在 adc_init 中配置), 不能用 hDMA1。
     * 这里重新 Start 是容错处理: 正常循环模式下 TCD1 的 Auto_Start 已使能,
     * 硬件完成一个 TCD 后会自动装载下一个 TCD 并继续传输。 */
    LHL_DMA_Start(&hDMA0);
}
 
/**
 * @brief  This function handles DMA CHANNEL1 interrupt request.
 * @param  None
 * @retval None
 */
void DMA1_CH1_IRQHandler(void)
{
    /* 本工程 DMA 只使用通道0(分散聚合模式), 通道1 未配置/未使能, 此函数保留备用 */
    LHL_DMA_ClearPending(DMA_CHANNEL_1); 
}

/*********************************End of File**********************************/
