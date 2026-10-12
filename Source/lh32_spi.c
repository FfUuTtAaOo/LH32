#include "lh32_spi.h"

/* DAC8562 为只写器件(无 MISO), SPI 采用阻塞式发送, 不再使用接收中断。
   以下缓冲区/变量供 lh32m0xx_it.c 中的 SPI0_IRQHandler 使用, 当前 DAC 场景下该中断未使能。 */
uint8_t SPI_TX_Buffer[MAX_SIZE] = {0xAA,0x55,0x08,0x00,0x00,0x00,0x00,0x00};
uint8_t SPI_RX_Buffer[MAX_SIZE];
__RW uint8_t counter, flag, index;

void spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* 1. SPI pin init ,SPI0-> Master */
    GPIO_InitStructure.Pin = GPIO_PIN_0;                                        // P0.0 MISO(闲置, DAC8562 无数据输出)
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = ENABLE;
    GPIO_InitStructure.Alternate = GPIO0_0_AF_MISO1;
    LHL_GPIO_Init(pGPIO0, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_1;                                        // P0.1-> SPI0(Master) SCLK
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIO0_1_AF_SCLK1;
    LHL_GPIO_Init(pGPIO0, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_2;                                        // P0.2-> SPI0(Master) MOSI
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIO0_2_AF_MOSI1;
    LHL_GPIO_Init(pGPIO0, &GPIO_InitStructure);

    /* P0.3 -> DAC8562 SYNC: 作为普通 GPIO 由软件手动控制帧同步 */
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_8mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIOx_AF_GPIO;
    LHL_GPIO_Init(pGPIO0, &GPIO_InitStructure);
    LHL_GPIO_SetPins(pGPIO0, GPIO_PIN_3);                                       // SYNC 空闲为高

    /* 2. Initial configuration of SPI peripheral */
    SPI_InitStructure.Mode = SPI_MODE_MASTER;                                   // SPI0 -> Master
    SPI_InitStructure.Direction = SPI_DIR_2LINES_FULLDUPLEX;                    // Dual-line full-duplex
    SPI_InitStructure.DataSize = SPI_DATASIZE_8BIT;                             // 8-bit width
    SPI_InitStructure.CPOL = SPI_CPOL_LOW;                                      // DAC8562: SCLK 空闲 low
    SPI_InitStructure.CPHA = SPI_CPHA_2EDGE;                                    // DAC8562: 下降沿采样 => SPI Mode 1
    SPI_InitStructure.BaudRatePrescaler = SPI_BAUDRATE_PSC_4;                   // SCLK = PCLK2/4 (当前 SYSCLK=16MHz 下 = 4MHz)
    SPI_InitStructure.NSS = SPI_NSS_SOFT;                                       // NSS software control
    SPI_InitStructure.FirstBit = SPI_FIRSTBIT_MSB;                              // MSB First
    LHL_SPI_Init(pSPI0, &SPI_InitStructure);
}
