#include "lh32_gpio.h"

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Configure pin parameters */    
    GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4;       // Different pins of the same port can be configured simultaneously. Pin1 & Pin4
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;          // ??????
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;          // Drive capability: 4mA (Can be adjusted according to the load.2~16mA)
    GPIO_InitStructure.Pull = GPIO_NOPULL;                  // Top/bottom pull-up/pull-down resistor
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIOx_AF_GPIO;           // AF represents the ordinary IO function
    LHL_GPIO_Init(pGPIO1, &GPIO_InitStructure);             // Initialize P1.5 and P1.6 to push-pull output mode
}
