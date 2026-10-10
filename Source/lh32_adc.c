#include "lh32_adc.h"

/* Private Define ------------------------------------------------------------*/
#define TCD_COUNT 2

typedef struct {
    uint32_t    ADC0_Data; 
    uint32_t    ADC1_Data;  
} ADC_DoubleData_t;                                     /* Dual ADC data */

/* Global Variable -----------------------------------------------------------*/
DMA_HandleTypeDef hDMA0, hDMA1;

void adc_init(void)
{
    REF_InitTypeDef REF_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMAMUX_InitTypeDef DMAMUX_InitStructure;
    DMA_InitTypeDef DMA_InitStructure[TCD_COUNT];       /* Multiple initialization structure parameters of the decentralized aggregation mode */
    static DMA_DES_N_TypeDef TCD_Quene[TCD_COUNT] __ALIGN(32); /* The DMA TCD descriptor must be 32-bit aligned.*/
    ADC_DoubleData_t dADCBuffer;

    /* Initialize internal benchmark */
    REF_InitStructure.VREF = REF_INTERNAL_2P5V;                     // Internal reference voltage 2.5V
    REF_InitStructure.VREF_Boost = DISABLE;
    REF_InitStructure.VDRIVE = REF_INTERNAL_OFF;                    // Disable VDrive output
    LHL_REF_Init(&REF_InitStructure);

    /* Initialize ADC0 and ADC1, set synchronization, and enable DMA */
    ADC_InitStructure.AINP = ADC0_AINP_AIN0;                        // ADC0 input channel
    ADC_InitStructure.AINM = ADC0_AINM_AVSS;
    ADC_InitStructure.FS = ADC_SPS_50;                              // Output rate:50SPS
    ADC_InitStructure.PGA = ADC_PGA_GAIN_1;                         // Internal PGA gain x1
    ADC_InitStructure.Code = ADC_CODE_BIPOLAR;                      // Bipolar data
    ADC_InitStructure.Mode = ADC_MODE_CONTINUOUS_CONVERSION;        // Continuous conversion mode
    ADC_InitStructure.Trigger = ADC_TRIGGER_SOFTWARE;               // software trigger
    ADC_InitStructure.ReferenceSelect = ADC_REF_REFP_to_REFN;       // Internal benchmark
    ADC_InitStructure.REF_BUFP = ENABLE;                            // Enable reference buffer
    ADC_InitStructure.REF_BUFM = ENABLE;
    ADC_InitStructure.REF_Precharge = DISABLE;
    ADC_InitStructure.Reference = 2500.0f;
    LHL_ADC_Init(ADC_0, &ADC_InitStructure);

    ADC_InitStructure.AINP = ADC1_AINP_AIN1;                        // ADC1 input channel
    ADC_InitStructure.AINM = ADC0_AINM_AVSS;
    LHL_ADC_Init(ADC_1, &ADC_InitStructure);                        // Apart from the channel, the configurations of ADC1 and ADC0 are exactly the same.

    LHL_ADC_SetSync(ENABLE);

    LHL_ADC_DMACmd(ADC_0, ENABLE);                                  // Enable the DMA function of ADC0

    /* Configure DMAUX: The request source for DMA channel 0 is from ADC0. */
    DMAMUX_InitStructure.DMA_Channel = DMA_CHANNEL_0;
    DMAMUX_InitStructure.Request_Source = REQUEST_SOURCE_ADC0;
    DMAMUX_InitStructure.Periodic_Trigger = DISABLE;
    DMAMUX_InitStructure.Cmd = ENABLE;
    LHL_DMAMUX_Init(&DMAMUX_InitStructure); 

    /* Configure DMA channel 0: Transfer data from ADC0 and ADC1 */
    hDMA0.Channel = DMA_CHANNEL_0;
    hDMA0.Mode = DMA_SCATTER_GATHER_MODE;
    hDMA0.Request = DMA_HARDWARE_REQUEST;
    hDMA0.TCD_Count = TCD_COUNT;                                        // Two TCDs are respectively used for ADC0 and ADC1.
    hDMA0.TCD_List = TCD_Quene;                                         // TCD linked list
    hDMA0.TCD_Init = DMA_InitStructure;                                 // TCD initialization queue
    
    DMA_InitStructure[0].Direction = DMA_PERIPH_TO_MEMORY;              // TCD0 is used for ADC0
    DMA_InitStructure[0].Src_Address = (uint32_t)(&pADC->ADC_DATA_0);
    DMA_InitStructure[0].Dest_Address = (uint32_t)&dADCBuffer.ADC0_Data;// Point to the cache area ADC0
    DMA_InitStructure[0].Data_Width = DMA_DATA_WIDTH_4B;        
    DMA_InitStructure[0].Data_Size = 1;                        
    DMA_InitStructure[0].Repetition = 1;
    DMA_InitStructure[0].Trans_Mode = DMA_CYCLIC_TRANSMISSION;          
    DMA_InitStructure[0].TCD_Address = (int32_t)&TCD_Quene[1];          // After completion, load TCD1
    DMA_InitStructure[0].Auto_Start = DISABLE;
    DMA_InitStructure[0].INT_Major = DISABLE;
    DMA_InitStructure[0].INT_Half = DISABLE;
    DMA_InitStructure[0].Chaining = TO_DMA_CHANNEL_NONE;                

    DMA_InitStructure[1].Direction = DMA_PERIPH_TO_MEMORY;              // TCD1 is used for ADC1
    DMA_InitStructure[1].Src_Address = (uint32_t)(&pADC->ADC_DATA_1);
    DMA_InitStructure[1].Dest_Address = (uint32_t)&dADCBuffer.ADC1_Data;// Point to the cache area ADC1
    DMA_InitStructure[1].Data_Width = DMA_DATA_WIDTH_4B;        
    DMA_InitStructure[1].Data_Size = 1;                        
    DMA_InitStructure[1].Repetition = 1;
    DMA_InitStructure[1].Trans_Mode = DMA_CYCLIC_TRANSMISSION;         
    DMA_InitStructure[1].TCD_Address = (int32_t)&TCD_Quene[0];          // After completion, load TCD0
    DMA_InitStructure[1].Auto_Start = ENABLE;
    DMA_InitStructure[1].INT_Major = ENABLE;                            // Trigger the DMA0 main interrupt upon completion
    DMA_InitStructure[1].INT_Half = DISABLE;
    DMA_InitStructure[1].Chaining = TO_DMA_CHANNEL_NONE;
}
