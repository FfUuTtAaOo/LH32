#include "lh32_adc.h"

/* Private Define ------------------------------------------------------------*/
#define TCD_COUNT 2

/* ---- AINM(负端)通道选择 ----
 * ⚠️ ADC0 与 ADC1 的通道枚举值**不一样**, 不能混用! 见 lh32m0g3x_adc.h:
 *      ADC0: AIN0..AIN11=0x00..0x0B, AVSS=0x10, REFP=0x11
 *      ADC1: AIN0..AIN11=0x00..0x0B, TEMPN=0x10, AVSS=0x11, REFP=0x12
 *    (原代码给 ADC1 写的是 ADC0_AINM_AVSS=0x10, 在 ADC1 里那是 TEMPN = 内部温度
 *     传感器负端, 并不是 AVSS —— 属于隐蔽的通道错配, 已一并修正。)
 * 接法含义与量程推导见 Include/lh32_adc.h 的 SENSOR_AINM_MODE 说明。 */
#if (SENSOR_AINM_MODE == 1)
  #define FZ_AINM_CH   ADC0_AINM_REFP       /* ADC0 = FZ: 负端接内部基准节点 */
  #define FX_AINM_CH   ADC1_AINM_REFP       /* ADC1 = FX: 负端接内部基准节点 */
#else
  #define FZ_AINM_CH   ADC0_AINM_AVSS       /* ADC0 = FZ: 单端, 负端接 AVSS */
  #define FX_AINM_CH   ADC1_AINM_AVSS       /* ADC1 = FX: 单端, 负端接 AVSS */
#endif

/* ⚠️ PGA 增益必须与"真正到达 AIN 引脚上的电压"匹配, 过大 -> 直接饱和!
 * 本工程外部已做信号调理电路, 送到引脚的是"伏级"电压 -> 用 ×1(见 lh32_adc.h)。
 * 正满量程 = 基准(ADC_VREF_MV, 实测约 2500mV) / 增益:
 *     ×1 -> 2500mV   ×2 -> 1250mV   ×4 -> 625mV
 *     ×32 -> 78.1mV  ×64 -> 39.1mV  ×128 -> 19.5mV
 * 饱和特征: 超量程后码值被削平在 +满码 0x7FFFFF / 负满码 0x800000, 表现为
 *           "输入电压变了, 读数却卡住不动"(例: 1.5V 输入 + ×64 -> 恒为 0x7FFF)。 */
/* SENSOR_PGA / ADC_VREF_MV / SENSOR_AINM_MODE 统一定义在 Include/lh32_adc.h */

/* Global Variable -----------------------------------------------------------*/
DMA_HandleTypeDef hDMA0, hDMA1;

/* Dual ADC data buffer
 * 注意: 必须是全局变量, 不能是 adc_init() 的局部变量。
 * DMA 会持续向该地址写数据, 若放在栈上, 函数返回后栈空间被复用,
 * 采样结果会丢失/破坏调用栈。 */
ADC_DoubleData_t dADCBuffer;

void adc_init(void)
{
    REF_InitTypeDef REF_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMAMUX_InitTypeDef DMAMUX_InitStructure;
    static DMA_InitTypeDef DMA_InitStructure[TCD_COUNT];       /* Multiple initialization structure parameters of the decentralized aggregation mode */
    static DMA_DES_N_TypeDef TCD_Quene[TCD_COUNT] __ALIGN(32); /* The DMA TCD descriptor must be 32-bit aligned.*/

    /* 关键提醒: 应变电桥必须有"桥压激励"(+EX/-EX, 通常 5V)才有输出!
     *   - 若板上用外部 5V 供桥, 这里不用管;
     *   - 若希望用芯片 VDRIVE 供桥, 取消下面这行注释(桥压 2.5V, 信号幅度减半):
     *         LHL_ADC_SetVDRIVE(REF_INTERNAL_2P5V, ENABLE);
     *   没有激励时, 电桥不工作 -> 引脚停在共模中点 -> 读数恒为 2.5V 不动。 */

    /* Initialize internal benchmark
     * 这里只能写"档位"。⚠️ 3.3V 供电时 4.096V 档给不出来(基准缓冲输出不能高于 AVDD),
     * 实测有效基准以 ADC_VREF_MV 为准(见 lh32_adc.h, 万用表量 REFP 引脚即可标定)。 */
    REF_InitStructure.VREF = REF_INTERNAL_4P096V;
#if (SENSOR_AINM_MODE == 1)
    /* 差分模式把 REFP 同时当成了 AINM 输入, 参考节点会被 ADC 采样网络加载 ->
     * 打开参考缓冲的 BOOST 提高驱动能力, 减小基准被拉低的风险。
     * 若发现基准反而变差(噪声/振荡), 改回 DISABLE 再试。 */
    REF_InitStructure.VREF_Boost = ENABLE;
#else
    REF_InitStructure.VREF_Boost = DISABLE;
#endif
    REF_InitStructure.VDRIVE = REF_INTERNAL_OFF;                    // Disable VDrive output
    LHL_REF_Init(&REF_InitStructure);

    /* Initialize ADC0 and ADC1, set synchronization, and enable DMA */
    ADC_InitStructure.AINP = ADC0_AINP_AIN0;                            // ADC0 = AIN0 = FZ
    ADC_InitStructure.AINM = FZ_AINM_CH;                                //   负端: 见 SENSOR_AINM_MODE
    ADC_InitStructure.FS = ADC_SPS_50;                              // Output rate:50SPS
    ADC_InitStructure.PGA = SENSOR_PGA;                             // 与外部调理电路输出范围匹配(见 lh32_adc.h)
    ADC_InitStructure.Code = ADC_CODE_BIPOLAR;                      // 力是双向的, 必须用双极性码(单极性会削掉负向力)
    ADC_InitStructure.Mode = ADC_MODE_CONTINUOUS_CONVERSION;        // Continuous conversion mode
    ADC_InitStructure.Trigger = ADC_TRIGGER_SOFTWARE;               // software trigger
    ADC_InitStructure.ReferenceSelect = ADC_REF_REFP_to_REFN;       // Internal benchmark
    ADC_InitStructure.REF_BUFP = ENABLE;                            // Enable reference buffer
    ADC_InitStructure.REF_BUFM = ENABLE;
    ADC_InitStructure.REF_Precharge = DISABLE;
    /* ⚠️ 这个字段是"死"的: LHL_ADC_Init() 从头到尾不读它, 而唯一使用它的
     * LHL_ADC_GetVoltage()/GetTemperature() 在本工程中无人调用(Keil map 显示已被
     * 链接器 Removing)。也就是说改这里的数不会碰任何硬件寄存器、也不参与本工程的
     * 码->电压换算(那是 main.c 的事)。写成 ADC_VREF_MV 只为消除"两处基准不一致"
     * 的误导, 别指望它能改变量程。 */
    ADC_InitStructure.Reference = ADC_VREF_MV;
    LHL_ADC_Init(ADC_0, &ADC_InitStructure);

    ADC_InitStructure.AINP = ADC1_AINP_AIN1;                            // ADC1 = AIN1 = FX
    ADC_InitStructure.AINM = FX_AINM_CH;                                //   负端: 见 SENSOR_AINM_MODE
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

    /* ---- 启动 DMA 与 ADC (原代码缺失这几步, 导致整条链路不工作) ---- */

    /* 按上面的参数生成 TCD 描述符, 并把首个 TCD 装载到 DMA 通道0 硬件寄存器。
     * 必须在配置完成后调用, 否则 TCD 只停留在软件变量里, DMA 不会工作。 */
    LHL_DMA_Init(&hDMA0);

    /* 使能 DMA 通道0 中断: TCD1 已置 INT_Major, 但 CPU 侧 NVIC 也必须打开 */
    NVIC_ClearPendingIRQ(DMA1_CH0_IRQn);
    NVIC_EnableIRQ(DMA1_CH0_IRQn);

    /* 使能 DMA 通道0 的请求 (ADC0 触发请求源) */
    LHL_DMA_Start(&hDMA0);

    /* 启动 ADC 转换: 软件触发 + 连续转换模式, 启动后即持续采样。
     * 不调用则 ADC 永远不转换, DMA 也就永远收不到请求。 */
    adc_start();
}

/**
  * @brief  启动 ADC0/ADC1 转换
  * @param  None
  * @retval None
  */
void adc_start(void)
{
    LHL_ADC_Start(ADC_0);
    LHL_ADC_Start(ADC_1);
}
