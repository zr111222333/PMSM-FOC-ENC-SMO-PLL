#include "ADC.h"

// 三 ADC 注入同步 + 规则组 DMA

 ADC_HandleTypeDef hadc1;
 ADC_HandleTypeDef hadc2;
 ADC_HandleTypeDef hadc3;
 DMA_HandleTypeDef hdma_adc1;

MotorADC_RawTypeDef ADC_Raw;


uint16_t g_adc_dma_buf[2] = {0}; // 母线电压、温度

/* 电流零偏值，停机校准后存入 */
static float  Iu_offset =1552.0f; // 初始设 1552 只是一个合理的默认值，防止校准前数值完全离谱
static float  Iv_offset =1552.0f;
static float  Iw_offset =1552.0f;

void Motor_ADC_RCC_GPIO_DMA_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* UVW三相电流          PB0(U相电流)、PA3(W相电流)、PA6(V相电流) */
    // GPIO_InitStruct.Alternate=ADC;   ADC模拟外设用模拟模式，复用是数字外设使用的
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_3;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /*  温度PA0-ADC1检测 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /*  电源电压PB1-ADC1检测 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1规则组(温度 + 母线电压 )的DMA配置 */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    // hdma_adc1.Init.MemBurst=;  //9.3.11手册
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    // hdma_adc1.Init.PeriphBurst=;//9.3.11手册
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE; // 外设端读取地址不变
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1); // 温度、母线电压用DMA搬运，ADC1规则组的DMA句柄hdma_adc1与ADC1句柄hadc1关联

    /* 中断优先级 */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0); /* ADC注入中断优先级最高，电流环入口 */
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/**
 * @brief  ADC1初始化：规则组(母线+温度) + 注入组(V相电流，TIM1_CC4触发)
 */
void Motor_ADC1_Init(void)
{
    /* 单ADC基础参数 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 21MHz : APB2--84MHz,ADC最大频率36MHz，DIV2超频不可以---数据手册5.3.20
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // 转换结束标志（EOC）触发时机选择
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.NbrOfConversion = 2; // 规则通道:母线+温度
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    HAL_ADC_Init(&hadc1);

    /* 多ADC协同工作模式配置 ： ADC123*/
    ADC_MultiModeTypeDef multimode = {0};
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED; // 注入组无需cpu-dma：仅 ADC1 的规则组独立用 DMA 搬运母线电压、温度，ADC2/3 规则组未使用 DMA
    multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    // ADC1通道配置
    ADC_ChannelConfTypeDef sConfig = {0};
    // ADC1规则通道配置---温度
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Offset = 0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    // ADC1规则通道配置---母线电压
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // ADC1注入通道配置  V相电流，TIM1_CC4上升沿触发
    ADC_InjectionConfTypeDef sConfigInj = {0};
    sConfigInj.AutoInjectedConv = DISABLE;
    sConfigInj.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // TIM1_CC4上升沿触发
    sConfigInj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInj.InjectedChannel = ADC_CHANNEL_6;
    sConfigInj.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInj.InjectedNbrOfConversion = 1; // ADC1是1个
    sConfigInj.InjectedOffset = 0;
    sConfigInj.InjectedRank = 1;
    sConfigInj.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInj);
}

/**
 * @brief  ADC2初始化：注入组(U相电流，同步触发)
 */
void Motor_ADC2_Init(void)
{
    /* 单ADC基础参数 */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 21MHz : APB2--84MHz,ADC最大频率36MHz，DIV2超频不可以---数据手册5.3.20
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // 转换结束标志（EOC）触发时机选择
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.NbrOfConversion = 1; // 注入模式下，规则通道调试预留
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    HAL_ADC_Init(&hadc2);

    // ADC2注入通道配置  U相电流
    ADC_InjectionConfTypeDef sConfigInj = {0};
    sConfigInj.AutoInjectedConv = DISABLE;
    sConfigInj.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInj.InjectedChannel = ADC_CHANNEL_8;
    sConfigInj.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInj.InjectedNbrOfConversion = 1; // ADC2是1个
    sConfigInj.InjectedOffset = 0;
    sConfigInj.InjectedRank = 1;
    sConfigInj.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInj);
}

/**
 * @brief  ADC3初始化：注入组(W相电流，同步触发)
 */
void Motor_ADC3_Init(void)
{
    /* 单ADC基础参数 */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 21MHz : APB2--84MHz,ADC最大频率36MHz，DIV2超频不可以---数据手册5.3.20
    hadc3.Init.ContinuousConvMode = ENABLE;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // 转换结束标志（EOC）触发时机选择
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.NbrOfConversion = 1; // 注入模式下，规则通道调试预留
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ENABLE;
    HAL_ADC_Init(&hadc3);

    // ADC3注入通道配置  W相电流，
    ADC_InjectionConfTypeDef sConfigInj = {0};
    sConfigInj.AutoInjectedConv = DISABLE;
    sConfigInj.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInj.InjectedChannel = ADC_CHANNEL_3;
    sConfigInj.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInj.InjectedNbrOfConversion = 1; // ADC3是1个
    sConfigInj.InjectedOffset = 0;
    sConfigInj.InjectedRank = 1;
    sConfigInj.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInj);
}

/**
 * @brief  电机ADC初始化
 */
void Motor_ADC_Init(void)
{
    Motor_ADC_RCC_GPIO_DMA_Init();
    Motor_ADC1_Init();
    Motor_ADC2_Init();
    Motor_ADC3_Init();
}

/**
 * @brief  电机ADC启动
 */
void Motor_ADC_Start(void)
{
    /* 启动规则组DMA */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buf, 2);
    /* 启动注入组，先让从机"就位等待"，最后再启动主机触发,ADC1开中断，ADC2/3同步触发 */
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc3);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/*  注入组外部触发中断 */
void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
    HAL_ADC_IRQHandler(&hadc3);
}

/*  规则组DMA传输完成中断 */
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* 1. 获取电流原始值 */
        Motor_ADC_Get_RawValue(&ADC_Raw);
        
        /*  母线电压计算  */ 
        float Udc = ADC_Raw.Vbus_raw / ADC_FULL_SCALE * ADC_REF_VOLTAGE * VBUS_DIVIDER_GAIN; 
        
        /*  只有采样值合理时才更新，防止上电初期数据尚未有效  */
        if ((Udc > 5.0f) && (Udc < 70.0f))
        {
            foc_input.Udc = Udc;
        }
        /* 10kHz电流环 */
        Motor_FastControlISR();
        
    }
}

/**
 * @brief  电流零偏校准，电机停机、下桥全关时调用，采样50次取平均
 */
void Motor_ADC_Get_Current_Offset(void)
{
    uint32_t Iu_sum = 0, Iv_sum = 0, Iw_sum = 0;
    
    for (uint16_t i = 0; i < 50; i++)
    {
        Iu_sum = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) + Iu_sum;
        Iv_sum = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) + Iv_sum;
        Iw_sum = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) + Iw_sum;
        HAL_Delay(1);
    }
    Iu_offset = (float)(Iu_sum / 50.0f);
    Iv_offset = (float)(Iv_sum / 50.0f);
    Iw_offset = (float)(Iw_sum / 50.0f);
}

/**
 * @brief  获取三相电流+母线+温度原始值
 */
void Motor_ADC_Get_RawValue(MotorADC_RawTypeDef *raw)
{
    raw->Iu_raw = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - Iu_offset;
    raw->Iv_raw = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - Iv_offset;
    raw->Iw_raw = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - Iw_offset;
    raw->Temp_raw = g_adc_dma_buf[0];
    raw->Vbus_raw = g_adc_dma_buf[1];
}


float Motor_GetTemperature(void)
{
    const uint16_t temp_adc = g_adc_dma_buf[0];
    float Rt, temp;

    /* 判断ADC是否断线、短路或达到异常满量程。 */
    if ((temp_adc == 0U) || (temp_adc >= 4095U))
    {
        return 200.0f;//失效保护值，并不代表真实测得200℃
    }

    Rt = 4700.0f * (ADC_FULL_SCALE / (float)temp_adc - 1.0f);
    if (Rt <= 0.0f)
    {
        return 200.0f;
    }

    temp = logf(Rt / 10000.0f) / 3380.0f;
    temp += 1.0f / (273.15f + 25.0f);
    temp = 1.0f / temp - 273.15f;
    return temp;
}

