#include "adc.h"
#include "delay.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#define ADC_DMA_ENABLE
uint16_t ADC_ConvertedValue[ADC_SAMPLE_LEN] = {0};

#ifndef ADC_DMA_ENABLE
//初始化ADC
void  Adc_Init2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

    //先初始化ADC1通道5 IO口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 通道5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);  //ADC1复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);   //复位结束


    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样阶段之间的延迟5个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
    ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1
    ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化


    ADC_Cmd(ADC1, ENABLE);//开启AD转换器

    //设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles);    //ADC1,ADC通道,480个周期,提高采样时间可以提高精确度
}
//获得ADC值
//ch: @ref ADC_channels
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)
{

    ADC_SoftwareStartConv(ADC1);        //使能指定的ADC1的软件转换启动功能

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//等待转换结束

    return ADC_GetConversionValue(ADC1);    //返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch, u8 times)
{
    u32 temp_val = 0;
    u8 t;
    for (t = 0; t < times; t++)
    {
        temp_val += Get_Adc(ch);
        delay_ms(5);
    }
    return temp_val / times;
}
uint8_t IsADCSampleEnd(void)
{
    uint16_t i;
    for (i = 0; i < ADC_SAMPLE_LEN; i++)
    {
        ADC_ConvertedValue[i] = Get_Adc(ADC_Channel_5);
    }
    return 1;
}
void MYADC_DMA_Enable(void)
{

}
#else

static uint8_t adc_sample_end = 0;
//初始化ADC
void  Adc_Init2(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE) {;}
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

//  //init GPIO ADC1, channel 14
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //先初始化ADC1通道5 IO口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 通道5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// DMA_Mode_Normal  DMA_Mode_Circular
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE); //使能数据流传输完成中断
    DMA_Cmd(DMA2_Stream0, ENABLE);

    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//84M/4 = 21M  不大于36MHZ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    //ADC_Channel_14  ADC_SampleTime_84Cycles
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_144Cycles); //转换时间84个ADC周期

    ADC_SoftwareStartConv(ADC1);
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);

    adc_sample_end = 0; //
}
uint8_t IsADCSampleEnd(void)
{
    return adc_sample_end;
}
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET)
    {
        printf("DMA end!!\r\n");
        adc_sample_end = 1;

        ADC_Cmd(ADC1, DISABLE);
        ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);
        ADC_DMACmd(ADC1, DISABLE);

        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    }
    return ;
}
//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//ndtr:数据传输量
void MYADC_DMA_Enable(void)
{
    adc_sample_end = 0; //clear
    DMA_Cmd(DMA2_Stream0, DISABLE);                      //关闭DMA传输

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE) {;}   //确保DMA可以被设置

//  DMA_SetCurrDataCounter(DMA2_Stream0,ADC_SAMPLE_LEN);          //数据传输量

    DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);      //使能数据流传输完成中断
    DMA_Cmd(DMA2_Stream0, ENABLE);                      //开启DMA传输

    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
}

#endif //#ifndef ADC_DMA_ENABLE
/*
#define FFT_LENGTH 4096
void FFTTestTask(void *arg)
{
    OS_ERR err;
    CPU_TS ts;
    arm_cfft_radix4_instance_f32 scfft;
    int i = 0;
    unsigned char str[10];

    arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//FFT初始化
    while(1)
    {
        OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,&ts,&err);//等待传输信号完成
        for(i=0;i<FFT_LENGTH;i++)
        {
            fft.fft_input[2*i] = (float)fft.ADC_ConvertedValue[i]*3.3f/4096.0f;//实部为ADC采样值
            fft.fft_input[2*i+1] = 0;//虚部为0
        }
        arm_cfft_radix4_f32(&scfft,fft.fft_input);//FFT运算
        arm_cmplx_mag_f32(fft.fft_input,fft.fft_output,FFT_LENGTH);//计算每个点的摸值
        for(i=0;i<FFT_LENGTH;i++)
        {
                sprintf((char*)str,"%.2f\r\n",fft.fft_output[i]);
            board.UART4Send(str,strlen((char*)str));//将数据打印至串口助手便于观察
            OSTimeDly(1,OS_OPT_TIME_DLY,&err);
        }
        OSTimeDly(500,OS_OPT_TIME_DLY,&err);
        board.ADC1_DMA2Enable();//重新启动ADC和DMA传输
    }
}
*/





