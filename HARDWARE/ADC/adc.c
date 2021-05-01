#include "adc.h"
#include "delay.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#define ADC_DMA_ENABLE
uint16_t ADC_ConvertedValue[ADC_SAMPLE_LEN] = {0};

#ifndef ADC_DMA_ENABLE
//��ʼ��ADC
void  Adc_Init2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

    //�ȳ�ʼ��ADC1ͨ��5 IO��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 ͨ��5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);  //ADC1��λ
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);   //��λ����


    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//���������׶�֮����ӳ�5��ʱ��
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1
    ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��


    ADC_Cmd(ADC1, ENABLE);//����ADת����

    //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles);    //ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��
}
//���ADCֵ
//ch: @ref ADC_channels
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)
{

    ADC_SoftwareStartConv(ADC1);        //ʹ��ָ����ADC1�����ת����������

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//�ȴ�ת������

    return ADC_GetConversionValue(ADC1);    //�������һ��ADC1�������ת�����
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ��
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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
//��ʼ��ADC
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

    //�ȳ�ʼ��ADC1ͨ��5 IO��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 ͨ��5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

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
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE); //ʹ����������������ж�
    DMA_Cmd(DMA2_Stream0, ENABLE);

    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//84M/4 = 21M  ������36MHZ
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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_144Cycles); //ת��ʱ��84��ADC����

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
//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//ndtr:���ݴ�����
void MYADC_DMA_Enable(void)
{
    adc_sample_end = 0; //clear
    DMA_Cmd(DMA2_Stream0, DISABLE);                      //�ر�DMA����

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE) {;}   //ȷ��DMA���Ա�����

//  DMA_SetCurrDataCounter(DMA2_Stream0,ADC_SAMPLE_LEN);          //���ݴ�����

    DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);      //ʹ����������������ж�
    DMA_Cmd(DMA2_Stream0, ENABLE);                      //����DMA����

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

    arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//FFT��ʼ��
    while(1)
    {
        OSTaskSemPend(0,OS_OPT_PEND_BLOCKING,&ts,&err);//�ȴ������ź����
        for(i=0;i<FFT_LENGTH;i++)
        {
            fft.fft_input[2*i] = (float)fft.ADC_ConvertedValue[i]*3.3f/4096.0f;//ʵ��ΪADC����ֵ
            fft.fft_input[2*i+1] = 0;//�鲿Ϊ0
        }
        arm_cfft_radix4_f32(&scfft,fft.fft_input);//FFT����
        arm_cmplx_mag_f32(fft.fft_input,fft.fft_output,FFT_LENGTH);//����ÿ�������ֵ
        for(i=0;i<FFT_LENGTH;i++)
        {
                sprintf((char*)str,"%.2f\r\n",fft.fft_output[i]);
            board.UART4Send(str,strlen((char*)str));//�����ݴ�ӡ���������ֱ��ڹ۲�
            OSTimeDly(1,OS_OPT_TIME_DLY,&err);
        }
        OSTimeDly(500,OS_OPT_TIME_DLY,&err);
        board.ADC1_DMA2Enable();//��������ADC��DMA����
    }
}
*/





