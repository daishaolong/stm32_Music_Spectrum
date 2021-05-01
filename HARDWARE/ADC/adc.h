#ifndef __ADC_H
#define __ADC_H
#include "sys.h"
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
#define ADC_SAMPLE_FREQ   (64024)//(42*1000UL)//ADCʱ��Ƶ��/(ADC����ʱ��+ADC�������)
#define ADC_SAMPLE_LEN  4096//1024//256//64//256//64//(sizeof(ADC_ConvertedValue) / sizeof(ADC_ConvertedValue[0]))
#define FREQ_SEG  (ADC_SAMPLE_FREQ/ADC_SAMPLE_LEN)

extern uint16_t ADC_ConvertedValue[ADC_SAMPLE_LEN];

void Adc_Init(void);                //ADCͨ����ʼ��
u16  Get_Adc(u8 ch);                //���ĳ��ͨ��ֵ
u16 Get_Adc_Average(u8 ch, u8 times); //�õ�ĳ��ͨ����������������ƽ��ֵ
void  Adc_Init2(void);
void MYADC_DMA_Enable(void);
uint8_t IsADCSampleEnd(void);
#endif















