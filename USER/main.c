#include "sys.h"
#include "delay.h"  
#include "usart.h"  
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "timer.h" 
#include "math.h" 
#include "arm_math.h"  
#include "adc.h"
#include "pwm.h"

//ALIENTEK ̽����STM32F407������ ʵ��47_2
//DSP FFT����ʵ��   -�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK


#define FFT_LENGTH		ADC_SAMPLE_LEN//(256) 		//FFT���ȣ�Ĭ����1024��FFT

float fft_inputbuf[FFT_LENGTH*2];	//FFT��������
float fft_outputbuf[FFT_LENGTH];	//FFT�������

u8 timeout;//��ʱ���������
#define BEEP_PIN  PFout(8)
void displayMusicSpectrum(float *arr,uint16_t size);
#if 0
int main(void)
{ 
  arm_cfft_radix4_instance_f32 scfft;
 	u8 key,t=0;
	float time; 
	u8 buf[50]; 
	u16 i; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	
	LED_Init();					//��ʼ��LED
	KEY_Init();					//��ʼ������
 	LCD_Init();					//��ʼ��LCD
	TIM3_Int_Init(65535,84-1);	//1Mhz����Ƶ��,����ʱ65ms���ҳ���
  	TIM14_PWM_Init(500-1,84-1);	//84M/84=1Mhz�ļ���Ƶ�ʣ���װ��ֵ500,����PWMƵ�� 1M/500=2Khz.    
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"����Ƶ��");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2021/05/01");	
	LCD_ShowString(30,130,200,16,16,"KEY0:Run FFT");//��ʾ��ʾ��Ϣ 
	LCD_ShowString(30,160,200,16,16,"FFT runtime:");//��ʾFFTִ��ʱ��
 	POINT_COLOR=BLUE;	//��������Ϊ��ɫ   
	Adc_Init2();
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//��ʼ��scfft�ṹ�壬�趨FFT��ز���
 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			for(i=0;i<FFT_LENGTH;i++)//�����ź�����
			{
//				 fft_inputbuf[2*i]=100+
//				                   10*arm_sin_f32(2*PI*i/FFT_LENGTH)+
//								   30*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
//				                   50*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//���������ź�ʵ��
				 fft_inputbuf[2*i]=(float)ADC_ConvertedValue[i]*3.3f/4096.0f;//ʵ��ΪADC����ֵ				
				 fft_inputbuf[2*i+1]=0;//�鲿ȫ��Ϊ0
			}
			TIM_SetCounter(TIM3,0);//����TIM3��ʱ���ļ�����ֵ
			timeout=0;
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT���㣨��4��
			time=TIM_GetCounter(TIM3)+(u32)timeout*65536; 			//��������ʱ��
			sprintf((char*)buf,"%0.3fms\r\n",time/1000);	
			LCD_ShowString(30+12*8,160,100,16,16,buf);	//��ʾ����ʱ��		
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//��������������ģ�÷�ֵ 
			printf("\r\n%d point FFT runtime:%0.3fms\r\n",FFT_LENGTH,time/1000);
			printf("FFT Result:\r\n");
			float max=-1,min=9999999.0;
			int16_t max_index=-1,min_index=-1;
			for(i=0;i<FFT_LENGTH;i++)
			{
				if(i!=0 && max<fft_outputbuf[i])
				{
					max=fft_outputbuf[i];
					max_index=i;
				}
				if(min>fft_outputbuf[i])
				{
					min=fft_outputbuf[i];
					min_index=i;
				}
				printf("fft_outputbuf[%d]:%f\r\n",i,fft_outputbuf[i]);
			}
			printf("max:%f,max_index:%d\r\n",max,max_index);
			printf("min:%f,min_index:%d\r\n",min,min_index);
			displayMusicSpectrum(fft_outputbuf+1,FFT_LENGTH/2);
		}
		else if(key == KEY1_PRES && IsADCSampleEnd())
		{
			for(i=0;i<ADC_SAMPLE_LEN;i++)
			{
				printf("ADC_ConvertedValue[%2d]:%d\r\n",i,ADC_ConvertedValue[i]);
			}			
			MYADC_DMA_Enable();
		}
		else if(key == KEY2_PRES)
		{
			BEEP_PIN=!BEEP_PIN;
		}
		else delay_ms(10);
		t++;
		if((t%10)==0)LED1=!LED1;		  
	}
}
#else

int main(void)
{ 
	arm_cfft_radix4_instance_f32 scfft;
	u32 t=0; 
	u16 i;
	u8 key;
	char buf[50];
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	
	LED_Init();					//��ʼ��LED
	KEY_Init();					//��ʼ������
 	LCD_Init();					//��ʼ��LCD
  	POINT_COLOR=RED; 
	LCD_ShowString(30,20,200,16,16,"Explorer STM32F4");	
	snprintf(buf,sizeof(buf)-1,"Music Spectrum! freq:%d,num:%d,freq_seg:%d",ADC_SAMPLE_FREQ,ADC_SAMPLE_LEN,FREQ_SEG);
	LCD_ShowString(30,40,450,16,16,(u8*)buf);	
	LCD_ShowString(30,60,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,80,200,16,16,"2021/05/04");	
	LCD_ShowString(30,100,450,16,16,"KEY0:change display style mode!");//�л���ʾģʽ
	LCD_ShowString(30,120,200,16,16,"KEY1:change band!");//�л�Ƶ��
	displayDropdownSpeed();
//	snprintf(buf,sizeof(buf)-1,"KEY2:change drop down speed! current speed:%d!",getDropdownSpeed());
// 	LCD_ShowString(30,140,450,16,16,(u8*)buf);//�л������ٶ�
//	POINT_COLOR=BLUE;	//��������Ϊ��ɫ   
	TIM14_PWM_Init(500-1,84-1);	//84M/84=1Mhz�ļ���Ƶ�ʣ���װ��ֵ500,����PWMƵ�� 1M/500=2Khz.   
	Adc_Init2();
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//��ʼ��scfft�ṹ�壬�趨FFT��ز���
 	while(1)
	{
		if(IsADCSampleEnd())
		{
			for(i=0;i<FFT_LENGTH;i++)//�����ź�����
			{
//				 fft_inputbuf[2*i]=100+
//				   0.1*arm_sin_f32(2*PI*i/FFT_LENGTH)+0.7*arm_sin_f32(2*PI*i*10/FFT_LENGTH)+
//				   0.3*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
//				   0.5*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//���������ź�ʵ��
				 fft_inputbuf[2*i]=(float)ADC_ConvertedValue[i]*3.3f/4096.0f;//ʵ��ΪADC����ֵ				
				 fft_inputbuf[2*i+1]=0;//�鲿ȫ��Ϊ0
//				printf("ADC_ConvertedValue[%2d]:%d\r\n",i,ADC_ConvertedValue[i]);
//				printf("%d\r\n",ADC_ConvertedValue[i]);
			}
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT���㣨��4��	
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//��������������ģ�÷�ֵ 
			displayMusicSpectrum(fft_outputbuf+1,FFT_LENGTH/2);			
			MYADC_DMA_Enable();
			t++;
			if((t%10)==0)LED1=!LED1;	
		}
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			changeDisplayStyleMode();
		}
		else if(key == KEY1_PRES)
		{
			changeBand();
		}
		else if(key == KEY2_PRES)
		{
			changeDropdownSpeed();
		}
//		delay_ms(80);
	}
}

#endif



