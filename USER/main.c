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

//ALIENTEK 探索者STM32F407开发板 实验47_2
//DSP FFT测试实验   -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


#define FFT_LENGTH		ADC_SAMPLE_LEN//(256) 		//FFT长度，默认是1024点FFT

float fft_inputbuf[FFT_LENGTH*2];	//FFT输入数组
float fft_outputbuf[FFT_LENGTH];	//FFT输出数组

u8 timeout;//定时器溢出次数
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					//初始化LED
	KEY_Init();					//初始化按键
 	LCD_Init();					//初始化LCD
	TIM3_Int_Init(65535,84-1);	//1Mhz计数频率,最大计时65ms左右超出
  	TIM14_PWM_Init(500-1,84-1);	//84M/84=1Mhz的计数频率，重装载值500,所以PWM频率 1M/500=2Khz.    
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"音乐频谱");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2021/05/01");	
	LCD_ShowString(30,130,200,16,16,"KEY0:Run FFT");//显示提示信息 
	LCD_ShowString(30,160,200,16,16,"FFT runtime:");//显示FFT执行时间
 	POINT_COLOR=BLUE;	//设置字体为蓝色   
	Adc_Init2();
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数
 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			for(i=0;i<FFT_LENGTH;i++)//生成信号序列
			{
//				 fft_inputbuf[2*i]=100+
//				                   10*arm_sin_f32(2*PI*i/FFT_LENGTH)+
//								   30*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
//				                   50*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//生成输入信号实部
				 fft_inputbuf[2*i]=(float)ADC_ConvertedValue[i]*3.3f/4096.0f;//实部为ADC采样值				
				 fft_inputbuf[2*i+1]=0;//虚部全部为0
			}
			TIM_SetCounter(TIM3,0);//重设TIM3定时器的计数器值
			timeout=0;
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT计算（基4）
			time=TIM_GetCounter(TIM3)+(u32)timeout*65536; 			//计算所用时间
			sprintf((char*)buf,"%0.3fms\r\n",time/1000);	
			LCD_ShowString(30+12*8,160,100,16,16,buf);	//显示运行时间		
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值 
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					//初始化LED
	KEY_Init();					//初始化按键
 	LCD_Init();					//初始化LCD
  	POINT_COLOR=RED; 
	LCD_ShowString(30,20,200,16,16,"Explorer STM32F4");	
	snprintf(buf,sizeof(buf)-1,"Music Spectrum! freq:%d,num:%d,freq_seg:%d",ADC_SAMPLE_FREQ,ADC_SAMPLE_LEN,FREQ_SEG);
	LCD_ShowString(30,40,450,16,16,(u8*)buf);	
	LCD_ShowString(30,60,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,80,200,16,16,"2021/05/04");	
	LCD_ShowString(30,100,450,16,16,"KEY0:change display style mode!");//切换显示模式
	LCD_ShowString(30,120,200,16,16,"KEY1:change band!");//切换频段
	displayDropdownSpeed();
//	snprintf(buf,sizeof(buf)-1,"KEY2:change drop down speed! current speed:%d!",getDropdownSpeed());
// 	LCD_ShowString(30,140,450,16,16,(u8*)buf);//切换下落速度
//	POINT_COLOR=BLUE;	//设置字体为蓝色   
	TIM14_PWM_Init(500-1,84-1);	//84M/84=1Mhz的计数频率，重装载值500,所以PWM频率 1M/500=2Khz.   
	Adc_Init2();
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数
 	while(1)
	{
		if(IsADCSampleEnd())
		{
			for(i=0;i<FFT_LENGTH;i++)//生成信号序列
			{
//				 fft_inputbuf[2*i]=100+
//				   0.1*arm_sin_f32(2*PI*i/FFT_LENGTH)+0.7*arm_sin_f32(2*PI*i*10/FFT_LENGTH)+
//				   0.3*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
//				   0.5*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//生成输入信号实部
				 fft_inputbuf[2*i]=(float)ADC_ConvertedValue[i]*3.3f/4096.0f;//实部为ADC采样值				
				 fft_inputbuf[2*i+1]=0;//虚部全部为0
//				printf("ADC_ConvertedValue[%2d]:%d\r\n",i,ADC_ConvertedValue[i]);
//				printf("%d\r\n",ADC_ConvertedValue[i]);
			}
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT计算（基4）	
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值 
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



