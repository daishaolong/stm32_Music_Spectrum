实验器材:
	探索者STM32F4开发板
	
实验目的:
	学习STM32F4 DSP功能(FFT计算)的使用
	
硬件资源:
	1,DS0(连接在PF9)
	2,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面)
	3,ALIENTEK 2.8/3.5/4.3/7寸TFTLCD模块(通过FSMC驱动,FSMC_NE4接LCD片选/A6接RS)  
	4,按键KEY0(PE4)
	
实验现象:
	本实验测试STM32F4的DSP 库的FFT函数，程序运行后，自动生成1024点测试序列，然后，每当KEY0
	按下后，调用DSP库的FFT算法（基4法）执行FFT运算，在LCD屏幕上面显示运算时间，同时将FFT结果
	输出到串口，DS0用于提示程序正在运行。

注意事项: 
	1,4.3寸和7寸屏需要比较大电流,USB供电可能不足,请用外部电源适配器(推荐外接12V 1A电源).
	2,本例程在LCD_Init函数里面(在ILI93xx.c),用到了printf,如果不初始化串口1,将导致液晶无法显示!! 
	   

					正点原子@ALIENTEK
					2014-10-25
					广州市星翼电子科技有限公司
					电话：020-38271790
					传真：020-36773971
					购买：http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					公司网站：www.alientek.com
					技术论坛：www.openedv.com


					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					