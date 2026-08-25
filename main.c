#include "stm32f10x.h"
#include "sys.h"

void LR_flash(void)   
{
	u8 i;
	while(1)
	{
		for(i=1;i<7;i++)
			L_ws2812_rgb(i, WS_RED);
		L_ws2812_refresh(led_num);
		for(i=1;i<7;i++)
			R_ws2812_rgb(i, WS_DARK);
		R_ws2812_refresh(led_num);
		delay_ms(300);

		for(i=1;i<7;i++)
			L_ws2812_rgb(i, WS_DARK);
		L_ws2812_refresh(led_num);
		for(i=1;i<7;i++)
			R_ws2812_rgb(i, WS_RED);
		R_ws2812_refresh(led_num);
		delay_ms(300);
	}
}

int main(void)
  { 
		Stm32_Clock_Init(9);						//外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组		
		uart_init(115200);	            //=====串口初始化为115200
		JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

		colorful_led_Init();            //=====炫彩灯初始化

		printf("QST青软\r\n");
		/**主要程序**/
	while(1)
	{
		LR_flash();   //左右灯交替闪烁
	}
}
	

