#include "led.h"
#include "rs232.h"	
#include "rplidar.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "ps.h"	
#include "lcd.h"
#include "motor.h"
#include "PWM.h"
#include "global.h"
#include "exti.h"
#include "math.h"
#include "Astar.h"

extern char buffer[100];
u8 main_stop(void);
void main_right(void);
void main_star(void);

int tt=0,mode=0;
 int main(void)
 {	 
	int i,j,t; 
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	LED_Init();		  		//初始化与LED连接的硬件接口	
	PS2_Init();	 
	delay_ms(1000);	
	PS2_2_Init();
	KEY_Init();				//按键初始化	
	USART2_Init(115200);
	EXTIX_Init();
	RS232_Init(115200);	    //初始化RS232为115200
	motor_init();
	motorio_init();
	TIM3_PWM_Init(2000-1,36-1);	 //不分频。PWM频率=72000000/900=80Khz 	
	USART1_Send_Data(RS232buf,2);//发送5个字节 	
	key=Search();
	
	while(1)
	{	
			PS2_CS();
			motor_excute();
	}
}



