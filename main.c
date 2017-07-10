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
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�	
	PS2_Init();	 
	delay_ms(1000);	
	PS2_2_Init();
	KEY_Init();				//������ʼ��	
	USART2_Init(115200);
	EXTIX_Init();
	RS232_Init(115200);	    //��ʼ��RS232Ϊ115200
	motor_init();
	motorio_init();
	TIM3_PWM_Init(2000-1,36-1);	 //����Ƶ��PWMƵ��=72000000/900=80Khz 	
	USART1_Send_Data(RS232buf,2);//����5���ֽ� 	
	key=Search();
	
	while(1)
	{	
			PS2_CS();
			motor_excute();
	}
}



