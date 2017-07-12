#ifndef __motor_H
#define __motor_H	

#include "sys.h"
#include "global.h"
#include "ps.h"
#include "led.h"
#include "PWM.h"
#include "global.h"
#include "rplidar.h"	
#include "math.h"
#include "Astar.h"
#include "rs232.h"
//#define M1_BRK_EN GPIO_SetBits(GPIOE,GPIO_Pin_0) //�ߵ�ƽɲ��
//#define M1_BRK_DIS GPIO_ResetBits(GPIOE,GPIO_Pin_0) //�͵�ƽ��ɲ��
#define M1_FR_POS GPIO_SetBits(GPIOC,GPIO_Pin_8) //�ߵ�ƽ��ת
#define M1_FR_NEG GPIO_ResetBits(GPIOC,GPIO_Pin_8) //�͵�ƽ��ת
//#define M1_EN_EN GPIO_ResetBits(GPIOE,GPIO_Pin_2) //�͵�ƽ�������
//#define M1_EN_DIS GPIO_SetBits(GPIOE,GPIO_Pin_2) //�ߵ�ƽͣ��

//#define M2_BRK_EN GPIO_SetBits(GPIOE,GPIO_Pin_1) //�ߵ�ƽɲ��
//#define M2_BRK_DIS GPIO_ResetBits(GPIOE,GPIO_Pin_1) //�͵�ƽ��ɲ��
#define M2_FR_POS GPIO_SetBits(GPIOF,GPIO_Pin_9) //�ߵ�ƽ��ת
#define M2_FR_NEG GPIO_ResetBits(GPIOF,GPIO_Pin_9) //�͵�ƽ��ת
//#define M2_EN_EN GPIO_ResetBits(GPIOE,GPIO_Pin_3) //�͵�ƽ�������
//#define M2_EN_DIS GPIO_SetBits(GPIOE,GPIO_Pin_3) //�ߵ�ƽͣ��

//#define M3_BRK_EN GPIO_SetBits(GPIOF,GPIO_Pin_0) //�ߵ�ƽɲ��
//#define M3_BRK_DIS GPIO_ResetBits(GPIOF,GPIO_Pin_0) //�͵�ƽ��ɲ��
#define M3_FR_POS GPIO_SetBits(GPIOF,GPIO_Pin_10) //�ߵ�ƽ��ת
#define M3_FR_NEG GPIO_ResetBits(GPIOF,GPIO_Pin_10) //�͵�ƽ��ת
//#define M3_EN_EN GPIO_ResetBits(GPIOF,GPIO_Pin_2) //�͵�ƽ�������
//#define M3_EN_DIS GPIO_SetBits(GPIOF,GPIO_Pin_2) //�ߵ�ƽͣ��

//#define M4_BRK_EN GPIO_SetBits(GPIOF,GPIO_Pin_6) //�ߵ�ƽɲ��
//#define M4_BRK_DIS GPIO_ResetBits(GPIOF,GPIO_Pin_6) //�͵�ƽ��ɲ��
#define M4_FR_POS GPIO_SetBits(GPIOF,GPIO_Pin_11) //�ߵ�ƽ��ת
#define M4_FR_NEG GPIO_ResetBits(GPIOF,GPIO_Pin_11) //�͵�ƽ��ת
//#define M4_EN_EN GPIO_ResetBits(GPIOF,GPIO_Pin_8) //�͵�ƽ�������
//#define M4_EN_DIS GPIO_SetBits(GPIOF,GPIO_Pin_8) //�ߵ�ƽͣ��


#define mode1_up  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)
#define mode1_down  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
#define mode2_up  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)
#define mode2_down  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define mode3_up  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
#define mode3_down  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define mode4_up  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)
#define mode4_down  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)

#define KAI1  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_12)
#define KAI2  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_13)
#define KAI3  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_14)
#define KAI4  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15)

typedef struct
{
	  __IO float err;                //����ƫ��ֵ
	  __IO float err_last;            //��������һ��ƫ��ֵ
	  __IO float err_next;            //������һ��ƫ��ֵ
	  __IO float err_sum;            //���������
	  __IO float Kp;            //������������֡�΢��ϵ��
	  __IO float Ki;
	  __IO float Kd;
	  __IO float x;             //������������ϵ��
	  __IO float y;
	  __IO float z;
	  __IO float output;              //���������
	  
} PID;

typedef struct
{
      __IO PID   MT_PID;             //pid
	  __IO float SetSpeed;           //�����趨ֵ
      __IO float ActualSpeed;        //����ʵ��ֵ
	  __IO float integral;            //���������
	  __IO float output_duty;         //���������
	  
} MT_Alg;

extern u8 mode_up;
extern u8 mode_down;
extern u8 mode_left;	
extern u8 mode_right;
extern u8 mode_shun;
extern u8 mode_ni;	
extern u8 mode_stop;	
extern u8 mode_theta;	
extern u8 mode_distance;	
extern char code[8];
extern float theta;
extern float distance_x;
extern float distance_y;
extern float M_cspeed[4];
extern u8 E_BRK[4][7];
extern u8 BRK[4][7];
extern u8 speed_stop[4][7];
extern u8 speed_up[4][7];
extern u8 speed_right[4][7];
extern u32 pu2;

extern float Runline[4]; //����ǰ�����˼�ʱ����ٶ�
extern MT_Alg Motor_Alg[4];
extern float MotorAcc1[4]; //ֱ������ģʽ�µļ��ٶ�

extern void motor_init(void);
extern void motor_excute(void);
extern float abs2(float num);
extern int LIMIT_int_L_H(int Num,int min,int max);
extern void motor_excute(void);
extern int abs_int(int num);
extern void motorio_init(void);
//extern int cur;
extern u8 key;
extern int cur;
extern int oldc;
#define M1_CCR				TIM3->CCR1	//���1��Ӧ��ʱ��ռ�ձȸ�ֵ�Ĵ���
#define M2_CCR				TIM3->CCR2	//���2��Ӧ��ʱ��ռ�ձȸ�ֵ�Ĵ���
#define M3_CCR				TIM3->CCR4	//���3��Ӧ��ʱ��ռ�ձȸ�ֵ�Ĵ���
#define M4_CCR				TIM4->CCR1	//���4��Ӧ��ʱ��ռ�ձȸ�ֵ�Ĵ���

#endif
