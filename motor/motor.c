#include "sys.h"	
#include "motor.h"


void motor_PID(MT_Alg *Motor_parameter,int i);
void motor_excute(void);
void AD(void);
void limit(void);
float abs2(float num);
int LIMIT_int_L_H(int Num,int min,int max);
int abs_int(int num);
int GETrep(float th);
void XY(void);
void THETA(void);
void THETA2(int *autospeed);

//static bool selectstate=FALSE; //Ö¸Ê¾slect°´¼üµÄ×´Ì¬
int cur=0;
int oldc=0;
float theta=0.0;
float distance_x=0.0;
float distance_y=0.0;
u8 mode123=0;
u8 mode345=0;
u8 mode71=0;
u8 mode72=0;
u8 mode73=0;
u8 mode74=0;
u8 key=0;
u8 key11=0;
u8 key22=0;
u8 key33=0;
u8 key44=0;
u8 nkey11=0;
u8 nkey22=0;
u8 nkey33=0;
u8 nkey44=0;

u16 motorspeed[4]={0,0,0,0};
u8 k1=50;
u8 k2=3;
u8 t=0;
u8 i=20;  		  //×ªËÙ±È
float r=0.15;   //°ë¾¶
float theta2=0.0;
float distance1=0.0;
float distance2=0.0;	
//static int cur=0;

u8 E_BRK[4][7]={{0x05,0x24,0x00,0x00,0x00,0x00,0x00},
				{0x09,0x24,0x00,0x00,0x00,0x00,0x00},
				{0x03,0x24,0x00,0x00,0x00,0x00,0x00},
				{0x07,0x24,0x00,0x00,0x00,0x00,0x00}};//É²³µ£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»		
u8 BRK[4][7]=  {{0x05,0x04,0x60,0x01,0x01,0x00,0x00},
				{0x09,0x04,0x60,0x01,0x01,0x00,0x00},
				{0x03,0x04,0x60,0x01,0x01,0x00,0x00},
				{0x07,0x04,0x60,0x01,0x01,0x00,0x00}};//²»É²³µ£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»   
u8 E_FR[4][7]= {{0x05,0x04,0x52,0x01,0x00,0x00,0x52},
				{0x09,0x04,0x52,0x01,0x00,0x00,0x5E},
				{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
				{0x00,0x00,0x00,0x00,0x00,0x00,0x00}};//Õý×ª£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
u8 FR[4][7]=   {{0x05,0x04,0x52,0x01,0x01,0x00,0x53},
				{0x09,0x04,0x52,0x01,0x01,0x00,0x5F},
				{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
				{0x00,0x00,0x00,0x00,0x00,0x00,0x00}};//·´×ª£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
u8 MO[4][7]= {{0x05,0x70,0x00,0x00,0x0A,0x00,0x7F},
				{0x09,0x70,0x00,0x00,0x0A,0x00,0x73},
				{0x03,0x70,0x00,0x00,0x0A,0x00,0x79},
				{0x07,0x70,0x00,0x00,0x0A,0x00,0x7D}};//ÇÐ»»µ½Ö¸ÁîËÙ¶ÈÄ£Ê½£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
		
u8 speed_up[4][7]={{0x05,0x22,0x00,0x00,0xF4,0x01,0xD2},
				   {0x09,0x22,0x00,0x00,0x0C,0xFE,0xD9},
				   {0x03,0x22,0x00,0x00,0xF4,0x01,0xD4},
				   {0x07,0x22,0x00,0x00,0x0C,0xFE,0xD7}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
//1041
u8 speed_up_low[4][7]={{0x05,0x22,0x00,0x00,0x11,0x04,0x32},
				   {0x09,0x22,0x00,0x00,0xEF,0xFB,0x3F},
				   {0x03,0x22,0x00,0x00,0x11,0x04,0x34},
				   {0x07,0x22,0x00,0x00,0xEF,0xFB,0x31}};//gai
//2083
u8 speed_up_mid[4][7]={{0x05,0x22,0x00,0x00,0x23,0x08,0x0C},
				   {0x09,0x22,0x00,0x00,0xDD,0xF7,0x01},
				   {0x03,0x22,0x00,0x00,0x23,0x08,0x0A},
				   {0x07,0x22,0x00,0x00,0xDD,0xF7,0x0F}};//gai
//3125
u8 speed_up_high[4][7]={{0x05,0x22,0x00,0x00,0x35,0x0C,0x1E},
				   {0x09,0x22,0x00,0x00,0xCB,0xF3,0x13},
				   {0x03,0x22,0x00,0x00,0x35,0x0C,0x18},
				   {0x07,0x22,0x00,0x00,0xCB,0xF3,0x1D}}; //ÐÞ¸ÄÐ£Ñé
//4167
u8 speed_up_highest[4][7]={{0x05,0x22,0x00,0x00,0x47,0x10,0x70},
				   {0x09,0x22,0x00,0x00,0xB9,0xEF,0x7D},
				   {0x03,0x22,0x00,0x00,0x47,0x10,0x76},
				   {0x07,0x22,0x00,0x00,0xB9,0xEF,0x73}};//gai

u8 speed_down[4][7]={{0x05,0x22,0x00,0x00,0x0C,0xFE,0xD5},
				     {0x09,0x22,0x00,0x00,0xF4,0x01,0xDE},
				     {0x03,0x22,0x00,0x00,0x0C,0xFE,0xD3},
				     {0x07,0x22,0x00,0x00,0xF4,0x01,0xD0}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
//1041
u8 speed_down_low[4][7]={{0x05,0x22,0x00,0x00,0xEF,0xFB,0x33},
				     {0x09,0x22,0x00,0x00,0x11,0x04,0x3E},
				     {0x03,0x22,0x00,0x00,0xEF,0xFB,0x35},
				     {0x07,0x22,0x00,0x00,0x11,0x04,0x30}};//gai
//2083
u8 speed_down_mid[4][7]={{0x05,0x22,0x00,0x00,0xDD,0xF7,0x0D},
				     {0x09,0x22,0x00,0x00,0x23,0x08,0x00},
				     {0x03,0x22,0x00,0x00,0xDD,0xF7,0x0B},
				     {0x07,0x22,0x00,0x00,0x23,0x08,0x0E}};//gai
//3125
u8 speed_down_high[4][7]={{0x05,0x22,0x00,0x00,0xCB,0xF3,0x1F},
				     {0x09,0x22,0x00,0x00,0x35,0x0C,0x12},
				     {0x03,0x22,0x00,0x00,0xCB,0xF3,0x19},
				     {0x07,0x22,0x00,0x00,0x35,0x0C,0x1C}};//gai
//4167
u8 speed_down_highest[4][7]={{0x05,0x22,0x00,0x00,0xB9,0xEF,0x71},
				     {0x09,0x22,0x00,0x00,0x47,0x10,0x7C},
				     {0x03,0x22,0x00,0x00,0xB9,0xEF,0x77},
				     {0x07,0x22,0x00,0x00,0x47,0x10,0x72}};//gai

u8 speed_right[4][7]={{0x05,0x22,0x00,0x00,0x0C,0xFE,0xD5},
										 {0x09,0x22,0x00,0x00,0x0C,0xFE,0xD9},
										 {0x03,0x22,0x00,0x00,0xF4,0x01,0xD4},
										 {0x07,0x22,0x00,0x00,0xF4,0x01,0xD0}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
u8 speed_rightlow[4][7]={{0x05,0x22,0x00,0x00,0x00,0x00,0x00},
										 {0x09,0x22,0x00,0x00,0x00,0x00,0x00},
										 {0x03,0x22,0x00,0x00,0x00,0x00,0x00},
										 {0x07,0x22,0x00,0x00,0x00,0x00,0x00}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
u8 speed_left[4][7]={{0x05,0x22,0x00,0x00,0xF4,0x01,0xD2},
				     {0x09,0x22,0x00,0x00,0xF4,0x01,0xDE},
				     {0x03,0x22,0x00,0x00,0x0C,0xFE,0xD3},
				     {0x07,0x22,0x00,0x00,0x0C,0xFE,0xD7}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»

u8 speed_ni[4][7]={{0x05,0x22,0x00,0x00,0x0C,0xFE,0xD5},
				   {0x09,0x22,0x00,0x00,0x0C,0xFE,0xD9},
				   {0x03,0x22,0x00,0x00,0x0C,0xFE,0xD3},
				   {0x07,0x22,0x00,0x00,0x0C,0xFE,0xD7}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»

u8 speed_shun[4][7]={{0x05,0x22,0x00,0x00,0xF4,0x01,0xD2},
				   {0x09,0x22,0x00,0x00,0xF4,0x01,0xDE},
				   {0x03,0x22,0x00,0x00,0xF4,0x01,0xD4},
				   {0x07,0x22,0x00,0x00,0xF4,0x01,0xD0}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»

u8 speed_stop[4][7]={{0x05,0x22,0x00,0x00,0x00,0x00,0x27},
				     {0x09,0x22,0x00,0x00,0x00,0x00,0x2B},
				     {0x03,0x22,0x00,0x00,0x00,0x00,0x21},
				     {0x07,0x22,0x00,0x00,0x00,0x00,0x25}};//ÉèÖÃËÙ¶ÈÎª³õÊ¼ËÙ¶È£ºÇý¶¯Æ÷ID¡¢Ö¸Áî¡¢µØÖ·µÍ8Î»¡¢µØÖ·¸ß8Î»¡¢Êý¾ÝµÍ8Î»¡¢Êý¾Ý¸ß8Î»ºÍÐ£ÑéÎ»
					 					 
int duty[4]={1999,1999,1999,1999};
float M_cspeed[4]={0,0,0,0};
float Runline[4]={20.0 , 20.0 ,20.0 , 20.0};//°´ÏÂÇ°½øºóÍË¼üÊ±ºòµÄËÙ
float MotorAcc1[4]={0.40,0.40,0.40,0.40};//Ö±ÏßÐÐ×ßÄ£Ê½ÏÂµÄ¼ÓËÙ¶È
MT_Alg Motor_Alg[4]={{{0.0 , 0.0 , 0.0 , 0.0 , 5.0 , 0.00 , 0.007 , 1.0 , 1.0 , 1.0 , 0.0} , 0.0 , 0.0 , 0.0 , 0.0},
					 {{0.0 , 0.0 , 0.0 , 0.0 , 5.0 , 0.000005 , 0.0125  , 1.0 , 1.0 , 1.0 , 0.0} , 0.0 , 0.0 , 0.0 , 0.0},
					 {{0.0 , 0.0 , 0.0 , 0.0 , 5.0 , 0.000005 , 0.0125 , 1.0 , 1.0 , 1.0 , 0.0} , 0.0 , 0.0 , 0.0 , 0.0},
					 {{0.0 , 0.0 , 0.0 , 0.0 , 5.0 , 0.000005 , 0.0125  , 1.0 , 1.0 , 1.0 , 0.0} , 0.0 , 0.0 , 0.0 , 0.0}}; 
                     //µç»ú1¡¢µç»ú2¡¢µç»ú3¡¢µç»ú4µÄ¿ØËÙ²ÎÊý
                     //err  err_last  err_next  err_sum  kp       ki         kd     x     y     z output setspeed actual_speed integral output_duty
u8 mode_kai=0;
u8 mode_shang=0;
u8 mode_xia=0;
u8 mode_ss=0;
u8 mode_up=0;
u8 mode_down=0;
u8 mode_left=0;	
u8 mode_right=0;
u8 mode_shun=0;
u8 mode_ni=0;	
u8 mode_stop=0;
u8 mode_theta=0;		
u8 mode_distance=0;	
u8 mode_distance2=0;	
					 
u8 mode_right2=1;
u8 mode_up2=1;
u8 mode_left2=1;
u32 pu={0x00000000};
u32 pu2={0x00000000};
u32 pu4={0x00000000};
u8 key;
					 
int repairvalue=0;
int repairvalue2=0;
int repairvalue3=0;				 
u8 count=0;
int pu3;
				 			 			 
void motor_init(void)
{

	int i;
	for(i=0;i<4;i++) 
	{
		//Ä£Ê½¡¢¼ÓËÙ¶ÈÉèÖÃ
		RS232_Send_Data(MO[i],7);//Ä£Ê½
		OR(AC[i]);RS232_Send_Data(AC[i],7);
	}	
		//RS232_Send_Data(AC[i],7);//¼ÓËÙ¶È
		mode_kai=0;
		mode_shang=0;
		mode_xia=0;
		mode_up=0;
		mode_down=0;
		mode_left=0;	
		mode_right=0;
		mode_shun=0;
		mode_ni=0;	
		mode_stop=0;
}

void motorio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);  //Ê¹ÄÜGPIOÍâÉèºÍAFIO¸´ÓÃ¹¦ÄÜÄ£¿éÊ±ÖÓ
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOE, ENABLE);
	
	//µç»ú·½Ïò	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_9;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);//³õÊ¼»¯GPIO 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//³õÊ¼»¯GPIO 	
	
	//µç»úÉ²³µ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//³õÊ¼»¯GPIO 
	
	GPIO_SetBits(GPIOC,GPIO_Pin_0);
	GPIO_SetBits(GPIOC,GPIO_Pin_1);
  GPIO_SetBits(GPIOC,GPIO_Pin_2);
	GPIO_SetBits(GPIOC,GPIO_Pin_3);
	
	//¹âµç¿ª¹Ø
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //ÉèÖÃ³ÉÉÏÀ­ÊäÈë
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//³õÊ¼»¯GPIOE2,3,4
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //ÉèÖÃ³ÉÉÏÀ­ÊäÈë
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//³õÊ¼»¯GPIOE2,3,4
	
	//»úÐµ¿ª¹Ø
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //ÉèÖÃ³ÉÉÏÀ­ÊäÈë
 	GPIO_Init(GPIOG, &GPIO_InitStructure);//³õÊ¼»¯GPIOE2,3,4

}

void motor_excute(void)
{  
	  int i;
		if(PS2_Sony.PS2_Startsignal == TRUE)   //Èç¹ûstart¼ü°´ÏÂ ÁÁÂÌµÆ
		{
			
			//Ö¸Ê¾µÆLED1ÁÁ
			LED1=0;
		
			//Ê¹ÄÜµç»ú  ²»É²³µ		
//				GPIO_ResetBits(GPIOC,GPIO_Pin_0);
//	      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
//      	GPIO_ResetBits(GPIOC,GPIO_Pin_2);
//	      GPIO_ResetBits(GPIOC,GPIO_Pin_3);
			//??????
				key11=KAI1;
				key22=KAI2;
				key33=KAI3;
				key44=KAI4;
				delay_ms(5);
				key11=KAI1;
				key22=KAI2;
				key33=KAI3;
				key44=KAI4;

			/******************************************  ????  ************************************************************/				
			if((PS2_Sony.PS2_Byte.PS2_L1==TRUE) && (PS2_Sony.PS2_Byte.PS2_L2==TRUE) &&(PS2_Sony.PS2_Byte.PS2_R1== FALSE)&& (PS2_Sony.PS2_Byte.PS2_R2== FALSE)&&key11==0&&nkey11==0&&key22==0&&nkey22==0&&key33==0&&nkey33==0&&key44==0&&nkey44==0)
			{
				//?????
				LED0=0;

				if(key11==0&&nkey11==0&&key22==0&&nkey22==0&&key33==0&&nkey33==0&&key44==0&&nkey44==0)
				{
					//???,???
					if(mode_kai==0)
					{
							mode_kai=1;				
							mode_ss=0;
							if(mode1_up==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
							if(mode2_up==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_1);
							if(mode3_up==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
							if(mode4_up==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
					}
					//???????
					if(mode_xia)
					{
						GPIO_ResetBits(GPIOC,GPIO_Pin_0);
						GPIO_ResetBits(GPIOC,GPIO_Pin_1);
						GPIO_ResetBits(GPIOC,GPIO_Pin_2);
						GPIO_ResetBits(GPIOC,GPIO_Pin_3);
						mode_xia=0;
					}
					
							if(mode1_up==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_0);mode_shang=1;} //????1 ?????
							if(mode2_up==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_1);mode_shang=1;} //????2 ?????
							if(mode3_up==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_2);mode_shang=1;} //????3 ?????
							if(mode4_up==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_3);mode_shang=1;} //????4 ?????
							
					M1_FR_NEG;M2_FR_POS;M3_FR_POS;M4_FR_NEG;//????
							
					M1_CCR=800;
					M2_CCR=800;
					M3_CCR=800;
					M4_CCR=800;
				}
			}
			/******************************************  ????  ************************************************************/
			else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_L2==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== TRUE) && (PS2_Sony.PS2_Byte.PS2_R2== TRUE)&&key11==0&&nkey11==0&&key22==0&&nkey22==0&&key33==0&&nkey33==0&&key44==0&&nkey44==0)
			{
				//?????
				LED0=0;
				//?????,???
					if(mode_kai==0)//&&mode1_down==1&&mode2_down==1&&mode3_down==1&&mode4_down==1)
					{
							mode_kai=1;
							mode_ss=0;
							if(mode1_down==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
							if(mode2_down==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_1);
							if(mode3_down==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
							if(mode4_down==1)		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
					}
					//???????
					if(mode_shang)
					{
						GPIO_ResetBits(GPIOC,GPIO_Pin_0);
						GPIO_ResetBits(GPIOC,GPIO_Pin_1);
						GPIO_ResetBits(GPIOC,GPIO_Pin_2);
						GPIO_ResetBits(GPIOC,GPIO_Pin_3);
						mode_shang=0;
					}
								
					if(mode1_down==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_0);mode_xia=1;}//????1 ?????
					if(mode2_down==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_1);mode_xia=1;}//????1 ?????
					if(mode3_down==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_2);mode_xia=1;}//????1 ?????
					if(mode4_down==0)  {GPIO_SetBits(GPIOC,GPIO_Pin_3);mode_xia=1;}//????1 ?????
					M1_FR_POS;M2_FR_NEG;M3_FR_NEG;M4_FR_POS;//????
						M1_CCR=800;
						M2_CCR=800;
						M3_CCR=800;
						M4_CCR=800;
			}

			
				/******************************************  ???????  ************************************************************/
			else if(PS2_Sony.PS2_Leftsignal == FALSE)
			{
				M1_CCR=1999;
				M2_CCR=1999;
				M3_CCR=1999;
				M4_CCR=1999;	
				if(key11==1&&nkey11==1)		GPIO_SetBits(GPIOB,GPIO_Pin_8);
				if(key22==1&&nkey22==1)   GPIO_SetBits(GPIOB,GPIO_Pin_8);
				if(key33==1&&nkey33==1)		GPIO_SetBits(GPIOB,GPIO_Pin_8);
				if(key44==1&&nkey44==1)   GPIO_SetBits(GPIOB,GPIO_Pin_8);

				if(mode_ss==0)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_0);//????1 ?????
					GPIO_SetBits(GPIOC,GPIO_Pin_1);//????1 ?????
					GPIO_SetBits(GPIOC,GPIO_Pin_2);//????1 ?????
					GPIO_SetBits(GPIOC,GPIO_Pin_3);//????1 ?????
					mode_ss=1;
					mode_kai=0;
				}
			}

/////////////////////////////////Ö÷Òª³ÌÐò//////////////////////////////////////////////////////////////////////////////////
			if(PS2_Sony.PS2_Leftsignal == TRUE&&PS2_Sony.PS2_Byte.PS2_Right == FALSE)
			{	
				get_code(code);
				for(t=0;t<=top;t++)
				{
					if(path_stack[t]->s_y==path_y && path_stack[t]->s_x==path_x)
					{
						oldc=cur;
						cur=t;break;
					}
				}
				//Í£ÏÂµ÷ÕûÎ»×Ë
				if( cur==0 || cur==top ||(path_stack[cur+1]->s_y!=path_stack[cur-1]->s_y && path_stack[cur+1]->s_x!=path_stack[cur-1]->s_x))
				{

					static int mode=-1;
					if(mode!=cur)
					{
						mode=cur;	
						for(i=0;i<4;i++)
						{
							RS232_Send_Data(speed_stop[i],7);			
						}
						GPIO_SetBits(GPIOB,GPIO_Pin_8);					
						delay_ms(500);
						GPIO_ResetBits(GPIOB,GPIO_Pin_8);
						mode123=0;mode345=0;
					}
					XY();THETA();
				}
				//¼ÓËÙÖ±ÐÐ
				if( cur>=2 && path_stack[cur-2]->s_y-path_stack[cur-1]->s_y==1 && path_stack[cur-1]->s_y-path_y==1)  
				{
					for(i=0;i<4;i++)
					{
						RS232_Send_Data(speed_up_high[i],7);			
					}	
				}
				//¼ÓËÙ×ó²àºáÒÆ
				else if(cur>=2 && path_stack[cur-1]->s_x-path_stack[cur-2]->s_x==1 && path_x-path_stack[cur-1]->s_x==1)  
				{
					autospeed[0]=1000;
					autospeed[1]=1000;
					autospeed[2]=-1000;
					autospeed[3]=-1000;
//					SPEED(2000,1,1);
//					SPEED(2000,1,2);
//					SPEED(2000,0,3);
//					SPEED(2000,0,4);
//					for(i=0;i<4;i++)
//					{
//						RS232_Send_Data(CMD_SPEED[i],7);
//					}					
				}
				//¼ÓËÙÓÒ²àºáÒÆ
				else if(cur>=2 && path_stack[cur-2]->s_x-path_stack[cur-1]->s_x==1 && path_stack[cur-1]->s_x-path_x==1)  
				{
					autospeed[0]=-1000;
					autospeed[1]=-1000;
					autospeed[2]=1000;
					autospeed[3]=1000;
//					SPEED(2000,0,1);
//					SPEED(2000,0,2);
//					SPEED(2000,1,3);
//					SPEED(2000,1,4);
					for(i=0;i<4;i++)
					{
						RS232_Send_Data(CMD_SPEED[i],7);
					}					
				}				
				//¼õËÙÖ±ÐÐ
				else if((cur>=2 &&  path_stack[cur-2]->s_y-path_stack[cur-1]->s_y==0 && path_stack[cur-1]->s_y-path_y==1)||(cur==1&&path_stack[cur-1]->s_y-path_y==1))  
				{
					for(i=0;i<4;i++)
					{
						RS232_Send_Data(speed_up[i],7);			
					}					
				}				
				//¼õËÙ×ó²àºáÒÆ
				else if((cur>=2 &&  path_stack[cur-1]->s_x-path_stack[cur-2]->s_x==0 && path_x-path_stack[cur-1]->s_x==1)||(cur==1&& path_x-path_stack[cur-1]->s_x==1)) 
				{
					autospeed[0]=400;
					autospeed[1]=400;
					autospeed[2]=-400;
					autospeed[3]=-400;
//					SPEED(400,1,1);
//					SPEED(400,1,2);
//					SPEED(400,0,3);
//					SPEED(400,0,4);
					for(i=0;i<4;i++)
					{
						RS232_Send_Data(CMD_SPEED[i],7);
					}					
				}
				//¼õËÙÓÒ²àºáÒÆ
				else if((cur>=2 &&  path_stack[cur-2]->s_x-path_stack[cur-1]->s_x==0 && path_stack[cur-1]->s_x-path_x==1)||(cur==1&& path_stack[cur-1]->s_x-path_x==1))
				{
					autospeed[0]=-400;
					autospeed[1]=-400;
					autospeed[2]=400;
					autospeed[3]=400;
//					SPEED(400,0,1);
//					SPEED(400,0,2);
//					SPEED(400,1,3);
//					SPEED(400,1,4);
					for(i=0;i<4;i++)
					{
						RS232_Send_Data(CMD_SPEED[i],7);
					}					
				}

			}
			
				
     ///////////°´ÏÂright°´¼üÍ£³µ///////////
			if(PS2_Sony.PS2_Byte.PS2_Right == TRUE)
			{
				for(i=0;i<4;i++)
					{
					  	RS232_Send_Data(speed_stop[i],7);//ËÙ¶È
  						
					}		
			}
			
			
			/******************************************  °´ÏÂÇ°½øºóÍË¼ü  ************************************************************/	
			if((PS2_Sony.PS2_Byte.PS2_Up == TRUE) && (PS2_Sony.PS2_Byte.PS2_Down== FALSE))
			{						

				mode_right2=1;
					//Ö¸Ê¾µÆÉÁË¸
					LED0=0;
					LED0=1;
					
				
					/****************Ö±ÏßÇ°½ø**********************/
					if((PS2_Sony.PS2_Byte.PS2_Square == FALSE)&&(PS2_Sony.PS2_Byte.PS2_O == FALSE)&&(PS2_Sony.PS2_Byte.PS2_Triangle== FALSE) && (PS2_Sony.PS2_Byte.PS2_X== FALSE)) 
					{		
						mode_down=0;
						mode_left=0;	
						mode_right=0;
						mode_shun=0;
						mode_ni=0;	
						mode_stop=0;
						//Éè¶¨·½Ïò
//						RS232_Send_Data(E_FR[0],7);
//						RS232_Send_Data(E_FR[1],7);
//						RS232_Send_Data(E_FR[2],7);
//						RS232_Send_Data(E_FR[3],7);
						//Éè¶¨ËÙ¶È
						
						if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
							if(mode_up==0)
							{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_up[i],7);//ËÙ¶È
									
								}	
								mode_up=1;
							}						
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==TRUE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_up==0)
							{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_up_low[i],7);//ËÙ¶È
									
								}	
								mode_up=1;
							}						
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== TRUE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_up==0)
							{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_up_mid[i],7);//ËÙ¶È
									
								}	
								mode_up=1;
							}						
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== TRUE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_up==0)
							{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_up_high[i],7);//ËÙ¶È
									
								}	
								mode_up=1;
							}						
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== TRUE))
						{	
   						if(mode_up==0)
							{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_up_highest[i],7);//ËÙ¶È
									
								}	
								mode_up=1;
							}						
					  }
						
				}
					/*****************×ó²àºáÒÆ**********************/
					else if((PS2_Sony.PS2_Byte.PS2_Square == TRUE)&& (PS2_Sony.PS2_Byte.PS2_O == FALSE)&& (PS2_Sony.PS2_Byte.PS2_Triangle== FALSE) && (PS2_Sony.PS2_Byte.PS2_X== FALSE)) 
					{
						mode_up=0;
						mode_down=0;
						mode_right=0;
						mode_shun=0;
						mode_ni=0;	
						mode_stop=0;
						//Éè¶¨·½Ïò
						//RS232_Send_Data(FR[0],7);
						//RS232_Send_Data(E_FR[1],7);
						//RS232_Send_Data(E_FR[2],7);
						//RS232_Send_Data(FR[3],7);
						//Éè¶¨ËÙ¶È
						if(mode_left==0)
						{
							for(i=0;i<4;i++)
							{
								RS232_Send_Data(speed_left[i],7);//ËÙ¶È
								
							}
					   }
						mode_left=1;
					}
					
					/******************ÓÒ²àºáÒÆ*********************/ 
					else if((PS2_Sony.PS2_Byte.PS2_Square == FALSE) && (PS2_Sony.PS2_Byte.PS2_O == TRUE)&& (PS2_Sony.PS2_Byte.PS2_Triangle== FALSE) && (PS2_Sony.PS2_Byte.PS2_X== FALSE)) 
					{
						mode_up=0;
						mode_down=0;
						mode_left=0;	
						mode_shun=0;
						mode_ni=0;	
						mode_stop=0;
						LED0=0;
						//Éè¶¨·½Ïò
						//RS232_Send_Data(E_FR[0],7);
						//RS232_Send_Data(FR[1],7);
						//RS232_Send_Data(FR[2],7);
						//RS232_Send_Data(E_FR[3],7);
						//Éè¶¨ËÙ¶È
						if(mode_right==0)
						{
							for(i=0;i<4;i++)
							{
								RS232_Send_Data(speed_right[i],7);//ËÙ¶È
								
							}
					    }
						mode_right=1;					
					 }
								
					/***************ÄæÊ±ÕëÔ­µØ×ª*********************/ 
					else if((PS2_Sony.PS2_Byte.PS2_Square == FALSE) && (PS2_Sony.PS2_Byte.PS2_O == FALSE)&& (PS2_Sony.PS2_Byte.PS2_Triangle== TRUE) && (PS2_Sony.PS2_Byte.PS2_X== FALSE)) 
					{
						mode_up=0;
						mode_down=0;
						mode_left=0;	
						mode_right=0;
						mode_shun=0;
						mode_stop=0;
						//Éè¶¨·½Ïò
						//RS232_Send_Data(FR[0],7);
						//RS232_Send_Data(E_FR[1],7);
						//RS232_Send_Data(FR[2],7);
						//RS232_Send_Data(E_FR[3],7);
						//Éè¶¨ËÙ¶È
						if(mode_left==0)
						{
							for(i=0;i<4;i++)
							{
								RS232_Send_Data(speed_ni[i],7);//ËÙ¶È
							}
					   }
						mode_ni=1;					
					 }
								 
					/******************Ë³Ê±ÕëÔ­µØ×ª*****************/ 
					else if((PS2_Sony.PS2_Byte.PS2_Square == FALSE) && (PS2_Sony.PS2_Byte.PS2_O == FALSE)&& (PS2_Sony.PS2_Byte.PS2_Triangle== FALSE) && (PS2_Sony.PS2_Byte.PS2_X== TRUE)) 
					{
						mode_up=0;
						mode_down=0;
						mode_left=0;	
						mode_right=0;
						mode_ni=0;	
						mode_stop=0;
						//Éè¶¨·½Ïò
						//RS232_Send_Data(E_FR[0],7);
						//RS232_Send_Data(FR[1],7);
						//RS232_Send_Data(E_FR[2],7);
						//RS232_Send_Data(FR[3],7);
						//Éè¶¨ËÙ¶È
						if(mode_left==0)
						{
							for(i=0;i<4;i++)
							{
								RS232_Send_Data(speed_shun[i],7);//ËÙ¶È
							}
					   }
						mode_shun=1;				    
					}															
			}	
			/******************************************  °´ÏÂºóÍË¼ü  ************************************************************/				
			else if((PS2_Sony.PS2_Byte.PS2_Up == FALSE) && (PS2_Sony.PS2_Byte.PS2_Down== TRUE))
			{
				 mode_up=0;
				 mode_left=0;	
				 mode_right=0;
				 mode_shun=0;
				 mode_ni=0;	
			     mode_stop=0;
				 //Ö¸Ê¾µÆÉÁË¸
				 LED0=0;
			   LED0=1;
			     
				 //if((PS2_Sony.PS2_Byte.PS2_Square == FALSE)&&(PS2_Sony.PS2_Byte.PS2_O == FALSE)&&(PS2_Sony.PS2_Byte.PS2_Triangle== FALSE) && (PS2_Sony.PS2_Byte.PS2_X== FALSE))
				 //{
						//Éè¶¨·½Ïò
						//RS232_Send_Data(FR[0],7);
						//RS232_Send_Data(FR[1],7);
						//RS232_Send_Data(FR[2],7);
						//RS232_Send_Data(FR[3],7);
						//Éè¶¨ËÙ¶È
						
						if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_down==0)
					  	{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_down[i],7);//ËÙ¶È
								}
					    }
							mode_down=1;				
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==TRUE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_down==0)
					  	{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_down_low[i],7);//ËÙ¶È
								}
					    }
							mode_down=1;					
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== TRUE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_down==0)
					  	{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_down_mid[i],7);//ËÙ¶È
								}
					    }
							mode_down=1;					
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== TRUE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== FALSE))
						{	
   						if(mode_down==0)
					  	{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_down_high[i],7);//ËÙ¶È
								}
					    }
							mode_down=1;				
					  }
						else if((PS2_Sony.PS2_Byte.PS2_L1==FALSE) && (PS2_Sony.PS2_Byte.PS2_R1== FALSE) && (PS2_Sony.PS2_Byte.PS2_L2== FALSE) && (PS2_Sony.PS2_Byte.PS2_R2== TRUE))
						{	
   						if(mode_down==0)
					  	{
								for(i=0;i<4;i++)
								{
									RS232_Send_Data(speed_down_highest[i],7);//ËÙ¶È
								}
					    }
							mode_down=1;			
					  }
				 //}
				 //else {}	
			}
			/****************************************************Î´°´ÏÂÇ°½øºóÍË¼ü***************************************************************/						
			else 
			{      
					mode_up=0;
					mode_down=0;
					mode_left=0;	
					mode_right=0;
					mode_shun=0;
					mode_ni=0;	
					//Éè¶¨·½Ïò
					//RS232_Send_Data(E_FR[0],7);
					//RS232_Send_Data(E_FR[1],7);
					//RS232_Send_Data(E_FR[2],7);
					//RS232_Send_Data(E_FR[3],7);
					//Éè¶¨ËÙ¶È
				if(mode_stop==0)
			  {
					for(i=0;i<4;i++)
					{
					  	RS232_Send_Data(speed_stop[i],7);//ËÙ¶È
  						
					}		
				}
				mode_stop=1;
			}	
		}
		else if(PS2_Sony.PS2_Startsignal == FALSE)  //Èç¹ûstart¼üÃ»ÓÐ°´ÏÂ£¬ÄÇÃ´ÁÁºìµÆ£¬ÖÆ¶¯
		{
			static u8 mode_ting=0;
			//Ö¸Ê¾µÆLED1Ãð£¬·äÃùÆ÷¹Ø
			LED1=1;
			GPIO_ResetBits(GPIOB,GPIO_Pin_8);
			//Ê¹ÄÜµç»
			if(mode_ting==0)
			{
			for(i=0;i<4;i++) 
			{
				 RS232_Send_Data(speed_stop[i],7);//ËÙ¶È
//				 delay_ms(1000); delay_ms(1000); delay_ms(1000); delay_ms(1000);
				//				RS232_Send_Data(E_BRK[i],7);   //É²³µ
			}	
			mode_ting=1;			
		}
			//Ê¹ÄÜµç»ú  ²»É²³µ		
//			GPIO_SetBits(GPIOC,GPIO_Pin_0);
//			GPIO_SetBits(GPIOC,GPIO_Pin_1);
//			GPIO_SetBits(GPIOC,GPIO_Pin_2);
//			GPIO_SetBits(GPIOC,GPIO_Pin_3);
		}
}

void motor_PID(MT_Alg *Motor_parameter,int i)
{
    Motor_parameter->ActualSpeed=M_cspeed[i];
	  Motor_parameter->MT_PID.err_last=Motor_parameter->MT_PID.err;
	  Motor_parameter->MT_PID.err_next=Motor_parameter->MT_PID.err_last;
	  Motor_parameter->MT_PID.err=Motor_parameter->SetSpeed-Motor_parameter->ActualSpeed;
	  Motor_parameter->MT_PID.err_sum+=Motor_parameter->MT_PID.err;
	  Motor_parameter->output_duty=Motor_parameter->MT_PID.x*Motor_parameter->MT_PID.Kp*Motor_parameter->MT_PID.err
          	+Motor_parameter->MT_PID.y*Motor_parameter->MT_PID.Ki*Motor_parameter->MT_PID.err_sum
				    +Motor_parameter->MT_PID.z*Motor_parameter->MT_PID.Kd*(Motor_parameter->MT_PID.err-Motor_parameter->MT_PID.err_last);
	  duty[i]-=Motor_parameter->output_duty;
	  duty[i]=LIMIT_int_L_H(duty[i],0,1999);
}

int GETrep(float th) 
{
	int rep=th*100;
	return rep;
}

void limit(void)
{
//	if(theta<-10)  theta=-10;
//	if(theta>10)  theta=10;
	if(distance_x<-50) distance_x=-50;
	if(distance_x>50) distance_x=50;
	if(distance_y<-50) distance_y=-50;
	if(distance_y>50) distance_y=50;
}

float abs2(float num)
{
    if(num>=0) return num;
	  else if(num<0) return -1*num;
	  else
			return 0.0;
}

int LIMIT_int_L_H(int Num,int min,int max)
{
		if(Num<=min)
			return min;
		else if(Num>=max)
			return max;
		else
			return Num;
}	

int abs_int(int num)
{
    if(num>=0) return num;
	  else if(num<0) return -1*num;
	  else
			return 0;
}

void XY(void)
{
	while(mode345==0)
	{				
		//ÏÞÎ»
		limit();
		if(distance_x>-5&&distance_x<5) distance_x=0;
		if(distance_y>-5&&distance_y<5) distance_y=0;
	
		if(mode_distance==0) distance1=distance_x;
		if(mode_distance2==0) distance2=distance_y;

		repairvalue2=distance_x*k2;    //distance*k2;
		repairvalue3=distance_y*k2;
		repairvalue=0;
		
			//²îËÙ
		motorspeed[0]=autospeed[0]+repairvalue+repairvalue2-repairvalue3;
		motorspeed[1]=autospeed[1]-repairvalue-repairvalue2-repairvalue3;
		motorspeed[2]=autospeed[2]+repairvalue-repairvalue2-repairvalue3;
		motorspeed[3]=autospeed[3]-repairvalue+repairvalue2-repairvalue3;
		//Éú³ÉÖ¸Áî²¢·¢ËÍ
		SPEED(motorspeed[0],1,1);
		SPEED(motorspeed[1],0,2);
		SPEED(motorspeed[2],1,3);
		SPEED(motorspeed[3],0,4);
		for(i=0;i<4;i++)
		{
			RS232_Send_Data(CMD_SPEED[i],7);//ËÙ¶È
		}
		
			//distance_x»ý·Ö
			if(distance1>0) 
			{	
				distance1=distance1-3*((4*repairvalue2)*100*360/4/2500/i)*r/4;
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
				mode_distance=1;
				if(distance1<0)
				{
					distance1=0;
					GPIO_ResetBits(GPIOB,GPIO_Pin_8);
					mode_distance=0;
				}
			}	
			else if(distance1<0)
			{
				distance1=distance1-3*((4*repairvalue2)*100*360/4/2500/i)*r/4;
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
				mode_distance=1;
				if(distance1>0)
				{
					distance1=0;
					GPIO_ResetBits(GPIOB,GPIO_Pin_8);
					mode_distance=0;
				}
			}
			
			//distance_y»ý·Ö
			if(distance2>0) 
			{	
				distance2=distance2-3*((4*repairvalue3)*100*360/4/2500/i)*r/4;
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
				mode_distance2=1;
				if(distance2<0)
				{
					distance2=0;
					GPIO_ResetBits(GPIOB,GPIO_Pin_8);
					mode_distance2=0;
				}
			}	
			else if(distance2<0)
			{
				distance2=distance2-3*((4*repairvalue3)*100*360/4/2500/i)*r/4;
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
				mode_distance2=1;
				if(distance2>0)
				{
					distance2=0;
					GPIO_ResetBits(GPIOB,GPIO_Pin_8);
					mode_distance2=0;
				}
			}
			if(distance_x==0&&distance_y==0)//mode123==1&&
			{
				mode345=1;
			}
		}	
}


void THETA(void)
{
	while(mode123==0)
	{
		limit();
		if(theta>-0.5&&theta<0.5) 
		{
			theta=0;
			mode123=1;
		}
		if(mode_theta==0) theta2=theta;
		repairvalue=theta*k1;
		repairvalue2=0;
		repairvalue3=0;
		motorspeed[0]=autospeed[0]+repairvalue+repairvalue2-repairvalue3;
		motorspeed[1]=autospeed[1]-repairvalue-repairvalue2-repairvalue3;
		motorspeed[2]=autospeed[2]+repairvalue-repairvalue2-repairvalue3;
		motorspeed[3]=autospeed[3]-repairvalue+repairvalue2-repairvalue3;
		//Éú³ÉÖ¸Áî²¢·¢ËÍ
		SPEED(motorspeed[0],1,1);
		SPEED(motorspeed[1],0,2);
		SPEED(motorspeed[2],1,3);
		SPEED(motorspeed[3],0,4);
		for(i=0;i<4;i++)
		{
			RS232_Send_Data(CMD_SPEED[i],7);//ËÙ¶È
		}
		//theta»ý·Ö
		if(theta2>0) 
		{	
			theta2=theta2-0.3*((4*repairvalue)*100*360/4/2500/i)*r/4/(0.6+1.7);//   °ë¾¶/4/(L1+L2)
			GPIO_SetBits(GPIOB,GPIO_Pin_8);
			mode_theta=1;
			if(theta2<0)
			{
				theta2=0;
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				mode_theta=0;
			}
		}	
		else if(theta2<0) 
		{
			theta2=theta2-0.3*((4*repairvalue)*100*360/4/2500/i)*r/4/(0.6+1.7);
			GPIO_SetBits(GPIOB,GPIO_Pin_8);
			mode_theta=1;
			if(theta2>0)
			{
				theta2=0;
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				mode_theta=0;
			}
		}	
	}
}

void THETA2(int *autospeed)
{

	if(theta>-0.3&&theta<0.3) 
	{
		theta=0;
		//mode123=1;
	}
	else mode_theta=0;
	while(mode_theta==0)
	{
		if(mode_theta==0) theta2=theta;
		repairvalue=theta*k1;
		repairvalue2=0;
		repairvalue3=0;
		motorspeed[0]=autospeed[0]+repairvalue+repairvalue2-repairvalue3;
		motorspeed[1]=autospeed[1]-repairvalue-repairvalue2-repairvalue3;
		motorspeed[2]=autospeed[2]+repairvalue-repairvalue2-repairvalue3;
		motorspeed[3]=autospeed[3]-repairvalue+repairvalue2-repairvalue3;
		//Éú³ÉÖ¸Áî²¢·¢ËÍ
		SPEED(motorspeed[0],1,1);
		SPEED(motorspeed[1],0,2);
		SPEED(motorspeed[2],1,3);
		SPEED(motorspeed[3],0,4);
		for(i=0;i<4;i++)
		{
			RS232_Send_Data(CMD_SPEED[i],7);//ËÙ¶È
		}
		//theta»ý·Ö
		if(theta2>0) 
		{	
			theta2=theta2-0.3*((4*repairvalue)*100*360/4/2500/i)*r/4/(0.6+1.7);//   °ë¾¶/4/(L1+L2)
			GPIO_SetBits(GPIOB,GPIO_Pin_8);
			mode_theta=1;
			if(theta2<=0)
			{
				theta2=0;
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				mode_theta=0;
			}
		}	
		else if(theta2<0) 
		{
			theta2=theta2-0.3*((4*repairvalue)*100*360/4/2500/i)*r/4/(0.6+1.7);
			GPIO_SetBits(GPIOB,GPIO_Pin_8);
			mode_theta=1;
			if(theta2>=0)
			{
				theta2=0;
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				mode_theta=0;
			}
		}	
	}
		SPEED(autospeed[0],autospeed[0]>0,1);
		SPEED(autospeed[1],autospeed[0]>0,2);
		SPEED(autospeed[2],autospeed[0]>0,3);
		SPEED(autospeed[3],autospeed[0]>0,4);
		for(i=0;i<4;i++)
		{
			RS232_Send_Data(CMD_SPEED[i],7);//ËÙ¶È
		}
}
