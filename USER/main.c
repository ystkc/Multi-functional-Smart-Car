#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "delay.h"
#include "motor.h"
#include "keysacn.h"
#include "IRSEARCH.h"
#include "IRAvoid.h"
#include "remote.h"   
#include "usart.h"
#include "UltrasonicWave.h"
#include "timer.h"
#include "Server.h"

#define KEYMODE_1   1  // 循迹
#define KEYMODE_2   2  // 红外避障
#define KEYMODE_3   3  // 红外遥控
#define KEYMODE_4   4  // 超声避障
#define KEYMODE_5   5  // 蓝牙遥控
uint8_t keyMode;      //指无符号8Bit整形数

//中断服务函数  
u8 i;
u8 flag;  //定义一个标志位
void USART1_IRQHandler(void)  
  
{  
     if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)      //检查指定的 USART1 中断发生与否
      { 
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);   //清除 USART1 的中断待处理位
			GPIO_ResetBits(GPIOC,GPIO_Pin_6);                 //设置LED灯作为中断发生与否的指示灯
			i=  USART_ReceiveData(USART1);                    //返回 USART1 最近接收到的数据
		//if语句的内容部分可以保证蓝牙接收到的flag数据是正确的，如果不加的话会导致接收不到正确的数据
		  if(i=='0')
				{                 
					 flag=0;
					 ZYSTM32_run(80,100);
				}
				
			if(i=='1')
				{
					 flag=1;
					 ZYSTM32_brake(100);
				}
				
			if(i=='2')
			   {
					 flag=2;
					 ZYSTM32_Right(80,100);
				}
			   
			if(i=='3')
			   {
								 
					flag=3;
					ZYSTM32_Left(80,100);
			   }
			   
			if(i=='4')
			  {
					flag=4;
					ZYSTM32_Spin_Left(80,100);
			  }
			  
			if(i=='5')
			  {
					flag=5;
					ZYSTM32_back(80,100);
			  }
			  
			if(i=='6')
			 {
					flag=6;
					ZYSTM32_Spin_Right(80,100);
			 }
		  }
		  
//		  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	  

// 超声波转头函数
int front_detection()
{

//	ZYSTM32_brake(0);
	SetJointAngle(90);
	delay_ms(100);
	return UltrasonicWave_StartMeasure();
}
int left_detection()
{

//	ZYSTM32_brake(0);
	SetJointAngle(175);
	delay_ms(300);
	return UltrasonicWave_StartMeasure();
}
int right_detection()
{

//	ZYSTM32_brake(0);
	SetJointAngle(5);
	delay_ms(300);
	return UltrasonicWave_StartMeasure();
}

void IR_IN()                             //红外遥控子程序
{
	  u8 key;	
    ZYSTM32_brake(10);	
	   key=Remote_Scan();	
			switch(key)
			{    
				case 98:ZYSTM32_run(80,100);break;	    
				case 2:ZYSTM32_brake(100);break;		 	  
				case 194:ZYSTM32_Right(80,100);break;	   
				case 34:ZYSTM32_Left(80,100);break;		  
				case 224:ZYSTM32_Spin_Left(80,100);break;		  
				case 168:ZYSTM32_back(80,100);break;		   
				case 144:ZYSTM32_Spin_Right(80,100);break;		    
			}
}

void KeyScanTask(void)//按键子程序
{
	static u8 keypre = 0;//按键被按下时置1
	
	if((keypre == 0)&& (KEY))
	{
		keypre = 1; //置1，避免持续按下按键时再次进入此函数。
		switch(keyMode)
		{
			case KEYMODE_1:keyMode = KEYMODE_2; break;
			case KEYMODE_2:keyMode = KEYMODE_3; break;
			case KEYMODE_3:keyMode = KEYMODE_4; break;
			case KEYMODE_4:keyMode = KEYMODE_5; break;
			case KEYMODE_5:keyMode = KEYMODE_1; break;
			default: break;
		}
	}
	if(!KEY)  //按键被放开
	{
		keypre = 0;//置0，允许再次切换LED模式
	}
}
void USAvoidRun() // 超声波避障模式
{
    int Q_temp,L_temp,R_temp;
	Q_temp = front_detection();
	if(Q_temp<60 && Q_temp>0) //测量距离值	
	{
		ZYSTM32_brake(500);		
		ZYSTM32_back(60,500);	
		ZYSTM32_brake(1000);	
		
		L_temp=left_detection();//测量左边障碍物的距离值
		delay_ms(500);
		R_temp=right_detection();//测量右边障碍物的距离值
		delay_ms(500);
		
		if((L_temp < 60 ) &&( R_temp < 60 ))//当左右两侧均有障碍物靠的比较近
		{
			ZYSTM32_Spin_Left(60,500);
		}				
	else if(L_temp > R_temp)
		{
			ZYSTM32_Left(60,700);
			ZYSTM32_brake(500);
		}	
	else
		{
			ZYSTM32_Right(60,700);
			ZYSTM32_brake(500);					
		}							
	}	
	else
	{
		ZYSTM32_run(60,10);
	}

}

//任务：循迹、避障、遥控模式处理
void LEDTask()
{
	switch(keyMode)
	{
		case KEYMODE_1:
			LED_D4_SET;
			LED_D3_RESET;
			SearchRun();
			break;
		case KEYMODE_2:
			LED_D4_RESET;
			LED_D3_SET;
			AVoidRun();
			break;
		case KEYMODE_3:
			LED_D4_SET;
			LED_D3_SET;
			IR_IN();			
			break;
		case KEYMODE_4:
			LED_D4_RESET;
			LED_D3_RESET;
			USAvoidRun();
			break;
		case KEYMODE_5:
			LED_D4_RESET;
			LED_D3_RESET;
			break;
		default:
			break;
	}
}
	


 int main(void)
 {	
	 delay_init();
	 KEY_Init();
	 IRSearchInit();
	 IRAvoidInit();
   Timerx_Init(5000,7199);                //10Khz的计数频率，计数到5000为500ms 
	 UltrasonicWave_Configuration();        //对超声波模块初始化
	 uart_init(115200);
	 TIM4_PWM_Init(7199,0);  //初始化PWM
	 TIM5_PWM_Init(9999,143);               //不分频，PWM频率=72*10^6/(9999+1)/(143+1)=50Hz	 
	 Remote_Init();			    //红外接收初始化	
	 ZYSTM32_brake(500);
	//  keysacn();		
	 keyMode = KEYMODE_1;
	 
	 
	//利用中断进行蓝牙的收发
	
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
// 	USART_DeInit(USART1);  //复位串口1
//  USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10     
	
	 
    /* USARTx configured as follow:
       - BaudRate = 9600 baud  波特率
       - Word Length = 8 Bits  数据长度
       - One Stop Bit          停止位
       - No parity             校验方式
       - Hardware flow control disabled (RTS and CTS signals) 硬件控制流
       - Receive and transmit enabled                         使能发送和接收
    */
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断

    //Usart1 NVIC 配置
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
      NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
      USART_Cmd(USART1, ENABLE);                    //使能串口 
	  
	  
	  //蓝牙数据接收判断函数
	  while(1)
	  {
		
		  switch(flag)
              {
					case 0:  Stop();         break  ;
					case 1:  Turnleft();     break  ;
					case 2:  Turnright();    break  ;
					case 3:  Turnback();     break  ;
					case 4:  Turnfront();    break  ;
					case 5:  Leftaround();   break  ;
					case 6:  Rightaround();  break  ;
                    default: Stop();         break  ;                                                                                                
	          }		  
	  }
	  

		while(1)
		{  
		 
   KeyScanTask();
   LEDTask();			

		}
 }

