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

#define KEYMODE_1   1  // ѭ��
#define KEYMODE_2   2  // �������
#define KEYMODE_3   3  // ����ң��
#define KEYMODE_4   4  // ��������
#define KEYMODE_5   5  // ����ң��
uint8_t keyMode;      //ָ�޷���8Bit������

//�жϷ�����  
u8 i;
u8 flag;  //����һ����־λ
void USART1_IRQHandler(void)  
  
{  
     if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)      //���ָ���� USART1 �жϷ������
      { 
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);   //��� USART1 ���жϴ�����λ
			GPIO_ResetBits(GPIOC,GPIO_Pin_6);                 //����LED����Ϊ�жϷ�������ָʾ��
			i=  USART_ReceiveData(USART1);                    //���� USART1 ������յ�������
		//if�������ݲ��ֿ��Ա�֤�������յ���flag��������ȷ�ģ�������ӵĻ��ᵼ�½��ղ�����ȷ������
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
	  

// ������תͷ����
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

void IR_IN()                             //����ң���ӳ���
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

void KeyScanTask(void)//�����ӳ���
{
	static u8 keypre = 0;//����������ʱ��1
	
	if((keypre == 0)&& (KEY))
	{
		keypre = 1; //��1������������°���ʱ�ٴν���˺�����
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
	if(!KEY)  //�������ſ�
	{
		keypre = 0;//��0�������ٴ��л�LEDģʽ
	}
}
void USAvoidRun() // ����������ģʽ
{
    int Q_temp,L_temp,R_temp;
	Q_temp = front_detection();
	if(Q_temp<60 && Q_temp>0) //��������ֵ	
	{
		ZYSTM32_brake(500);		
		ZYSTM32_back(60,500);	
		ZYSTM32_brake(1000);	
		
		L_temp=left_detection();//��������ϰ���ľ���ֵ
		delay_ms(500);
		R_temp=right_detection();//�����ұ��ϰ���ľ���ֵ
		delay_ms(500);
		
		if((L_temp < 60 ) &&( R_temp < 60 ))//��������������ϰ��￿�ıȽϽ�
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

//����ѭ�������ϡ�ң��ģʽ����
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
   Timerx_Init(5000,7199);                //10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms 
	 UltrasonicWave_Configuration();        //�Գ�����ģ���ʼ��
	 uart_init(115200);
	 TIM4_PWM_Init(7199,0);  //��ʼ��PWM
	 TIM5_PWM_Init(9999,143);               //����Ƶ��PWMƵ��=72*10^6/(9999+1)/(143+1)=50Hz	 
	 Remote_Init();			    //������ճ�ʼ��	
	 ZYSTM32_brake(500);
	//  keysacn();		
	 keyMode = KEYMODE_1;
	 
	 
	//�����жϽ����������շ�
	
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
// 	USART_DeInit(USART1);  //��λ����1
//  USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10     
	
	 
    /* USARTx configured as follow:
       - BaudRate = 9600 baud  ������
       - Word Length = 8 Bits  ���ݳ���
       - One Stop Bit          ֹͣλ
       - No parity             У�鷽ʽ
       - Hardware flow control disabled (RTS and CTS signals) Ӳ��������
       - Receive and transmit enabled                         ʹ�ܷ��ͺͽ���
    */
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�

    //Usart1 NVIC ����
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
      NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
      USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 
	  
	  
	  //�������ݽ����жϺ���
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

