#include "Server.h"
#include "stm32f10x.h"                  // Device header

void TIM5_PWM_Init(u16 arr,u16 psc)
{
	//�����ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	//ʹ�ܶ�ʱ��ʱ�� TIM5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//ʹ��GPIOA����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//��ʼ��GPIO
	//���÷����������TIM5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM5 PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//��ʼ��ʱ��
	//��������һ�������¼�װ�����Զ���װ�ؼĴ�������ֵ50HZ
	 TIM_TimeBaseStructure.TIM_Period = arr;
	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ ����Ƶ
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;
	//����ʱ�ӷָTDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 //TIM���ϼ���ģʽ
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	 //���ģʽ����
	 //ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ1
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 //�Ƚ����ʹ��
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 0;//���ô�װ�벶��ȽϼĴ���������ֵ
	 //������ԣ�TIM����Ƚϼ��Ը�
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	 TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	 
	 TIM_CtrlPWMOutputs(TIM5,ENABLE);	//MOE �����ʹ��	
	 
	 TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //CH1Ԥװ��ʹ��
   TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	 
	 //ʹ��ARR��MOE����ʱ��
	 TIM_ARRPreloadConfig(TIM5, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
   TIM_Cmd(TIM5, ENABLE); //ʹ��TIM4
	 	
}

void SetJointAngle(float angle)
{
	angle=(u16)(50.0*angle/9.0+249.0);
	TIM_SetCompare1(TIM5,angle);
}
