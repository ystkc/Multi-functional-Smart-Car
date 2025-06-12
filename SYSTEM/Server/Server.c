#include "Server.h"
#include "stm32f10x.h"                  // Device header

void TIM5_PWM_Init(u16 arr,u16 psc)
{
	//定义初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	//使能定时器时钟 TIM5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//使能GPIOA外设时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//初始化GPIO
	//设置服用输出功能TIM5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM5 PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//初始化时基
	//设置在下一个更新事件装入活动的自动重装载寄存器周期值50HZ
	 TIM_TimeBaseStructure.TIM_Period = arr;
	//设置用来作为TIMx时钟频率除数的预分频值 不分频
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;
	//设置时钟分割：TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 //TIM向上计数模式
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	 //输出模式配置
	 //选择定时器模式：TIM脉冲宽度调制模式1
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 //比较输出使能
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 0;//设置待装入捕获比较寄存器的脉冲值
	 //输出极性：TIM输出比较级性高
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	 TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	 
	 TIM_CtrlPWMOutputs(TIM5,ENABLE);	//MOE 主输出使能	
	 
	 TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //CH1预装载使能
   TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	 
	 //使能ARR、MOE及定时器
	 TIM_ARRPreloadConfig(TIM5, ENABLE); //使能TIMx在ARR上的预装载寄存器
   TIM_Cmd(TIM5, ENABLE); //使能TIM4
	 	
}

void SetJointAngle(float angle)
{
	angle=(u16)(50.0*angle/9.0+249.0);
	TIM_SetCompare1(TIM5,angle);
}
