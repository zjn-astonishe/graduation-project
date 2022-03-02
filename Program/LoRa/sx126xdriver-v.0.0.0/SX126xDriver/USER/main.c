#include "stm32f10x.h"
#include "delay.h"
#include "HAL_uart.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "project_config.h"
#include "sx126x_example_send.h"
#include "sx126x_example_recive.h"

//硬件初始化
void SysInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//NVIC(中断优先级管理)分组配置,注意:这个分组整个程序只能有一次,配置后不要修改,否则会出现很多问题 这里直接用4,0~15优先级
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	    //JTAG复用为GPIO需要使用 RCC_APB2Periph_AFIO 时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//关闭JTAG功能(需要打开 RCC_APB2Periph_AFIO 时钟)

	//led指示灯
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13); 			 //PB.12 输出高
	
	HALUart1Init();
	SysTick_Config(SystemCoreClock/1000);
}

int main(void){
	SysInit();	//硬件初始化
	
	printf("SysInit OK，version:%s\r\n",SOFT_VERSION);

	//测试demo，一个程序只能打开一条测试demo，进入测试demo后将进入死循环，不会返回了
	ExampleSX126xReciveDemo();	//循环接收demo
		// ExampleSX126xSendDemo();	//定时发送demo
	
	//开启测试demo后代码就执行不到这里了
	while(1){
		printf("systick=%d\r\n",Get_SysTick());
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay_ms(100);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay_ms(100);
	}
}
