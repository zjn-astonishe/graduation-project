#include "stm32f10x.h"
#include "delay.h"
#include "HAL_uart.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "project_config.h"
#include "sx126x_example_send.h"
#include "sx126x_example_recive.h"

//Ӳ����ʼ��
void SysInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//NVIC(�ж����ȼ�����)��������,ע��:���������������ֻ����һ��,���ú�Ҫ�޸�,�������ֺܶ����� ����ֱ����4,0~15���ȼ�
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	    //JTAG����ΪGPIO��Ҫʹ�� RCC_APB2Periph_AFIO ʱ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//�ر�JTAG����(��Ҫ�� RCC_APB2Periph_AFIO ʱ��)

	//ledָʾ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13); 			 //PB.12 �����
	
	HALUart1Init();
	SysTick_Config(SystemCoreClock/1000);
}

int main(void){
	SysInit();	//Ӳ����ʼ��
	
	printf("SysInit OK��version:%s\r\n",SOFT_VERSION);

	//����demo��һ������ֻ�ܴ�һ������demo���������demo�󽫽�����ѭ�������᷵����
	ExampleSX126xReciveDemo();	//ѭ������demo
		// ExampleSX126xSendDemo();	//��ʱ����demo
	
	//��������demo������ִ�в���������
	while(1){
		printf("systick=%d\r\n",Get_SysTick());
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay_ms(100);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay_ms(100);
	}
}
