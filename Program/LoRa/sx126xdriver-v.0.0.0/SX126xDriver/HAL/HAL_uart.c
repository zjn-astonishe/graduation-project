#include "stm32f10x.h"
#include "stdio.h"

/*��δ���������֧��printf��ӡ�����ڵĴ���*/
#pragma import(__use_no_semihosting)             
//��׼����Ҫ֧�ֵĺ���
struct __FILE 
{
	int handle;
};

FILE __stdout;       
//����_sys_exit()�Ա��⹤���ڰ�����״̬
void _sys_exit(int x) 
{
	x = x; 
}
//�ض���fputc����
//�����Ҫ����MCU������ϣ��printf���ĸ����������ȷ�� __WAIT_TODO__
int fputc(int ch, FILE *f)
{
	//ע�⣺USART_FLAG_TXE�Ǽ�鷢�ͻ������Ƿ�Ϊ�գ����Ҫ�ڷ���ǰ��飬������������߷���Ч�ʣ����������ߵ�ʱ����ܵ������һ���ַ���ʧ
	//USART_FLAG_TC�Ǽ�鷢����ɱ�־������ڷ��ͺ��飬����������˯�߶�ʧ�ַ����⣬����Ч�ʵͣ����͹����з��ͻ������Ѿ�Ϊ���ˣ����Խ�����һ�������ˣ�������ΪҪ�ȴ�������ɣ�����Ч�ʵͣ�
	//��Ҫ����һ���ã�һ����Ч�����
	
	//ѭ���ȴ�ֱ�����ͻ�����Ϊ��(TX Empty)��ʱ���Է������ݵ�������
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {}
	USART_SendData(USART1, (uint8_t) ch);

  /* ѭ���ȴ�ֱ�����ͽ���*/
  //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  //{}

	return ch;
}

//USART1�жϺ���(����)
void USART1_IRQHandler(void){
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)  
	{
		USART_SendData(USART1, USART_ReceiveData(USART1));
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
}

//uart1(PA_9 TX,PA_10 RX)��ʼ������
uint8_t HALUart1Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART1);  //��λ����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);//��������ʱ��,ע��APB1��APB2ʱ��ʹ�ܺ�����һ��
	
	//PA_9 ��ʼ��Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//Ƭ�����裬����ģʽ(��Ҫ���PSEL���ò���ȷ���Ǹ����ĸ�����)
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��GPIO
	//PA_10��ʼ��Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��GPIO

	//�����жϻص�
	//�ж����ȼ�������ǰ�ж����ȼ�����Ϊ2(NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2))�Ѿ����壬��Ҫ�޸�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	//ָ�������ĸ��ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�Ϊ3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//���� NVIC


	//USART����
	USART_InitStructure.USART_BaudRate = 115200;	//���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�Ϊ8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //����USART����

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//���������ж�(�������޷���������,û�н��ջص������Ͳ������ˣ���Ϊû�лص���û�д�����յĵ����ݣ�������Ҳû������)
	USART_Cmd(USART1, ENABLE);                    //ʹ��USART
	
	return 0;
}
