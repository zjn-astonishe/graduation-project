/*!
 * \file      sx1262mbxcas-board.c
 *
 * \brief     Target board SX1262MBXCAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "project_config.h"
#include "delay.h"
#include "radio.h"
#include "sx126x-board.h"
#include "HAL_SPI.h"
#include "stdio.h"

typedef struct __softTimer_s{
	uint8_t isRun;		//0ֹͣ��1����
	uint32_t delayMs;	//��ʱ����Ҫ�೤ʱ�����
	uint32_t startMs;	//��ʱ����ʼʱ��
}SoftTimer_t;

static DioIrqHandler *dio1IrqCallback=NULL;	//��¼DIO1�Ļص��������
static uint32_t timer_count=0;
static SoftTimer_t txTimerHandle,rxTimerHandle;

/* ��ʼ��sx126x��Ҫ�õ���GPIO��ʼ������ BUSY ��������Ϊ����ģʽ
 */
void SX126xIoInit( void )
{
	uint8_t u8_ret=255;
	GPIO_InitTypeDef  GPIO_InitStructure;

	//��ʼ��BUSYΪ��������ģʽ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //ʹ��ָ���˿�ʱ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//��������
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO4_BUSY_PIN;
	GPIO_Init(RADIO_DIO4_BUSY_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	//�����⼸������û���õ�������Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO0_TXEN_PIN;
	GPIO_Init(RADIO_DIO0_TXEN_PORT, &GPIO_InitStructure);	//��ʼ��GPIO

	GPIO_InitStructure.GPIO_Pin = RADIO_DIO2_PIN;
	GPIO_Init(RADIO_DIO2_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO3_PIN;
	GPIO_Init(RADIO_DIO3_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO5_RXEN_PIN;
	GPIO_Init(RADIO_DIO5_RXEN_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	u8_ret=Spi1Init();
	if(u8_ret){
		printf("spi1 init error\r\n");
	}else{
		printf("SX126xIoInit init sucess\r\n");
	}
}

/* ��ʼ�� DIO1
 * ��DIO1����Ϊ�ⲿ�ж�(�����ش���)�����һص�����Ϊ dioIrq ����ԭ�� void RadioOnDioIrq( void* context )
 */
void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	dio1IrqCallback=dioIrq;
	
	//gpio(DIO1)��ʼ��Ϊ��������ģʽ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //ʹ��ָ���˿�ʱ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;//��������
	GPIO_InitStructure.GPIO_Pin = RADIO_DIO1_PIN;
	GPIO_Init(RADIO_DIO1_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	//�ж�����
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��
	//GPIOE.2 �ж����Լ��жϳ�ʼ������   �����ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11);
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//�����ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line11);  //���LINE11�ϵ��жϱ�־λ 
}

//���жϻص������лص� void SX126xIoIrqInit( DioIrqHandler dioIrq ) ��ע��� dioIrq �ص�
void EXTI15_10_IRQHandler(void){
	//printf("enter irq\r\n");
	if(GPIO_ReadInputDataBit(RADIO_DIO1_PORT,RADIO_DIO1_PIN)){
		printf("DIO1 irq\r\n");
		if(NULL!=dio1IrqCallback){
			dio1IrqCallback(NULL);
		}
		EXTI_ClearITPendingBit(EXTI_Line11);//����жϱ�־λ
	}
}

void SX126xIoDeInit( void )
{
    //GPIOȥ��ʼ������
}

//��λ��������
void SX126xReset( void )
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	SX126xDelayMs( 10 );
	
	//��RST����͵�ƽ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //ʹ��ָ���˿�ʱ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = RADIO_nRESET_PIN;
	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	GPIO_WriteBit( RADIO_nRESET_PORT, RADIO_nRESET_PIN,Bit_RESET);	//RST��Ϊ�͵�ƽ���
	
	SX126xDelayMs( 20 );
	
	//��RST����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	
	SX126xDelayMs( 10 );
}

//��ȡBusy���ŵ�ƽ״̬
void SX126xWaitOnBusy( void )
{
	uint32_t u32_count=0;
	
	while( GPIO_ReadInputDataBit(RADIO_DIO4_BUSY_PORT,RADIO_DIO4_BUSY_PIN) == 1 ){
		if(u32_count++>1000){
			printf("wait busy pin timeout\r\n");
			u32_count=0;
		}
		SX126xDelayMs(1);
	}
}

//����SPI��NSS���ŵ�ƽ��0�͵�ƽ������ߵ�ƽ
void SX126xSetNss(uint8_t lev ){
	if(lev){
		GPIO_SetBits(RADIO_NSS_PORT,RADIO_NSS_PIN);	//����ߵ�ƽ
	}else{
		GPIO_ResetBits(RADIO_NSS_PORT,RADIO_NSS_PIN);	//����͵�ƽ
	}
}

//spi����һ���ֽڵ�����
uint8_t SX126xSpiInOut(uint8_t data){
	return HALSpi1InOut(data);
}

//���Ƶ���Ƿ����Ҫ���������Ҫ�ж������ֱ�ӷ���true
bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

//������ʱ
void SX126xDelayMs(uint32_t ms){
	delay_ms(ms);
}

//tx/rx��ʱ������

//��ʱ��3�жϷ������
//tx��ʱ������Ҫ�ص� RadioOnTxTimeoutIrq
//rx��ʱ������Ҫ�ص� RadioOnRxTimeoutIrq
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	uint32_t diffMs=0;
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		timer_count++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־
		
		//printf("timer_count=%d\r\n",timer_count);
		
		//����tx��ʱ��
		if(txTimerHandle.isRun){
			if(timer_count>=txTimerHandle.startMs){
				diffMs=timer_count-txTimerHandle.startMs;
			}else{//�����
				diffMs=0xffffffff - txTimerHandle.startMs + timer_count;
			}
			if(diffMs>txTimerHandle.delayMs){
				//��ʱ����
				//printf("----------------- tx timer irq ---------------\r\n");
				SX126xTxTimerStop();
				txTimerHandle.startMs=timer_count;
				RadioOnTxTimeoutIrq(NULL);	//tx��ʱ������Ҫ�ص� RadioOnTxTimeoutIrq
			}
		}
		
		//����rx��ʱ��
		if(rxTimerHandle.isRun){
			if(timer_count>=rxTimerHandle.startMs){
				diffMs=timer_count-rxTimerHandle.startMs;
			}else{//�����
				diffMs=0xffffffff - rxTimerHandle.startMs + timer_count;
			}
			if(diffMs>rxTimerHandle.delayMs){
				//��ʱ����
				//printf("----------------- rx timer irq ---------------\r\n");
				SX126xRxTimerStop();
				rxTimerHandle.startMs=timer_count;
				RadioOnRxTimeoutIrq(NULL);	//rx��ʱ������Ҫ�ص� RadioOnRxTimeoutIrq
			}
		}
	}
}

//��ʼ����ʱ��(������TIM3��ʱ��ģ���������ms��ʱ��)
//tx��ʱ������Ҫ�ص� RadioOnTxTimeoutIrq
//rx��ʱ������Ҫ�ص� RadioOnRxTimeoutIrq
void SX126xTimerInit(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//ʹ��TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
 
	//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = 10; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/10000; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ(1s10000�Σ�Ҳ����1��0.1ms)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//�ر������ʱ��
	txTimerHandle.isRun=0;
	rxTimerHandle.isRun=0;
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�
}

void SX126xSetTxTimerValue(uint32_t nMs){
	printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	txTimerHandle.delayMs=nMs;
}

void SX126xTxTimerStart(void){
	printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	txTimerHandle.startMs=timer_count;
	txTimerHandle.isRun=1;
}

void SX126xTxTimerStop(void){
	printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	txTimerHandle.isRun=0;
}

void SX126xSetRxTimerValue(uint32_t nMs){
	printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	rxTimerHandle.delayMs=nMs;
}

void SX126xRxTimerStart(void){
	printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	rxTimerHandle.startMs=timer_count;
	rxTimerHandle.isRun=1;
}

void SX126xRxTimerStop(void){
	printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	rxTimerHandle.isRun=0;
}
