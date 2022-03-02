#include "sx126x_example_send.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "delay.h"
#include "string.h"

/*!
 * Radio events function pointer
 * ����Ǵ��ν��������������ˣ�������ȫ�ֱ���(�ֲ�����ʹ�������ڴ��ͷſ��ܵ����쳣)
 */
static RadioEvents_t SX126xRadioEvents;

static void SX126xOnTxDone( void );
static void SX126xOnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void SX126xOnTxTimeout( void );
static void SX126xOnRxTimeout( void );
static void SX126xOnRxError( void );

//����һ����ʱ��������ÿ��1S����һ������
void ExampleSX126xSendDemo(void){
	uint32_t u32_count=0;
	
	printf("start %s() example\r\n",__func__);
	
	SX126xRadioEvents.TxDone = SX126xOnTxDone;
	SX126xRadioEvents.RxDone = SX126xOnRxDone;
	SX126xRadioEvents.TxTimeout = SX126xOnTxTimeout;
	SX126xRadioEvents.RxTimeout = SX126xOnRxTimeout;
	SX126xRadioEvents.RxError = SX126xOnRxError;

	Radio.Init( &SX126xRadioEvents );
	Radio.SetChannel(LORA_FRE);
	Radio.SetTxConfig( MODEM_LORA, LORA_TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );//������loraģʽ,���书��,fsk�õ�lora����Ϊ0�Ϳ��ԣ�������������ʣ�ǰ���볤�ȣ��̶��������ݰ�(һ���ǲ��̶�������ѡfalse)��crcУ�飬0��ʾ�ر���Ƶ����Ƶ֮��ķ�����(�ر���Ƶ�������û������)�����Ӧ���Ǳ�ʾ�Ƿ�Ҫ��ת�жϵ�ƽ�ģ���ʱʱ��

	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SX126X_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );
	
	while(1){
		Radio.IrqProcess( ); // Process Radio IRQ
		
		if(0==u32_count%1000){
			printf("systick=%d ,send u32 data:%d\r\n",Get_SysTick(),u32_count);
			Radio.Send((uint8_t *)&u32_count,4);
		}
		u32_count++;
		delay_ms(1);
	}
}

static void SX126xOnTxDone( void )
{
	printf("TxDone\r\n");
	Radio.Standby();
	
	//���������˸һ��led��ʾ
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay_ms(500);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay_ms(500);
}

static void SX126xOnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	uint32_t reciveNumber=0;
	Radio.Standby();
	printf("RxDone\r\nsize:%d\r\nrssi:%d\r\nsnr:%d\r\n",size,rssi,snr);
	Radio.Rx( LORA_RX_TIMEOUT_VALUE );
	if(size!=4){
		printf("recive size !=4 is error\r\n");
	}else{
		memcpy(&reciveNumber,payload,4);
		printf("recive u32 data=%d\r\n",reciveNumber);
		//���ճɹ���˸һ��led��ʾ
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay_ms(500);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay_ms(500);
	}
}

static void SX126xOnTxTimeout( void )
{
	printf("TxTimeout\r\n");
}

static void SX126xOnRxTimeout( void )
{
	Radio.Standby();
	printf("RxTimeout retry recive\r\n");
	Radio.Rx( LORA_RX_TIMEOUT_VALUE ); 
}

static void SX126xOnRxError( void )
{
	Radio.Standby();
	printf("RxError retry recive\r\n");
	Radio.Rx(LORA_RX_TIMEOUT_VALUE); 
}
