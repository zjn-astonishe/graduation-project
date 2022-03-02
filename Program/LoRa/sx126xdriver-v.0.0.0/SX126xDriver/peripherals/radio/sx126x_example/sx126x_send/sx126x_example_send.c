#include "sx126x_example_send.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
#include "stm32f10x_it.h"
#include "delay.h"
#include "string.h"

/*!
 * Radio events function pointer
 * 这个是传参进入其他函数中了，所以用全局变量(局部变量使用完了内存释放可能导致异常)
 */
static RadioEvents_t SX126xRadioEvents;

static void SX126xOnTxDone( void );
static void SX126xOnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void SX126xOnTxTimeout( void );
static void SX126xOnRxTimeout( void );
static void SX126xOnRxError( void );

//开启一个定时发送任务，每隔1S发送一条数据
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
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );//参数：lora模式,发射功率,fsk用的lora设置为0就可以，带宽，纠错编码率，前导码长度，固定长度数据包(一般是不固定的所以选false)，crc校验，0表示关闭跳频，跳频之间的符号数(关闭跳频这个参数没有意义)，这个应该是表示是否要翻转中断电平的，超时时间

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
	
	//发送完成闪烁一下led提示
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
		//接收成功闪烁一下led提示
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
