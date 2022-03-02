#ifndef __PROJECT_CONFIG_H__
#define __PROJECT_CONFIG_H__

//����semtech����������ֲ
//����������ַ https://github.com/Lora-net/LoRaMac-node/tree/master/src/radio  �������� 2021/2/3
#define SOFT_VERSION	"sx126x driver for stm32f103 V0.0.0"

//--------------------------------------------- ����Ĭ������ ---------------------------------------------
#define LORA_FRE																		470000000	// �շ�Ƶ��
#define LORA_TX_OUTPUT_POWER                        22        // ����Ĭ��ʹ�õķ��书�ʣ�126x���书��0~22dbm��127x���书��2~20dbm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,	����Ĭ��ʹ�õĴ���sx126x��[0: 125 kHz,1: 250 kHz,2: 500 kHz,3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // ����Ĭ��ʹ�õ���Ƶ���ӷ�Χ7~12
#define LORA_CODINGRATE                             2         // ����Ĭ��ʹ�õľ��������[1: 4/5,2: 4/6,3: 4/7,4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // ǰ���볤��
#define LORA_SX126X_SYMBOL_TIMEOUT                  0         // Symbols(sx126x�õ�����0,127x�õ�����5)
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false			// �Ƿ�Ϊ�̶����Ȱ�(��ʱֻ��sx126x�õ���)
#define LORA_IQ_INVERSION_ON                        false			// ���Ӧ���������Ƿ�ת�жϵ�ƽ��(��ʱֻ��sx126x�õ���)
#define LORA_RX_TIMEOUT_VALUE                       2000

/*!
 * Board MCU pins definitions
 */
//SPI
#define RADIO_NSS_PIN       GPIO_Pin_4
#define RADIO_NSS_PORT      GPIOA
#define RADIO_MOSI_PIN      GPIO_Pin_7
#define RADIO_MOSI_PORT     GPIOA
#define RADIO_MISO_PIN      GPIO_Pin_6
#define RADIO_MISO_PORT     GPIOA
#define RADIO_SCK_PIN       GPIO_Pin_5
#define RADIO_SCK_PORT      GPIOA
//RST��λ��
#define RADIO_nRESET_PIN    GPIO_Pin_1
#define RADIO_nRESET_PORT   GPIOB
//DIO1 ����
#define RADIO_DIO1_PIN      GPIO_Pin_11
#define RADIO_DIO1_PORT     GPIOB
//BUSY ����
#define RADIO_DIO4_BUSY_PIN      GPIO_Pin_0
#define RADIO_DIO4_BUSY_PORT     GPIOA


//�����⼸������û���õ�������Ϊ��������ģʽ
//TXEN
#define RADIO_DIO0_TXEN_PIN      GPIO_Pin_10
#define RADIO_DIO0_TXEN_PORT     GPIOB
//DIO2
#define RADIO_DIO2_PIN      GPIO_Pin_8
#define RADIO_DIO2_PORT     GPIOB
//DIO3
#define RADIO_DIO3_PIN      GPIO_Pin_9
#define RADIO_DIO3_PORT     GPIOB
//RXEN
#define RADIO_DIO5_RXEN_PIN      GPIO_Pin_1
#define RADIO_DIO5_RXEN_PORT     GPIOA

#endif // __PROJECT_CONFIG_H__
