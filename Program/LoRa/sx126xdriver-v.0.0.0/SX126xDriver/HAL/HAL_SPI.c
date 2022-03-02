#include "HAL_SPI.h"
#include "project_config.h"

//SPI��ʼ��(sx126x��sx127xʹ�õ�SPI������ͬ�ģ�����ͳһ��ʼ��)
uint8_t Spi1Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	//��ʼ��SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//SPIʱ��ʹ��
	//��ʼ��MISO,MOSI,SCKΪ��������ģʽ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    //ʹ��ָ���˿�ʱ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;	//Ƭ����������ģʽ
	GPIO_InitStructure.GPIO_Pin = RADIO_SCK_PIN|RADIO_MISO_PIN|RADIO_MOSI_PIN;
	GPIO_Init(RADIO_SCK_PORT, &GPIO_InitStructure);	//��ʼ��GPIO

	//����SPI
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ʱ�Ӽ���(����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ���ǵ͵�ƽ)
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//������λ(����ͬ��ʱ�ӵĵڼ��������أ��������½������ݱ�����)
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:����Ϊ�������(SSI����)
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256(256�����,�����������֮������ٶ�,����ٶȹ��쵼��ͨ��ʧ���ٵ�С)
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ(CRCУ�����)
	SPI_Init(SPI1,&SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

	SPI_Cmd(SPI1,ENABLE); //ʹ��SPI����
	
	//��ʼ��NSS��Ϊ����ߵ�ƽ RADIO_NSS_PIN		RADIO_NSS_PORT
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    //ʹ��ָ���˿�ʱ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz�����ﲻ�ô��Σ�ֱ��д��������ٶ�50MHZ
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = RADIO_NSS_PIN;
	GPIO_Init(RADIO_NSS_PORT, &GPIO_InitStructure);	//��ʼ��GPIO
	//GPIO_ResetBits(gpioObj->portIndex,gpioObj->pinIndex);	//����͵�ƽ
	GPIO_SetBits(RADIO_NSS_PORT,RADIO_NSS_PIN);	//����ߵ�ƽ
	
	return 0;
}

//spi��������(����ֵ�ǽ��յ�������)
//������
//     spiName	:��Ҫͨ���ĸ�spi��������
//     data		:Ҫ���͵�����
//����ֵ��
//       ���յ�������
//ע�⣺����û�п���CS�źţ���Ҫ�Լ����ƣ�ͨ��һ����ͨ��GPIO���ƾͿ��ԣ�
uint8_t HALSpi1InOut(uint8_t data){
	uint32_t retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){//���ָ����SPI��־λ�������:���ͻ���ձ�־λ(���������˾Ϳ��Կ���������)
		if((retry++)>2000){
			//��ʱ��
			return 0xff;
		}
	}
	SPI_I2S_SendData(SPI1, data); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){ //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ���ǿ��˾ͱ�ʾ������������ˣ�
		if((retry++)>2000){
			//��ʱ��
			return 0xff;
		}
	}
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����	
}
