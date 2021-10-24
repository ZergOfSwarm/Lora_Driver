/*
 * LoRa.h
 *
 *  Created on: 2 нояб. 2020 г.
 *      Author: Enik
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "libs.h"
#include "LoRa_defines.h"

#define LORA_TIMEOUT 300
#define MAX_PACKET	256

class LORA {
private:
	typedef enum _LORA_STATUS {
		SLEEP, STANDBY, TX, RX
	} LORA_Status;

	struct _LORA_STRUCT{
		GPIO_TypeDef *dio0_port;
		uint16_t dio0_pin;
		GPIO_TypeDef *nss_port;
		uint16_t nss_pin;
		GPIO_TypeDef *reset_port;
		uint16_t reset_pin;

		SPI_HandleTypeDef *hspix;

		uint8_t power;
		uint8_t spread;
		uint8_t BW;
		uint8_t packet_len;

		LORA_Status status;

		uint8_t rxBuffer[MAX_PACKET];
		uint8_t readBytes;

	} LORA_St;

	void hw_Reset(void);
	int	hw_GetDIO0(void);
	void setLoRa(void);
	void clearLoRaIrq(void);
	void config(void);

	void setNSS(bool _val);
	uint8_t hw_SPIReadByte(void);
	void hw_SPICommand(uint8_t cmd);
	uint8_t SPIRead(uint8_t addr);
	void SPIWrite(uint8_t addr, uint8_t cmd);
	void SPIBurstRead(uint8_t addr, uint8_t *rxBuf, uint8_t length);
	void SPIBurstWrite(uint8_t addr, uint8_t *txBuf, uint8_t length);

public:
	LORA(){
	}

	LORA(GPIO_TypeDef *dio_po, uint16_t dio_pin,GPIO_TypeDef *nss_po, uint16_t nss_pin, GPIO_TypeDef *rst_po,uint16_t rst_pin, SPI_HandleTypeDef *spix) {
		LORA_St.dio0_port = dio_po;
		LORA_St.dio0_pin = dio_pin;
		LORA_St.nss_port = nss_po;
		LORA_St.nss_pin = nss_pin;
		LORA_St.reset_port = rst_po;
		LORA_St.reset_pin = rst_pin;
		LORA_St.hspix = spix;
	}

	void init(uint8_t pack_len, uint8_t pwr = LORA_POWER_20DBM, uint8_t sprd = LORA_SF_7, uint8_t brnd = LORA_BW_500KHZ);
	bool check_con(void);

	uint8_t RxPacket(void);
	int TxPacket(uint8_t *txBuffer, uint8_t length, uint32_t timeout= LORA_TIMEOUT) ;
	int setTX(uint8_t length, uint32_t timeout = LORA_TIMEOUT) ;
	int setRX(uint8_t length, uint32_t timeout = LORA_TIMEOUT);
	uint8_t available(void);
	uint8_t read(uint8_t *rxBuf, uint8_t length);
	uint8_t RSSI_LoRa(void) ;
	uint8_t RSSI(void);
	void standby(void) ;
	void sleep(void);


};

#endif /* INC_LORA_H_ */
