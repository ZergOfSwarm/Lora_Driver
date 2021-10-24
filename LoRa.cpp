/*
 * LoRa.cpp
 *
 *  Created on: 2 нояб. 2020 г.
 *      Author: Enik
 */

#include "LoRa.h"

/*__________________________________*/
#define PUBLIC_METHODS
/*__________________________________*/

/*	LORA::init function
 * 	args: packet_len; frequency; power; spread_factor; brandwidth
 * 	search for: LoRa_defines
 * 	only packet_len you need to customize
 */
void LORA::init(uint8_t pack_len, uint8_t pwr, uint8_t sprd, uint8_t brnd) {
	setNSS(1);
	HAL_GPIO_WritePin(LORA_St.reset_port, LORA_St.reset_pin, GPIO_PIN_SET);

	LORA_St.packet_len = pack_len;
	LORA_St.power = pwr;
	LORA_St.spread = sprd;
	LORA_St.BW = brnd;

	config();
}

bool LORA::check_con(void){
	uint8_t vers = SPIRead(REG_LR_VERSION);
	if (vers != 0x12) {
	    return 0;
	  }
	else return 1;


}

uint8_t LORA::RxPacket(void) { // OK; add to .h
	uint8_t addr, packet_size;

	if (hw_GetDIO0()) {
		memset(LORA_St.rxBuffer, 0x00, MAX_PACKET);

		addr = SPIRead(LR_RegFifoRxCurrentaddr); //last packet addr
		SPIWrite(LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (LORA_St.spread == LORA_SF_6) { //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
			packet_size = LORA_St.packet_len;
		} else {
			packet_size = SPIRead(LR_RegRxNbBytes); //Number for received bytes
		}

		SPIBurstRead(0x00, LORA_St.rxBuffer, packet_size);
		LORA_St.readBytes = packet_size;
		clearLoRaIrq();
	}
	return LORA_St.readBytes;
}

int LORA::TxPacket(uint8_t *txBuffer, uint8_t length, uint32_t timeout) {
	SPIBurstWrite(0x00, txBuffer, length);
	SPIWrite(LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		if (hw_GetDIO0()) { //if(Get_NIRQ()) //Packet send over
			SPIRead(LR_RegIrqFlags);
			clearLoRaIrq(); //Clear irq
			standby(); //Entry Standby mode
			return 1;
		}

		if (--timeout == 0) {
			hw_Reset();
			return 0;
		}
		delay(1);
	}
}

int LORA::setTX(uint8_t length, uint32_t timeout) { // OK; add to .h; timeout to std
	LORA_St.packet_len = length;

	config(); //setting base parameter
	SPIWrite(REG_LR_PADAC, 0x87);		// 20 dBm boost
	SPIWrite(LR_RegHopPeriod, 0x00); //Отключить скачки частоты (Freq hopping)
	SPIWrite(REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	clearLoRaIrq();
	SPIWrite(LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SPIWrite(LR_RegPayloadLength, length); // PUT len of str
	uint8_t addr = SPIRead(LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SPIWrite(LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	uint8_t temp = SPIRead(LR_RegPayloadLength); // CHECK len of str

	while (1) {
		if (temp == length) {
			LORA_St.status = TX;
			return 1;
		}
		if (--timeout == 0) {
			hw_Reset();
			return 0;
		}
		delay(1);
	}

}

int LORA::setRX(uint8_t length, uint32_t timeout) { // OK; add to .h; timeout to std
	LORA_St.packet_len = length;

	config();		//Setting base parameter
	SPIWrite(REG_LR_PADAC, 0x84);	//Normal and RX
	SPIWrite(LR_RegHopPeriod, 0xFF);	//No FHSS
	SPIWrite(REG_LR_DIOMAPPING1, 0x01);	//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SPIWrite(LR_RegIrqFlagsMask, 0x3F);	//Open RxDone interrupt & Timeout
	clearLoRaIrq();
	SPIWrite(LR_RegPayloadLength, length);//Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
	uint8_t addr = SPIRead(LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SPIWrite(LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SPIWrite(LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	SPIWrite(LR_RegOpMode, 0x05);	//Continuous Rx Mode //High Frequency Mode
	LORA_St.readBytes = 0;

	while (1) {
		if ((SPIRead(LR_RegModemStat) & 0x04) == 0x04) {//Rx-on going RegModemStat
			LORA_St.status = RX;
			return 1;
		}
		if (--timeout == 0) {
			hw_Reset();
			return 0;
		}
		delay(1);
	}
}

uint8_t LORA::available(void) { // OK; add to .h
	return RxPacket();
}

uint8_t LORA::read(uint8_t *rxBuf, uint8_t length) { // OK; add to .h
	if (length != LORA_St.readBytes)
		length = LORA_St.readBytes;
	memcpy(rxBuf, LORA_St.rxBuffer, length);
	rxBuf[length] = '\0';
	LORA_St.readBytes = 0;
	return length;
}

uint8_t LORA::RSSI_LoRa(void) { // OK; add to .h
	uint32_t temp = 10;
	temp = SPIRead(LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t LORA::RSSI(void) { // OK; add to .h
	uint8_t temp = 0xff;
	temp = SPIRead(0x11);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}

void LORA::standby(void) { // OK; add to .h
	SPIWrite(LR_RegOpMode, 0x09);
	LORA_St.status = STANDBY;
}

void LORA::sleep(void) { // OK; add to .h
	SPIWrite(LR_RegOpMode, 0x08);
	LORA_St.status = SLEEP;
}

/*__________________________________*/
#define PRIVATE_METHODS
/*__________________________________*/
void LORA::hw_Reset(void) {	// OK
	LORA_Status _st_buf = LORA_St.status;
	setNSS(1);
	HAL_GPIO_WritePin(LORA_St.reset_port, LORA_St.reset_pin, GPIO_PIN_RESET);
	delay(1);
	HAL_GPIO_WritePin(LORA_St.reset_port, LORA_St.reset_pin, GPIO_PIN_SET);
	delay(100);

	if (_st_buf == TX)
		setTX(LORA_St.packet_len);
	else if (_st_buf == RX)
		setRX(LORA_St.packet_len);
	else if (_st_buf == SLEEP) {
		config();
		sleep();
	} else if (_st_buf == STANDBY) {
		config();
		standby();
	} else {
		config();
	}

}

int LORA::hw_GetDIO0(void) { // OK
	return (HAL_GPIO_ReadPin(LORA_St.dio0_port, LORA_St.dio0_pin)
			== GPIO_PIN_SET);
}

void LORA::setLoRa(void) { // OK
	SPIWrite(LR_RegOpMode, 0x88);
}

void LORA::clearLoRaIrq(void) { // OK
	SPIWrite(LR_RegIrqFlags, 0xFF);
}

void LORA::config(void) {
	setNSS(1);
	HAL_GPIO_WritePin(LORA_St.reset_port, LORA_St.reset_pin, GPIO_PIN_RESET);
	delay(1);
	HAL_GPIO_WritePin(LORA_St.reset_port, LORA_St.reset_pin, GPIO_PIN_SET);
	delay(100);


/*
	SPIWrite(LR_RegOpMode, 0x80 | 0x00);

	uint8_t fq_mas[3] = { 0x6C, 0x80, 0x00 }; // ONLY 433 MHz
	SPIBurstWrite(LR_RegFrMsb, (uint8_t*) fq_mas, 3); //setting  frequency parameter

	SPIWrite(LR_RegFifoTxBaseAddr, 0);
	SPIWrite(LR_RegFifoRxBaseAddr, 0);

	SPIWrite(LR_RegLna,SPIRead(LR_RegLna) | 0x03);

	// set auto AGC
	SPIWrite(0x26, 0x04);  // REG_MODEM_CONFIG_3


	SPIWrite(REG_LR_PADAC , 0x84);

	uint8_t ocp = (100-45)/5;
	SPIWrite(LR_RegOcp, 0x20 | (0x1F & ocp));

	SPIWrite(LR_RegPaConfig, 0x80 | 0x00);

	SPIWrite(LR_RegOpMode, 0x80 | 0x01);
*/


	 sleep(); //Change modem mode Must in Sleep mode
	 delay(15);
	 setLoRa();
	 //SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	 uint8_t fq_mas[3] = { 0x6C, 0x80, 0x00 }; // ONLY 433 MHz
	 SPIBurstWrite(LR_RegFrMsb, (uint8_t*) fq_mas, 3); //setting  frequency parameter

	 //setting base parameter
	 SPIWrite(LR_RegPaConfig, LORA_St.power); //Setting output power parameter

	 SPIWrite(LR_RegOcp, 0x0B);			//RegOcp,Close Ocp
	 SPIWrite(LR_RegLna, 0x23);		//RegLNA,High & LNA Enable
	 if (LORA_St.spread == 6) {	//SFactor=6
	 SPIWrite(LR_RegModemConfig1,
	 ((LORA_St.BW << 4) + (LORA_CR << 1) + 0x01));
	 //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

	 SPIWrite(LR_RegModemConfig2,
	 ((LORA_St.spread << 4) + (LORA_CRC << 2) + 0x03));

	 uint8_t tmp = SPIRead(0x31);
	 tmp &= 0xF8;
	 tmp |= 0x05;
	 SPIWrite(0x31, tmp);
	 SPIWrite(0x37, 0x0C);
	 } else {
	 SPIWrite(LR_RegModemConfig1,
	 ((LORA_St.BW << 4) + (LORA_CR << 1) + 0x00));
	 //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

	 SPIWrite(LR_RegModemConfig2,
	 ((LORA_St.spread << 4) + (LORA_CRC << 2) + 0x03));
	 //SFactor &  LNA gain set by the internal AGC loop
	 }

	 SPIWrite(LR_RegSymbTimeoutLsb, 0xFF); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	 SPIWrite(LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	 SPIWrite(LR_RegPreambleLsb, 12); //RegPreambleLsb 8+4=12byte Preamble
	 SPIWrite(REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	 LORA_St.readBytes = 0;
	 standby(); //Entry standby mode

}

/*__________________________________*/
#define BUS_METHODS
/*__________________________________*/

void LORA::setNSS(bool _val) { // OK

	HAL_GPIO_WritePin(LORA_St.nss_port, LORA_St.nss_pin,
			(_val == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

}

uint8_t LORA::hw_SPIReadByte(void) { // OK; add to .h
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	setNSS(0);
	HAL_SPI_TransmitReceive(LORA_St.hspix, &txByte, &rxByte, 1, 1000);
	while (HAL_SPI_GetState(LORA_St.hspix) != HAL_SPI_STATE_READY)
		;
	return rxByte;
}

void LORA::hw_SPICommand(uint8_t cmd) { // OK; add to .h
	setNSS(0);
	HAL_SPI_Transmit(LORA_St.hspix, &cmd, 1, 1000);
	while (HAL_SPI_GetState(LORA_St.hspix) != HAL_SPI_STATE_READY)
		;
}

uint8_t LORA::SPIRead(uint8_t addr) { // OK; add to .h
	uint8_t tmp;

	hw_SPICommand(addr);
	tmp = hw_SPIReadByte();
	setNSS(1);
	return tmp;
}

void LORA::SPIWrite(uint8_t addr, uint8_t cmd) { // OK; add to .h
	setNSS(0);
	hw_SPICommand(addr | 0x80);
	hw_SPICommand(cmd);
	setNSS(1);
}

void LORA::SPIBurstRead(uint8_t addr, uint8_t *rxBuf, // OK; add to .h
		uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		hw_SPICommand(addr);
		for (uint8_t i = 0; i < length; i++) {
			*(rxBuf + i) = hw_SPIReadByte();
		}
		setNSS(1);
	}
}

void LORA::SPIBurstWrite(uint8_t addr, uint8_t *txBuf, uint8_t length) { // OK; add to .h
	if (length <= 1) {
		return;
	} else {
		setNSS(0);
		hw_SPICommand(addr | 0x80);
		for (uint8_t i = 0; i < length; i++) {
			hw_SPICommand(*(txBuf + i));
		}
		setNSS(1);
	}
}

#define END

