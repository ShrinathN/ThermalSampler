#include "nrf_driver.h"

extern SPI_HandleTypeDef hspi2;
uint8_t buf[0x1c] = {0,};

void SPI_Transmit(uint8_t *data, uint32_t length_of_data);

void SPI_Transmit(uint8_t *data, uint32_t length_of_data)
{
	HAL_SPI_TransmitReceive(&hspi2, data, buf, length_of_data, 1000);
}

NRF24L01_Register_Map nrf_register_map = {
	0,
};

uint8_t NRF_ReadRegister(uint8_t register_addr)
{
	uint8_t byte_read = 0xFF;
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_R_REGISTER | register_addr}, 1);
	HAL_SPI_Receive(&hspi2, &byte_read, 1, 100);
	NRF_CS_HIGH;


	return byte_read;
}

void NRF_WriteRegister(uint8_t register_addr, uint8_t data_to_write)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_W_REGISTER | register_addr, data_to_write}, 2);
	NRF_CS_HIGH;

}

void NRF_SetTxAddress(uint8_t *addr, uint8_t len)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_W_REGISTER | NRF_REG_TX_ADDR}, 1);
	SPI_Transmit(addr, len);
	NRF_CS_HIGH;

}

void NRF_SetRxAddress(uint8_t *addr, uint8_t len)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_W_REGISTER | NRF_REG_RX_ADDR_P0}, 1);
	SPI_Transmit(addr, len);
	NRF_CS_HIGH;
}

void NRF_WriteTxBuffer(uint8_t *data, uint8_t len, uint8_t pipe)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_W_TX_PAYLOAD | pipe}, 1);
	SPI_Transmit(data, len);
	NRF_CS_HIGH;
}

void NRF_ReadRxBuffer(uint8_t *buffer, uint8_t len)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_R_RX_PAYLOAD}, 1);
	HAL_SPI_Receive(&hspi2, buffer, len, 100);
	NRF_CS_HIGH;
}

void NRF_FlushTxBuffer(void)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_FLUSH_TX}, 1);
	NRF_CS_HIGH;
}

void NRF_FlushRxBuffer(void)
{
	NRF_CS_LOW;
	SPI_Transmit((uint8_t[]){NRF_COMMAND_FLUSH_RX}, 1);
	NRF_CS_HIGH;
}

void NRF_Execute(void)
{
	NRF_CE_HIGH;
	NRF_DELAY(1);
	NRF_CE_LOW;
}

void NRF_Execute_Rx(void)
{
	NRF_CE_HIGH;
}

void NRF_Tx_Init(void)
{
	NRF_CS_HIGH;
	NRF_CE_LOW;


	//waking up
	while((NRF_ReadRegister(NRF_REG_CONFIG) & 0b10) != 0b10)
	{
		nrf_register_map.CONFIG.CONFIG_Byte = 0;		//setting byte as 0
		nrf_register_map.CONFIG.CONFIG_Bits.PWR_UP = 1; //powering up
		NRF_WriteRegister(NRF_REG_CONFIG, nrf_register_map.CONFIG.CONFIG_Byte);
	}


	nrf_register_map.CONFIG.CONFIG_Bits.CRCO = 0;		 //CRC 1 byte
	nrf_register_map.CONFIG.CONFIG_Bits.EN_CRC = 1;		 //disable CRC
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_MAX_RT = 1; //do not IRQ on MAX_RT
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_TX_DS = 0;	 //set IRQ on TX_DS
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_RX_DR = 1;	 //do not IRQ on RX_DR
	nrf_register_map.CONFIG.CONFIG_Bits.PRIM_RX = 0;	 //set as TX
	NRF_WriteRegister(NRF_REG_CONFIG, nrf_register_map.CONFIG.CONFIG_Byte);
	

	//no auto ACK
	nrf_register_map.EN_AA.EN_AA_Byte = 0;
	NRF_WriteRegister(NRF_REG_EN_AA, nrf_register_map.EN_AA.EN_AA_Byte);

	//no RX enable
	nrf_register_map.EN_RXADDR.EN_RXADDR_Byte = 0;
	NRF_WriteRegister(NRF_REG_EN_RXADDR, nrf_register_map.EN_RXADDR.EN_RXADDR_Byte);

	//setting address width as 5 bytes
	nrf_register_map.SETUP_AW.SETUP_AW_Bits.AW = 3;
	NRF_WriteRegister(NRF_REG_SETUP_AW, nrf_register_map.SETUP_AW.SETUP_AW_Byte);

	//auto retransmission
	nrf_register_map.SETUP_RETR.SETUP_RETR_Byte = 0;
	NRF_WriteRegister(NRF_REG_SETUP_RETR, nrf_register_map.SETUP_RETR.SETUP_RETR_Byte);

	//setting channel
	nrf_register_map.RF_CH.RF_CH_Bits.RF_CH = 2; //3rd channel
	NRF_WriteRegister(NRF_REG_RF_CH, nrf_register_map.RF_CH.RF_CH_Byte);

	//enabling LNA, Tx power, air data rate
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.DONT_CARE = 1;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.CONT_WAVE = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_DR_LOW = 1;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.PLL_LOCK = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_DR_HIGH = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_PWR = 0b11;
	NRF_WriteRegister(NRF_REG_RF_SETUP, nrf_register_map.RF_SETUP.RF_SETUP_Byte);

	//setting address
	nrf_register_map.TX_ADDR[0] = 0x12;
	nrf_register_map.TX_ADDR[1] = 0x34;
	nrf_register_map.TX_ADDR[2] = 0x56;
	nrf_register_map.TX_ADDR[3] = 0x78;
	nrf_register_map.TX_ADDR[4] = 0x9A;
	NRF_SetTxAddress(nrf_register_map.TX_ADDR, 5);

	//no dynamic payload length
	nrf_register_map.DYNPD.DYNPD_Byte = 0;
	NRF_WriteRegister(NRF_REG_DYNPD, nrf_register_map.DYNPD.DYNPD_Byte);

	nrf_register_map.FEATURE.FEATURE_Byte = 0;
	NRF_WriteRegister(NRF_REG_FEATURE, nrf_register_map.FEATURE.FEATURE_Byte);
}

void NRF_Rx_Init(void)
{
	NRF_CS_HIGH;
	NRF_CE_LOW;

	//waking up
	nrf_register_map.CONFIG.CONFIG_Byte = 0;		//setting byte as 0
	nrf_register_map.CONFIG.CONFIG_Bits.PWR_UP = 1; //powering up
	NRF_WriteRegister(NRF_REG_CONFIG, nrf_register_map.CONFIG.CONFIG_Byte);
	NRF_DELAY(20);										 //wakeup delay 20ms
	
	nrf_register_map.CONFIG.CONFIG_Bits.CRCO = 0;		 //CRC 1 byte
	nrf_register_map.CONFIG.CONFIG_Bits.EN_CRC = 1;		 //disable CRC
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_MAX_RT = 1; //do not set IRQ on MAX_RT
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_TX_DS = 1;	 //do not set IRQ on TX_DS
	nrf_register_map.CONFIG.CONFIG_Bits.MASK_RX_DR = 0;	 //set IRQ on RX_DR
	nrf_register_map.CONFIG.CONFIG_Bits.PRIM_RX = 1;	 //set as RX
	NRF_WriteRegister(NRF_REG_CONFIG, nrf_register_map.CONFIG.CONFIG_Byte);

	//no auto ACK
	nrf_register_map.EN_AA.EN_AA_Byte = 0;
	NRF_WriteRegister(NRF_REG_EN_AA, nrf_register_map.EN_AA.EN_AA_Byte);

	//no RX enable
	nrf_register_map.EN_RXADDR.EN_RXADDR_Bits.ERX_P0 = 1;
	NRF_WriteRegister(NRF_REG_EN_RXADDR, nrf_register_map.EN_RXADDR.EN_RXADDR_Byte);

	//setting address width as 5 bytes
	nrf_register_map.SETUP_AW.SETUP_AW_Bits.AW = 3;
	NRF_WriteRegister(NRF_REG_SETUP_AW, nrf_register_map.SETUP_AW.SETUP_AW_Byte);

	//auto retransmission
	nrf_register_map.SETUP_RETR.SETUP_RETR_Byte = 0;
	NRF_WriteRegister(NRF_REG_SETUP_RETR, nrf_register_map.SETUP_RETR.SETUP_RETR_Byte);

	//setting channel
	nrf_register_map.RF_CH.RF_CH_Bits.RF_CH = 2; //3rd channel
	NRF_WriteRegister(NRF_REG_RF_CH, nrf_register_map.RF_CH.RF_CH_Byte);

	//enabling LNA, Tx power, air data rate
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.DONT_CARE = 1;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.CONT_WAVE = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_DR_LOW = 1;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.PLL_LOCK = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_DR_HIGH = 0;
	nrf_register_map.RF_SETUP.RF_SETUP_Bits.RF_PWR = 0b11;
	NRF_WriteRegister(NRF_REG_RF_SETUP, nrf_register_map.RF_SETUP.RF_SETUP_Byte);

	//setting address
	nrf_register_map.RX_ADDR_P0[0] = 0x12;
	nrf_register_map.RX_ADDR_P0[1] = 0x34;
	nrf_register_map.RX_ADDR_P0[2] = 0x56;
	nrf_register_map.RX_ADDR_P0[3] = 0x78;
	nrf_register_map.RX_ADDR_P0[4] = 0x9A;
	NRF_SetRxAddress(nrf_register_map.RX_ADDR_P0, 5);

	//payload length
	nrf_register_map.RX_PW_P0.RX_PW_P0_Bits.RX_PW_P0 = 0;
	NRF_WriteRegister(NRF_REG_RX_PW_P0, nrf_register_map.RX_PW_P0.RX_PW_P0_Byte);

	//no dynamic payload length
	nrf_register_map.DYNPD.DYNPD_Byte = 0;
	NRF_WriteRegister(NRF_REG_DYNPD, nrf_register_map.DYNPD.DYNPD_Byte);

	nrf_register_map.FEATURE.FEATURE_Byte = 0;
	NRF_WriteRegister(NRF_REG_FEATURE, nrf_register_map.FEATURE.FEATURE_Byte);
}

void NRF_SetPayloadLength(uint8_t payload_length, uint8_t rx_pipe_number)
{
	if (rx_pipe_number >= 0 && rx_pipe_number <= 5)
	{
		NRF_WriteRegister(NRF_REG_RX_PW_P0 + rx_pipe_number, payload_length);
	}
}