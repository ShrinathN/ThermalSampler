#ifndef NRF_DRIVER_H
#define NRF_DRIVER_H

//MCU related include
#include "main.h"

//lines macros
#define NRF_DELAY(X) HAL_Delay(X)
#define NRF_CS_LOW HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_RESET)
#define NRF_CS_HIGH HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET)
#define NRF_CE_LOW HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define NRF_CE_HIGH HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)

//register map
typedef struct
{
	union
	{
		struct
		{
			uint8_t PRIM_RX : 1;
			uint8_t PWR_UP : 1;
			uint8_t CRCO : 1;
			uint8_t EN_CRC : 1;
			uint8_t MASK_MAX_RT : 1;
			uint8_t MASK_TX_DS : 1;
			uint8_t MASK_RX_DR : 1;
			uint8_t Reserved : 1;
		} CONFIG_Bits;
		uint8_t CONFIG_Byte;
	} CONFIG;

	union
	{
		struct
		{
			uint8_t ENAA_P0 : 1;
			uint8_t ENAA_P1 : 1;
			uint8_t ENAA_P2 : 1;
			uint8_t ENAA_P3 : 1;
			uint8_t ENAA_P4 : 1;
			uint8_t ENAA_P5 : 1;
			uint8_t Reserved : 2;
		} EN_AA_Bits;
		uint8_t EN_AA_Byte;
	} EN_AA;

	union
	{
		struct
		{
			uint8_t ERX_P0 : 1;
			uint8_t ERX_P1 : 1;
			uint8_t ERX_P2 : 1;
			uint8_t ERX_P3 : 1;
			uint8_t ERX_P4 : 1;
			uint8_t ERX_P5 : 1;
			uint8_t Reserved : 2;
		} EN_RXADDR_Bits;
		uint8_t EN_RXADDR_Byte;
	} EN_RXADDR;

	union
	{
		struct
		{
			uint8_t AW : 4;
			uint8_t Reserved : 4;
		} SETUP_AW_Bits;
		uint8_t SETUP_AW_Byte;
	} SETUP_AW;

	union
	{
		struct
		{
			uint8_t ARC : 4;
			uint8_t ARD : 4;
		} SETUP_RETR_Bits;
		uint8_t SETUP_RETR_Byte;
	} SETUP_RETR;

	union
	{
		struct
		{
			uint8_t RF_CH : 7;
			uint8_t Reserved : 1;
		} RF_CH_Bits;
		uint8_t RF_CH_Byte;
	} RF_CH;

	union
	{
		struct
		{
			uint8_t DONT_CARE : 1;
			uint8_t RF_PWR : 2;
			uint8_t RF_DR_HIGH : 1;
			uint8_t PLL_LOCK : 1;
			uint8_t RF_DR_LOW : 1;
			uint8_t Reserved : 1;
			uint8_t CONT_WAVE : 1;
		} RF_SETUP_Bits;
		uint8_t RF_SETUP_Byte;
	} RF_SETUP;

	union
	{
		struct
		{
			uint8_t TX_FULL : 1;
			uint8_t RX_P_NO : 3;
			uint8_t MAX_RT : 1;
			uint8_t TX_DS : 1;
			uint8_t RX_DR : 1;
			uint8_t Reserved : 1;
		} STATUS_Bits;
		uint8_t STATUS_Byte;
	} STATUS;

	union
	{
		struct
		{
			uint8_t ARC_CNT : 4;
			uint8_t PLOS_CNT : 4;
		} OBSERVE_TX_Bits;
		uint8_t OBSERVE_TX_Byte;
	} OBSERVE_TX;

	union
	{
		struct
		{
			uint8_t CD : 1;
		} CD_Bits;
		uint8_t CD_Byte;
	} CD;

	uint8_t RX_ADDR_P0[5];
	uint8_t RX_ADDR_P1[5];
	uint8_t RX_ADDR_P2;
	uint8_t RX_ADDR_P3;
	uint8_t RX_ADDR_P4;
	uint8_t RX_ADDR_P5;
	uint8_t TX_ADDR[5];

	union
	{
		struct
		{
			uint8_t RX_PW_P0 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P0_Bits;
		uint8_t RX_PW_P0_Byte;
	} RX_PW_P0;

	union
	{
		struct
		{
			uint8_t RX_PW_P1 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P1_Bits;
		uint8_t RX_PW_P1_Byte;
	} RX_PW_P1;

	union
	{
		struct
		{
			uint8_t RX_PW_P2 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P2_Bits;
		uint8_t RX_PW_P2_Byte;
	} RX_PW_P2;

	union
	{
		struct
		{
			uint8_t RX_PW_P3 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P3_Bits;
		uint8_t RX_PW_P3_Byte;
	} RX_PW_P3;

	union
	{
		struct
		{
			uint8_t RX_PW_P4 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P4_Bits;
		uint8_t RX_PW_P4_Byte;
	} RX_PW_P4;

	union
	{
		struct
		{
			uint8_t RX_PW_P5 : 6;
			uint8_t Reserved : 2;
		} RX_PW_P5_Bits;
		uint8_t RX_PW_P5_Byte;
	} RX_PW_P5;

	union
	{
		struct
		{
			uint8_t RX_EMPTY : 1;
			uint8_t RX_FULL : 1;
			uint8_t Reserved : 2;
			uint8_t TX_EMPTY : 1;
			uint8_t TX_FULL : 1;
			uint8_t TX_REUSE : 1;
			uint8_t Reserved2 : 1;

		} FIFO_STATUS_Bits;
		uint8_t FIFO_STATUS_Byte;
	} FIFO_STATUS;

	union
	{
		struct
		{
			uint8_t DPL_P0 : 1;
			uint8_t DPL_P1 : 1;
			uint8_t DPL_P2 : 1;
			uint8_t DPL_P3 : 1;
			uint8_t DPL_P4 : 1;
			uint8_t DPL_P5 : 1;
			uint8_t Reserved : 2;

		} DYNPD_Bits;
		uint8_t DYNPD_Byte;
	} DYNPD;

	union
	{
		struct
		{
			uint8_t EN_DYN_ACK : 1;
			uint8_t EN_ACK_PAY : 1;
			uint8_t EN_DPL : 1;
			uint8_t Reserved : 5;
		} FEATURE_Bits;
		uint8_t FEATURE_Byte;
	} FEATURE;
} NRF24L01_Register_Map;

//registers
enum
{
	NRF_REG_CONFIG = 0x00,
	NRF_REG_EN_AA,
	NRF_REG_EN_RXADDR,
	NRF_REG_SETUP_AW,
	NRF_REG_SETUP_RETR,
	NRF_REG_RF_CH,
	NRF_REG_RF_SETUP,
	NRF_REG_STATUS,
	NRF_REG_OBSERVE_TX,
	NRF_REG_CD,
	NRF_REG_RX_ADDR_P0,
	NRF_REG_RX_ADDR_P1,
	NRF_REG_RX_ADDR_P2,
	NRF_REG_RX_ADDR_P3,
	NRF_REG_RX_ADDR_P4,
	NRF_REG_RX_ADDR_P5,
	NRF_REG_TX_ADDR,
	NRF_REG_RX_PW_P0,
	NRF_REG_RX_PW_P1,
	NRF_REG_RX_PW_P2,
	NRF_REG_RX_PW_P3,
	NRF_REG_RX_PW_P4,
	NRF_REG_RX_PW_P5,
	NRF_REG_FIFO_STATUS,
	NRF_REG_DYNPD = 0x1C,
	NRF_REG_FEATURE
};

enum
{
	NRF_PIPE_P0,
	NRF_PIPE_P1,
	NRF_PIPE_P2,
	NRF_PIPE_P3,
	NRF_PIPE_P4,
	NRF_PIPE_P5
};

//commands
#define NRF_COMMAND_R_REGISTER (0x00)
#define NRF_COMMAND_W_REGISTER (0x20)
#define NRF_COMMAND_R_RX_PAYLOAD (0x61)
#define NRF_COMMAND_W_TX_PAYLOAD (0xA0)
#define NRF_COMMAND_FLUSH_TX (0xE1)
#define NRF_COMMAND_FLUSH_RX (0xE2)
#define NRF_COMMAND_REUSE_TX_PL (0xE3)
#define NRF_COMMAND_ACTIVATE (0x50)
#define NRF_COMMAND_R_RX_PL_WID (0x60)
#define NRF_COMMAND_W_ACK_PAYLOAD (0xA8)
#define NRF_COMMAND_W_TX_PAYLOAD_NO_ACK (0xB0)
#define NRF_COMMAND_NOP (0xFF)

uint8_t NRF_ReadRegister(uint8_t register_addr);
void NRF_WriteRegister(uint8_t register_addr, uint8_t data_to_write);
void NRF_Tx_Init(void);
void NRF_Rx_Init(void);
void NRF_SetTxAddress(uint8_t *addr, uint8_t len);
void NRF_SetRxAddress(uint8_t *addr, uint8_t len);
void NRF_FlushTxBuffer(void);
void NRF_FlushRxBuffer(void);
void NRF_Execute(void);
void NRF_WriteTxBuffer(uint8_t *data, uint8_t len, uint8_t pipe);
void NRF_Execute_Rx(void);
void NRF_ReadRxBuffer(uint8_t *buffer, uint8_t len);
void NRF_SetPayloadLength(uint8_t payload_length, uint8_t rx_pipe_number);

#endif