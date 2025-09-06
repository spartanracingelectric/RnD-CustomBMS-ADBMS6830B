#include <stdint.h>
#include "main.h"
#include "adbms.h"
#include <stdio.h>

/* ADBMS Register addresses */

#define WRCFGA      0x0001 // Write Configuration Register Group A
#define WRCFGB      0x0024 // Write Configuration Register Group B
#define RDCFGA      0x0002 // Read Configuration Register Group A
#define RDCFGB      0x0026 // Read Configuration Register Group B
#define RDCVA       0x0004 // Read Cell Voltage Register Group A
#define RDCVB       0x0006 // Read Cell Voltage Register Group B
#define RDCVC       0x0008 // Read Cell Voltage Register Group C
#define RDCVD       0x000A // Read Cell Voltage Register Group D
#define RDCVE       0x0009 // Read Cell Voltage Register Group E
#define RDCVF       0x000B // Read Cell Voltage Register Group F
#define RDACA       0x0044 // Read Averaged Cell Voltage Register Group A
#define RDACB       0x0046 // Read Averaged Cell Voltage Register Group B
#define RDACC       0x0048 // Read Averaged Cell Voltage Register Group C
#define RDACD       0x004A // Read Averaged Cell Voltage Register Group D
#define RDACE       0x0049 // Read Averaged Cell Voltage Register Group E
#define RDACF       0x004B // Read Averaged Cell Voltage Register Group F
#define RDSVA       0x0003 // Read S Voltage Register Group A
#define RDSVB       0x0005 // Read S Voltage Register Group B
#define RDSVC       0x0007 // Read S Voltage Register Group C
#define RDSVD       0x000D // Read S Voltage Register Group D
#define RDSVE       0x000E // Read S Voltage Register Group E
#define RDSVF       0x000F // Read S Voltage Register Group F
#define RDFCA       0x0012 // Read Filter Cell Voltage Register Group A
#define RDFCB       0x0013 // Read Filter Cell Voltage Register Group B
#define RDFCC       0x0014 // Read Filter Cell Voltage Register Group C
#define RDFCD       0x0015 // Read Filter Cell Voltage Register Group D
#define RDFCE       0x0016 // Read Filter Cell Voltage Register Group E
#define RDFCF       0x0017 // Read Filter Cell Voltage Register Group F
#define RDAUXA      0x0019 // Read Auxiliary Register Group A
#define RDAUXB      0x001A // Read Auxiliary Register Group B
#define RDAUXC      0x001B // Read Auxiliary Register Group C
#define RDAUXD      0x001F // Read Auxiliary Register Group D
#define RDRAXA      0x001C // Read Redundant Auxiliary Register Group A
#define RDRAXB      0x001D // Read Redundant Auxiliary Register Group B
#define RDRAXC      0x001E // Read Auxiliary Redundant Register Group C
#define RDRAXD      0x0025 // Read Auxiliary Redundant Register Group D
#define RDSTATA     0x0030 // Read Status Register Group A
#define RDSTATB     0x0031 // Read Status Register Group B
#define RDSTATC     0x0032 // Read Status Register Group C
#define RDSTATD     0x0033 // Read Status Register Group D
#define RDSTATE     0x0034 // Read Status Register Group E
#define WRPWMA      0x0020 // Write PWM Register Group A
#define RDPWMA      0x0022 // Read PWM Register Group A
#define WRPWMB      0x0021 // Write PWM Register Group B
#define RDPWMB      0x0023 // Read PWM Register Group B
#define CMDIS       0x0040 // LPCM Disable
#define CMEN        0x0041 // LPCM Enable
#define CMHB2       0x0043 // LPCM Heartbeat
#define WRCMCFG     0x0058 // Write LPCM Configuration Register
#define RDCMCFG     0x0059 // Read LPCM Configuration Register
#define WRCMCELLT   0x005A // Write LPCM Cell Threshold
#define RDCMCELLT   0x005B // Read LPCM Cell Threshold
#define WRCMGPIOT   0x005C // Write LPCM GPIO Threshold
#define RDCMGPIOT   0x005D // Read LPCM GPIO Threshold
#define CLRCMFLAG   0x005E // Clear LPCM Flags
#define RDCMFLAG    0x005F // Read LPCM Flags
#define ADCV        0x0260 // Start Cell Voltage ADC Conversion and Poll Status
#define ADSV        0x0168 // Start S-ADC Conversion and Poll Status
#define ADAX        0x0410 // Start AUX ADC Conversions and Poll Status
#define ADAX2       0x0400 // Start AUX2 ADC Conversions and Poll Status
#define CLRCELL     0x0711 // Clear Cell Voltage Register Groups
#define CLRFC       0x0714 // Clear Filtered Cell Voltage Register Groups
#define CLRAUX      0x0712 // Clear Auxiliary Register Groups
#define CLRSPIN     0x0716 // Clear S-Voltage Register Groups
#define CLRFLAG     0x0717 // Clear Flags
#define CLOVUV      0x0715 // Clear OVUV
#define WRCOMM      0x0721 // Write COMM Register Group
#define RDCOMM      0x0722 // Read COMM Register Group
#define STCOMM      0x0723 // Start I2C/SPI Communication
#define MUTE        0x0028 // Mute Discharge
#define UNMUTE      0x0029 // Unmute Discharge
#define RDSID       0x002C // Read Serial ID Register Group
#define RSTCC       0x002E // Reset Command Counter
#define SNAP        0x002D // Snapshot
#define UNSNAP      0x002F // Release Snapshot
#define SRST        0x0027 // Soft Reset
#define ULRR        0x0038 // Unlock Retention Register
#define WRRR        0x0039 // Write Retention Registers
#define RDRR        0x003A // Read Retention Registers

#define ADV         0x0430
#define ADX         0x0530
#define CLRO        0x0713

/* END ADBMS Register addresses */

#define NUM_OF_DEVICES 1

#define COMMAND_SIZE_BYTES 2
#define PEC_SIZE_BYTES 2
#define COMMAND_PACKET_SIZE_BYTES (COMMAND_SIZE_BYTES + PEC_SIZE_BYTES)

#define PEC_INITIAL_SEED 0x0010
#define PEC_CMD_SIZE 16
#define PEC_LUT_SIZE 256

#define BITS_IN_BYTE 8

#define COMMAND_SIZE_BYTES 2

#define REGISTER_SIZE_BYTES      6
#define REGISTER_PACKET_LENGTH   (REGISTER_SIZE_BYTES + PEC_SIZE_BYTES)

#define NUM_CELL_VOLTAGE_TYPES 3
#define NUM_CELLV_REGISTERS 6


extern SPI_HandleTypeDef hspi1;

uint8_t rxBuffer[256];

static const uint16_t cellVoltageCode[NUM_CELLV_REGISTERS] =
{
    RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF

};

const uint16_t commandPecTable[PEC_LUT_SIZE] =
{
    0x0000, 0x8B32, 0x9D56, 0x1664, 0xB19E, 0x3AAC, 0x2CC8, 0xA7FA, 0xE80E, 0x633C, 0x7558, 0xFE6A, 0x5990, 0xD2A2, 0xC4C6, 0x4FF4,
    0x5B2E, 0xD01C, 0xC678, 0x4D4A, 0xEAB0, 0x6182, 0x77E6, 0xFCD4, 0xB320, 0x3812, 0x2E76, 0xA544, 0x02BE, 0x898C, 0x9FE8, 0x14DA,
    0xB65C, 0x3D6E, 0x2B0A, 0xA038, 0x07C2, 0x8CF0, 0x9A94, 0x11A6, 0x5E52, 0xD560, 0xC304, 0x4836, 0xEFCC, 0x64FE, 0x729A, 0xF9A8,
    0xED72, 0x6640, 0x7024, 0xFB16, 0x5CEC, 0xD7DE, 0xC1BA, 0x4A88, 0x057C, 0x8E4E, 0x982A, 0x1318, 0xB4E2, 0x3FD0, 0x29B4, 0xA286,
    0xE78A, 0x6CB8, 0x7ADC, 0xF1EE, 0x5614, 0xDD26, 0xCB42, 0x4070, 0x0F84, 0x84B6, 0x92D2, 0x19E0, 0xBE1A, 0x3528, 0x234C, 0xA87E,
    0xBCA4, 0x3796, 0x21F2, 0xAAC0, 0x0D3A, 0x8608, 0x906C, 0x1B5E, 0x54AA, 0xDF98, 0xC9FC, 0x42CE, 0xE534, 0x6E06, 0x7862, 0xF350,
    0x51D6, 0xDAE4, 0xCC80, 0x47B2, 0xE048, 0x6B7A, 0x7D1E, 0xF62C, 0xB9D8, 0x32EA, 0x248E, 0xAFBC, 0x0846, 0x8374, 0x9510, 0x1E22,
    0x0AF8, 0x81CA, 0x97AE, 0x1C9C, 0xBB66, 0x3054, 0x2630, 0xAD02, 0xE2F6, 0x69C4, 0x7FA0, 0xF492, 0x5368, 0xD85A, 0xCE3E, 0x450C,
    0x4426, 0xCF14, 0xD970, 0x5242, 0xF5B8, 0x7E8A, 0x68EE, 0xE3DC, 0xAC28, 0x271A, 0x317E, 0xBA4C, 0x1DB6, 0x9684, 0x80E0, 0x0BD2,
    0x1F08, 0x943A, 0x825E, 0x096C, 0xAE96, 0x25A4, 0x33C0, 0xB8F2, 0xF706, 0x7C34, 0x6A50, 0xE162, 0x4698, 0xCDAA, 0xDBCE, 0x50FC,
    0xF27A, 0x7948, 0x6F2C, 0xE41E, 0x43E4, 0xC8D6, 0xDEB2, 0x5580, 0x1A74, 0x9146, 0x8722, 0x0C10, 0xABEA, 0x20D8, 0x36BC, 0xBD8E,
    0xA954, 0x2266, 0x3402, 0xBF30, 0x18CA, 0x93F8, 0x859C, 0x0EAE, 0x415A, 0xCA68, 0xDC0C, 0x573E, 0xF0C4, 0x7BF6, 0x6D92, 0xE6A0,
    0xA3AC, 0x289E, 0x3EFA, 0xB5C8, 0x1232, 0x9900, 0x8F64, 0x0456, 0x4BA2, 0xC090, 0xD6F4, 0x5DC6, 0xFA3C, 0x710E, 0x676A, 0xEC58,
    0xF882, 0x73B0, 0x65D4, 0xEEE6, 0x491C, 0xC22E, 0xD44A, 0x5F78, 0x108C, 0x9BBE, 0x8DDA, 0x06E8, 0xA112, 0x2A20, 0x3C44, 0xB776,
    0x15F0, 0x9EC2, 0x88A6, 0x0394, 0xA46E, 0x2F5C, 0x3938, 0xB20A, 0xFDFE, 0x76CC, 0x60A8, 0xEB9A, 0x4C60, 0xC752, 0xD136, 0x5A04,
    0x4EDE, 0xC5EC, 0xD388, 0x58BA, 0xFF40, 0x7472, 0x6216, 0xE924, 0xA6D0, 0x2DE2, 0x3B86, 0xB0B4, 0x174E, 0x9C7C, 0x8A18, 0x012A
};

// CRC lookup table for data PEC
const uint16_t dataPecTable[PEC_LUT_SIZE] =
{
    0x0000, 0x048F, 0x091E, 0x0D91, 0x123C, 0x16B3, 0x1B22, 0x1FAD, 0x24F7, 0x2078, 0x2DE9, 0x2966, 0x36CB, 0x3244, 0x3FD5, 0x3B5A,
    0x49EE, 0x4D61, 0x40F0, 0x447F, 0x5BD2, 0x5F5D, 0x52CC, 0x5643, 0x6D19, 0x6996, 0x6407, 0x6088, 0x7F25, 0x7BAA, 0x763B, 0x72B4,
    0x93DC, 0x9753, 0x9AC2, 0x9E4D, 0x81E0, 0x856F, 0x88FE, 0x8C71, 0xB72B, 0xB3A4, 0xBE35, 0xBABA, 0xA517, 0xA198, 0xAC09, 0xA886,
    0xDA32, 0xDEBD, 0xD32C, 0xD7A3, 0xC80E, 0xCC81, 0xC110, 0xC59F, 0xFEC5, 0xFA4A, 0xF7DB, 0xF354, 0xECF9, 0xE876, 0xE5E7, 0xE168,
    0x2737, 0x23B8, 0x2E29, 0x2AA6, 0x350B, 0x3184, 0x3C15, 0x389A, 0x03C0, 0x074F, 0x0ADE, 0x0E51, 0x11FC, 0x1573, 0x18E2, 0x1C6D,
    0x6ED9, 0x6A56, 0x67C7, 0x6348, 0x7CE5, 0x786A, 0x75FB, 0x7174, 0x4A2E, 0x4EA1, 0x4330, 0x47BF, 0x5812, 0x5C9D, 0x510C, 0x5583,
    0xB4EB, 0xB064, 0xBDF5, 0xB97A, 0xA6D7, 0xA258, 0xAFC9, 0xAB46, 0x901C, 0x9493, 0x9902, 0x9D8D, 0x8220, 0x86AF, 0x8B3E, 0x8FB1,
    0xFD05, 0xF98A, 0xF41B, 0xF094, 0xEF39, 0xEBB6, 0xE627, 0xE2A8, 0xD9F2, 0xDD7D, 0xD0EC, 0xD463, 0xCBCE, 0xCF41, 0xC2D0, 0xC65F,
    0x4EE1, 0x4A6E, 0x47FF, 0x4370, 0x5CDD, 0x5852, 0x55C3, 0x514C, 0x6A16, 0x6E99, 0x6308, 0x6787, 0x782A, 0x7CA5, 0x7134, 0x75BB,
    0x070F, 0x0380, 0x0E11, 0x0A9E, 0x1533, 0x11BC, 0x1C2D, 0x18A2, 0x23F8, 0x2777, 0x2AE6, 0x2E69, 0x31C4, 0x354B, 0x38DA, 0x3C55,
    0xDD3D, 0xD9B2, 0xD423, 0xD0AC, 0xCF01, 0xCB8E, 0xC61F, 0xC290, 0xF9CA, 0xFD45, 0xF0D4, 0xF45B, 0xEBF6, 0xEF79, 0xE2E8, 0xE667,
    0x94D3, 0x905C, 0x9DCD, 0x9942, 0x86EF, 0x8260, 0x8FF1, 0x8B7E, 0xB024, 0xB4AB, 0xB93A, 0xBDB5, 0xA218, 0xA697, 0xAB06, 0xAF89,
    0x69D6, 0x6D59, 0x60C8, 0x6447, 0x7BEA, 0x7F65, 0x72F4, 0x767B, 0x4D21, 0x49AE, 0x443F, 0x40B0, 0x5F1D, 0x5B92, 0x5603, 0x528C,
    0x2038, 0x24B7, 0x2926, 0x2DA9, 0x3204, 0x368B, 0x3B1A, 0x3F95, 0x04CF, 0x0040, 0x0DD1, 0x095E, 0x16F3, 0x127C, 0x1FED, 0x1B62,
    0xFA0A, 0xFE85, 0xF314, 0xF79B, 0xE836, 0xECB9, 0xE128, 0xE5A7, 0xDEFD, 0xDA72, 0xD7E3, 0xD36C, 0xCCC1, 0xC84E, 0xC5DF, 0xC150,
    0xB3E4, 0xB76B, 0xBAFA, 0xBE75, 0xA1D8, 0xA557, 0xA8C6, 0xAC49, 0x9713, 0x939C, 0x9E0D, 0x9A82, 0x852F, 0x81A0, 0x8C31, 0X88BE
};

void BMS_csLow()
{
	HAL_GPIO_WritePin(PORTA_CS_GPIO_Port, PORTA_CS_Pin, GPIO_PIN_RESET);
}

void BMS_csHigh()
{
	HAL_GPIO_WritePin(PORTA_CS_GPIO_Port, PORTA_CS_Pin, GPIO_PIN_SET);
}

void BMS_wakeUpChain()
{
	for(uint32_t i = 0; i < NUM_OF_DEVICES + 1; i++)
	{
		BMS_csLow();
		HAL_Delay(1); //only need microseconds delay
		BMS_csHigh();
		HAL_Delay(1);
	}
}

void BMS_printRxBuffer(uint32_t numBytes)
{
	printf("RX Buffer (%lu bytes): ", numBytes);
	for(uint32_t i = 0; i < numBytes; i++) {
		printf("%02X ", rxBuffer[i]);
	}
	printf("\n");

}

uint16_t BMS_calculateCommandPEC(uint8_t *packet, uint32_t numBytes) {

	uint16_t pec = PEC_INITIAL_SEED;

	for (uint32_t i = 0; i < numBytes; i++) {
		uint8_t index = (uint8_t)((pec >> (PEC_CMD_SIZE - BITS_IN_BYTE)) ^ packet[i]);
		pec = ((pec << BITS_IN_BYTE) ^ (uint16_t)(commandPecTable[index]));
	}

	return pec;
}

void BMS_sendCommand(uint16_t command)
{
	uint8_t txBuffer[COMMAND_PACKET_SIZE_BYTES]; //2 CMD Bytes + 2 PEC Bytes

	txBuffer[0] = (uint8_t)(command >> BITS_IN_BYTE);
	txBuffer[1] = (uint8_t)(command);

    uint16_t commandPEC = BMS_calculateCommandPEC(txBuffer, COMMAND_SIZE_BYTES);
    txBuffer[2] = (uint8_t)(commandPEC >> BITS_IN_BYTE);
    txBuffer[3] = (uint8_t)(commandPEC);

    BMS_csLow();
    HAL_SPI_Transmit(&hspi1, txBuffer, COMMAND_PACKET_SIZE_BYTES, HAL_MAX_DELAY);
    BMS_csHigh();

}

void BMS_startCellConversions(ADC_MODE_REDUNDANT_E redundantMode, ADC_MODE_CONTINOUS_E continousMode, ADC_MODE_DISCHARGE_E dischargeMode, ADC_MODE_FILTER_E filterMode, ADC_MODE_CELL_OPEN_WIRE_E openWireMode)
{
	return BMS_sendCommand((uint16_t)(ADCV | redundantMode | continousMode | dischargeMode | filterMode | openWireMode));
}

void BMS_readRegister(uint16_t command, uint32_t numDevs)
{
    uint32_t packetLength = COMMAND_PACKET_SIZE_BYTES + (numDevs * REGISTER_PACKET_LENGTH);

    uint8_t txBuffer[packetLength];

    txBuffer[0] = (uint8_t)(command >> BITS_IN_BYTE);
    txBuffer[1] = (uint8_t)(command);

    uint16_t commandPEC = BMS_calculateCommandPEC(txBuffer, COMMAND_SIZE_BYTES);
    txBuffer[2] = (uint8_t)(commandPEC >> BITS_IN_BYTE);
    txBuffer[3] = (uint8_t)(commandPEC);

    BMS_csLow();
    HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, packetLength, HAL_MAX_DELAY);
    BMS_csHigh();
}


void BMS_parseVoltages()
{
	return;
}

void BMS_readVoltages()
{
	for (uint32_t i = 0; i < NUM_CELLV_REGISTERS; i++) {
		BMS_readRegister(cellVoltageCode[i], NUM_OF_DEVICES);
	    BMS_printRxBuffer(100);
		BMS_parseVoltages();
	}
}

void BMS_readSerialID()
{
	BMS_readRegister(RDSID, NUM_OF_DEVICES);
    BMS_printRxBuffer(100);

}




