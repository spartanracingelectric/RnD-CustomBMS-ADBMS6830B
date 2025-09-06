typedef enum
{
    NON_REDUNDANT_MODE = 0,
    REDUNDANT_MODE = (1 << 8)
} ADC_MODE_REDUNDANT_E;

typedef enum
{
    SINGLE_SHOT_MODE = 0,
    CONTINOUS_MODE = (1 << 7)
} ADC_MODE_CONTINOUS_E;

typedef enum
{
    DISCHARGE_DISABLED = 0,
    DISCHARGE_PERMITTED = (1 << 4)
} ADC_MODE_DISCHARGE_E;

typedef enum
{
    NO_FILTER_RESET = 0,
    FILTER_RESET = (1 << 2)
} ADC_MODE_FILTER_E;

typedef enum
{
    CELL_OPEN_WIRE_DISABLED = 0,
    CELL_OPEN_WIRE_EVEN,
    CELL_OPEN_WIRE_ODD
} ADC_MODE_CELL_OPEN_WIRE_E;

typedef enum
{
    AUX_OPEN_WIRE_DISABLED = 0,
    AUX_OPEN_WIRE_PULL_DOWN = (1 << 8),
    AUX_OPEN_WIRE_PULL_UP = ((1 << 8) | (1 << 7))
} ADC_MODE_AUX_OPEN_WIRE_E;

typedef enum
{
    PACK_OPEN_WIRE_DISABLED = 0,
    PACK_OPEN_WIRE_POSITIVE = (1 << 6),
    PACK_OPEN_WIRE_NEGATIVE = (1 << 7)
} ADC_MODE_PACK_OPEN_WIRE_E;

void BMS_wakeUpChain(void);

void BMS_csLow(void);

void BMS_csHigh(void);

void BMS_printRxBuffer(uint32_t numBytes);

uint16_t BMS_calculateCommandPEC(uint8_t *packet, uint32_t numBytes);

void BMS_sendCommand(uint16_t command);

void BMS_startCellConversions(ADC_MODE_REDUNDANT_E redundantMode, ADC_MODE_CONTINOUS_E continousMode, ADC_MODE_DISCHARGE_E dischargeMode, ADC_MODE_FILTER_E filterMode, ADC_MODE_CELL_OPEN_WIRE_E openWireMode);

void BMS_readRegister(uint16_t command, uint32_t numDevs);

void BMS_parseVoltages(void);

void BMS_readVoltages(void);

void BMS_readSerialID(void);






