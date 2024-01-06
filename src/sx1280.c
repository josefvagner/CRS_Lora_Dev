/* FreeRTOS Includes */
#include "FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include "pico/stdlib.h"
#include "hardware/adc.h"/*"../../pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h"*/
#include "hardware/spi.h"/*"../../pico-sdk/src/rp2_common/hardware_spi/include/hardware/spi.h"*/
#include "hardware/gpio.h"

#include "sx1280.h"

void spiSend(buff_t buff) {
    gpio_put( CS_PIN, 0 );
    spi_write_blocking( spi1, buff.data, buff.len*sizeof(uint8_t));
    gpio_put( CS_PIN, 1 );

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
    }
}

buff_t spiSendRecv(buff_t buff) {
    uint8_t response[buff.len] = { 0 };
    buff_t ret;
    ret.data = response;
    ret.len = buff.len;

    gpio_put( CS_PIN, 0 );
    spi_write_read_blocking( spi1, buff.data, ret.data, buff.len*sizeof(uint8_t));
    gpio_put( CS_PIN, 1 );

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
    }

    return ret;
}

void SetPacketLen(uint8_t packetLen) {
    uint8_t data[8] = {SET_PACKET_PARAMS, PREAMBLE_LENGTH, HEADER_TYPE, packetLen, CYCLICAL_REDUNDANCY_CHECK, CHIRP_INVERT, 0x00, 0x00};
    buff_t buff = {data, 8}
    spiSend(buff);
}

uint8_t GetStatus() {
    uint8_t data[1] = { GET_STATUS };
    buff_t buff = {data, 1};
    buff_t ret = spiSendRecv(buff);
    return *(ret.data);
}

void WriteRegister(uint16_t addr, buff_t buff) {
    
}
buff_t ReadRegister(uint16_t addr, size_t n);
void WriteBuffer(buff_t buff);
buff_t ReadBuffer(uint8_t n);
void SetSleep(uint8_t sleepConfig);
void SetStandby(uint8_t standbyConfig);
void SetFs();
void SetTx(uint8_t periodBase, uint16_t periodBaseCount);
void SetRx(uint8_t periodBase, uint16_t periodBaseCount);
void SetRxDutyCycle(uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount);
void SetCad();
void SetTxContinuousWave();
void SetTxContinuousPreamble();
void SetPacketType(uint8_t packetType);
uint8_t GetPacketType();
void SetRfFrequency(uint8_t rfFrequency[3]);
void SetTxParams(uint8_t power, uint8_t rampTime);
void SetCadParams(uint8_t cadSymbolNum);
void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void SetModulationParams(uint8_t modParam[3]);
// void SetPacketParams(uint8_t packetParam[7]);
buff_t GetRxBufferStatus();
buff_t GetPacketStatus();
uint8_t GetRssiInst();
void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
uint16_t GetIrqStatus();
void ClrIrqStatus(uint16_t irqMask);
void SetRegulatorMode(uint8_t regulatorMode);
void SetSaveContext();
void SetAutoFS(uint8_t state);
void SetAutoTx(uint8_t time);
void SetPerfCounterMode(uint8_t perfCounterMode);
void SetLongPreamble(uint8_t enable);
void SetUartSpeed(uint8_t uartSpeed);
void SetRangingRole(uint8_t mode);
void SetAdvancedRanging(uint8_t state);