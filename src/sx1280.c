/* FreeRTOS Includes */
#include "FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include "pico/stdlib.h"
#include "hardware/adc.h" /*"../../pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h"*/
#include "hardware/spi.h" /*"../../pico-sdk/src/rp2_common/hardware_spi/include/hardware/spi.h"*/
#include "hardware/gpio.h"

#include "sx1280.h"

void spiSend(buff_t buff)
{
    gpio_put(CS_PIN, 0);
    spi_write_blocking(spi1, buff.data, buff.len * sizeof(uint8_t));
    gpio_put(CS_PIN, 1);

    while (gpio_get(BUSY_PIN) == 1)
    {
        vTaskDelay(10);
    }
}

void myMemcpy(void *dest, void *src, size_t n)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < n; i++)
        cdest[i] = csrc[i];
}

buff_t spiSendRecv(buff_t buff)
{
    uint8_t response[buff.len] = {0};
    buff_t ret;
    ret.data = response;
    ret.len = buff.len;

    gpio_put(CS_PIN, 0);
    spi_write_read_blocking(spi1, buff.data, ret.data, buff.len * sizeof(uint8_t));
    gpio_put(CS_PIN, 1);

    while (gpio_get(BUSY_PIN) == 1)
    {
        vTaskDelay(10);
    }

    return ret;
}

uint8_t GetStatus()
{
    uint8_t data[1] = {GET_STATUS};
    buff_t sendBuff = {data, 1};
    buff_t ret = spiSendRecv(sendBuff);
    return *(ret.data);
}

void WriteRegister(uint16_t addr, buff_t buff)
{
    size_t len = buff.len + 3;
    uint8_t data[len] = {0x00};
    data[0] = WRITE_REGISTER;
    data[1] = addr & 0xff;
    data[2] = (addr >> 8) & 0xff;
    myMemcpy(data + 3, buff.data, buff.len);
    buff_t sendBuff = {data, len};
    spiSend(sendBuff);
}

buff_t ReadRegister(uint16_t addr, size_t n)
{
    uint8_t data[n + 4] = {0x00};
    data[0] = READ_REGISTER;
    data[1] = addr & 0xff;
    data[2] = (addr >> 8) & 0xff;
    buff_t sendBuff = {data, n + 4};
    buff_t ret = spiSendRecv(sendBuff);
    ret.data = ret.data + 4;
    ret.len = ret.len - 4;
    return ret;
}

void WriteBuffer(buff_t buff)
{
    size_t len = buff.len + 2;
    uint8_t data[len];
    data[0] = WRITE_BUFFER;
    data[1] = TX_BASE_ADDR;
    myMemcpy(data + 2, buff.data, buff.len);
    buff_t sendBuff = {data, len};
    spiSend(sendBuff);
}

buff_t ReadBuffer(uint8_t n)
{
    uint8_t data[n + 3] = {0x00};
    data[0] = READ_BUFFER;
    data[1] = RX_BASE_ADDR;
    buff_t sendBuff = {data, n + 3};
    buff_t ret = spiSendRecv(sendBuff);
    ret.data = ret.data + 3;
    ret.len = ret.len - 3;
    return ret;
}

void SetSleep(uint8_t sleepConfig)
{
    uint8_t data[2] = {SET_SLEEP, sleepConfig};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetStandby(uint8_t standbyConfig)
{
    uint8_t data[2] = {SET_STANDBY, standbyConfig};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetFs()
{
    uint8_t data[1] = {SET_FS};
    buff_t sendBuff = {data, 1};
    spiSend(sendBuff);
}

void SetTx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t data[4] = {
        SET_TX,
        periodBase,
        (uint8_t)(periodBaseCount & 0xff),
        (uint8_t)((periodBaseCount >> 8) & 0xff)};
    buff_t sendBuff = {data, 4};
    spiSend(sendBuff);
}

void SetRx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t data[4] = {
        SET_RX,
        periodBase,
        (uint8_t)(periodBaseCount & 0xff),
        (uint8_t)((periodBaseCount >> 8) & 0xff)};
    buff_t sendBuff = {data, 4};
    spiSend(sendBuff);
}

void SetRxDutyCycle(uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t data[7] = {
        SET_RX_DUTY_CYCLE,
        periodBase,
        (uint8_t)(rxPeriodBaseCount & 0xff),
        (uint8_t)((rxPeriodBaseCount >> 8) & 0xff),
        (uint8_t)(sleepPeriodBaseCount & 0xff),
        (uint8_t)((sleepPeriodBaseCount >> 8) & 0xff),
    };
    buff_t sendBuff = {data, 7};
    spiSend(sendBuff);
}

void SetCad()
{
    uint8_t data[1] = {SET_CAD};
    buff_t sendBuff = {data, 1};
    spiSend(sendBuff);
}

void SetTxContinuousWave()
{
    uint8_t data[1] = {SET_TX_CONTINUOUS_WAVE};
    buff_t sendBuff = {data, 1};
    spiSend(sendBuff);
}

void SetTxContinuousPreamble()
{
    uint8_t data[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    buff_t sendBuff = {data, 1};
    spiSend(sendBuff);
}

void SetPacketType(uint8_t packetType)
{
    uint8_t data[2] = {SET_PACKET_TYPE, packetType};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

uint8_t GetPacketType()
{
    uint8_t data[3] = {0x00};
    data[0] = GET_PACKET_TYPE;
    buff_t sendBuff = {data, 3};
    buff_t ret = spiSendRecv(sendBuff);
    return ret.data[2];
}

void SetRfFrequency(uint8_t rfFrequency[3])
{
    uint8_t data[4] = {0x00};
    data[0] = SET_RF_FREQUENCY;
    myMemcpy(data + 1, rfFrequency, 3);
    buff_t sendBuff = {data, 4};
    spiSend(sendBuff);
}

void SetTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t data[3] = {SET_TX_PARAMS, power, rampTime};
    buff_t sendBuff = {data, 3};
    spiSend(sendBuff);
}

void SetCadParams(uint8_t cadSymbolNum)
{
    uint8_t data[2] = {SET_CAD_PARAMS, cadSymbolNum};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t data[3] = {SET_BUFFER_BASE_ADDRESS, txBaseAddress, rxBaseAddress};
    buff_t sendBuff = {data, 3};
    spiSend(sendBuff);
}

void SetModulationParams(uint8_t modParam[3])
{
    uint8_t data[4] = {0x00};
    data[0] = SET_MODULATION_PARAMS;
    myMemcpy(data + 1, modParam, 3);
    buff_t sendBuff = {data, 4};
    spiSend(sendBuff);
}

void SetPacketParams(uint8_t packetParams[7])
{
    uint8_t data[8] = {0x00};
    data[0] = SET_PACKET_PARAMS;
    myMemcpy(data + 1, packetParam, 7);
    buff_t sendBuff = {data, 8};
    spiSend(sendBuff);
}

buff_t GetRxBufferStatus()
{
    uint8_t data[4] = {0x00};
    data[0] = GET_RX_BUFFER_STATUS;
    buff_t sendBuff = {data, 4};
    buff_t ret = spiSendRecv(sendBuff);
    ret.data = ret.data + 2;
    ret.len = ret.len - 2;
    return ret;
}

buff_t GetPacketStatus()
{
    uint8_t data[7] = {0x00};
    data[0] = GET_PACKET_STATUS;
    buff_t sendBuff = {data, 7};
    buff_t ret = spiSendRecv(sendBuff);
    ret.data = ret.data + 2;
    ret.len = ret.len - 2;
    return ret;
}

uint8_t GetRssiInst()
{
    uint8_t data[3] = {0x00};
    data[0] = GET_RSSI_LNST;
    buff_t sendBuff = {data, 3};
    buff_t ret = spiSendRecv(sendBuff);
    ret.data = ret.data + 2;
    ret.len = ret.len - 2;
    return ret;
}

void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t data[9] = {
        SET_DIO_IRQ_PARAMS,
        (uint8_t)(irqMask & 0xff),
        (uint8_t)((irqMask >> 8) & 0xff),
        (uint8_t)(dio1Mask & 0xff),
        (uint8_t)((dio1Mask >> 8) & 0xff),
        (uint8_t)(dio2Mask & 0xff),
        (uint8_t)((dio2Mask >> 8) & 0xff),
        (uint8_t)(dio3Mask & 0xff),
        (uint8_t)((dio3Mask >> 8) & 0xff)};
    buff_t sendBuff = {data, 9};
    spiSend(sendBuff);
}

uint16_t GetIrqStatus()
{
    uint8_t data[4] = {0x00};
    data[0] = GET_IRQ_STATUS;
    buff_t sendBuff = {data, 4};
    buff_t ret = spiSendRecv(sendBuff);
    return (uint16_t)((ret.data[2] << 8) | ret.data[3]);
}

void ClrIrqStatus(uint16_t irqMask)
{
    uint8_t data[3] = {
        CLR_IRQ_STATUS,
        (uint8_t)(irqMask & 0xff),
        (uint8_t)((irqMask >> 8) & 0xff)};
    buff_t sendBuff = {data, 3};
    spiSend(sendBuff);
}

void SetRegulatorMode(uint8_t regulatorMode)
{
    uint8_t data[2] = {SET_REGULATOR_MODE, regulatorMode};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetSaveContext()
{
    uint8_t data[1] = {SET_SAVE_CONTEXT};
    buff_t sendBuff = {data, 1};
    spiSend(sendBuff);
}

void SetAutoFS(uint8_t state)
{
    uint8_t data[2] = {SET_AUTO_FS, state};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetAutoTx(uint8_t time)
{
    uint8_t data[3] = {
        SET_AUTO_TX,
        (uint8_t)(time & 0xff),
        (uint8_t)((time >> 8) & 0xff)};
    buff_t sendBuff = {data, 3};
    spiSend(sendBuff);
}

void SetPerfCounterMode(uint8_t perfCounterMode)
{
    uint8_t data[2] = {SET_PERF_COUNTER_MODE, perfCounterMode};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetLongPreamble(uint8_t enable)
{
    uint8_t data[2] = {SET_LONG_PREAMBLE, enable};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetUartSpeed(uint8_t uartSpeed)
{
    uint8_t data[2] = {SET_UART_SPEED, uartSpeed};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetRangingRole(uint8_t mode)
{
    uint8_t data[2] = {SET_RANGING_ROLE, mode};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}

void SetAdvancedRanging(uint8_t state)
{
    uint8_t data[2] = {SET_ADVANCED_RANGING, state};
    buff_t sendBuff = {data, 2};
    spiSend(sendBuff);
}
