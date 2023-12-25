
/*
 * Author: Chris Schorn
 * Open Lora Mesh Network
 * Version 
 * Versioning Reason: 
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* FreeRTOS Includes */
#include "FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include <stdio.h> /* pico/stdio.h" */
#include "pico/stdlib.h"
#include "hardware/adc.h"/*"../../pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h"*/
#include "hardware/spi.h"/*"../../pico-sdk/src/rp2_common/hardware_spi/include/hardware/spi.h"*/
#include "hardware/gpio.h"

static TaskHandle_t xSimpleLEDTaskHandle = NULL;
static TaskHandle_t xUsbIOTaskHandle = NULL;
static TaskHandle_t xSx1280TaskHandle = NULL;

#define BUSY_PIN 26
#define RESET_PIN 8

#define SCK_PIN 10
#define MOSI_PIN 11
#define MISO_PIN 12
#define CS_PIN 13

#define RX_BASE_ADDR 0x00
#define TX_BASE_ADDR 0x80

#define BUFFER_LEN 126

/* ------------ Defining SD Card Command Index with Hexadecimel Commands ------------ */

/*  Retrieve the transceiver status
    Cannot be the first command sent over the interface
    Is not strictly necessary for SPI b/c device returns status info
        also on cammand bytes
    params( void ) return( status ) */
#define GETSTATUS 0xC0

/*  Writes a block of bytes in a data memory space starting 
        at a specific address.
    params( address[15:8], address[7:0], data[0:n] ) return( void ) */
#define WRITEREGISTER 0x18 

/*  Reads a block of data starting at a given address
    The host must send a NOP after the address to receive data!!!!!!!!!!!!!!!
    params( address[15:8], address[7:0] ) return( data[0:n-1] )  */
#define READREGISTER 0x19

/*  Write the data payload to be transmitted
    Data sent in hex, most likely translated using ascii for text
    Audio data tbd
    params( offset, data[0:n] ) return( void ) */
#define WRITEBUFFER 0x1A

/*  Function allows reading (n-3) bytes of payload received 
        starting at offset.
    Data received in hex, most likely translated using ascii for text
    params( offset ) return( data[0:n-1] ) */
#define READBUFFER 0x1B

/*  Set transceiver to Sleep mode
    Lowest current consumption
    params( sleepConfig ) return( void )
    sleepConfig[7:4] unused 
    sleepConfig[1] 1: Data buffer in retention mode
    sleepConfig[0] 0: Data Ram is flushed 1: Data Ram in retention  */
#define SETSLEEP 0x84

/*  Set the device in either STDBY_RC or STDBY_XOSC mode
    Used to configure the transceiver 
    Intermediate levels of power consumption
    params( standbyConfig )
    standbyConfig 0: STDBY_RC mode 1: STDBY_XOSC mode
    return( void ) */
#define SETSTANDBY 0x80

/*  Set the device in Frequency Synthesizer mode
    PLL(Phase Locked Loop) is locked to the carrier frequency
    For test purposes of PLL
    params( void ) return( void ) */
#define SETFS 0xC1

/*  Set the device in Transmit mode
    Clear IRQ status before using this command
    Timeout = periodBase * periodBaseCount
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out Other: Time out active
    return( void ) */
#define SETTX 0x83

/*  Set the device in Receiver mode
    Timeout = periodBase * periodBaseCount 
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          0xFFFF: Rx Continuous mode, multi-packet Rx
                          Other: Time out active
    return( void ) */
#define SETRX 0x82

/*  Set transceiver in sniff mode
    setLongPreamble must be issued prior to setRxDutyCycle
    RxPeriod = periodBase * rxPeriodBaseCount
    SleepPeriod = periodBase * sleepPeriodBaseCount
    params( rxPeriodBase, rxPeriodBaseCount[15:8], 
        rxPeriodBaseCount[7:0], sleepPeriodBase,
        sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          Other: Device will stay in Rx Mode for 
                                 RxPeriod and return 
                                 to Sleep Mode for SleepPeriod
    return( void ) */
#define SETRXDUTYCYCLE 0x94

/*  Set transceiver to Channel Activity Detection mode
    Device searches for a Lora signal
    Returns to STDBY_RC mode when finished
    Always sends CadDone IRQ, sends CadDetected IRQ if signal found
    Useful in Listen before Talk Applications
    params( void ) return( void ) */
#define SETCAD 0xC5

/*  Test command to generate a Continuous Wave (RF tone)
    Frequency and power settings from setRfFrequency, and setTxParams
    params( void ) return( void ) */
#define SETTXCONTINUOUSWAVE 0xD1

/*  Test command to generate infinite sequence pf symbol 0 in Lora
    params( void ) return( void ) */
#define SETTXCONTNIOUSPREAMBLE 0xD2

/*  Sets the transceiver radio frame
    MUST BE THE FIRST IN A RADIO CONFIGURATION SEQUENCE!!!!!!!
    params( packetType )
    packetType[8:0] 0x00: GFSK
                    0x01: Lora 
                    0x02: Ranging 
                    0x03: FLRC
                    0x04: BLE
    return( void ) */
#define SETPACKETTYPE 0x8A

/*  Returns the current operation packet type of the radio
    packetType probly comes in same format as setPacketType
    params( void ) return( packetType ) */
#define GETPACKETTYPE 0x03

/*  Set the frequency of the RF frequency mode
    rfFrequency sets the number of PLL steps
    Frf = ( Fxosc/2^18 ) * rfFrequency
        Gives frequency in kilohertz
    params( rfFrequency[23:16], rfFrequency[15:8], rfFrequency[7:0] )
    return( void ) */
#define SETRFFREQUENCY 0x86

/*  Sets the Tx output power and the Tx ramp time
    params( power, rampTime )
    power  Pout[dB] = -18 + power i.e. -18 + 0x1F(31) = 13dbm
    rampTime 0x00: 2um 0x20: 4us 0x40: 5us 0x60: 8us 
            0x80: 10us 0xA0: 12us 0xC0: 16us 0xE0: 20us
    return( void ) */
#define SETTXPARAMS 0x8E

/*  Sets number of symbols which Channel Activity Detected operates
    For symbols 1 & 2, there are higher risks of false detection.
    params( cadSymbolNum )
    cadSymbolNum 0x00: 1 symbol
                 0x20: 2 symbols
                 0x40: 4 symbols
                 0x60: 8 symbols
                 0x80: 16 symbols
    return( void ) */
#define SETCADPARAMS 0x88

/*  Fixes the base address for the packet handing operation
        in Tx and Rx mode for all packet types
    params( txBaseAddress, rxBaseAddress ) return( void ) */
#define SETBUFFERBASEADDRESS 0x8F

/*  Configure the modulation parameters of the radio
    Params passed will be interpreted depending on the frame type
    Frame Type 
    params( modParam1, modParam2, modParam3 )
    modParam1 BLE: BitrateBandwidth   Lora/Ranging: Spreading Factor
    modParam2 BLE: ModulationIndex    Lora/Ranging: Bandwith
    modParam3 BLE: ModulationShaping  Lora & Ranging: Coding Rate
    return( void ) */
#define SETMODULATIONPARAMS 0x8B

/*  Set the parameters of the packet handling block
    params( packetParam1, packetParam2, packetParam3, packetParam4,
        packetParam5, packetParam6, packetParam7 )
    packetParam1 BLE: ConnectionState Lora/Ranging: Preambl Length
    packetParam2 BLE: CrcLength       Lora/Ranging: Header Type
    packetParam3 BLE: BleTestPayload  Lora/Ranging: PayloadLength
    packetParam4 BLE: Whitening       Lora/Ranging: CRC
    packetParam5 BLE: Not Used     Lora/Ranging: InvertIQ/chirp invert
    packetParam6 BLE: Not Used        Lora/Ranging: Not Used
    packetParam7 BLE: Not Used        Lora/Ranging: not Used
    return( void ) */ 
#define SETPACKETPARAMS 0x8C

/*  Returns the length of the last received packet 
        and the address of the first byte received
    In Lora packet type, 0x00 always returned for rxPayloadLength.
        Instead read register 0x901, for Lora payload length
    params( void ) return( payloadLength, rxBufferOffset ) */
#define GETRXBUFFERSTATUS 0x17

/*  Retrieve information about the last received packet
    rssiSync: RSSI value latched upon  detection of sync address.
        Actual signal power is –(rssiSync)/2dBm
    snr: Estimation of Signal to Noise Ratio on last packet received. 
        In two’s compliment format multiplied by 4. 
        Actual Signal to Noise Ratio(SNR) is (snr)/4dB. If SNR ≤ 0, 
        RSSI_{packet, real} = RSSI_{packet,measured} – SNR_{measured}
    params( void ) 
    return( packetStatus[39:32], packetStatus[31:24],
        packetStatus[23:16], packetStatus[15:8], packetStatus[7:0] ) 
    packetStatus[7:0]   BLE: RFU        Lora/Ranging: rssiSync
    packetStatus[15:8]  BLE: rssiSync   Lora/Ranging: snr
    packetStatus[16:23] BLE: errors     Lora/Ranging: -
    packetStatus[24:31] BLE: status     Lora/Ranging: -
    packetStatus[32:39] BLE: sync       Lora/Ranging: - */
#define GETPACKETSTATUS 0x1D

/*  Returns instantaneous RSSI value during reception of a  packet
    rssilnst: Signal power is (–rssiInst)/2dBm
    params( void ) return( rssilnst ) */
#define GETRSSILNST 0x1F

/*  Enable IRQs and to route IRQs to DIO pins
    An interrupt is flagged in IRQ register if the corresponding 
        bit in flag register is set
    irqMask[15:0] set which IRQ's are active, 
        pg 95 in sx1280 manual has IRQ table
    dioMasks active bits correspond to the active bits irqMasks
        If coresponding bits are both on, IRQ is sent through that DIO
    params( irqMask[15:8], irqMask[7:0], dio1Mask[15:8],dio1Mask[7:0],
    dio2Mask[15:8], dio2Mask[7:0], dio3Mask[15:8], dio3Mask[7:0] )
    return( void ) */
#define SETDIOIRQPARAMS 0x8D

/*  Returns the value of the IRQ register
    IRQ register is only interacatable through these commands
    params( void ) return( irqStatus[15:8], irqStatus[7:0] ) */
#define GETIRQSTATUS 0x15

/*  Clears an IRQ flag in IRQ register
    Corresponding bits in irqMask will clear flag of that IRQ
    params( irqMask[15:8], irqMask[7:0] ) return( void ) */
#define CLRIRQSTATUS 0x97

/*  Why Kansas but no arkansas
    Havent found in book
    params( regulatorMode ) return( void ) */
#define SETREGULATORMODE 0x96

/*  Havent found in book
    params( void ) return( void ) */
#define SETSAVECONTEXT 0xD5

/*  Set the state following a Rx or Tx operation is FS, not STDBY
    Reduces switching time between consecutive Rx and/or Tx operations
    params( 0x00=disable or 0x01=enable ) return( void ) */
#define SETAUTOFS 0x9E

/*  Allows transceiver to send a packet at a user programmable time 
        after the end of a packet reception
    Must be issued in STDBY_RC mode
    TxDelay = time + 33us(time needed for transceiver to switch modes)
    params( time[15:8], time[7:0] ) return( void ) */
#define SETAUTOTX 0x98

/*  Set the transceiver into Long Preamble mode
    RxDutyCycle is modified so that if a preamble is detected,
         the Rx window is extended by SleepPeriod + 2 * RxPeriod
    params( enable )
    enable 0x00: disable 0x01: enable
    return( void ) */
#define SETLONGPREAMBLE 0x9B

/* #define SETUARTSPEED 0x9D, using spi not uart interface */

/*  params( 0x00=slave or 0x01=master ) return( void ) */
#define SETRANGINGROLE 0xA3

/* params( 0x00=slave or 0x01=master ) return( void ) */
#define SETADVANCEDRANGING 0x9A


/* --------------------------- sx1280 2.4GHz Lora Operation -------------------------------- */

/*  Driving the chip select pin low 
    Transactions with sx1280 start with chip select low */
static inline void sx1280Select(){

    asm volatile ("nop \n nop \n nop");/* Find out what it does */
    gpio_put( CS_PIN, 0 );
    asm volatile ("nop \n nop \n nop");
}

/*  Driving the chip select pin high 
    Transactions with sx1280 end with chip select high */
static inline void sx1280Deselect(){

     asm volatile ("nop \n nop \n nop");
     gpio_put( CS_PIN, 1 );
     asm volatile ("nop \n nop \n nop");
}

void setPacketParam(uint8_t preambleLength, 
                    uint8_t headerType,
                    uint8_t bufferLen, 
                    uint8_t cyclicalRedundancyCheck, 
                    uint8_t chirpInvert){
    uint8_t *setupWriteData = ( uint8_t * ) pvPortMalloc( 8*sizeof( uint8_t ) );
    *( setupWriteData ) = SETPACKETPARAMS;
    *( setupWriteData + 1 ) = preambleLength; /* Preamble Length */
    *( setupWriteData + 2 ) = headerType; /* Header Type */
    *( setupWriteData + 3 ) = bufferLen; /* Payload Length */
    *( setupWriteData + 4 ) = cyclicalRedundancyCheck; /* Cyclical Redundancy Check */
    *( setupWriteData + 5 ) = chirpInvert; /* Invert IQ/chirp invert */
    *( setupWriteData + 6 ) = 0x00; /* Not Used */
    *( setupWriteData + 7 ) = 0x00; /* Not Used */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 8*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
    }
}

/* Function sending common transciever settings to sx1280 */ 
void sx1280Setup( uint8_t standbyMode, 
                  uint8_t packetType, 
                  uint8_t rfFrequency2316,
                  uint8_t rfFrequency158, 
                  uint8_t rfFrequency70, 
                  uint8_t spreadingFactor,
                  uint8_t bandwidth, 
                  uint8_t codingRate, 
                  uint8_t preambleLength, 
                  uint8_t headerType, 
                  uint8_t cyclicalRedundancyCheck, 
                  uint8_t chirpInvert, 
                  uint8_t *outboundMessage ){

    uint8_t *setupWriteData;
    uint8_t *setupReadData;

    /* Waiting till the busy pin is driven low */
    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
    }

    /* Setting sx1280 Standby mode */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( setupWriteData ) = SETSTANDBY;
    *( setupWriteData + 1 ) = standbyMode; /* Setting STDBY_RC Mode 0x01, STDBY_XOSC */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETSTANDBY\n");
    }

    /* Setting sx1280 Packet Type */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( setupWriteData ) = SETPACKETTYPE;
    *( setupWriteData + 1 ) = packetType;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETPACKETTYPE\n");
    }

    /* Setting RF Frequency */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = SETRFFREQUENCY;
    *( setupWriteData + 1 ) = rfFrequency2316; /* rfFrequency[23:16], bits 23 to 16 */
    *( setupWriteData + 2 ) = rfFrequency158; /* rfFrequency[15:8], bits 15 to 8 */
    *( setupWriteData + 3 ) = rfFrequency70; /* rfFrequency[7:0], bits 7 to 0 */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETRFFREQUENCY\n");
    }

    /* Setting Tx and Rx Buffer Base Addresses
       Putting both at 0 since messages can be size of buffer */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( setupWriteData ) = SETBUFFERBASEADDRESS;
    *( setupWriteData + 1 ) = TX_BASE_ADDR;
    *( setupWriteData + 2 ) = RX_BASE_ADDR;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETBUFFERBASEADDRESS\n");
    }

    /* Setting the Modulation Params */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = SETMODULATIONPARAMS;
    *( setupWriteData + 1 ) = spreadingFactor; /* Spreading Factor */
    *( setupWriteData + 2 ) = bandwidth; /* Bandwidth */
    *( setupWriteData + 3 ) = codingRate; /* Coding Rate */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;
    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if( spreadingFactor == 0x50 || spreadingFactor == 0x60 ){

        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x1E;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
        setupWriteData = NULL;
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if( spreadingFactor == 0x70 || spreadingFactor == 0x80 ){

        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x37;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
        setupWriteData = NULL;
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if( spreadingFactor == 0x90 || spreadingFactor == 0xA0 || spreadingFactor == 0xB0 || spreadingFactor == 0xC0 ){
        
        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x32;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
        setupWriteData = NULL;
    }
    /* 0x01 must be written to register 0x093C */
    /* Nenašel jsem v datasheetu
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = WRITEREGISTER;
    *( setupWriteData + 1 ) = 0x09;
    *( setupWriteData + 2 ) = 0x3C;
    *( setupWriteData + 3 ) = 0x01;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    setupWriteData = NULL;
    */

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETMODULATIONPARAMS\n");
    }

    uint8_t bufferLen = BUFFER_LEN;
    setPacketParam(preambleLength, headerType, bufferLen, cyclicalRedundancyCheck, chirpInvert);

    /* Testing connecting from pico to sx1280 by writing to and reading from buffer
       Working output should be "status status FF" */

    /* writeData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( writeData ) = WRITEBUFFER;
    *( writeData + 1 ) = 0x00;
    *( writeData + 2 ) = 0xFF;
    sx1280Select();
    spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );
    writeData = NULL; */

    /* Must use two NOP's for reads because data is
            returned beginning on the second NOP */
    /*readData = ( uint8_t * ) pvPortMalloc( 5*sizeof( uint8_t ) );
    writeData = ( uint8_t * ) pvPortMalloc( 5*sizeof( uint8_t ) );
    *( writeData ) = READBUFFER;
    *( writeData + 1 ) = 0x00;
    *( writeData + 2 ) = 0x00;
    *( writeData + 3 ) = 0x00;
    *( writeData + 4 ) = 0x00;
    sx1280Select();
    spi_write_read_blocking( spi1, writeData, readData, 5*sizeof( uint8_t ) );
    sx1280Deselect(); 
    printf( "%X %X %X %X %X\n", *( readData ), *( readData + 1 ), *( readData + 2 ), *( readData + 3 ), *( readData + 4 ) );
    vPortFree( writeData );
    writeData = NULL;
    vPortFree( readData );
    readData = NULL; */
}


/* Function setting up and running tx operation on an sx1280, taking 255 byte message packets */
void sx1280Tx( uint8_t power, 
               uint8_t rampTime,
               uint8_t *outboundMessage,
               uint8_t msgLen,
               uint8_t txIrq158, 
               uint8_t txIrq70, 
               uint8_t txPeriodBase,
               uint8_t txPeriodBaseCount158, 
               uint8_t txPeriodBaseCount70 ){

    uint8_t *txWriteData = 0;
    uint8_t *txReadData = 0;
    /* Iterators */
    uint32_t i = 0;

    /* Setting the tx parameters necessary for sending a message */
    txWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( txWriteData ) = SETTXPARAMS;
    *( txWriteData + 1 ) = power;    /* power       */
    *( txWriteData + 2 ) = rampTime; /* rampTime    */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETTXPARAMS\n");
    }

    /* Allocating msgLen+3 bytes to writeData, payloadLength is indexed from zero
            and space is needed for the WRITEBUFFER command and nop */
    txWriteData = ( uint8_t * )pvPortMalloc( ( msgLen+2 )*sizeof( uint8_t ) );
    *( txWriteData ) = WRITEBUFFER;
    *( txWriteData + 1 ) = TX_BASE_ADDR;
    /* Looping payloadLength times, writing outboundMessage data to WRITEBUFFER command */
    for( i = 0; i < msgLen; i++ ){
        *(txWriteData + i + 2) = *(outboundMessage + i);
        printf("Outbound Message: 0x%X %c\n", *( txWriteData + i + 2), *( txWriteData + i + 2));
    }
    printf("Outbound Message Address: 0x%X\n", outboundMessage );
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, ( msgLen+2 )*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after tx WRITEBUFFER\n");
    }

    setPacketParam( 0x0C,          /* uint8_t preambleLength           */
                    0x00,          /* uint8_t headerType               */
                    msgLen,
                    0x20,          /* uint8_t cyclicalRedundancyCheck  */
                    0x40           /* uint8_t chirpInvert              */);

    /* setting IRQ parameters for the outgoing message, looping SPI not DIO pins to check*/
    txWriteData = ( uint8_t * ) pvPortMalloc( 9*sizeof( uint8_t ) );
    *( txWriteData ) = SETDIOIRQPARAMS;
    *( txWriteData + 1 ) = txIrq158; /* IRQ Mask for bits 15:8 of IRQ register */
    *( txWriteData + 2 ) = txIrq70; /* IRQ Mask for bits 7:0 of IRQ register */
    *( txWriteData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
    *( txWriteData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
    *( txWriteData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
    *( txWriteData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
    *( txWriteData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
    *( txWriteData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 9*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after tx SETDIOIRQPARAMS\n");
    }

    /* Putting sx1280 in transmit mode to send the message in sx1280's message buffer 
       Timeout is periodBase * periodBaseCount */
    txWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( txWriteData ) = SETTX;
    *( txWriteData + 1 ) = txPeriodBase; /* setting periodBase, RTC step */
    *( txWriteData + 2 ) = txPeriodBaseCount158; /* setting periodBaseCount[15:8] */
    *( txWriteData + 3 ) = txPeriodBaseCount70; /* setting periodBaseCount[8:0] */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after tx SETTX\n");
    }

    /* Looping over GETIRQSTATUS using SPI, till TxDone bit is high */
    for( i = 0; i <= 25; i++){

        vPortFree( txReadData );
        vTaskDelay( 50 );

        txWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        txReadData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( txWriteData ) = GETIRQSTATUS;
        *( txWriteData + 1 ) = 0x00;
        *( txWriteData + 2 ) = 0x00;
        *( txWriteData + 3 ) = 0x00;
        sx1280Select();
        spi_write_read_blocking( spi1, txWriteData, txReadData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( txWriteData );
        txWriteData = NULL;
 
        while( gpio_get( BUSY_PIN ) == 1 ){
            vTaskDelay( 10 );
            // printf("Busy after tx GETIRQSTATUS\n");
        }

        printf("IRQ Check: 0x%X %i\n", *( txReadData + 3 ), i );

        /* Checking bits [7:0] to see if the TxDone bit in the IRQ register is high
           Doing bitwise 'and' operation with 0x01 to mask the rest of the bits in 
                the IRQ register, giving a clear indication that a message has been sent
            Bits [15:8] would be in  *( readData + 4 ) */
        if( *( txReadData + 3 ) != 0x00 ){ /* GETIRQSTATUS TxDone == 1 */

            printf("IRQ: 0x%X %i \n", *( txReadData + 3 ), i );
            vPortFree( txReadData );
            txReadData = NULL;
            break;
        }

    }

    /* Clearing the IRQ register, reseting IRQ Mask bits to 0 */
    txWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( txWriteData ) = CLRIRQSTATUS;
    *( txWriteData + 1 ) = 0xFF; /* clearing bits 15:8 of IRQ mask */
    *( txWriteData + 2 ) = 0xFF; /* clearing bits 7:0 of IRQ mask */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after tx CLRIRQSTATUS\n");
    }

    /* Tx SETSANDBY */
    txWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( txWriteData ) = SETSTANDBY;
    *( txWriteData + 1 ) = 0x00;
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );
    txWriteData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after tx SETSTANDBY\n");
    }
}

/* Function setting up and running rx operation on an sx1280, 2.4Ghz LORA Modem*/
void sx1280Rx( uint8_t rxIrq158, 
               uint8_t rxIrq70, 
               uint8_t rxPeriodBase,
               uint8_t rxPeriodBaseCount158, 
               uint8_t rxPeriodBaseCount70){

    uint8_t *writeData;
    uint8_t *readData;
    uint8_t totalSizeOfMessage = 0;
    uint8_t rxPayloadLength = 0;
    uint8_t rxStartBufferPointer = 0;
    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;
    

    /* setting IRQ parameters for Rx mode */
    writeData = ( uint8_t * ) pvPortMalloc( 9*sizeof( uint8_t ) );
    *( writeData ) = SETDIOIRQPARAMS;
    *( writeData + 1 ) = rxIrq158; /* IRQ Mask for bits 15:8 of IRQ register */
    *( writeData + 2 ) = rxIrq70; /* IRQ Mask for bits 7:0 of IRQ register */ 
    *( writeData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
    *( writeData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
    *( writeData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
    *( writeData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
    *( writeData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
    *( writeData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
    sx1280Select();
    spi_write_blocking( spi1, writeData, 9*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );
    writeData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after rx SETDIOIRQPARAMS\n");
    }

    /* setting sx1280 to Rx mode */
    writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( writeData ) = SETRX;
    *( writeData + 1 ) = rxPeriodBase; /* Setting the RTC step */
    *( writeData + 2 ) = rxPeriodBaseCount158; /* perdiodBase[15:8] for rx */
    *( writeData + 3 ) = rxPeriodBaseCount70; /* perdiodBase[7:0] for rx */
    sx1280Select();
    spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );
    writeData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETRX\n");
    }

    /* Using the GETIRQSTATUS command, the RxDone flag will be set to 1 when
            a new message has been received 
       Assuming that the rx buffer is cleared when each new message is received */

    /* Loop polling 100 times over rx mode, 100 is arbitrarily picked 
       Could try to grab the amount of time the sx1280 will listen for and loop for that many ms
            Will need conditionals for 0x0000, 0xFFFF, and everything between
    if( rxPeriodBaseCount158 == 0x00 && rxPeriodBaseCount70 == 0x00 ){ // Single Rx

    }
    else if( rxPeriodBaseCount158 == 0xFF && rxPeriodBaseCount70 == 0xFF ){ // Continuous Rx

    }
    else{ // Rx with a timeout

    }
        */
    for( i = 0; i <= 25; i++ ){ /* we want to keep listening */

        printf("Listening: %i\n", i );
        vTaskDelay( 50 ); 
        rxPayloadLength = 0;

        /* Using GETIRQSTATUS to check if there is a new message in the rx buffer */
        writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        readData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( writeData ) = GETIRQSTATUS;
        *( writeData + 1 ) = 0x00;
        *( writeData + 2 ) = 0x00;
        *( writeData + 3 ) = 0x00;
        sx1280Select();
        spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( writeData );
        writeData = NULL;

        /* Checking to see if the RxDone bit in the IRQ register is high, with 0x02 bitmask */
        if(*( readData + 3) & 0x02){ /* GETIRQSTATUS RxDone == 1 */
            if(*(readData + 3) & 0b01000000){
                printf("CRC error...\n");
                break;
            }
 
            vPortFree( readData );
            readData = NULL;

            while( gpio_get( BUSY_PIN ) == 1 ){
                vTaskDelay( 10 );
                // printf("Busy after rx GETIRQSTATUS\n");
            }

            /* using GETPACKETSTATUS which returns rssiSync, and Signal to Noise Ratio ( SNR )
               Not currently using but it's in sx1280 Documentation for Rx operation
                    pretty sure it's used to see if the received message is useable or not */
            /*
            writeData = ( uint8_t * ) pvPortMalloc( 7*sizeof( uint8_t ) );
            readData = ( uint8_t * ) pvPortMalloc( 7*sizeof( uint8_t ) );
            *( writeData ) = GETPACKETSTATUS;
            *( writeData + 1 ) = 0x00;
            *( writeData + 2 ) = 0x00;
            *( writeData + 3 ) = 0x00;
            *( writeData + 4 ) = 0x00;
            *( writeData + 5 ) = 0x00;
            *( writeData + 6 ) = 0x00;
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, 7*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );
            writeData = NULL;
            vPortFree( readData );
            readData = NULL;

            while( gpio_get( BUSY_PIN ) == 1 ){
                vTaskDelay( 10 );
                // printf("Busy after rx GETPACKETSTATUS\n");
            }
            */

            /* Clearing the IRQ register on the sx1280
               Not sure why it's done here in the rx operation in sx1280 documentation */
            writeData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
            *( writeData ) = CLRIRQSTATUS;
            *( writeData + 1 ) = 0xFF;
            *( writeData + 2 ) = 0xFF;
            sx1280Select();
            spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );
            writeData = NULL;

            while( gpio_get( BUSY_PIN ) == 1 ){
                vTaskDelay( 10 );
                // printf("Busy after rx CLRIRQSTATUS\n");
            }

            /* Getting the length of the newly received message
               GETRXBUFFERSTATUS only works for LORA messages with headers, 
                    otherwise read register 0x0901 */
            writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
            readData = ( uint8_t * ) pvPortMalloc( 4*sizeof(uint8_t ) );
            *( writeData ) = GETRXBUFFERSTATUS; 
            *( writeData + 1 ) = 0x00;
            *( writeData + 2 ) = 0x00;
            *( writeData + 3 ) = 0x00;
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
            sx1280Deselect();
            /* Grabbing message size for correct memory allocation for incoming message */
            rxPayloadLength = *( readData + 2 );
            rxStartBufferPointer = *( readData + 3 );
            vPortFree( writeData );
            writeData = NULL;
            vPortFree( readData );
            readData = NULL;

            while( gpio_get( BUSY_PIN ) == 1 ){
                vTaskDelay( 10 );
                // printf("Busy after rx READREGISTER\n");
            }

            /* Reading message buffer of sx1280
               Allocating the size of the message in the sx1280 buffer plus 3 because over 
                    spi you must send an opcode, the buffer offset, and a nop to receive the
                    payload on the buffer */
            writeData = ( uint8_t * ) pvPortMalloc((rxPayloadLength + 3)*sizeof(uint8_t));
            readData = ( uint8_t * ) pvPortMalloc((rxPayloadLength + 3)*sizeof(uint8_t));
            for(int i = 0; i < (rxPayloadLength + 3); i++){
                *(writeData + i) = 0x00;
            }
            *( writeData ) = READBUFFER;
            *( writeData + 1 ) = rxStartBufferPointer; /* sx1280 message buffer offset */
            printf("Final Address = 0x%X\n", rxStartBufferPointer);
            printf("size of msg buffer = %X\n", rxPayloadLength);
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, ( rxPayloadLength + 3 )*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );
            writeData = NULL;
            for( i = 0; i < rxPayloadLength; i++){
                printf("Inbound Message: 0x%X %c %i\n", *( readData + i + 3), *( readData + i + 3), i);
            }
            /* vPortFree( readData );
            readData = NULL; */

            while( gpio_get( BUSY_PIN ) == 1 ){
                vTaskDelay( 10 );
                // printf("Busy after rx READBUFFER\n");
            }

            break;

        }
    }

    /* Rx SETSANDBY */
    writeData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( writeData ) = SETSTANDBY;
    *( writeData + 1 ) = 0x00;
    sx1280Select();
    spi_write_blocking( spi1, writeData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );
    writeData = NULL;

    while( gpio_get( BUSY_PIN ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after rx SETSTANDBY\n");
    }
}


/*  Function setting up sx1280 connection to rp2040 
    Initializing SPI interface on rp2040
    Using GP10-GP13
        GP10: SPI1 SCK
        GP11: SPI1 Tx
        GP12: SPI1 Rx
        GP13: SPI1 CSn */
void sx1280Rp2040Setup( ){

    /* Setting up Busy Pin */

    /* void gpio_init (uint gpio)
       Initializing GP22 to input pin for Busy */
    gpio_init( BUSY_PIN );
    /* static void gpio_set_dir (uint gpio, bool out)
       Setting GP22 direction to input, True is out False is in
       Use gpio_get( uint gpio ) to get the state of a gpio */
    gpio_set_dir( BUSY_PIN, 0 );

    /* Setting up Reset Pin */

    /* Initializing GP21 to output pin for Reset */
    gpio_init( RESET_PIN );
    /* Setting GP21 direction to output, True is out False is in */
    gpio_set_dir( RESET_PIN, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP21 High
       A resit is initiated by driving Reset Low */
    gpio_put( RESET_PIN, 1 );

    /* Setting up SPI1 Interface */

    /* Inializing spi1 at 1MHz */
    spi_init( spi1, 1000000 );

    /* void gpio_set_function( uint gpio, enum gpio_function fn )
       Setting  GP10-GP12 as SCK, TX, and RX respectively */
    gpio_set_function( SCK_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MOSI_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MISO_PIN, GPIO_FUNC_SPI );

    /* Initializing GP13 to output pin for Chip Select */
    gpio_init( CS_PIN );
    /* Setting GP13 direction to output, True is out False is in */
    gpio_set_dir( CS_PIN, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP13 High
       A data transfer is started by driving Chip Select low */
    gpio_put( CS_PIN, 1 );
}


/*  Task setting up and running sx1280
    Also now adding some elements of mesh in */
void vSx1280Task( void *pvParameters ){

    /* 8 bit pointer variables for dynamic arrays */
    uint8_t *writeData = NULL;
    uint8_t *readData = NULL;

    /* 32 bit pointer for address received from vUsbIOTask task notification */
    uint32_t *taskNotificationFromUSB = ( uint32_t * ) pvPortMalloc( 1*sizeof( uint32_t ) );

    struct sx1280MessageStorageTillUse{
        uint8_t *message;
    };
    /* Instantiating struct, containing an 8 bit pointer to temporarily store a message */
    struct sx1280MessageStorageTillUse messageStorageTillUse = { NULL };

    sx1280Rp2040Setup();

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;

    gpio_put( RESET_PIN, 0 ); /* Resetting sx1280 during startup, says in documentation FIND PAGE */
    asm volatile ("nop \n nop \n nop");
    gpio_put( RESET_PIN, 1 );

    while( true ){

        xTaskNotifyWait(
                0xffffffff,               /* uint32_t ulBitsToClearOnEntry */
                0,                        /* uint32_t ulBitsToClearOnExit */
                taskNotificationFromUSB,  /* uint32_t *pulNotificationValue */
                100 );                    /* TickType_t xTicksToWait */
 
        if( *( taskNotificationFromUSB ) != 0 ){
            for( j = 0; *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) != 0x00; j++){
                printf("sx1280 task notification = 0x%X %c\n", *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ), *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) );
            }
            printf("sx1280 task notification held address: 0x%X\n", *(taskNotificationFromUSB));
            messageStorageTillUse.message = ( uint8_t * ) *( taskNotificationFromUSB );
        }

        vTaskDelay( 10 );

        /* Arbitrarily allocating BUFFER_LEN bytes to writeData for payloadLength in sx1280Setup
           Payload length does not matter for messages with headers */
        writeData = ( uint8_t * ) pvPortMalloc( BUFFER_LEN*sizeof( uint8_t ) );
        for(int i = 0; i < BUFFER_LEN; i++){
            *(writeData + i) = 0x00;
        }

        sx1280Setup( 0x00,          /* uint8_t standbyMode              */
                     0x01,          /* uint8_t packetType               */
                     0xB8,          /* uint8_t rfFrequency2316          */
                     0x9D,          /* uint8_t rfFrequency158           */
                     0x89,          /* uint8_t rfFrequency70            */
                     0x50,          /* uint8_t spreadingFactor          */
                     0x0A,          /* uint8_t bandwidth                */
                     0x01,          /* uint8_t codingRate               */
                     0x0C,          /* uint8_t preambleLength           */
                     0x00,          /* uint8_t headerType               */
                     0x20,          /* uint8_t cyclicalRedundancyCheck  */
                     0x40,          /* uint8_t chirpInvert              */
                     writeData );   /* uint8_t *outboundMessage         */

        vPortFree( writeData );
        writeData = NULL;

        sx1280Rx( 0b01000000,   /* uint8_t rxIrq158                 */
                  0b01110011,   /* uint8_t rxIrq70                  */
                  0x02,         /* uint8_t rxPeriodBase             */
                  0xFF,         /* uint8_t rxPeriodBaseCount158     */
                  0xFF          /* uint8_t rxPeriodBaseCount70      */);

        /* Checking received message data */
        // *( readData + i ) != 0x00
        /*
        if(dataLen){
            for( i = 0; i < dataLen; i++){
                printf("Inbound Message: 0x%X %c %i\n", *( readData + i), *( readData + i), i);
            }
        }
        else{
            printf("No inbound message\n");
        }
        */

        if( messageStorageTillUse.message != NULL ){

            int msgLen = 0;
            /* Looping and printing outbound message till NULL, strings are NULL terminated */
            for( i = 0; *( messageStorageTillUse.message + i ) != 0x00; i++ ){
                msgLen += 1;
                printf("Outbound Check: 0x%X %c\n", *( messageStorageTillUse.message + i ), *( messageStorageTillUse.message + i ) );
            }
            if(msgLen > BUFFER_LEN){
                msgLen = BUFFER_LEN;
                printf("Msg over BUFFER_LEN...sending only BUFFER_LEN\n");
            }

            sx1280Tx( 0x1F,         /* uint8_t power                    */
                      0xE0,         /* uint8_t rampTime                 */
                                    /* uint8_t *outboundMessage         */
                      messageStorageTillUse.message,
                      (uint8_t)msgLen,    
                      0x40,         /* uint8_t txIrq158                 */
                      0x01,         /* uint8_t txIrq70                  */
                      0x02,         /* uint8_t txPeriodBase             */
                      0x01,         /* uint8_t txPeriodBaseCount158     */
                      0xF4 );       /* uint8_t txPeriodBaseCount70      */

            vPortFree( writeData );
            writeData = NULL;
            vPortFree( messageStorageTillUse.message );
            messageStorageTillUse.message = NULL;
        }
        vTaskDelay( 10 );
    }
}

/* ----------------------------- Pi Pico Onboard LED Task ------------------------------- */

void vSimpleLEDTask( void *pvParameters ){

    gpio_init( PICO_DEFAULT_LED_PIN );
    gpio_set_dir( PICO_DEFAULT_LED_PIN, GPIO_OUT );

    while( true ){
        gpio_put( PICO_DEFAULT_LED_PIN, 1 );
        vTaskDelay( 100 );
        gpio_put( PICO_DEFAULT_LED_PIN, 0 );
        vTaskDelay( 100 );
    }
}

/* --------------------------- Serial Monitor USB IO Task -------------------------------- */

/*  Task running usb serial input
    Sends task notification to vSx1280Task with pointer to 
        buffer holding the message to be sent from the sx1280 */
void vUsbIOTask( void *pvParameters ){

    uint8_t currentChar = 0x00; /* 8 bit integer to hold hex value from getchar() */

    uint8_t *messageBuffer = NULL; /* 8 bit pointer to hold outgoing message */
    uint8_t *messageBufferRealloc = NULL; /* 8 bit pointer for dynamic message input */

    /* 32 bit integer for placing input characters in the correct places in messageBuffer */
    uint32_t messageCounter = 0;

    /* Allocating single byte to messageBuffer, done before realloc is called */
    messageBuffer = ( uint8_t * ) pvPortMalloc( 1*sizeof( uint8_t ) );

    /* Iterators */
    uint32_t i = 0;

    while( true ){

        /* Setting currentChar to the character being read in by getchar() */
        currentChar = getchar_timeout_us( 1000 );

        /* Checking currentChar for the error code 0xFF, isn't on the ascii keyboard, < 0x20,
           and isn't a new line character */
        if( currentChar == 0xFF || ( currentChar < 0x20 && currentChar != 0x0A ) ){
            currentChar = 0x00;
        }

        /* Checking if the character being read in is "\n" */
        if( currentChar == 0x0A ){ 

            messageBufferRealloc = messageBuffer;
            /* An extra byte is added to messageBuffer for "0x00", NULL terminated strings */
            messageBuffer = ( uint8_t * ) pvPortMalloc( ( messageCounter+2 )*sizeof( uint8_t ));
            for( i = 0; i <= messageCounter; i++ ){
                *( messageBuffer + i ) = *( messageBufferRealloc + i );
            }
            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + messageCounter ) = currentChar;
            *( messageBuffer + messageCounter + 1 ) = 0x00; 
            vPortFree( messageBufferRealloc ); /* Freeing reallocation holder pointer */
            messageBufferRealloc = NULL;
            for( i = 0; i <= messageCounter + 1; i++ ){
                printf( "Typed Message: 0x%X %c %i\n", *( messageBuffer + i ), *( messageBuffer + i ), i );
            }
            printf("Typed Message Address: 0x%X\n", messageBuffer );

            /* FreeRTOS function updating a receiving tasks notification value */
            xTaskNotify(
                xSx1280TaskHandle,                /* TaskHandle_t xTaskToNotify */ 
                ( uint32_t ) messageBuffer,       /* uint32_t ulValue, (int)&buffer[0] */
                eSetValueWithoutOverwrite );      /* eNotifyAction eAction */

            vTaskDelay( 500 ); /* To allow other tasks time to grab the notification values */
            messageBuffer = ( uint8_t * ) pvPortMalloc( 1*sizeof( uint8_t ) );
            messageCounter = 0;
        }
        else if( currentChar != 0x0A && currentChar != 0x00 ){ /* if the character being read in is not "\n" */
            /* Increasing messageBuffer size until "/n"
               messageCounter is indexed from 0, add one for correct sized message */
            messageBufferRealloc = messageBuffer;
            messageBuffer = ( uint8_t * ) pvPortMalloc( ( messageCounter+1 )*sizeof( uint8_t ));
            for( i = 0; i <= messageCounter; i++ ){
                *( messageBuffer + i ) = *( messageBufferRealloc + i );
            }
            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + messageCounter ) = currentChar;
            messageCounter = messageCounter + 1;
            vPortFree( messageBufferRealloc );
            messageBufferRealloc = NULL;
        }
        currentChar = 0x00;
        vTaskDelay( 10 );
    }
}


/* ------------------------------------- MAIN ------------------------------------------- */

int main( void ){

    stdio_init_all();

    uint32_t status = xTaskCreate(
                    vSimpleLEDTask,  
                    "Green Led",    
                    1024,           
                    NULL,           
                    1,              
                    &xSimpleLEDTaskHandle ); 

    uint32_t ioStatus = xTaskCreate(
                    vUsbIOTask,         /* TaskFunction_t pvTaskCode */
                    "Simple IO",        /* const char * const pcName */
                    4096,               /* uint16_t usStackDepth */
                    NULL,               /* void *pvParameters */
                    1,                  /* UBaseType_t uxPriority */
                    &xUsbIOTaskHandle );/*TaskHandle_t *pxCreatedTask*/

    uint32_t sx1280Status = xTaskCreate(
                    vSx1280Task,
                    "sx1280",
                    4096, /* usStackDepth * 4 = stack in bytes, because pico is 32 bits wide */
                    NULL,
                    1,
                    &xSx1280TaskHandle );

    vTaskStartScheduler();

    while( true ){

    }

    return 0;
}
