/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Jesse Marroquin ( Maxim Integrated )
 */
// #include "stm32l1xx.h"
#include "mxc_device.h"
#include "uart.h"
#include "utilities.h"
#include "board.h"
#include "uart-board.h"

#include "mxc_uart.h"

typedef struct {
    void *Instance;
} UART_HandleTypeDef;

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
// #define TX_BUFFER_RETRY_COUNT                       10

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    // switch (uartId) {
    //     case UART_1: obj->Instance = MXC_UART0; break;
    //     case UART_2: obj->Instance = MXC_UART1; break;
    // }
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    uart_cfg_t cfg;
    sys_cfg_uart_t sys_cfg;

    sys_cfg.map = MAP_A;
    sys_cfg.flow_flag = UART_FLOW_DISABLE;

    switch (parity) {
        default:
        case NO_PARITY:   cfg.parity = UART_PARITY_DISABLE; break;
        case EVEN_PARITY: cfg.parity = UART_PARITY_EVEN;    break;
        case ODD_PARITY:  cfg.parity = UART_PARITY_ODD;     break;
    }

    switch (wordLength) {
        default:
        case UART_8_BIT: cfg.size = UART_DATA_SIZE_8_BITS; break;
    }

    switch (stopBits) {
        default:
        case UART_1_STOP_BIT:   cfg.stop = UART_STOP_1;   break;
        case UART_1_5_STOP_BIT: cfg.stop = UART_STOP_1P5; break;
        case UART_2_STOP_BIT:   cfg.stop = UART_STOP_2;   break;
    }

    cfg.pol = UART_FLOW_POL_EN;

    switch (flowCtrl) {
        default:
        case NO_FLOW_CTRL:      cfg.flow = UART_FLOW_CTRL_DIS; break;
        case RTS_CTS_FLOW_CTRL: cfg.flow = UART_FLOW_CTRL_EN;  break;
    }

    cfg.baud = baudrate;

    UART_Init(MXC_UART1, &cfg, &sys_cfg);
}

void UartMcuDeInit( Uart_t *obj )
{
    UART_Shutdown(MXC_UART1);
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    UART_Write(MXC_UART1, &data, 1);

    return 0;
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    UART_Read(MXC_UART1, data, 1, 0);

    return 0;
}

uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
{
    UART_Write(MXC_UART1, buffer, size);

    return 0;
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
{
    UART_Read(MXC_UART1, buffer, size, 0);

    return 0;
}
