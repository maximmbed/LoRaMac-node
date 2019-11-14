/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
#include <stddef.h>
#include <stdio.h>
#include "mxc_sys.h"
#include "mxc_uart.h"
#include "mxc_lp.h"
#include "board-config.h"
#include "rtc-board.h"
#include "sx126x-board.h"
#include "board.h"
#include "mxc_gpio.h"
#include "mxc_flc.h"
#include "mxc_rtc.h"
#include "mxc_tmr_utils.h"

#define MAXDAP_DEBUG_ENABLE    1

#define CONSOLE

#define CONSOLE_UART 1
#define CONSOLE_BAUD 115200

mxc_uart_regs_t *ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);

const uart_cfg_t console_uart_cfg = {
    UART_PARITY_DISABLE,
    UART_DATA_SIZE_8_BITS,
    UART_STOP_1,
    UART_FLOW_CTRL_DIS,
    UART_FLOW_POL_DIS,
    CONSOLE_BAUD,
    // UART_CLKSEL_SYSTEM
};

const sys_cfg_uart_t console_uart_sys_cfg = {
#if MAXDAP_DEBUG_ENABLE
        MAP_A,
#else
        MAP_B,
#endif
    UART_FLOW_DISABLE,
};

/* Deepsleep wakeup signals */
const gpio_cfg_t radio_intr_pin = { PORT_0, PIN_9, GPIO_PAD_NONE, GPIO_FUNC_IN };
const gpio_cfg_t gpio_all = { PORT_0, 0x3FFF, GPIO_PAD_NONE, GPIO_FUNC_IN };

void BoardCriticalSectionBegin( uint32_t *mask )
{
    // *mask = __get_PRIMASK( );
    // __disable_irq( );

    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_DisableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(0));
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    // __set_PRIMASK( *mask );

    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(0));
}

void BoardInitMcu( void )
{
    /* Protection against Deepsleeping quickly after POR */
    TMR_Delay(MXC_TMR1, SEC(2), NULL);

#if !MAXDAP_DEBUG_ENABLE
    /* Disable SWD */
    MXC_GCR->scon &= ~MXC_F_GCR_SCON_SWD_DIS;
#endif

#ifdef CONSOLE
    UART_Init(ConsoleUART, &console_uart_cfg, &console_uart_sys_cfg);
#endif

    /* Don't buffer stdout */
    setbuf(stdout, NULL);

    RtcInit( );

    SpiInit( &SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );

    SX126xIoInit( );

    NVIC_DisableIRQ( GPIOWAKE_IRQn );
    NVIC_ClearPendingIRQ( GPIOWAKE_IRQn );
    LP_DisableGPIOWakeup( &gpio_all );
    LP_ClearWakeStatus( );
    MXC_GPIO0->wake_en_clr = gpio_all.mask;

    /* Configure radio interrupt pin as wakeup source but don't enable here */
    GPIO_Config( &radio_intr_pin );
    GPIO_IntConfig( &radio_intr_pin, GPIO_INT_EDGE, GPIO_INT_RISING );
}

void BoardResetMcu( void )
{

}

void BoardInitPeriph( void )
{

}

void BoardDeInitMcu( void )
{

}

uint8_t BoardGetPotiLevel( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

uint32_t BoardGetRandomSeed( void )
{
    uint32_t *ptr;
    uint32_t rs[2];

    BoardGetUniqueId((uint8_t *)rs);

    ptr = (uint32_t *)0x20000008;

    while (ptr < (uint32_t *)0x20018000) {
        rs[0] += *ptr++;
    }

    return rs[0] ^ rs[1];
}

void BoardGetUniqueId( uint8_t *id )
{
    int i;
    uint8_t *ptr = (uint8_t *)MXC_INFO_MEM_BASE;

    FLC_UnlockInfoBlock();

    for (i = 0; i < 8; i++) {
        id[i]  = *ptr;
        id[i] ^= *ptr + 8;
        id[i] ^= *ptr + 16;
        ptr++;
    }

    FLC_LockInfoBlock();
}

void BoardLowPowerHandler( void )
{
    NVIC_ClearPendingIRQ( GPIOWAKE_IRQn );
    LP_ClearWakeStatus();

    MXC_GPIO0->wake_en_set |= radio_intr_pin.mask;
    LP_EnableGPIOWakeup( &radio_intr_pin );

    NVIC_EnableIRQ( GPIOWAKE_IRQn );

    LP_EnableRTCAlarmWakeup();

    LP_DisableBandGap();
    LP_DisableBlockDetect( );

    LP_EnableVCorePORSignal( );
    LP_EnableFastWk( );

    LP_EnableRamRetReg( );
    LP_EnableSRamRet0( );
    LP_EnableSRamRet1( );
    LP_EnableSRamRet2( );
    LP_EnableSRamRet3( );

    LP_DisableSysRAM0LightSleep( );

    while ( UART_PrepForSleep( ConsoleUART ) == E_BUSY );
    while ( RTC_CheckBusy( ) == E_BUSY );

    LP_EnterDeepSleepMode();
}

uint8_t GetBoardPowerSource( void )
{
    return USB_POWER;
}

Version_t BoardGetVersion( void )
{
    static Version_t BoardVersion = {{ 0 }};

    return BoardVersion;
}

/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
#ifdef CONSOLE
    UART_Write(MXC_UART_GET_UART(CONSOLE_UART), buf, count);
#endif
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
#ifdef CONSOLE
    size_t n;
    uint8_t *ptr = (uint8_t *)buf;

    for (n = 0; n < count; n++, ptr++) {
        *ptr = UART_ReadByte(MXC_UART_GET_UART(CONSOLE_UART));
        UART_WriteByte(MXC_UART_GET_UART(CONSOLE_UART), *ptr);
    }

    return count;
#else
    return 0;
#endif
}
