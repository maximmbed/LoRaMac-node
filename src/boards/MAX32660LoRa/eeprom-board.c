/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
#include "utilities.h"
#include "eeprom-board.h"
#include <stdio.h>
#include <string.h>
#include "mxc_flc.h"
#include "board-config.h"

static const uint32_t ERASE_BUFFER_SIZE = MXC_FLASH_PAGE_SIZE;
static uint8_t EraseBuf[MXC_FLASH_PAGE_SIZE];


uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    static int first = 1;
    uint32_t WriteAddr = MXIM_NVM_BASE + addr;
    int err;

    if( first )
    {
        memset( &EraseBuf, 0xFF, sizeof( EraseBuf ) );
        first = 0;
    }

    err = FLC_BufferErase( WriteAddr, WriteAddr + size, EraseBuf, ERASE_BUFFER_SIZE );
    if( err )
    {
        printf( "Flash Erase ERROR: %d\n", err  );
        return FAIL;
    }

    err = FLC_Write( WriteAddr, size, buffer );
    if( err )
    {
        printf( "Flash Write ERROR: %d\n", err );
        return FAIL;
    }

    return SUCCESS;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    memcpy( buffer, ( void* )( MXIM_NVM_BASE + addr ), size );

    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    while( 1 );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    return -1;
}
