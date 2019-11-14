/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
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
#include <stdint.h>
#include "utilities.h"
#include "spi-board.h"
#include "mxc_spi.h"

typedef struct
{
    spi_type Instance;
    IRQn_Type IrqNum;
    uint32_t hz;
    int8_t bits;
    int8_t mode;
    int8_t master;
    spi_req_t req;
} SPI_HandleTypeDef;

static SPI_HandleTypeDef SpiHandle[MXC_SPIMSS_INSTANCES + MXC_SPI17Y_INSTANCES];

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    CRITICAL_SECTION_BEGIN( );

    obj->SpiId = spiId;
    if (spiId == SPI_1) {
        SpiHandle[spiId].Instance = SPI0A;
        SpiHandle[spiId].IrqNum = SPI0_IRQn;
    } else {
        SpiHandle[spiId].Instance = SPI1A;
        SpiHandle[spiId].IrqNum = SPI1_IRQn;
    }

    SpiFormat(obj, 8, 0, 0, 0);
    SpiFrequency(obj, 10E6);
    SpiHandle[spiId].req.len = 1;
    SpiHandle[spiId].req.bits = SpiHandle[spiId].bits;
    SPI_Init(SpiHandle[spiId].Instance, SpiHandle[obj->SpiId].mode, SpiHandle[obj->SpiId].hz);

    CRITICAL_SECTION_END( );
}

void SpiDeInit( Spi_t *obj )
{
    SPI_Shutdown(SpiHandle[obj->SpiId].Instance);
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    SpiHandle[obj->SpiId].bits = bits;
    SpiHandle[obj->SpiId].mode = !!cpol << 1 | !!cpha;
    SpiHandle[obj->SpiId].master = !slave;
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    SpiHandle[obj->SpiId].hz = hz;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    uint8_t inData;
    spi_req_t *req = &SpiHandle[obj->SpiId].req;

    req->tx_data = &outData;
    req->rx_data = &inData;

    SPI_MasterTrans(SpiHandle[obj->SpiId].Instance, req);

    return inData;
}

