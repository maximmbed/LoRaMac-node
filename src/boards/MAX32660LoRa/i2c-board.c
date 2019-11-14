/*!
 * \file      i2c-board.c
 *
 * \brief     Target board I2C driver implementation
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
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_assert.h"
#include "mxc_i2c.h"
#include "utilities.h"
#include "board-config.h"
#include "i2c-board.h"


typedef struct I2cHandle I2cHandle_t;
static struct I2cHandle {
    mxc_i2c_regs_t *Instance;
    IRQn_Type IrqNum;   // used? delete if not
    uint32_t Frequency;
    I2cAddrSize InternalAddressSize;
} I2cHandle[MXC_I2C_INSTANCES] = {
    { NULL, 0, 0, I2C_ADDR_SIZE_8 },
    { NULL, 0, 0, I2C_ADDR_SIZE_8 }
};

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj    I2C object
 * \param [IN] i2cId  I2C peripheral ID to be used
 * \param [IN] scl    I2C Scl pin name to be used
 * \param [IN] sda    I2C Sda pin name to be used
 */
void I2cMcuInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
    const gpio_cfg_t *cfg;

    switch (i2cId) {
        case I2C_1: cfg = &gpio_cfg_i2c0; break;
        case I2C_2: cfg = &gpio_cfg_i2c1; break;
        default: MXC_ASSERT_FAIL(); break;
    }

    // MXC_ASSERT(cfg->mask & (1 << scl));
    if (!(cfg->mask & (1 << scl))) {
        MXC_ASSERT_FAIL();
    }

    // MXC_ASSERT(cfg->mask & (1 << sda));
    if (!(cfg->mask & (1 << sda))) {
        MXC_ASSERT_FAIL();
    }

    obj->I2cId = i2cId;
    obj->Scl.pin = scl;
    obj->Sda.pin = sda;

    I2cHandle[i2cId].Instance = (i2cId == I2C_1) ? MXC_I2C0 : MXC_I2C1;
    I2cHandle[i2cId].IrqNum = (i2cId == I2C_1) ? I2C0_IRQn : I2C1_IRQn;
}

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj              I2C object
 * \param [IN] mode             Mode of operation for the I2C Bus
 * \param [IN] dutyCycle        Signal duty cycle
 * \param [IN] I2cAckEnable     Enable or Disable to ack
 * \param [IN] AckAddrMode      7bit or 10 bit addressing
 * \param [IN] I2cFrequency     I2C bus clock frequency
 */
void I2cMcuFormat( I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency )
{
    MXC_ASSERT(AckAddrMode == I2C_ACK_ADD_7_BIT);
    I2C_Shutdown(I2cHandle[obj->I2cId].Instance);
    if (I2C_Init(I2cHandle[obj->I2cId].Instance, I2cFrequency, NULL) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
    }
    I2cHandle[obj->I2cId].Frequency = I2cFrequency;
}

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cMcuDeInit( I2c_t *obj )
{
    I2C_Shutdown(I2cHandle[obj->I2cId].Instance);
}

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cMcuResetBus( I2c_t *obj )
{
    I2C_Shutdown(I2cHandle[obj->I2cId].Instance);
    I2C_Init(I2cHandle[obj->I2cId].Instance, I2cHandle[obj->I2cId].Frequency, NULL);
}

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of data bytes to write
 */
uint8_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t buf[16];
    uint16_t i;
    int err;


    if (size > 15) {
        return 0;
    }

    buf[0] = addr;

    for (i = 1; i <= size; i++) {
        buf[i] = *buffer++;
    }

    err = I2C_MasterWrite(I2cHandle[obj->I2cId].Instance, deviceAddr, buf, size + 1, 0);
    if (err != (size + 1)) {
        return 0;
    } else {
        return 1;
    }
}

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to read
 * \param [IN] size             number of data bytes to read
 */
uint8_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t buf[16];
    uint16_t i;
    int err;

    buf[0] = addr;

    err = I2C_MasterWrite(I2cHandle[obj->I2cId].Instance, deviceAddr, buf, 1, 1);
    if (err != 1) {
        return 0;
    }

    err = I2C_MasterRead(I2cHandle[obj->I2cId].Instance, deviceAddr, buf, size, 0);
    if (err != size) {
        return 0;
    } else {
        for (i = 0; i < size; i++) {
            *buffer++ = buf[i];
        }

        return 1;
    }
}

/*!
 * \brief Waits until the given device is in standby mode
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 */
uint8_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr )
{
    return 1;
}

/*!
 * \brief Sets the internal device address size
 *
 * \param [IN] obj              I2C object
 * \param [IN] addrSize         Internal address size
 */
void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
    I2cHandle[obj->I2cId].InternalAddressSize = addrSize;
}
