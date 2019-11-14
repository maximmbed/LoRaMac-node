/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \author    Jesse Marroquin ( Maxim Integrated )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      1 // 5

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 13

#define RADIO_MOSI                                  5
#define RADIO_MISO                                  4
#define RADIO_SCLK                                  6

#define RADIO_NSS                                   7
#define RADIO_BUSY                                  8
#define RADIO_DIO_1                                 9

#define RADIO_ANT_SWITCH_POWER                      NC
#define RADIO_FREQ_SEL                              NC
#define RADIO_XTAL_SEL                              NC
#define RADIO_DEVICE_SEL                            NC

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            10
#define RADIO_DBG_PIN_RX                            11

#define I2C_SCL                                     2
#define I2C_SDA                                     3

/*!
 * Flash address where NVM module will store/load data
 * Note: Address must be beginning of a page
 */
#define MXIM_NVM_BASE                               0x0001C000

#endif // __BOARD_CONFIG_H__
