/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
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
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio_regs.h"
#include "gpio-board.h"

static Gpio_t *GpioIrq[16];

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    mxc_gpio_regs_t *port;

    obj->pin = pin;

    if (pin == NC) {
        return;
    }

    obj->mask = 1 << (pin & 0xFF);
    obj->port = MXC_GPIO0;

    port = (mxc_gpio_regs_t *)obj->port;

    if (config == PIN_OPEN_DRAIN) {
        if (type == PIN_NO_PULL) {
            port->pad_cfg1 &= ~obj->mask;
            port->pad_cfg2 &= ~obj->mask;
            port->ps &= ~obj->mask;
        } else if (type == PIN_PULL_UP) {
            port->pad_cfg1 |=  obj->mask;
            port->pad_cfg2 &= ~obj->mask;
            port->ps |= obj->mask;
        } else {
            port->pad_cfg1 &= ~obj->mask;
            port->pad_cfg2 |=  obj->mask;
            port->ps &= ~obj->mask;
        }
    } else {
        // No resitors on push-pull
        port->pad_cfg1 &= ~obj->mask;
        port->pad_cfg2 &= ~obj->mask;
        port->ps &= ~obj->mask;
    }

    if ((mode == PIN_INPUT) || (mode == PIN_ANALOGIC)) {
        port->out_en_clr = obj->mask;
        port->en_set = obj->mask;
        port->en1_clr = obj->mask;
        port->en2_clr = obj->mask;
    } else if (mode == PIN_OUTPUT) {
        GpioMcuWrite(obj, value);
        port->out_en_set = obj->mask;
        port->en_set = obj->mask;
        port->en1_clr = obj->mask;
        port->en2_clr = obj->mask;
    }
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    mxc_gpio_regs_t *port;

    if (obj && irqHandler) {
        port = (mxc_gpio_regs_t *)obj->port;

        obj->IrqHandler = irqHandler;

        // edge sensitive
        port->int_mod |= obj->mask;

        if (irqMode == IRQ_RISING_EDGE) {
            port->int_pol |= obj->mask;
            port->int_dual_edge &= ~obj->mask;
        } else if (irqMode == IRQ_FALLING_EDGE) {
            port->int_pol &= ~obj->mask;
            port->int_dual_edge &= ~obj->mask;
        } else {
            port->int_dual_edge |= obj->mask;
        }

        GpioIrq[obj->pin & 0xFF] = obj;
        port->int_en_set = obj->mask;
        NVIC_SetPriority((IRQn_Type)MXC_GPIO_GET_IRQ(0), 4);
        NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(0));

    } else {
        MXC_ASSERT_FAIL( );
    }
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    mxc_gpio_regs_t *port;

    if (obj) {
        port = (mxc_gpio_regs_t *)obj->port;

        port->int_en_clr = obj->mask;

        GpioIrq[obj->pin & 0xFF] = NULL;

    } else {
        MXC_ASSERT_FAIL( );
    }
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    mxc_gpio_regs_t *port;

    if (obj) {
        if (obj->pin != NC) {
            port = (mxc_gpio_regs_t *)obj->port;

            if (value) {
                port->out_set = obj->mask;
            } else {
                port->out_clr = obj->mask;
            }
        }
    } else {
        MXC_ASSERT_FAIL( );
    }
}

void GpioMcuToggle( Gpio_t *obj )
{
    mxc_gpio_regs_t *port;

    if (obj) {
        if (obj->pin != NC) {
            port = (mxc_gpio_regs_t *)obj->port;

            if (port->out & obj->mask) {
                port->out_clr = obj->mask;
            } else {
                port->out_set = obj->mask;
            }
        }
    } else {
        MXC_ASSERT_FAIL( );
    }
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if (obj) {
        if (obj->pin != NC) {
            return !!(((mxc_gpio_regs_t *)obj->port)->in & obj->mask);
        }
    } else {
        MXC_ASSERT_FAIL( );
    }

    return 0;
}

void GPIO0_IRQHandler(void)
{
    uint32_t stat;
    unsigned int pin;

    mxc_gpio_regs_t *gpio = MXC_GPIO_GET_GPIO(0);

    stat = gpio->int_stat;
    gpio->int_clr = stat;

    pin = 0;

    while (stat) {
        if (stat & 1) {
            if (GpioIrq[pin]->IrqHandler != NULL) {
                GpioIrq[pin]->IrqHandler(GpioIrq[pin]->Context);
            }
        }

        pin++;
        stat >>= 1;
    }
}
