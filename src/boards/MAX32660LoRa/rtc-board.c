/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *              (C)2013-2017 Semtech - STMicroelectronics
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
#include <stdbool.h>
#include "mxc_rtc.h"
#include "utilities.h"
#include "timer.h"
#include "systime.h"
#include "rtc-board.h"
#include "mxc_assert.h"

#define MIN_TIMEOUT_TICKS 4

#define TICKS_PER_MS (1000.0f/256.0f)

static bool RtcInitialized = false;

/*!
 * RTC timer context
 */
typedef struct
{
    uint32_t Time;  // Reference time
    // uint32_t Delay; // Reference Timeout duration
}RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * Used to store the Seconds and SubSeconds.
 *
 * WARNING: Temporary fix fix. Should use MCU NVM internal
 *          registers
 */
uint32_t RtcBkupRegisters[] = { 0, 0 };

void RtcInit( void )
{
    if (RtcInitialized) {
        return;
    }

    while (RTC_Init(MXC_RTC, 0, 0, NULL) == E_BUSY) {}
    while (RTC_EnableRTCE(MXC_RTC) == E_BUSY) {}
    NVIC_SetPriority(RTC_IRQn, 2);
    NVIC_EnableIRQ(RTC_IRQn);
    RtcSetTimerContext();
    RtcInitialized = true;
}

/*!
 * \brief Returns the minimum timeout value
 *
 * \retval minTimeout Minimum timeout value in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return MIN_TIMEOUT_TICKS;
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return milliseconds / TICKS_PER_MS;
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
TimerTime_t RtcTick2Ms( uint32_t tick )
{
    return tick * TICKS_PER_MS;
}

/*!
 * \brief Performs a delay of milliseconds by polling RTC
 *
 * \param[IN] milliseconds Delay in ms
 */
void RtcDelayMs( TimerTime_t milliseconds )
{
    uint32_t ticks;
    uint32_t reference;

    reference = RtcGetTimerValue();
    ticks = RtcMs2Tick(milliseconds);

    while ((RtcGetTimerValue() - reference) < ticks) {}
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this funtion) + timeout
 *
 * \param timeout [IN] Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    uint32_t alarm;

    CRITICAL_SECTION_BEGIN( );

    RtcStopAlarm( );

    if (timeout < MIN_TIMEOUT_TICKS) {
        timeout += MIN_TIMEOUT_TICKS;
    }
    alarm = -timeout;
    while (RTC_SetSubsecondAlarm(MXC_RTC, alarm) == E_BUSY) {}

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Stops the Alarm
 */
void RtcStopAlarm( void )
{
    CRITICAL_SECTION_BEGIN( );
    while (RTC_DisableSubsecondInterrupt(MXC_RTC) == E_BUSY) {}
    RTC_ClearFlags(MXC_F_RTC_CTRL_ALSF);
    CRITICAL_SECTION_END( );
}

/*!
 * \brief Starts wake up alarm
 *
 * \note  Alarm in RtcTimerContext.Time + timeout
 *
 * \param [IN] timeout Timeout value in ticks
 */
void RtcStartAlarm( uint32_t timeout )
{
    uint32_t now;
    uint32_t future;
    uint32_t alarm;

    CRITICAL_SECTION_BEGIN( );

    RtcStopAlarm( );

    future = RtcTimerContext.Time + timeout;
    now = RtcGetTimerValue();

    if (now >= future) {
        alarm = -(int)MIN_TIMEOUT_TICKS;
    } else {
        alarm = -(future - now);
    }

    while (RTC_SetSubsecondAlarm(MXC_RTC, alarm) == E_BUSY) {}

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Sets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = RtcGetTimerValue( );
    return RtcTimerContext.Time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \retval value Timer value in ticks
 */
uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

/*!
 * \brief Gets the system time with the number of seconds elapsed since epoch
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since epoch
 * \retval seconds Number of seconds elapsed since epoch
 */
uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    uint32_t ticks;

    ticks = RtcGetTimerValue( );
    *milliseconds = RtcTick2Ms(ticks);
    return ticks >> 8;
}

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
uint32_t RtcGetTimerValue( void )
{
    uint32_t seconds;
    uint32_t subseconds;

    while (RTC_GetTime(&seconds, &subseconds) == E_BUSY) {}
    return (seconds << 8) | subseconds;
}

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm in ticks.
 */
uint32_t RtcGetTimerElapsedTime( void )
{
    return RtcGetTimerValue() - RtcTimerContext.Time;
}

/*!
 * \brief Writes data0 and data1 to the RTC backup registers
 *
 * \param [IN] data0 1st Data to be written
 * \param [IN] data1 2nd Data to be written
 */
void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    CRITICAL_SECTION_BEGIN( );
    RtcBkupRegisters[0] = data0;
    RtcBkupRegisters[1] = data1;
    CRITICAL_SECTION_END( );
}

/*!
 * \brief Reads data0 and data1 from the RTC backup registers
 *
 * \param [OUT] data0 1st Data to be read
 * \param [OUT] data1 2nd Data to be read
 */
void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{
    CRITICAL_SECTION_BEGIN( );
    *data0 = RtcBkupRegisters[0];
    *data1 = RtcBkupRegisters[1];
    CRITICAL_SECTION_END( );
}

/*!
 * \brief Processes pending timer events
 */
void RtcProcess( void )
{
    MXC_ASSERT_FAIL();
}

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate in milliseconds
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return period;
}

void RTC_IRQHandler(void)
{
    extern void LP_DisableRTCAlarmWakeup(void);

    LP_DisableRTCAlarmWakeup();
    while (RTC_ClearFlags(MXC_F_RTC_CTRL_ALSF) == E_BUSY) {}
    while (RTC_DisableSubsecondInterrupt(MXC_RTC) == E_BUSY) {}

    // NOTE: The handler should take less then 1 ms otherwise the clock shifts
    TimerIrqHandler( );
}
