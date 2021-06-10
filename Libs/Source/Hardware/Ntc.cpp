/**
 ******************************************************************************
 * @file    Ntc.cpp
 * @date    08-June-2021
 * @author  Ivan Lopashchenko <ivan.lopashchenko@droid-technologies.com>
 * @brief   Temperature driver
 *
 ******************************************************************************
 *
 * COPYRIGHT(c) 2021 Droid-Technologies LLC
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of Droid-Technologies LLC nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************
 */

#include "Ntc.hpp"
#include "Logger.h"
#include <math.h>

#define NTC_LOG_PREFIX  "NTC: "
/* R1 resistance */
#define NTC_UP_R 10000.0f
/* constants of Steinhart-Hart equation */
#define A 0.001111f
#define B 0.000237987f
#define C 0.000000065f
#define FILTER_SMA_ORDER 10

uint16_t Filter_Buffer[FILTER_SMA_ORDER] = {0,};

// This macro stops the program if the expr is 0, NULL, false
#define DT_ASSERT_PARAM(expr) do{if((expr) == 0) {while(1);}}while(0)

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define NTC_LOG_DEBUG(...)             LOG_DEBUG(NTC_LOG_PREFIX __VA_ARGS__)
#define NTC_LOG_DEBUG_WP(...)          LOG_DEBUG_WP(__VA_ARGS__)
#define NTC_LOG_DEBUG_DUMP(buf, size)  LOG_DEBUG_DUMP(buf, size)
#else
#define NTC_LOG_DEBUG(...)             do{}while(0)
#define NTC_LOG_DEBUG_WP(...)          do{}while(0)
#define NTC_LOG_DEBUG_DUMP(buf, size)  do{}while(0)
#endif

Driver::Ntc* Driver::Ntc::getInstance()
{
    if (!m_instance) {
        m_instance = new Ntc();
        m_instance->init();
    }

    return m_instance;
}

bool Driver::Ntc::init()
{
    if (!m_initialized) {
        m_initialized = true;
        m_instance->adc.registerEvent(*m_instance);
        m_instance->adc.start();
    }
    return true;
}

Driver::Ntc::Ntc() :
        adc()
{

}

Driver::Ntc::~Ntc()
{

}

void Driver::Ntc::convertionComplete(uint16_t* values, uint16_t len)
{
    m_instance->m_ntcResistance = ((NTC_UP_R)/((4095.0/values[0]) - 1));
    float Ntc_Ln = log(m_instance->m_ntcResistance);
    // calc temperature
    m_instance->m_ntcTemp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15;
}

float Driver::Ntc::getTemperature()
{
    return (m_instance->m_ntcTemp);
}

uint16_t Driver::Ntc::getResistance()
{
    return (m_instance->m_ntcResistance);
}

Driver::Ntc *Driver::Ntc::m_instance = NULL;
bool Driver::Ntc::m_initialized = false;
uint16_t Driver::Ntc::m_ntcResistance = 0;
float Driver::Ntc::m_ntcTemp = 0;
