/**
 ******************************************************************************
 * @file    Ntc.hpp
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

#ifndef __NTC_H
#define __NTC_H

#include "AdcEvent.hpp"
#include "Adc.hpp"
#include "cmsis_os.h"

namespace Driver {
class Ntc: public Interface::AdcEvent
{
public:
    static Ntc* getInstance();

    uint16_t getResistance();
    float getTemperature();

protected:
    void convertionComplete(uint16_t* values, uint16_t len);

private:
    Driver::Adc adc;

    static bool m_initialized;
    static Ntc *m_instance;
    static uint16_t m_ntcResistance;
    static float m_ntcTemp;

    Ntc();
    virtual ~Ntc();

    static void NtcTask(void* arg);
    bool init();

    uint16_t FilterSMA(uint16_t* forFiltered, uint16_t len);
};

}
;

#endif //__NTC_H
