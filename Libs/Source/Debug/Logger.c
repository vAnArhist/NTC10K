/**
 ******************************************************************************
 * @file    Logger.c
 * @date    3/11/2020
 * @brief   Implements log for info, debug and error
 ******************************************************************************
 *
 * COPYRIGHT(c) 2019 Droid-Technologies LTD
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Droid-Technologies LTD nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdarg.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "Logger.h"

#if (LOG_LEVEL > LOG_LEVEL_NO_LOG)


/**
 * @brief:  Logger data structure.
 */
typedef struct {
    osMutexId_t         m_mutHandle;
    StaticSemaphore_t   m_mutControlBlock;
    char                m_buffer[LOG_BUFFER_SIZE];
    bool                (*pExternalCbfn)(const uint8_t * data, uint32_t len);
    bool                m_isInit;
} Logger_t;

/**
 * @brief:  Logger instance.
 */
static Logger_t Logger;

/* Private function prototypes */


/**
 * @brief:  Initialize Logger.
 * @param:  pExternalCbfn - External function to output data.
 * @return: None.
 */
void Logger_init(bool (*pExternalCbfn)(const uint8_t * data, uint32_t len))
{
    if (!Logger.m_mutHandle) {
        osMutexAttr_t l_mutAttr = {
            .name = "mutLogger",
            .cb_mem = &Logger.m_mutControlBlock,
            .cb_size = sizeof(Logger.m_mutControlBlock)
        };
        Logger.m_mutHandle = osMutexNew(&l_mutAttr);
    }

    Logger.pExternalCbfn = pExternalCbfn;
}

/**
 * @brief:  Logger printf implementation.
 * @param:  format - Pointer to format string.
 * @param:  ... - list of arguments for the format string.
 * @return: None.
 */
void Logger_printf(const char *format, ...)
{
    if (Logger.pExternalCbfn) {
        va_list args;
        va_start(args, format);

        vsnprintf(Logger.m_buffer, LOG_BUFFER_SIZE, format, args);
        Logger.m_buffer[LOG_BUFFER_SIZE-1] = 0;

        uint32_t len = strlen(Logger.m_buffer);

        Logger.pExternalCbfn((const uint8_t *)Logger.m_buffer, len);

        va_end(args);
    }
}


/**
 * @brief:  Wait logger mutex.
 * @return: 'true' if mutex is acquired, 'false' - otherwise.
 */
bool Logger_wait(void)
{
    if (!Logger.pExternalCbfn) {
        return false;
    }

    osStatus l_status =  osMutexWait(Logger.m_mutHandle, 1000);

    return  (l_status == osOK || l_status == osErrorISR);   //  ignore mutex if log function call from ISR
}

/**
 * @brief:  Release logger mutes.
 * @return: None.
 */
void Logger_release(void)
{
    osMutexRelease(Logger.m_mutHandle);
}

/**
 * @brief:  Print dump bytes.
 * @param:  bptr - Pointer to data.
 * @param:  len - Data length.
 * @return: None.
 */
void Logger_printDumpBytes(const void* bptr, uint32_t len)
{
    int i = 0;

    uint8_t* p = (uint8_t*) bptr;
    if (len > 0 && bptr != NULL) {
        for (i = 0; i < len;) {
            if ((i & 0x0f) == 0) {
                Logger_printf("%s   ", (i == 0) ? "" : "\n");
            } else if ((i & 0x07) == 0 && i != 0) {
                Logger_printf(" ");                            // 8 byte separator
            }
            Logger_printf(" %02X", p[i++]);
        }
        Logger_printf("\n");
    }
}


#endif // (LOG_LEVEL > LOG_LEVEL_NO_LOG)
