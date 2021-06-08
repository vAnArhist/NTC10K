/**
 ******************************************************************************
 * @file    Logger.h
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

/*******************************************************************************
    @usage:

                1) Select debug level.
                There are 5 levels for messages:
                    LOG_LEVEL_NO_LOG  = 0
                    LOG_LEVEL_ERROR   = 1
                    LOG_LEVEL_WARNING = 2
                    LOG_LEVEL_INFO    = 3
                    LOG_LEVEL_DEBUG   = 4
                All messages above the selected level will be excluded from the
                code by conditional compilation.
                To enable debug level add global define, like:
                    LOG_LEVEL=LOG_LEVEL_DEBUG


                2) It is possible to get an extended log with information about
                the file and the line in which the message was generated.
                To enable this mode add global define
                        LOG_VERBOSE=1
                Example normal log:
                        [D]  Message
                Example extended log:
                        [D]  154: Main.cpp                      : Message


                3) Three types of messages are available:
                no suffix - the message is printed with additional markers:
                logging level, line, file name (if LOG_VERBOSE = 1), message.

                suffix _WP - prints message without any additional markers.
                suffix _DUMP - prints bytes dump in HEX. Like:
                    01 02 03 04 05 06 07 08  09 0A 0B 0C 0D 0E 01 10
                    11 12 13 14 15 16 17 18  19 1A 1B 1C 1D 1E 1F 20


                4) In order to print float add linker flag '-u_printf_float'


                5) Initialize the logger using the LOG_INIT() macro,
                specifying the function to output as a parameter.

*******************************************************************************/
#ifndef __LOGGER_H_
#define __LOGGER_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Log level */
#define LOG_LEVEL_NO_LOG        0
#define LOG_LEVEL_ERROR         1
#define LOG_LEVEL_WARNING       2
#define LOG_LEVEL_INFO          3
#define LOG_LEVEL_DEBUG         4

#define LOG_LEVEL LOG_LEVEL_DEBUG
#if !defined(LOG_LEVEL)
#define LOG_LEVEL               LOG_LEVEL_NO_LOG
#endif


#if (LOG_LEVEL > LOG_LEVEL_NO_LOG)


#if !defined(LOG_VERBOSE)
#define LOG_VERBOSE             0
#endif

#define LOG_BUFFER_SIZE         256


#if (LOG_VERBOSE > 0)
/* Get file name without full path */
#define __FILENAME__                    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG_FORMAT_STR(LEVEL, FMT, ...) Logger_printf("[%s] %4d: %-30s: " FMT, #LEVEL, __LINE__, __FILENAME__, ##__VA_ARGS__);
#else
#define LOG_FORMAT_STR(LEVEL, FMT, ...) Logger_printf("[%s] " FMT,  ##__VA_ARGS__);
#endif


#define LOG(LEVEL, FMT, ...)            do {                                                        \
                                            if (Logger_wait()) {                                    \
                                                LOG_FORMAT_STR(LEVEL, FMT, #LEVEL, ##__VA_ARGS__)   \
                                                Logger_release();                                   \
                                            }                                                       \
                                        } while(0)

#define LOG_WP(...)                     do {                                \
                                            if (Logger_wait()) {            \
                                                Logger_printf(__VA_ARGS__); \
                                                Logger_release();           \
                                            }                               \
                                        } while(0)

#define LOG_DUMP(DATA,LEN)              do {                                        \
                                            if (Logger_wait()) {                    \
                                                Logger_printDumpBytes(DATA, LEN);   \
                                                Logger_release();                   \
                                            }                                       \
                                        } while(0)

void Logger_printf(const char *format, ...);
bool Logger_wait(void);
void Logger_release(void);
void Logger_printDumpBytes(const void *bptr, uint32_t len);

void Logger_init(bool (*pExternalCbfn)(const uint8_t *data, uint32_t len));

#endif // (LOG_LEVEL > LOG_LEVEL_NO_LOG)




/* User MACRO */

#if (LOG_LEVEL > LOG_LEVEL_NO_LOG)
#define LOG_INIT(X)  Logger_init(X)
#else
#define LOG_INIT(X)  do { } while(0)
#endif // (LOG_LEVEL > LOG_LEVEL_NO_LOG)



#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
#define LOG_DEBUG(FMT, ...)             LOG(D, FMT "", ##__VA_ARGS__)
#define LOG_DEBUG_WP(...)               LOG_WP(__VA_ARGS__)
#define LOG_DEBUG_DUMP(DATA, LEN)       LOG_DUMP(DATA,LEN)
#else
#define LOG_DEBUG(...)                  do { } while(0)
#define LOG_DEBUG_WP(...)               do { } while(0)
#define LOG_DEBUG_DUMP(...)             do { } while(0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_INFO)
#define LOG_INFO(FMT, ...)              LOG(I, FMT "", ##__VA_ARGS__)
#define LOG_INFO_WP(...)                LOG_WP(__VA_ARGS__)
#define LOG_INFO_DUMP(DATA, LEN)        LOG_DUMP(DATA,LEN)
#else
#define LOG_INFO(...)                   do { } while(0)
#define LOG_INFO_WP(...)                do { } while(0)
#define LOG_INFO_DUMP(...)              do { } while(0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_WARNING)
#define LOG_WARNING(FMT, ...)           LOG(W, FMT "", ##__VA_ARGS__)
#define LOG_WARNING_WP(...)             LOG_WP(__VA_ARGS__)
#define LOG_WARNING_DUMP(DATA, LEN)     LOG_DUMP(DATA,LEN)
#else
#define LOG_WARNING(...)                do { } while(0)
#define LOG_WARNING_WP(...)             do { } while(0)
#define LOG_WARNING_DUMP(...)           do { } while(0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_ERROR)
#define LOG_ERROR(FMT, ...)             LOG(E, FMT "", ##__VA_ARGS__)
#define LOG_ERROR_WP(...)               LOG_WP(__VA_ARGS__)
#define LOG_ERROR_DUMP(DATA, LEN)       LOG_DUMP(DATA,LEN)
#else
#define LOG_ERROR(...)                  do { } while(0)
#define LOG_ERROR_WP(...)               do { } while(0)
#define LOG_ERROR_DUMP(...)             do { } while(0)
#endif



#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H_ */
