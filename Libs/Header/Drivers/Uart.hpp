/**
 ******************************************************************************
 * @file    Uart.hpp
 * @date    10 March 2020
 * @brief   UART DRIVER
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 Droid-Technologies LTD
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

#ifndef __DRIVERS_UART_H
#define __DRIVERS_UART_H

#include "stm32l4xx_hal.h"
#include "main.h"

#include "RingBuffer.hpp"
#include <map>
#include "IUart.hpp"

#define UART_TX_TIMEOUT  200

namespace Driver {

    class Uart : public Interfaces::IComport
	{

        public:

    		typedef enum {
    	    	UART_2 = 0,
				UART_CNT
    		} Uart_t;

    	    static Uart *getInstance(Uart_t uart);

    	    bool write(const uint8_t* buf, uint16_t len);
    	    bool read(uint8_t* buf, uint16_t len, uint16_t& lenReceived);
    	    uint16_t bytesAvailable();
    	    void clearRxBuffer();

            bool writeData(uint8_t* data, uint16_t length);
            bool readData(uint8_t* data, uint16_t& recLen, uint16_t size);
            void flushRx(void);
            uint16_t availableRx(void);
            bool isBusyTx(void);
            void rxBufferAdd(void);
            static void rxBufferAddCallback(UART_HandleTypeDef* huart);
            static void rxRestartReceive(UART_HandleTypeDef* huart);
            static UART_HandleTypeDef* getHandle(Uart_t uart);

        private:

            uint8_t m_tmp;
            Uart_t m_uartNumb;

            // Receiver buffer size
            Lib::RingBuffer m_buffer;

            static Uart* m_objects[Uart_t::UART_CNT];
            static UART_HandleTypeDef m_hUart[Uart_t::UART_CNT];

            Uart(Uart_t Uart_t);
            ~Uart();

            bool init();

            bool uartInit();
            bool mspInit();
            bool mspDeInit();
    };
}

#endif //__DRIVERS_UART_H
