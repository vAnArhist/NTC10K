/**
 ******************************************************************************
 * @file    <RingBuffer.cpp>
 * @date    <02 August 2019>
 * @brief   <Lib for RingBuffer>
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

#ifndef __TOOLS_RINGBUFFER_H
#define __TOOLS_RINGBUFFER_H

#include <stdint.h>

namespace Lib {

    class RingBuffer {
    
        public:
            RingBuffer();
            virtual ~RingBuffer(){}
        
            void put(uint8_t element);
            void get(uint8_t* data, uint16_t& length, uint16_t maxSize);
            bool get(uint8_t &byte);
            uint16_t getDataLenght();
            void clear();
        
        private:
            
            uint8_t m_buffer[512];
            uint16_t m_bufferSize;
            uint16_t m_putIndex;
            uint16_t m_getIndex;
            uint16_t m_dataLength;
        
            bool getElement(uint8_t &byte);
            void putElement(uint8_t &byte);

    };
}

#endif

