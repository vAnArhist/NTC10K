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

#include "RingBuffer.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define RING_ENTER_CRITICAL() taskENTER_CRITICAL()
#define RING_EXIT_CRITICAL() taskEXIT_CRITICAL()

#define RING_ENTER_CRITICAL_FROM_ISR() taskENTER_CRITICAL_FROM_ISR()
#define RING_EXIT_CRITICAL_FROM_ISR(x) taskEXIT_CRITICAL_FROM_ISR(x)


/**
* @brief  The constructor of RingBuffer class
* @param  bufferSize: uint16_t value of ring buffer size
* @retval none
*/
Lib::RingBuffer::RingBuffer()
{
    m_bufferSize = sizeof(m_buffer)/sizeof(m_buffer[0]);
    m_getIndex = 0;
    m_putIndex = 0;
    m_dataLength = 0;
}

/**
* @brief   Put new element to ring buffer. Use this method from UART IRQ handler to put new element
* @param   element:  new element.
* @retval  none.
*/
void Lib::RingBuffer::put(uint8_t element)
{
    uint32_t mask = RING_ENTER_CRITICAL_FROM_ISR();
    putElement(element);
    RING_EXIT_CRITICAL_FROM_ISR(mask);
}

/**
* @brief   Get first element from ring buffer. 
*          Use this method from task to get data from buffer.
* @retval  none.
*/
bool Lib::RingBuffer::get(uint8_t &byte)
{
    bool rv = false;

    RING_ENTER_CRITICAL();
    rv = getElement(byte);
    RING_EXIT_CRITICAL();

    return rv;
}

/**
* @brief   Get element from ring buffer.
*          You must check return value indicating if there was data to get.
* @note    This method is not thread safe!
* @retval  true on success. false - no data in buffer.
*/
bool Lib::RingBuffer::getElement(uint8_t &byte)
{
    if (m_dataLength == 0) {
        return false;
    }

    byte = m_buffer[m_getIndex++];
    if (m_getIndex >= m_bufferSize){
        m_getIndex = 0;
    }
    m_dataLength--;

    return true;
}

/**
* @brief   Get element from ring buffer.
*          You must check return value indicating if there was data to get.
* @note    This method is not thread safe!
* @retval  none
*/
void Lib::RingBuffer::putElement(uint8_t &byte)
{
    //if buffer is not empty
    if (m_dataLength != 0) {
        //if buffer is full
        if (m_putIndex == m_getIndex) {
            m_getIndex++;
            m_dataLength--;
            if (m_getIndex == m_bufferSize){
                m_getIndex = 0;
            }
        }
    }
    //put element to buffer
    m_buffer[m_putIndex++] = byte;
    if (m_putIndex == m_bufferSize){
        m_putIndex = 0;
    }
    m_dataLength++;
}

/**
* @brief   Get a number of elements from ring buffer. 
* @param   data:     Specify the pointer to array, where to save data.
* @param   length:   Specify the link to variable, where to set length of
*                    read data from ring buffer.
* @param   maxSize:  Specify  number of data to read from buffer.
* @retval  none.
*/
void Lib::RingBuffer::get(uint8_t* data, uint16_t& length, uint16_t maxSize)
{
    bool l_flag = false;

    length = 0;
    if (m_dataLength == 0) {
        return;
    }
    
    RING_ENTER_CRITICAL();
    //Protects against sudden change of m_dataLength
    uint32_t dataLength = m_dataLength;
    
    if(maxSize > dataLength)
        maxSize = dataLength;
    
    for(uint32_t i = 0; i < maxSize; i++) {

        l_flag  = getElement(data[i]);

        if (l_flag == false) {
            break;
        }
        length++;
    }
    RING_EXIT_CRITICAL();
}

/**
* @brief   Get a length of received elements in ring buffer. 
* @param   none.
* @retval  uint16_t: length of received elements.
*/
uint16_t Lib::RingBuffer::getDataLenght()
{
    return m_dataLength;
}

/**
* @brief   Clear the ring buffer. 
* @param   none.
* @retval  none.
*/
void Lib::RingBuffer::clear()
{
    RING_ENTER_CRITICAL();
    m_getIndex = m_putIndex = m_dataLength = 0;
    RING_EXIT_CRITICAL();
}
