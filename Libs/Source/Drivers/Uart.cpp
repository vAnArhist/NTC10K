/**
 ******************************************************************************
 * @file    <Uart.cpp>
 * @date    <10 March 2020>
 * @brief   <Driver for UART>
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

#include "Uart.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include "Logger.h"

static bool txIsBusy = true;

Driver::Uart* Driver::Uart::getInstance(Driver::Uart::Uart_t uartNumb)
{

    if (uartNumb >= Uart_t::UART_CNT) {
        return nullptr;
    }

    if (m_objects[uartNumb] == nullptr) {
        m_objects[uartNumb] = new Uart(uartNumb);
        m_objects[uartNumb]->init();
    }

    return m_objects[uartNumb];
}

/**
 * @brief  The constructor of UART class
 * @param  port: Instance of UartNumber, UART port (kUartNumber1 ... kUartNumber2)
 * @param  baud: Instance of UartBaudRate (kUartBaudRate_9600 ... kUartBaudRate_256000)
 * @retval none
 */
Driver::Uart::Uart(Uart_t uartNumb)
{
    m_tmp = 0;
    m_uartNumb = uartNumb;
}

Driver::Uart::~Uart()
{

    mspDeInit();
}

bool Driver::Uart::init(void)
{
    if (mspInit() == false) {
        return false;
    }
    if (uartInit() == false) {
        return false;
    }

    HAL_UART_Receive_IT(&m_hUart[m_uartNumb], &m_tmp, 1);

    return true;
}

/**
 * @brief  Send a data to UARTx
 * @param  buf: pointer to data source array
 * @param  len: array length
 * @retval none
 */
bool Driver::Uart::write(const uint8_t* buf, uint16_t len)
{
    HAL_StatusTypeDef l_error = HAL_ERROR;

    if (buf == NULL || len == 0) {
        return false;
    }

    taskENTER_CRITICAL();
    l_error = HAL_UART_Transmit_IT(&m_hUart[m_uartNumb], (uint8_t*) buf, len);
    taskEXIT_CRITICAL();

    if (l_error == HAL_OK) {
        return true;
    }

    return false;
}

/**
 * @brief  Receive a data from UARTx
 * @param  buf: pointer to destination data array
 * @param  len: maximum number of bytes that data buffer can received
 * @param  lenReceived: result array length
 * @retval none
 */
bool Driver::Uart::read(uint8_t* buf, uint16_t len, uint16_t& lenReceived)
{

    lenReceived = 0;

    if (buf == nullptr || len == 0) {

        return false;
    }

    m_buffer.get(buf, lenReceived, len);
    return true;
}

/**
 * @brief  How byte in the RingBuffer
 * @param  none
 * @retval number byte
 */
uint16_t Driver::Uart::bytesAvailable(void)
{

    uint16_t l_len;

    l_len = this->m_buffer.getDataLenght();

    return l_len;
}

/**
 * @brief  Clear RingBuffer
 * @param  none
 * @retval none
 */
void Driver::Uart::clearRxBuffer(void)
{

    this->m_buffer.clear();
}

/**
 * @brief  Send a data to UARTx
 * @param  data: pointer to data source array
 * @param  length: array length
 * @retval none
 */
bool Driver::Uart::writeData(uint8_t* data, uint16_t length)
{
    HAL_StatusTypeDef l_error = HAL_ERROR;

    if (data == NULL || length == 0) {
        return false;
    }

    taskENTER_CRITICAL();
    l_error = HAL_UART_Transmit_IT(&m_hUart[m_uartNumb], data, length);
    taskEXIT_CRITICAL();

    if (l_error == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief  Receive a data from UARTx
 * @param  data: pointer to destination data array
 * @param  length: result array length
 * @param  maxSize: maximum number of bytes that data buffer can received
 * @retval none
 */
bool Driver::Uart::readData(uint8_t* data, uint16_t& recLen, uint16_t size)
{

    recLen = 0;

    if (size == 0) {
        return false;
    }

    m_buffer.get(data, recLen, size);
    return true;
}

/**
 * @brief  Status of Transmitting
 * @param  none
 * @retval true - previos transmittion in progress (busy),
 *         false - can start new Transmition
 */
bool Driver::Uart::isBusyTx(void)
{

    if (m_hUart[m_uartNumb].gState == HAL_UART_STATE_READY) {
        return false;
    }
    return true;
}

/**
 * @brief  How byte in the RingBuffer
 * @param  none
 * @retval number byte
 */
uint16_t Driver::Uart::availableRx(void)
{

    uint16_t l_len;

    l_len = this->m_buffer.getDataLenght();

    return l_len;
}

/**
 * @brief  Clear RingBuffer
 * @param  none
 * @retval none
 */
void Driver::Uart::flushRx(void)
{

    this->m_buffer.clear();
}

/**
 * @brief  Add new byte from UART in to Receiver buffer
 * @param  none
 * @retval none
 * @notes  This function called from rxBufferAddCallback() static function
 */
void Driver::Uart::rxBufferAdd(void)
{
    this->m_buffer.put(m_tmp);
    // Restart receiving procedure
    HAL_StatusTypeDef stat1 = HAL_UART_Receive_IT(&m_hUart[m_uartNumb], &m_tmp,
                                                  1);

    if (stat1 != HAL_OK) {
        Driver::Uart::rxRestartReceive(&m_hUart[m_uartNumb]);
    }

}

/**
 * @brief  Find UART handle from in number
 * @param  u_number: kUartNumber1...kUartNumber2...
 * @retval UART handle
 * @notes  This function must be called from USARTx_IRQHandler() ISR
 */
//UART_HandleTypeDef* Driver::Uart::getHandle(Uarts u_number)
UART_HandleTypeDef* Driver::Uart::getHandle(Uart_t uartNumb)
{
    if (uartNumb >= UART_CNT) {
        return nullptr;
    }

    return &m_hUart[uartNumb];
}
;

/**
 * @brief  Find coresponding UART input buffer from handle
 * @param  huart: handle
 * @retval none
 * @notes  This function must be called from HAL_UART_RxCpltCallback() function
 */
void Driver::Uart::rxBufferAddCallback(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < UART_CNT; i++) {
        if (huart == &m_hUart[i]) {

            m_objects[i]->rxBufferAdd();

            return;
        }
    }
}

/**
 * @brief   Restart receive if error occured.
 * @param   huart: handle.
 * @retval  none.
 */
void Driver::Uart::rxRestartReceive(UART_HandleTypeDef* m_hUARTx)
{
    for (uint8_t i = 0; i < UART_CNT; i++) {
        if (m_hUARTx == &m_hUart[i]) {
            HAL_UART_Receive_IT(&m_objects[i]->m_hUart[i], &m_objects[i]->m_tmp,
                                1);
            return;
        }
    }
}

/*
 * @brief   USART IRQ Handler, defined in vector table
 * @param   none
 * @retval  none
 */
extern "C" void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(Driver::Uart::getHandle(Driver::Uart::UART_2));
}

// Transmitted OK.
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    txIsBusy = false;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    Driver::Uart::rxBufferAddCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != HAL_UART_ERROR_NONE) {
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        Driver::Uart::rxRestartReceive(huart);
    }
}

bool Driver::Uart::uartInit()
{

    if (m_uartNumb == Uart_t::UART_2) {
        m_hUart[UART_2].Instance = USART2;
        m_hUart[UART_2].Init.BaudRate = 115200;
        m_hUart[UART_2].Init.WordLength = UART_WORDLENGTH_8B;
        m_hUart[UART_2].Init.StopBits = UART_STOPBITS_1;
        m_hUart[UART_2].Init.Parity = UART_PARITY_NONE;
        m_hUart[UART_2].Init.Mode = UART_MODE_TX_RX;
        m_hUart[UART_2].Init.HwFlowCtl = UART_HWCONTROL_NONE;
        m_hUart[UART_2].Init.OverSampling = UART_OVERSAMPLING_16;
        m_hUart[UART_2].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        m_hUart[UART_2].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&m_hUart[UART_2]) != HAL_OK) {
            Error_Handler();
        }
    } else {
        return false;
    }

    return true;
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
bool Driver::Uart::mspInit()
{

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if (m_uartNumb == Uart_t::UART_2) {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**UART5 GPIO Configuration
         PA2     ------> USART2_TX
         PA15    ------> USART2_RX
         */
        GPIO_InitStruct.Pin = VCP_TX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = VCP_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
        HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

        /* UART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ (USART2_IRQn);
    } else {
        return false;
    }

    return true;
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
bool Driver::Uart::mspDeInit()
{

    if (m_uartNumb == Uart_t::UART_2) {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA15 (JTDI)     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOA, VCP_TX_Pin|VCP_RX_Pin);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    } else {
        return false;
    }

    return true;
}

Driver::Uart *Driver::Uart::m_objects[Uart_t::UART_CNT] = { nullptr };
UART_HandleTypeDef Driver::Uart::m_hUart[Uart_t::UART_CNT];
