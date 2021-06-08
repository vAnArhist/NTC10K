#include "stdint.h"

#ifndef __INTERFACES_IUART_H
#define __INTERFACES_IUART_H

namespace Interfaces {

class IComport {
public:
    /**
     * @brief writes data to the bus. May be blocking or not.
     * @param data pointer to the data
     * @param size number of bytes to write
     * @return true in the case of success. Otherwise false.
     */
    virtual bool write(const uint8_t* buf, uint16_t len) = 0;

    /**
     * @brief read reads data from input buffer.
     * @param data pointer to the data storage
     * @param size number of bytes in storage
     * @param lenReceived number of bytes acutally read
     * @return true in the case of success. Otherwise false.
     */
    virtual bool read(uint8_t* buf, uint16_t len, uint16_t& lenReceived) = 0;

    /**
     * @brief isBusyTx checks if there is ongoing sata transmission.
     * @return true if Uart sends data now.
     */
    virtual bool isBusyTx() = 0;

    /**
     * @brief bytesAvailable get amount of data available for read
     * @return number of bytes in input buffer
     */
    virtual uint16_t bytesAvailable() = 0;

    /**
     * @brief clearRxBuffer clear input buffer
     */
    virtual void clearRxBuffer() = 0;
};
}

#endif // __INTERFACES_IUART_H
