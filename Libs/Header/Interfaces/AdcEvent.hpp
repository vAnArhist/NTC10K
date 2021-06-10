#ifndef __ADC_EVENT_H
#define __ADC_EVENT_H

#include "stdint.h"

namespace Interface{
class AdcEvent{
public:
	virtual void convertionComplete(uint16_t* values, uint16_t len) = 0;
};

}

#endif // __EVENT_H
