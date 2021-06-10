#ifndef __DRIVERS_ADC_H
#define __DRIVERS_ADC_H

#include "AdcEvent.hpp"
#include "stdint.h"

#include "stm32l4xx_hal.h"

namespace Driver{

class Adc
{
public:
	typedef enum {
		ADC_CHANNEL0 = 0,
		ADC_CHANNELS_COUNT
	}AdcChannel_t;

    static ADC_HandleTypeDef *getAdcHandle(){return &hadc1;};
    static DMA_HandleTypeDef *getDmaHandle(){return &hdma_adc1;};

	Adc();
	virtual ~Adc();

	void start();
	void stop();
	uint16_t getRawVal();
	uint16_t getMV();
	bool registerEvent(Interface::AdcEvent& evt);

	virtual void callback(uint16_t* chValues, uint16_t chNum) {};

	static void dmaHalfCompl();
	static void dmaCompl();
private:

	Interface::AdcEvent* event;
	bool m_isInitialized;

	static ADC_HandleTypeDef hadc1;
	static DMA_HandleTypeDef hdma_adc1;
	static Adc* m_adcInstance;

	static uint16_t m_values[ADC_CHANNELS_COUNT];
	static uint16_t m_adcDatau16[10];

	void init();
	void process(uint16_t* values, uint16_t len);
	void adcInit();
	void dmaInit();

	void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
	void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle);
};

}

#endif // __DRIVERS_ADC_H
