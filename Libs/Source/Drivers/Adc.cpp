#include "Adc.hpp"
#include "string.h"
#include "Logger.h"
#include "cmsis_os.h"

#define ADC_RESOLUTION 4095

Driver::Adc::Adc()
{
    m_adcInstance = this;
    m_isInitialized = false;
    event = nullptr;
}

Driver::Adc::~Adc()
{
    stop();

    m_isInitialized = false;
    HAL_ADC_MspDeInit(&hadc1);
    m_adcInstance = nullptr;
}

void Driver::Adc::start()
{
    if (!m_isInitialized) {
        init();
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) m_adcDatau16, 10);
}

void Driver::Adc::stop()
{
    HAL_ADC_Stop_DMA(&hadc1);
}

void Driver::Adc::init()
{
    dmaInit();
    adcInit();
    m_isInitialized = true;
}

void Driver::Adc::process(uint16_t* values, uint16_t len)
{
    if (m_adcInstance == nullptr) {
        // Not initialized
        return;
    }

    int sum = 0;
    for(int i = 0; i<len; i++){
        sum += values[i];
    }
    m_values[0] = (uint16_t)(sum/len);
    if (m_adcInstance->event != NULL) {
        m_adcInstance->event->convertionComplete(m_adcInstance->m_values,
                                                 ADC_CHANNELS_COUNT);
    }
}

bool Driver::Adc::registerEvent(Interface::AdcEvent& evt)
{
    this->event = &evt;
    return true;
}

void Driver::Adc::adcInit()
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    /** Configure the global features of the ADC (Clock, Resolution,
     * Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION12b;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    // Sequencer disabled (ADC conversion on only 1 channel)
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    // Continuous mode enabled (automatic conversion restart after each conversion)
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    // ADC DMA continuous request to match with DMA circular mode
    hadc1.Init.DMAContinuousRequests = ENABLE;
    // DR register is overwritten with the last conversion result in
    // case of overrun
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    HAL_ADC_MspInit(&hadc1);

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        LOG_ERROR("ADC init fail\n");
    }

    /* ### - 2 - Start calibration ############################################ */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        LOG_ERROR("ADC calibration fail\n");
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_9; // PA4 (A3 on devboard)
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        LOG_ERROR("ADC channel init fail\n");
    }
}

void Driver::Adc::dmaInit()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void Driver::Adc::HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (adcHandle->Instance == ADC1) {
        /* ADC1 clock enable */
        __HAL_RCC_ADC_CLK_ENABLE();
        /* ADC Periph interface clock configuration */
        __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC1 GPIO Configuration
         PA1     ------> ADC1_IN6
         */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        hdma_adc1.Instance = DMA1_Channel1;
        hdma_adc1.Init.Request = DMA_REQUEST_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;

        /* Deinitialize  & Initialize the DMA for new transfer */
        HAL_DMA_DeInit(&hdma_adc1);
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            LOG_ERROR("ADC init fail\n");
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void Driver::Adc::HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

    if (adcHandle->Instance == ADC1) {
        // Peripheral clock disable
        __HAL_RCC_ADC_CLK_DISABLE();

        /**ADC1 GPIO Configuration
         PA4     ------> ADC1_IN6
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }
}

void Driver::Adc::dmaCompl()
{
    if (m_adcInstance != nullptr && m_adcInstance->m_isInitialized) {
        m_adcInstance->process(m_adcDatau16, 10);
    }
}

uint16_t Driver::Adc::getRawVal()
{
    return m_values[0];
}

uint16_t Driver::Adc::getMV()
{
    uint16_t mv = (3300 * getRawVal())/ADC_RESOLUTION;
    return mv;
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(Driver::Adc::getDmaHandle());
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    Driver::Adc::dmaCompl();
}

Driver::Adc *Driver::Adc::m_adcInstance = nullptr;
ADC_HandleTypeDef Driver::Adc::hadc1;
DMA_HandleTypeDef Driver::Adc::hdma_adc1;
uint16_t Driver::Adc::m_adcDatau16[10];
uint16_t Driver::Adc::m_values[AdcChannel_t::ADC_CHANNELS_COUNT];

