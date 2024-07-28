#pragma once

#include <cstdint>

#include "stm32g4xx.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_comp.h"
#include "stm32g4xx_hal_cordic.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_crc.h"
#include "stm32g4xx_hal_crc_ex.h"
#include "stm32g4xx_hal_cryp.h"
#include "stm32g4xx_hal_cryp_ex.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_dac_ex.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_dma_ex.h"
#include "stm32g4xx_hal_exti.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
// #include "stm32g4xx_hal_flash_ramfunc.h"
#include "stm32g4xx_hal_fmac.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_gpio_ex.h"
#include "stm32g4xx_hal_hrtim.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_i2c_ex.h"
#include "stm32g4xx_hal_i2s.h"
#include "stm32g4xx_hal_irda.h"
#include "stm32g4xx_hal_irda_ex.h"
#include "stm32g4xx_hal_iwdg.h"
#include "stm32g4xx_hal_lptim.h"
#include "stm32g4xx_hal_nand.h"
#include "stm32g4xx_hal_nor.h"
#include "stm32g4xx_hal_opamp.h"
#include "stm32g4xx_hal_opamp_ex.h"
#include "stm32g4xx_hal_pcd.h"
#include "stm32g4xx_hal_pcd_ex.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_qspi.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_rcc_ex.h"
#include "stm32g4xx_hal_rng.h"
#include "stm32g4xx_hal_rtc.h"
#include "stm32g4xx_hal_rtc_ex.h"
#include "stm32g4xx_hal_sai.h"
#include "stm32g4xx_hal_sai_ex.h"
#include "stm32g4xx_hal_smartcard.h"
#include "stm32g4xx_hal_smartcard_ex.h"
#include "stm32g4xx_hal_smbus.h"
#include "stm32g4xx_hal_smbus_ex.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_spi_ex.h"
#include "stm32g4xx_hal_sram.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include "stm32g4xx_hal_usart.h"
#include "stm32g4xx_hal_usart_ex.h"
#include "stm32g4xx_hal_wwdg.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_crc.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_fmc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_lptim.h"
#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_rng.h"
#include "stm32g4xx_ll_rtc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_usb.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_wwdg.h"

namespace hal
{
    enum class GpioBase : uint32_t
    {
        kUnknown = 0,
#ifdef GPIOA
        kPortA = GPIOA_BASE,
#endif
#ifdef GPIOB
        kPortB = GPIOB_BASE,
#endif
#ifdef GPIOC
        kPortC = GPIOC_BASE,
#endif
#ifdef GPIOD
        kPortD = GPIOD_BASE,
#endif
#ifdef GPIOE
        kPortE = GPIOE_BASE,
#endif
#ifdef GPIOF
        kPortF = GPIOF_BASE,
#endif
#ifdef GPIOG
        kPortG = GPIOG_BASE,
#endif
#ifdef GPIOH
        kPortH = GPIOH_BASE,
#endif
    };

    enum class DmaBase : uint32_t
    {
        kUnknown = 0,
#ifdef DMA1_BASE
        kDma1 = DMA1_BASE,
#endif
#ifdef DMA2_BASE
        kDma2 = DMA2_BASE
#endif
    };

    enum class DmaChBase : uint32_t
    {
        kUnknown = 0,
#ifdef DMA1_Channel1_BASE
        kDma1Ch1 = DMA1_Channel1_BASE,
#endif
#ifdef DMA1_Channel2_BASE
        kDma1Ch2 = DMA1_Channel2_BASE,
#endif
#ifdef DMA1_Channel3_BASE
        kDma1Ch3 = DMA1_Channel3_BASE,
#endif
#ifdef DMA1_Channel4_BASE
        kDma1Ch4 = DMA1_Channel4_BASE,
#endif
#ifdef DMA1_Channel5_BASE
        kDma1Ch5 = DMA1_Channel5_BASE,
#endif
#ifdef DMA1_Channel6_BASE
        kDma1Ch6 = DMA1_Channel6_BASE,
#endif
#ifdef DMA1_Channel7_BASE
        kDma1Ch7 = DMA1_Channel7_BASE,
#endif
#ifdef DMA1_Channel8_BASE
        kDma1Ch8 = DMA1_Channel8_BASE,
#endif
#ifdef DMA2_Channel1_BASE
        kDma2Ch1 = DMA2_Channel1_BASE,
#endif
#ifdef DMA2_Channel2_BASE
        kDma2Ch2 = DMA2_Channel2_BASE,
#endif
#ifdef DMA2_Channel3_BASE
        kDma2Ch3 = DMA2_Channel3_BASE,
#endif
#ifdef DMA2_Channel4_BASE
        kDma2Ch4 = DMA2_Channel4_BASE,
#endif
#ifdef DMA2_Channel5_BASE
        kDma2Ch5 = DMA2_Channel5_BASE,
#endif
#ifdef DMA2_Channel6_BASE
        kDma2Ch6 = DMA2_Channel6_BASE,
#endif
#ifdef DMA2_Channel7_BASE
        kDma2Ch7 = DMA2_Channel7_BASE,
#endif
#ifdef DMA2_Channel8_BASE
        kDma2Ch8 = DMA2_Channel8_BASE,
#endif
    };

    enum class ComparatorBase : uint32_t
    {
        kUnknown = 0,
#ifdef COMP1_BASE
        kComp1 = COMP1_BASE,
#endif
#ifdef COMP2_BASE
        kComp2 = COMP2_BASE,
#endif
#ifdef COMP3_BASE
        kComp3 = COMP3_BASE,
#endif
#ifdef COMP4_BASE
        kComp4 = COMP4_BASE,
#endif
#ifdef COMP5_BASE
        kComp5 = COMP5_BASE,
#endif
#ifdef COMP6_BASE
        kComp6 = COMP6_BASE,
#endif
#ifdef COMP7_BASE
        kComp7 = COMP7_BASE,
#endif
    };

    enum class AdcCh : uint32_t
    {
        kUnknown = 0,
#ifdef ADC_CHANNEL_TEMPSENSOR
        kAdcChTempSensor = ADC_CHANNEL_TEMPSENSOR,
#endif
#ifdef ADC_CHANNEL_TEMPSENSOR_ADC1
        kAdc1ChTempSensor = ADC_CHANNEL_TEMPSENSOR_ADC1,
#endif
#ifdef ADC_CHANNEL_TEMPSENSOR_ADC5
        kAdc5ChTempSensor = ADC_CHANNEL_TEMPSENSOR_ADC5,
#endif
#ifdef ADC_CHANNEL_VREFINT
        kAdcChVrefInt = ADC_CHANNEL_VREFINT,
#endif
#ifdef ADC_CHANNEL_0
        kAdcCh0 = ADC_CHANNEL_0,
#endif
#ifdef ADC_CHANNEL_1
        kAdcCh1 = ADC_CHANNEL_1,
#endif
#ifdef ADC_CHANNEL_2
        kAdcCh2 = ADC_CHANNEL_2,
#endif
#ifdef ADC_CHANNEL_3
        kAdcCh3 = ADC_CHANNEL_3,
#endif
#ifdef ADC_CHANNEL_4
        kAdcCh4 = ADC_CHANNEL_4,
#endif
#ifdef ADC_CHANNEL_5
        kAdcCh5 = ADC_CHANNEL_5,
#endif
#ifdef ADC_CHANNEL_6
        kAdcCh6 = ADC_CHANNEL_6,
#endif
#ifdef ADC_CHANNEL_7
        kAdcCh7 = ADC_CHANNEL_7,
#endif
#ifdef ADC_CHANNEL_8
        kAdcCh8 = ADC_CHANNEL_8,
#endif
#ifdef ADC_CHANNEL_9
        kAdcCh9 = ADC_CHANNEL_9,
#endif
#ifdef ADC_CHANNEL_10
        kAdcCh10 = ADC_CHANNEL_10,
#endif
#ifdef ADC_CHANNEL_11
        kAdcCh11 = ADC_CHANNEL_11,
#endif
#ifdef ADC_CHANNEL_12
        kAdcCh12 = ADC_CHANNEL_12,
#endif
#ifdef ADC_CHANNEL_13
        kAdcCh13 = ADC_CHANNEL_13,
#endif
#ifdef ADC_CHANNEL_14
        kAdcCh14 = ADC_CHANNEL_14,
#endif
#ifdef ADC_CHANNEL_15
        kAdcCh15 = ADC_CHANNEL_15,
#endif
#ifdef ADC_CHANNEL_16
        kAdcCh16 = ADC_CHANNEL_16,
#endif
#ifdef ADC_CHANNEL_17
        kAdcCh17 = ADC_CHANNEL_17,
#endif
#ifdef ADC_CHANNEL_18
        kAdcCh18 = ADC_CHANNEL_18,
#endif
#ifdef ADC_CHANNEL_19
        kAdcCh19 = ADC_CHANNEL_19,
#endif
#ifdef ADC_CHANNEL_20
        kAdcCh20 = ADC_CHANNEL_20,
#endif
    };

    enum class AdcBase : uint32_t
    {
        kUnknown = 0,
#ifdef ADC1_BASE
        kAdc1 = ADC1_BASE,
#endif
#ifdef ADC2_BASE
        kAdc2 = ADC2_BASE,
#endif
#ifdef ADC3_BASE
        kAdc3 = ADC3_BASE,
#endif
#ifdef ADC4_BASE
        kAdc4 = ADC4_BASE,
#endif
#ifdef ADC5_BASE
        kAdc5 = ADC5_BASE,
#endif
    };

    enum class SpiBase : uint32_t
    {
        kUnknown = 0,
#ifdef SPI1_BASE
        kSpi1 = SPI1_BASE,
#endif
#ifdef SPI2_BASE
        kSpi2 = SPI2_BASE,
#endif
#ifdef SPI3_BASE
        kSpi3 = SPI3_BASE,
#endif
#ifdef SPI4_BASE
        kSpi4 = SPI4_BASE,
#endif
#ifdef SPI5_BASE
        kSpi5 = SPI5_BASE,
#endif
#ifdef SPI6_BASE
        kSpi6 = SPI6_BASE,
#endif
    };

    enum class CanBase : uint32_t
    {
        kUnknown = 0,
#ifdef FDCAN1_BASE
        kCan1 = FDCAN1_BASE,
#endif
#ifdef FDCAN2_BASE
        kCan2 = FDCAN2_BASE,
#endif
#ifdef FDCAN3_BASE
        kCan3 = FDCAN3_BASE,
#endif
    };

    enum class CrcBase : uint32_t
    {
        kUnknown = 0,
#ifdef CRC_BASE
        kCrc = CRC_BASE
#endif
    };

    enum class DacBase : uint32_t
    {
        kUnknown = 0,
#ifdef DAC1_BASE
        kDac1 = DAC1_BASE,
#endif
#ifdef DAC2_BASE
        kDac2 = DAC2_BASE,
#endif
#ifdef DAC3_BASE
        kDac3 = DAC3_BASE,
#endif
#ifdef DAC4_BASE
        kDac4 = DAC4_BASE,
#endif
    };

    enum class DacCh : uint32_t
    {
        kUnknown = 0,
#ifdef DAC_CHANNEL_1
        kDacCh1 = DAC_CHANNEL_1,
#endif
#ifdef DAC_CHANNEL_2
        kDacCh2 = DAC_CHANNEL_2,
#endif
#ifdef DAC_CHANNEL_3
        kDacCh3 = DAC_CHANNEL_3,
#endif
#ifdef DAC_CHANNEL_4
        kDacCh4 = DAC_CHANNEL_4
#endif
    };

    enum class I2cBase : uint32_t
    {
        kUnknown = 0,
#ifdef I2C1_BASE
        kI2c1 = I2C1_BASE,
#endif
#ifdef I2C2_BASE
        kI2c2 = I2C2_BASE,
#endif
#ifdef I2C3_BASE
        kI2c3 = I2C3_BASE,
#endif
#ifdef I2C4_BASE
        kI2c4 = I2C4_BASE
#endif
    };

    enum class LpTimerBase : uint32_t
    {
        kUnknown = 0,
#ifdef LPTIM1_BASE
        kLpTimer1 = LPTIM1_BASE
#endif
    };

    enum class QSpiBase : uint32_t
    {
        kUnknown = 0,
#ifdef QSPI_R_BASE
        kQspi1 = QSPI_R_BASE
#endif
    };

    enum class RngBase : uint32_t
    {
        kUnknown = 0,
#ifdef RNG_BASE
        kRng = RNG_BASE
#endif
    };

    enum class TimerCh : uint32_t
    {
        kUnknown = 0,
#ifdef TIM_CHANNEL_1
        kTimerCh1 = TIM_CHANNEL_1,
#endif
#ifdef TIM_CHANNEL_2
        kTimerCh2 = TIM_CHANNEL_2,
#endif
#ifdef TIM_CHANNEL_3
        kTimerCh3 = TIM_CHANNEL_3,
#endif
#ifdef TIM_CHANNEL_4
        kTimerCh4 = TIM_CHANNEL_4,
#endif
#ifdef TIM_CHANNEL_5
        kTimerCh5 = TIM_CHANNEL_5,
#endif
#ifdef TIM_CHANNEL_6
        kTimerCh6 = TIM_CHANNEL_6,
#endif
#ifdef TIM_CHANNEL_ALL
        kTimerChAll = TIM_CHANNEL_ALL
#endif
    };

    enum class TimerBase : uint32_t
    {
        kUnknown = 0,

#ifdef TIM1_BASE
        kTimer1 = TIM1_BASE,
#endif
#ifdef TIM2_BASE
        kTimer2 = TIM2_BASE,
#endif
#ifdef TIM3_BASE
        kTimer3 = TIM3_BASE,
#endif
#ifdef TIM4_BASE
        kTimer4 = TIM4_BASE,
#endif
#ifdef TIM5_BASE
        kTimer5 = TIM5_BASE,
#endif
#ifdef TIM6_BASE
        kTimer6 = TIM6_BASE,
#endif
#ifdef TIM7_BASE
        kTimer7 = TIM7_BASE,
#endif
#ifdef TIM8_BASE
        kTimer8 = TIM8_BASE,
#endif
#ifdef TIM9_BASE
        kTimer9 = TIM9_BASE,
#endif
#ifdef TIM10_BASE
        kTimer10 = TIM10_BASE,
#endif
#ifdef TIM11_BASE
        kTimer11 = TIM11_BASE,
#endif
#ifdef TIM12_BASE
        kTimer12 = TIM12_BASE,
#endif
#ifdef TIM13_BASE
        kTimer13 = TIM13_BASE,
#endif
#ifdef TIM14_BASE
        kTimer14 = TIM14_BASE,
#endif
#ifdef TIM15_BASE
        kTimer15 = TIM15_BASE,
#endif
#ifdef TIM16_BASE
        kTimer16 = TIM16_BASE,
#endif
#ifdef TIM17_BASE
        kTimer17 = TIM17_BASE,
#endif
#ifdef TIM18_BASE
        kTimer18 = TIM18_BASE,
#endif
#ifdef TIM19_BASE
        kTimer19 = TIM19_BASE,
#endif
#ifdef TIM20_BASE
        kTimer20 = TIM20_BASE,
#endif
    };

    // enum class LpUartBase : uint32_t
    // {
    //     kUnknown = 0,
    // #ifdef LPUART1_BASE
    //     kLpUart1 = LPUART1_BASE
    // #endif
    // };

    enum class UartBase : uint32_t
    {
        kUnknown = 0,
#ifdef LPUART1_BASE
        kLpUart1 = LPUART1_BASE,
#endif
#ifdef USART1_BASE
        kUart1 = USART1_BASE,
#endif
#ifdef UART1_BASE
        kUart1 = UART1_BASE,
#endif
#ifdef USART2_BASE
        kUart2 = USART2_BASE,
#endif
#ifdef UART2_BASE
        kUart2 = UART2_BASE,
#endif
#ifdef USART3_BASE
        kUart3 = USART3_BASE,
#endif
#ifdef UART3_BASE
        kUart3 = UART3_BASE,
#endif
#ifdef USART4_BASE
        kUart4 = USART4_BASE,
#endif
#ifdef UART4_BASE
        kUart4 = UART4_BASE,
#endif
#ifdef USART5_BASE
        kUart5 = USART5_BASE,
#endif
#ifdef UART5_BASE
        kUart5 = UART5_BASE,
#endif
#ifdef USART6_BASE
        kUart6 = USART6_BASE,
#endif
#ifdef UART6_BASE
        kUart6 = UART6_BASE,
#endif
#ifdef USART7_BASE
        kUart7 = USART7_BASE,
#endif
#ifdef UART7_BASE
        kUart7 = UART7_BASE,
#endif
#ifdef USART8_BASE
        kUart8 = USART8_BASE,
#endif
#ifdef UART8_BASE
        kUart8 = UART8_BASE,
#endif
#ifdef USART9_BASE
        kUart9 = USART9_BASE,
#endif
#ifdef UART9_BASE
        kUart9 = UART9_BASE,
#endif
#ifdef USART10_BASE
        kUart10 = USART10_BASE,
#endif
#ifdef UART10_BASE
        kUart10 = UART10_BASE,
#endif
    };

    enum class WatchdogBase : uint32_t
    {
        kUnknown = 0,
#ifdef WWDG_BASE
        kWWdg = WWDG_BASE,
#endif
#ifdef WWDG1_BASE
        kWWdg1 = WWDG1_BASE,
#endif
#ifdef IWDG_BASE
        kIWdg = IWDG_BASE,
#endif
#ifdef IWDG1_BASE
        kIWdg1 = IWDG1_BASE,
#endif
    };

}  // namespace hal
