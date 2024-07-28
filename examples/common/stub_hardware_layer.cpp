#include "hardware_layer.h"

#ifdef USE_STUB_HARDWARE_LAYER

// This would usually be implemented by the hardware target, but this is only meant to show an example
#include "hal/target/stub/adc.h"
#include "hal/target/stub/can.h"
#include "hal/target/stub/gpio.h"
#include "hal/target/stub/pwm.h"
#include "hal/target/stub/timer.h"
#include "hal/target/stub/uart.h"

class StubHardwareLayer : public HardwareLayer
{
  public:
    hal::analog::IAdc& GetAdc() override { return adc_; }

    // connectivity
    hal::connectivity::ICan& GetCan() override { return can_; }
    hal::connectivity::IUart& GetUart() override { return uart_; }

    // gpio
    hal::gpio::IInput& GetInput() override { return gpio_; }
    hal::gpio::IInputInterrupt& GetInputInterrupt() override { return gpio_; }
    hal::gpio::IOutput& GetOutput() override { return gpio_; }

    // timers
    hal::timers::IBasicTimer& GetBasicTimer() override { return basic_timer_; }
    hal::timers::IPwm& GetPwm() override { return pwm_; }

  private:
    hal::analog::AdcStub adc_;
    hal::connectivity::CanStub can_;
    hal::connectivity::UartStub uart_;

    hal::gpio::GpioStub gpio_;

    hal::timers::BasicTimerStub basic_timer_;
    hal::timers::PwmStub pwm_;
};

StubHardwareLayer hw_impl;
HardwareLayer& s_hw{hw_impl};

#endif  // USE_STUB_HARDWARE_LAYER