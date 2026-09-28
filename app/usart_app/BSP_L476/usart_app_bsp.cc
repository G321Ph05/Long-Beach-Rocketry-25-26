#include <cstdint>
#include "board.h"
#include "st_gpio.h"
#include "st_sys_clock.h"
#include "st_usart.h"


using namespace LBR::Stml4;
namespace LBR
{
LBR::Stml4::StGpioSettings rx_settings{
    LBR::Stml4::GpioMode::ALT_FUNC, LBR::Stml4::GpioOtype::PUSH_PULL,
    LBR::Stml4::GpioOspeed::LOW, LBR::Stml4::GpioPupd::NO_PULL, 0x7};
LBR::Stml4::StGpioSettings tx_settings{
    LBR::Stml4::GpioMode::ALT_FUNC, LBR::Stml4::GpioOtype::PUSH_PULL,
    LBR::Stml4::GpioOspeed::LOW, LBR::Stml4::GpioPupd::NO_PULL, 0x7};

LBR::Stml4::StGpioParams rx_param{rx_settings, 3, GPIOA};
LBR::Stml4::StGpioParams tx_param{tx_settings, 2, GPIOA};

LBR::Stml4::HwGpio rx_gpio{rx_param};
LBR::Stml4::HwGpio tx_gpio{tx_param};
StUsart usart{USART2, 4000000, 9600};

Board board{.usart = usart, .rx = rx_gpio, .tx = tx_gpio};

Stml4::HwClock clock;

bool bsp_init() {
    bool result = true;
    result &= clock.init(Stml4::HwClock::configuration::DEFAULT_4MHZ);

    //Enable clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    //Enable UART2,3
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_USART3EN;
    //Enable UART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;


    result &= usart.init();
    result &= rx_gpio.init();
    result  &= tx_gpio.init();

    //Muse use USART2 b/c of virtual com to connect to minicom
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);

    return result;
   
}

Board& get_board()
{
    return board;
}

extern "C" void USART2_IRQHandler(void)
{
    // Check if data is available
    if (usart.get_addr()->ISR & USART_ISR_RXNE)
    {
        if (board.usart.receive(rx_byte))
        {
            // received 1 byte, echo it back
            std::span<const uint8_t> tx_span(&rx_byte, 1);
            board.usart.send(tx_span);
        }
    }
}


// namespace Stml4
}  // namespace LBR
