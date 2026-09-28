#include "st_usart.h"


namespace LBR
{
namespace Stml4
{
//Constructor
StUsart::StUsart(USART_TypeDef* base_addr, uint32_t clk_freq,
    uint32_t baud_rate) : _base_addr(base_addr),
      uartdiv(clk_freq / baud_rate)
{
}


//Send
bool StUsart::send(std::span<const uint8_t> txbuf)
{
    size_t count = 0;
    //Iterate through each byte in the received data
    for (const uint8_t byte : txbuf)
    {
        //Wait until TXE == 0
        // AKA: until the transmit register has data to send over the tx pin
        // (Transmitter data Empty) flag
        while (!(_base_addr->ISR & USART_ISR_TXE));
        //TDR: Transmit data value = byte of the message
        _base_addr->TDR = byte;
        //Keep track of how many bytes has been sent
        count++;
    }
    //Wait till the transmission is complete = The final bit has left the register
    // TC: Transmission complete flag
    while (!(_base_addr->ISR & USART_ISR_TC));
    return (count == txbuf.size());
}


//Receive
bool StUsart::receive(uint8_t& byte){
    //Clear overun error
    if (_base_addr->ISR & USART_ISR_ORE) {
        _base_addr->ICR |= USART_ICR_ORECF;
    }


    byte = _base_addr->RDR;
    return true;
}


bool StUsart::init() {
    //Check if base_addr is a valid pointer to USART_TypeDef*
    if (_base_addr == nullptr)
        return false;
   
    //Disable UART to configure necessary feature
    _base_addr->CR1 &= ~USART_CR1_UE;
    //Input baud rate to BRR register
    _base_addr->BRR = uartdiv;
    //Clear M0 & M1 register to enable 8-bit in each UART data word
    _base_addr->CR1 &= ~USART_CR1_M0;
    _base_addr->CR1 &= ~USART_CR1_M1;
    //Configure 1 stop bit - a 2 bit flag if 00 = 1 stop bit
    _base_addr->CR2 &= ~USART_CR2_STOP;
    //Enable TX, RX, UART
    _base_addr->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    //RX interrupt enable
    _base_addr->CR1 |= USART_CR1_RXNEIE;


    return true;
}


USART_TypeDef* StUsart::get_addr()
{
    return this->_base_addr;
}

}  // namespace Stml4
}  // namespace GIA
