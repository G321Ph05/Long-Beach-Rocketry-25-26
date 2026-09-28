/**
* @brief Gia's Prototyping UART based on the general class UART by LBR
*/
#pragma once


#include "stm32l476xx.h"
#include "usart.h"


namespace LBR
{
namespace Stml4
{
class StUsart : public Usart {
    public:
        StUsart(USART_TypeDef* base_addr, uint32_t clk_freq,
                uint32_t baud_rate);
       
        /**
        * @brief Receive data from serial input at rx pin
        * @param byte refernce to store received byte
        * @return true if data is received
         */
        bool receive(uint8_t& byte) override;

        /**
        * @brief Send data throuhg serial tx pin
        * @param txbuf an uint8_t std:array data to be sent
        * @param size variable of size_t specifying the length of message
        */
        bool send(std::span<const uint8_t> txbuf) override;


        /**
        * @brief Init UART and its associated pins
        * @return True if successful initilization, False otherwise
         */
        bool init();

        /**
        * @brief Get base address of UART object
        * @return pointer to USART_Typedef address
         */
        USART_TypeDef* get_addr();
       


    private:
        USART_TypeDef* _base_addr;
        uint16_t uartdiv;


};

} //Stml4
} //Gia
