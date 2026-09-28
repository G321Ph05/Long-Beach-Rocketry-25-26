#include "board.h"

using namespace LBR;

std::array<uint8_t, 17> txb{"Hello, Gia\r\n"};
uint8_t rx_byte;

int main(int argc, char** argv)
{
    bsp_init();

    Board hw = get_board();

    while (1)
    {
        hw.usart.send(txb);

        // Busy wait
        for (volatile uint32_t i = 0; i < 500000; i++)
        {
        }
    }

    return 0;
}
