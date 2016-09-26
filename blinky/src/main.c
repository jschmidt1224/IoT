#include <stdint.h>
#include "stm32f407xx.h"

void delay()
{
  // need volatile, otherwise won't delay like we want to do
  // static
    static volatile uint32_t counter;
    uint32_t i;

    counter = 0;
    for(i = 0; i < 0xFFFF; i++) {
        counter = i;
        counter = counter;
    }
}

int main()
{

    while(1) {

    }

    return 0;
}
