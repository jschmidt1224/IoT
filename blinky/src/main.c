#include <stdint.h>
#include "stm32f407xx.h"

void led_init()
{
  //Enables the clock going to the GPIO D peripheral
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  //Sets GPIOD ports 12-15 to output
  GPIOD->MODER &= ~(GPIO_MODER_MODER12) & ~(GPIO_MODER_MODER13);
  GPIOD->MODER &= ~(GPIO_MODER_MODER14) & ~(GPIO_MODER_MODER14);
  GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
  GPIOD->MODER |= GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;

  //Sets the speed of the GPIO to very high because why the fuck not
  GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13;
  GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;

  //Resetting all LEDs to ON
  GPIOD->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
  GPIOD->BSRR = GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
}

void led_update(uint8_t n)
{
  if (n & 0x01) {
    GPIOD->BSRR = GPIO_BSRR_BS_12;
  } else {
    GPIOD->BSRR = GPIO_BSRR_BR_12;
  }
  if (n & 0x02) {
    GPIOD->BSRR = GPIO_BSRR_BS_13;
  } else {
    GPIOD->BSRR = GPIO_BSRR_BR_13;
  }
  if (n & 0x04) {
    GPIOD->BSRR = GPIO_BSRR_BS_14;
  } else {
    GPIOD->BSRR = GPIO_BSRR_BR_14;
  }
  if (n & 0x08) {
    GPIOD->BSRR = GPIO_BSRR_BS_15;
  } else {
    GPIOD->BSRR = GPIO_BSRR_BR_15;
  }
}

void delay()
{
    static volatile uint32_t counter;
    uint32_t i;

    counter = 0;
    for(i = 0; i < 0xFFFFF; i++) {
        counter = i;
        counter = counter;
    }
}

int main()
{
    uint8_t counter = 0;
    led_init();
    led_update(counter);
    while(1) {
        delay();
        counter += 1;
        led_update(counter);
    }

    return 0;
}
