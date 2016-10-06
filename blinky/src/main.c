#include <stdint.h>
#include "stm32f407xx.h"

uint8_t counter = 0;
void usart_init()
{
  // make sure the relevant pins are appropriately set up.
  GPIOA->AFR[1] |= (7 << 8) | (7 << 4);
  GPIOA->AFR[0] |= (7 << 8) | (7 << 12);
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER9) & ~(GPIO_MODER_MODER10);
  GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;
  GPIOA->MODER &= ~(GPIO_MODER_MODER2) & ~(GPIO_MODER_MODER3);
  GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;            // enable clock for USART1
  USART1->BRR  = 16000000L/9600L;                // set baudrate
  USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // RX, TX enable
  USART1->CR1 |= USART_CR1_UE;                    // USART enable
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  USART2->BRR  = 16000000L/115200L;
  USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  USART2->CR1 |= USART_CR1_UE;
  }

int send_char(int ch)  {
  while (!(USART2->SR & USART_SR_TXE));
  USART2->DR = (ch & 0xFF);
  return (ch);
}

int get_char(void)  {
  while (!(USART2->SR & USART_SR_RXNE));
  send_char(USART2->DR);
  return ((int)(USART2->DR & 0xFF));
}

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

void button_init()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
}

void exti_init()
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->IMR |= EXTI_IMR_MR0;
  NVIC_EnableIRQ(EXTI0_IRQn);
}

void led_update(uint8_t n)
{
  uint32_t BS = (n << 12) & (0xF000);
  uint32_t BR = ((~n << 28) & (0xF0000000));
  GPIOD->BSRR = BS | BR;
}

void delay()
{
    static volatile uint32_t counter;
    uint32_t i;

    counter = 0;
    for(i = 0; i < 0xFFFF; i++) {
        counter = i;
        counter = counter;
    }
}

int EXTI0_IRQHandler()
{
  counter++;
  NVIC_ClearPendingIRQ(EXTI0_IRQn);
  EXTI->PR |= EXTI_PR_PR0;
  send_char('a');
  return 0;
}

int main()
{
    led_init();
    button_init();
    exti_init();
    usart_init();
    GPIOD->BSRR = GPIO_BSRR_BR_12;
    led_update(counter);
    while(1) {
        //get_char();
        //delay();
        //counter += 1;
        led_update(counter);
        //SendChar('U');
    }
    return 0;
}
