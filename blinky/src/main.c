#include <stdint.h>
#include "stm32f407xx.h"


void delay();
uint8_t counter = 0;
void usart_init()
{
  // make sure the relevant pins are appropriately set up.
  GPIOA->AFR[0] |= (7 << 8) | (7 << 12);
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER2) & ~(GPIO_MODER_MODER3);
  GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  USART2->BRR  = 25;
  USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  USART2->CR1 |= USART_CR1_UE;
}

static void clk_init()
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_VOS;

  RCC->CR |= RCC_CR_HSEON;  //Enable external oscillator
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  uint32_t PLL_M = 4;
  uint32_t PLL_P = 2;
  uint32_t PLL_N = 100;
  uint32_t PLL_Q = 7;
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
           (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

  RCC->CR |= RCC_CR_PLLON;
  GPIOD->BSRR = GPIO_BSRR_BR_12;
  while(!(RCC->CR & RCC_CR_PLLRDY));
  GPIOD->BSRR = GPIO_BSRR_BR_13;

  RCC->CFGR &= RCC_CFGR_PPRE2;
  RCC->CFGR |= RCC_CFGR_PPRE2_2;

  RCC->CFGR &= RCC_CFGR_PPRE1;
  RCC->CFGR |= RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2;

  RCC->CFGR &= RCC_CFGR_HPRE;
  RCC->CFGR |= RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_3;
  //Select System clock
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  GPIOD->BSRR = GPIO_BSRR_BR_14;
  while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

  GPIOD->BSRR = GPIO_BSRR_BR_15;
  return;
}

int send_char(int ch)
{
  while (!(USART2->SR & USART_SR_TXE));
  USART2->DR = (ch & 0xFF);
  return (ch);
}

int get_char(void)
{
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
  GPIOD->MODER &= ~(GPIO_MODER_MODER14) & ~(GPIO_MODER_MODER15);
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
    for(i = 0; i < 0xFFFFF; i++) {
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

void i2c_init()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI3EN;
  RCC->CR |= RCC_CR_PLLI2SON;
  while (!(RCC->CR | RCC_CR_PLLI2SRDY));

  //AF4 is I2C1, AF6 is SPI3
  GPIOB->MODER &= ~GPIO_MODER_MODER6 & ~GPIO_MODER_MODER9;
  GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 |  GPIO_OTYPER_OT_9;
}

int main()
{
    led_init();
    clk_init();
    //led_init();
    button_init();
    exti_init();
    usart_init();
    //GPIOD->BSRR = GPIO_BSRR_BR_12;
    //led_update(counter);
    while(1) {

    }
    return 0;
}
