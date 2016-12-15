#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "Audio.h"
#include "mp3dec.h"

// Private variables
volatile uint32_t time_var1, time_var2;
MP3FrameInfo mp3FrameInfo;
HMP3Decoder hMP3Decoder;
static char *string = "Hello UART!\n\r";

// Private function prototypes
static void AudioCallback(void *context,int buffer);
void Delay(volatile uint32_t nCount);
void init();
void init_clock();
void init_uart();
int send_char(int ch);
int get_char();

// External variables
#define MP3_SIZE	87323
char mp3_data[MP3_SIZE];

// Some macros

#define BUTTON		(GPIOA->IDR & GPIO_Pin_0)

int main(void) {
	init_clock();
	init();
	init_uart();
	int volume = 0;
	int b;	//B is for Bridget

	// Play mp3
	hMP3Decoder = MP3InitDecoder();
	InitializeAudio(Audio44100HzSettings);
	SetAudioVolume(0xCF);
	//PlayAudioWithCallback(AudioCallback, 0);
	//GPIOD->BSRRL = 1<<12;
	//for(;;) {
		char* tmp = string;
		while(*tmp) {
			send_char(*tmp);
			tmp++;
		}

	//}
	for(b = 0; b < MP3_SIZE; b++)
	{
		mp3_data[b] = get_char();
	}
	GPIOD->BSRRL = 1<<12;
	PlayAudioWithCallback(AudioCallback, 0);

	while(1) {
		
	}

	return 0;
}

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
static void AudioCallback(void *context, int buffer) {
	static int16_t audio_buffer0[4096];
	static int16_t audio_buffer1[4096];

	int offset, err;
	int outOfData = 0;
	static const char *read_ptr = mp3_data;
	static int bytes_left = MP3_SIZE;

	int16_t *samples;

	if (buffer) {
		samples = audio_buffer0;
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	} else {
		samples = audio_buffer1;
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;

	if (bytes_left <= 2889) {
		read_ptr = mp3_data;
		bytes_left = MP3_SIZE;
		offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
		bytes_left -= offset;
	}

	read_ptr += offset;
	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, &bytes_left, samples, 0);

	if (err) {
		/* error occurred */
		GPIOD->BSRRL = 1 << 15;
		switch (err) {
		case ERR_MP3_INDATA_UNDERFLOW:
			outOfData = 1;
			break;
		case ERR_MP3_MAINDATA_UNDERFLOW:
			/* do nothing - next call to decode will provide more mainData */
			break;
		case ERR_MP3_FREE_BITRATE_SYNC:
		default:
			outOfData = 1;
			break;
		}
	} else {
		/* no error */
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
	}

	if (!outOfData) {
		GPIOD->BSRRL = 1<<15;
		ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);
	}
}

void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// Enable full access to FPU (Should be done automatically in system_stm32f4xx.c):
	//SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// ------ UART ------ //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//RCC_AHB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART1);

/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
*/

	// Conf
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);
	//USART_Init(USART3, &USART_InitStructure);

	// Enable
	//USART_Cmd(USART2, ENABLE);
	//USART_Cmd(USART3, ENABLE);
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;

	while(time_var1){};
}

void init_clock()
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_VOS;

  RCC->CR |= RCC_CR_HSEON;  //Enable external oscillator
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  uint32_t PLL_M = 4;
  uint32_t PLL_P = 2;
  uint32_t PLL_N = 168;
  uint32_t PLL_Q = 7;
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
           (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

  RCC->CR |= RCC_CR_PLLON;
  while(!(RCC->CR & RCC_CR_PLLRDY));

  RCC->CFGR &= RCC_CFGR_PPRE2;
  RCC->CFGR |= RCC_CFGR_PPRE2_2;

  RCC->CFGR &= RCC_CFGR_PPRE1;
  RCC->CFGR |= RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2;

  RCC->CFGR &= RCC_CFGR_HPRE;
  RCC->CFGR |= RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_3;
  //Select System clock
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

  return;
}


void init_uart()
{
  // make sure the relevant pins are appropriately set up.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->AFR[1] |= (7 << 8) | (7 << 12);
  GPIOB->MODER &= ~(GPIO_MODER_MODER10) & ~(GPIO_MODER_MODER11);
  GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  USART3->BRR  = 42;
  USART3->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  USART3->CR1 |= USART_CR1_UE;
}


int send_char(int ch)
{
  while (!(USART3->SR & USART_SR_TXE));
  USART3->DR = (ch & 0xFF);
  return (ch);
}

int get_char()
{
	//GPIOD->BSRRL = 1<<14;
  while (!(USART3->SR & USART_SR_RXNE));
	//GPIOD->BSRRL = 1<<13;
  //send_char(USART2->DR);
	//GPIOD->BSRRL = 1<<12;
  return ((int)(USART3->DR & 0xFF));
}
/*
 * Dummy function to avoid compiler error
 */
void _init() {

}
