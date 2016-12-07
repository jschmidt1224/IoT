#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "flowControl.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_dma.h"

static DMA_HandleTypeDef DMA_HandleStructure;

void flowControl_TXinit()
{
  //DMA config
  //Stream6, TX
  DMA_HandleStructure.Instance = DMA1_Stream6;
  DMA_HandleStructure.Init.Channel = DMA_Channel_4;   //DMA_HandleTypeDef has DMA_InitTypeDef in it
  DMA_HandleStructure.Init.Direction = DMA_MEMORY_TO_PERIPH;
  DMA_HandleStructure.Init.PeriphInc = DMA_PINC_DISABLE;
  DMA_HandleStructure.Init.MemInc = DMA_MINC_DISABLE;
  DMA_HandleStructure.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DMA_HandleStructure.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  DMA_HandleStructure.Init.Mode = DMA_NORMAL;
  DMA_HandleStructure.Init.Priority = DMA_PRIORITY_HIGH;
  DMA_HandleStructure.Init.FIFOMode = DMA_FIFOMODE_DISABLE; //We don't care about out of order data

  HAL_DMA_Init(&DMA_HandleStructure);
}

void flowControl_RXinit()
{
  //Stream5, RX
  DMA_HandleStructure.Instance = DMA1_Stream6;
  DMA_HandleStructure.Init.Channel = DMA_Channel_4;   //DMA_HandleTypeDef has DMA_InitTypeDef in it
  DMA_HandleStructure.Init.Direction = DMA_PERIPH_TO_MEMORY;

}

int flowControl_transmit(uint8_t *msg, uint16_t len)
{
  //Create packet

  //Get datagram from buffer

  //Send packet over UART
}

int flowControl_receive(uint8_t *buf, uint16_t *len)
{
  //Retrieve packet from UART

  //Get length information from packet

  //Put datagram into buffer

  //Send ACK
}
