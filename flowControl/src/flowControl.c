#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "flowControl.h"
#include "flowControl.h"


void flowControl_init()
{
  //DMA stuff here
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
