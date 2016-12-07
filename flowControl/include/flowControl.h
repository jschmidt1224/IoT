#ifndef __FLOWCONTROL_H
#define __FLOWCONTROL_H

void flowControl_init();
int flowControl_transmit(uint8_t *msg, uint16_t len);
int flowControl_receive(uint8_t *buf, uint16_t *len);

#endif
