/**
 * Created by Manfred Langemann, Peter Fleury, Theo Theodoridis.
 * File     : twi.h
 * Version  : v1.0
 * Date     : © Copyright 04-01-2008
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

#ifndef TWI_H
#define TWI_H

#include "types.h"

#define READ    1
#define WRITE   0

BOOL twi_init(LONG bitrate);
BOOL twi_start(BYTE address, BYTE twi_type);
void twi_stop(void);

BOOL twi_write(BYTE byte);
BYTE twi_read_ack(void);
BYTE twi_read_nak(void);

BYTE twi_read_dev(BYTE address, BYTE reg);
void twi_write_dev(BYTE address, BYTE reg, BYTE data);

void twi_read_bytes(BYTE *data, BYTE size);
void twi_write_bytes(BYTE *data, BYTE size);

void twi_srf02(BYTE from, BYTE to);

#endif
