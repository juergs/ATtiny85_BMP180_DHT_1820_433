/*
  twi.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


  Changes for ATtiny 85:
  http://www.dexterindustries.com/howto/working-with-avr/any-port-any-pin-a-twi-master-for-attiny-atmega/
  
  http://www.daemon.de/blog/2012/10/10/182/error-twsr-undeclared-first-use-function/
  http://playground.arduino.cc/Code/USIi2c
  
  
*/

#ifndef twi_h
#define twi_h

  #include <inttypes.h>

  //---defines for ATtiny85
  #define SDA               0            // SDA Port B.0, Pin 5
  #define SCL               2            // SCL Port B.2, Pin 7
  #define SDA_PORT          PORTB           //SDA Port D
  #define SCL_PORT          PORTB           // SCL Port D

  //#define ATMEGA8

  #ifndef TWI_FREQ
  #define TWI_FREQ 100000L
  #endif

  #ifndef TWI_BUFFER_LENGTH
  #define TWI_BUFFER_LENGTH 32
  #endif

  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4
  
  void twi_init(void);
  void twi_disable(void);
  void twi_setAddress(uint8_t);
  void twi_setFrequency(uint32_t);
  uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
  uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
  uint8_t twi_transmit(const uint8_t*, uint8_t);
  void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
  void twi_attachSlaveTxEvent( void (*)(void) );
  void twi_reply(uint8_t);
  void twi_stop(void);
  void twi_releaseBus(void);

#endif

