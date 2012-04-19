// VirtualWireMessageTransport.h
//
// Simple Message Transport implementation for Arduino
// 
// uses VirtualWire library by Mike McCauley
//  See http://www.open.com.au/mikem/arduino/VirtualWire.pdf 
//  for full documentation of virtual wire
// 
// Author: Tom Klenke (tklenke@gmail.com)
// Copyright (C) 2012 Thomas R. Klenke

#ifndef VirtualWireMessageTransport_h
#define VirtualWireMessageTransport_h

#include <VirtualWire.h>

bool vwmt_setup( uint8_t bId, uint8_t rx_pin, uint8_t tx_pin, uint16_t speed );

//you need to call this periodically...like every loop at the top would be best.  
//handles peer discover, keep alives, shared clocks, stuff like that.
bool vwmt_network_mx(); 

bool vwmt_listen_for_message();

bool vwmt_send_my_next_message();

void vwmt_queue_msg_all( byte bMsgType, void* pBuf, uint8_t bufsz );

bool vwmt_queue_msg( byte bDest, byte bMsgType, void* pBuf, uint8_t bufsz );

void vwmt_clear_incoming_message();

uint8_t vwmt_get_number_queued_messages();

uint8_t vwmt_get_incoming_message_type();

uint8_t vwmt_get_incoming_source_id();

unsigned long vwmt_get_incoming_received_time();

//returns just the length of the data buffer in the incoming message
uint8_t vwmt_get_incoming_buffer_size();

//returns pointer to incoming buffer
uint8_t* vwmt_get_incoming_buffer();

char* vwmt_get_error_buffer();

void vwmt_clear_error_buffer();

uint8_t vwmt_get_id_status( uint8_t bId );

uint8_t vwmt_increment_id_status ( uint8_t bId );

uint8_t vwmt_get_my_id();



//SYSTEM RESERVED VALUES FOR MessageTypes -- I'll start from the biggest and get smaller.  You can use from zero upwards.
// I use this one for discover of peers.  In my own code, I'm negotiating a shared clock.  But that probably isn't a normal
// use case.  So I'll leave that out of the library unless someone wants it.
#define VWMT_MSG_BROADCAST_AGE 0xFF


#endif
