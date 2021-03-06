// VirtualWireMessageTransport.cpp
//
// Simple Message Transport implementation for Arduino
// See the README file in this directory fdor documentation
// 
// Author: Tom Klenke (tklenke@gmail.com)
// Copyright (C) 2012 Thomas R. Klenke

#include "VirtualWireMessageTransport.h"

//Uncomment if you want to see debug print traffic on Serial.print
#define VWMTDEBUG 

//positions of Bytes in the incoming/outgoing message buffers.
#define VWMT_CONTROL_POS 0
#define VWMT_ADDRESS_POS 1
#define VWMT_MSG_TYPE_POS 2
#define VWMT_BUF_POS 3

//Control Byte
// syn/ack 1000 0000 = 0x80
#define VWMT_SYN_ACK_MASK 0x80
// id 0110 0000 = 0xE0
#define VWMT_UNUSED_MASK 0xE0
// n bytes 0001 1111 = 0x1F
#define VWMT_N_BYTES_MASK 0x1F

//Address Byte
//source 1111 0000 = 0xF0
#define VWMT_SOURCE_MASK 0xF0
//destination 0000 1111 = 0x0F
#define VWMT_DEST_MASK 0x0F

byte vwmt_bState = 0x00;
byte vwmt_bRetries = 0;

//Message Buffer
#define VWMT_MAX_OUT_QUEUE 5
uint8_t vwmt_acInBuf[VW_MAX_MESSAGE_LEN];
uint8_t vwmt_aOutBuf[VWMT_MAX_OUT_QUEUE][VW_MAX_MESSAGE_LEN];
byte vwmt_nOutMsgs = 0;

#define VWMT_ERROR_SZ 32
char vwmt_szError[VWMT_ERROR_SZ];

//Time Out milliseconds
// listen for traffic
#define VWMT_TO_LISTEN 100

//Retry Minimums
#define VWMT_RM_SEND 10

unsigned int vwmt_nBackoff = 1;
unsigned char vwmt_bClearAir = 0;

//ids of units on 'network'
#define VWMT_MAX_IDS 16
byte vwmt_aKnownIds[VWMT_MAX_IDS];

unsigned int vwmt_id;
byte vwmt_nFriends = 0;
byte vwmt_bNewFriend = 0;

//Local Time at reception of message
unsigned long vwmt_ulTimeMsgReceived;


/////////////////
// DEBUG UTILITIES
////////////////

void vwmt_printBuf( uint8_t *pBuf )
{
    int i;
    if (pBuf[0]&VWMT_SYN_ACK_MASK)
        Serial.print("ack "); //ack = 1
    else
        Serial.print("syn ");
    //Serial.print("Unused:");
    //Serial.print( ((pBuf[0]&UNUSED_MASK)>>5), HEX );
    Serial.print(" Len:");
    Serial.print( pBuf[VWMT_CONTROL_POS]&VWMT_N_BYTES_MASK, DEC );
    Serial.print(" Srce:");
    Serial.print( (pBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4, DEC );
    Serial.print(" Dest:");
    Serial.print( (pBuf[VWMT_ADDRESS_POS]&VWMT_DEST_MASK), DEC );
    Serial.print(" MsgType:");
    Serial.print( pBuf[VWMT_MSG_TYPE_POS], HEX );
    Serial.print(": Buf-->");
    for (i = 0; i < ((pBuf[VWMT_CONTROL_POS]&VWMT_N_BYTES_MASK)-VWMT_BUF_POS); i++)
    {
        Serial.print(" 0x");
        Serial.print( (pBuf[i+VWMT_BUF_POS]), HEX );
    }
    Serial.println("<--");
}

/////////////////////////
// INTERNAL FUNCTIONS
////////////////////////

void vwmt_send_ack()
{
    #define VWMT_ACK_SZ 3
    uint8_t aAckBuf[VWMT_ACK_SZ];
    
    //build ack response
    // 1000 0011 = 0x83 (ack and 3 bytes long)
    aAckBuf[VWMT_CONTROL_POS] = 0x83;
    aAckBuf[VWMT_ADDRESS_POS] = (vwmt_id<<4) | ((vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4);
    aAckBuf[VWMT_MSG_TYPE_POS] = vwmt_acInBuf[VWMT_MSG_TYPE_POS];
    
    //send ack response
    vw_rx_stop();
        vw_send( aAckBuf, VWMT_ACK_SZ );
        vw_wait_tx();
    vw_rx_start();
}

void vwmt_pop_queue()
{
    int i;
    //remove top of queue and shuffle up one
    vwmt_nOutMsgs--;
    for (i = 0; i < vwmt_nOutMsgs; i++)
        memcpy(&(vwmt_aOutBuf[i]), &(vwmt_aOutBuf[i+1]), sizeof(vwmt_aOutBuf[0]));
    memset(&(vwmt_aOutBuf[vwmt_nOutMsgs]),0,sizeof(vwmt_aOutBuf[0]));
}

bool vwmt_valid_message()
{
    uint8_t buflen = VW_MAX_MESSAGE_LEN;
    //read the buffer into inBuf
    if (vw_get_message(vwmt_acInBuf, &buflen))
    {
        //write the time received
        vwmt_ulTimeMsgReceived = micros();
        
        //checksum okay, now check for validity
        
        if (!(buflen == (vwmt_acInBuf[VWMT_CONTROL_POS]&VWMT_N_BYTES_MASK)))
        {
            #ifdef VWMTDEBUG
            Serial.println("vm:bad buflen");
            #endif
            ;
        }
        else if ( !(vwmt_id == (vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_DEST_MASK)) )
        {
            #ifdef VWMTDEBUG
            Serial.println("vm:bad dest not for me");
            #endif
            //make sure I know about this source
            if ( ( ((vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4) < VWMT_MAX_IDS ) 
                 && ( !(vwmt_aKnownIds[(vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4])) )
            {
                vwmt_aKnownIds[(vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4] = 1;
                vwmt_nFriends++;
                vwmt_bNewFriend++;
            }
            //if it's friend broadcasting presence. let him know I heard him.
            vwmt_send_ack();
        }
        else if (vwmt_id == (vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK))
        {
            //to me...from me!  conflict!  this should not be possible!
            #ifdef VWMTDEBUG
            Serial.println("Conflict!");
            #endif
            snprintf(vwmt_szError, VWMT_ERROR_SZ, "vm:id conflict detected");
        }
        else
        {
            //it all checks out...make sure I know about this source before passing up
            if ( ( ((vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4) < VWMT_MAX_IDS ) 
                && ( !(vwmt_aKnownIds[(vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4])) )
            {
                vwmt_aKnownIds[(vwmt_acInBuf[VWMT_ADDRESS_POS]&VWMT_SOURCE_MASK)>>4] = 1;
                vwmt_nFriends++;
                vwmt_bNewFriend++;
            }
            return true;
        }
    }
    else
    {
        #ifdef VWMTDEBUG   
        Serial.println("vm:invalid checksum ");
        #endif        
    }
    
    #ifdef VWMTDEBUG   
    Serial.print("vm:bad msg:");
    vwmt_printBuf( vwmt_acInBuf );
    #endif
    //don't leave garbage lying around
    memset(vwmt_acInBuf,0,VW_MAX_MESSAGE_LEN);
    return false;
}

bool vwmt_send_message( uint8_t *pBuf )
{
    int i = 0;  
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    while ( i < VWMT_RM_SEND )
    {
        i++;
        vw_rx_stop();
            vw_send( pBuf, pBuf[0]&VWMT_N_BYTES_MASK );
            vw_wait_tx();
        vw_rx_start();
        
        //listen for ack of this message
        if (vw_wait_rx_max( VWMT_TO_LISTEN ))
        {
            if (vwmt_valid_message())
            {
                if (vwmt_acInBuf[0]&VWMT_SYN_ACK_MASK)
                    return true;
            }
        }
    }

    #ifdef VWMTDEBUG
    Serial.println("sm:max retries waiting for ack on my send");
    #endif
    return false;
}

///////////////////////
// EXTERNAL FUNCTIONS
//////////////////////
bool vwmt_setup( uint8_t bId, uint8_t rx_pin, uint8_t tx_pin, uint16_t speed )
{
    int i;
    //check in valid range
    if ( (bId > VWMT_MAX_IDS-1) || (bId < 0)|| (speed == 0) )
        return false;
        
    //set my id -- and I know about myself    
    vwmt_id = bId;
    vwmt_aKnownIds[vwmt_id] = 1;
    
    //setup virtual wire
    vw_set_rx_pin(rx_pin);
    vw_set_tx_pin(tx_pin);
    vw_setup(5000); // Bits per sec

    //need a true random seed
    randomSeed(analogRead(0));

    //make sure the buffers are cleared
    memset(vwmt_acInBuf, 0, VW_MAX_MESSAGE_LEN);
    memset(vwmt_szError, 0, VWMT_ERROR_SZ);
    for (i = 0; i < VWMT_MAX_OUT_QUEUE; i++)
        memset(vwmt_aOutBuf[i], 0, VW_MAX_MESSAGE_LEN);
        
        
    #ifdef VWMTDEBUG
    Serial.println("Debug prints on: comment out #define VWMTDEBUG in VirtualWireMessageTransport.cpp file to silence");
    #endif
}

bool vwmt_listen_for_message()
{
    //listen for a message for me
    vw_rx_start();
    if (vw_wait_rx_max( VWMT_TO_LISTEN * vwmt_nBackoff ))
    {
        vw_rx_stop();
        if (vwmt_valid_message())
        {
            vwmt_bClearAir++;
            vwmt_send_ack(); //got it!
            return true;
        }
        else
        {
            //if heard unexpected traffic...wait for all clear
            vwmt_bClearAir = 0;
            vwmt_nBackoff++;
            return false;
        }
    }
    else
    {
        vw_rx_stop();
        //if didn't hear anything then continue
        vwmt_nBackoff = 1;
        vwmt_bClearAir++;
        return false;
    }
}

bool vwmt_send_my_next_message()
{
    if (vwmt_bClearAir && vwmt_nOutMsgs)
    {
        if (vwmt_send_message( (uint8_t*)&(vwmt_aOutBuf[0]) ))
        {
            //sent successfully -- remove from queue
            #ifdef VWMTDEBUG
            Serial.println("smnm:msg sent");
            #endif
            vwmt_pop_queue();
            return true;
        }
    }
    return false;
}

bool vwmt_network_mx()
{
    int i;
    unsigned long ulMyAge = 0;
    
    //check for detected errors
    if (vwmt_szError[0])
        return false;
    
    if (vwmt_nOutMsgs)
        //clean up any old broadcast message
        if ((VWMT_MSG_BROADCAST_AGE == vwmt_aOutBuf[0][VWMT_MSG_TYPE_POS]) && ( vwmt_id == (vwmt_aOutBuf[0][VWMT_ADDRESS_POS]&VWMT_DEST_MASK) ))
            vwmt_pop_queue(); //remove top message on queue

    if( (!vwmt_nOutMsgs) && (!vwmt_nFriends))
    {
        //nothing in my queue & i have no friends-- send my age to myself (i.e.broadcast my presence)
        ulMyAge = micros();
        #ifdef VWMTDEBUG
        Serial.print("nmx:broadcasting age ");
        Serial.println(ulMyAge);
        #endif
        vwmt_queue_msg( vwmt_id, VWMT_MSG_BROADCAST_AGE, &ulMyAge, sizeof(ulMyAge) ); 
        return true;
    }
    
    if (vwmt_bNewFriend)
    {
        //negotiate shared clock or other stuff we need to agree upon.
        vwmt_bNewFriend = 0;
        
        #ifdef VWMTDEBUG
        Serial.print("nmx:I know about these ids:");
        for (i = 0; i < VWMT_MAX_IDS; i++)
            if ( vwmt_aKnownIds[i] )
            {
                Serial.print(" ");
                Serial.print(i);
            }
        Serial.println();
        #endif
    }
    

    return true;
}

uint8_t vwmt_get_incoming_message_type()
{
    return vwmt_acInBuf[VWMT_MSG_TYPE_POS];
}

uint8_t vwmt_get_incoming_source_id()
{
    return ( (vwmt_acInBuf[ VWMT_ADDRESS_POS ]&VWMT_SOURCE_MASK)>>4 );
}

//returns just the length of the data buffer in the incoming message
uint8_t vwmt_get_incoming_buffer_size()
{
    return ( (vwmt_acInBuf[VWMT_CONTROL_POS]&VWMT_N_BYTES_MASK) - VWMT_BUF_POS );
}

//returns pointer to incoming buffer
uint8_t* vwmt_get_incoming_buffer()
{
    return &vwmt_acInBuf[VWMT_BUF_POS];
}

void vwmt_queue_msg_all( byte bMsgType, void* pBuf, uint8_t bufsz )
{
    int i;

    for (i = 0; i < VWMT_MAX_IDS; i++)
        if ( (i != vwmt_id) && (vwmt_aKnownIds[i]) )
        {
            #ifdef VWMTDEBUG
            Serial.print("qma: to id: ");
            Serial.println(i);
            #endif
            vwmt_queue_msg( i, bMsgType, pBuf, bufsz );
        }
}

bool vwmt_queue_msg( byte bDest, byte bMsgType, void* pBuf, uint8_t bufsz )
{
    if (bufsz+3 > VW_MAX_MESSAGE_LEN)
        return false; //msg too big
    
    if ( (bDest > VWMT_MAX_IDS) || (!(vwmt_aKnownIds[bDest])) )
        return false; //invalid destination
        
    if ( vwmt_nOutMsgs >= VWMT_MAX_OUT_QUEUE )
        return false; //no room in the queue
    
    vwmt_aOutBuf[vwmt_nOutMsgs][VWMT_CONTROL_POS] = bufsz + 3; //flag, dest, type bytes
    vwmt_aOutBuf[vwmt_nOutMsgs][VWMT_ADDRESS_POS] = (vwmt_id << 4) | bDest;
    vwmt_aOutBuf[vwmt_nOutMsgs][VWMT_MSG_TYPE_POS] = bMsgType;
    memcpy( &(vwmt_aOutBuf[vwmt_nOutMsgs][VWMT_BUF_POS]), pBuf, bufsz );
    #ifdef VWMTDEBUG
    Serial.print("qm:Output Queued->");
    vwmt_printBuf( vwmt_aOutBuf[vwmt_nOutMsgs] );
    #endif
    vwmt_nOutMsgs++;
}

void vwmt_clear_incoming_message()
{
    memset(vwmt_acInBuf, 0, VW_MAX_MESSAGE_LEN);
}

uint8_t vwmt_get_number_queued_messages()
{
    return vwmt_nOutMsgs;
}

char* vwmt_get_error_buffer()
{
    return ((char*)&vwmt_szError);
}

void vwmt_clear_error_buffer()
{
    memset(vwmt_szError, 0, VWMT_ERROR_SZ);
}

unsigned long vwmt_get_incoming_received_time()
{
    return vwmt_ulTimeMsgReceived;
}

uint8_t vwmt_get_id_status( uint8_t bId )
{
    return vwmt_aKnownIds[bId];
}

uint8_t vwmt_increment_id_status ( uint8_t bId )
{
    if ( bId < VWMT_MAX_IDS )
    {
        vwmt_aKnownIds[bId]++;
        return vwmt_aKnownIds[bId];
    }
    return 0;
}

uint8_t vwmt_get_my_id()
{
    return vwmt_id;
}
