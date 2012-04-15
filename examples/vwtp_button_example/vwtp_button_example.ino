/* Simple Arduindo example program using VirtualWireMessageTransport library
    for peer to peer networking
    program sends button-up/button-down message to all known peers on network
    when receiving a button-up/button-down message, sets output on 
    pin 13 (built in LED) to HIGH/LOW depending on value of data received.
*/

#include <VirtualWireMessageTransport.h>
#include <VirtualWire.h>

#define TX_PIN 9
#define RX_PIN 8
#define LED11 11
#define LED12 12
#define LED13 13
#define ID0_PIN 2
#define BUTTON_PIN 10




//Msg type
#define MSG_BUTTON_CHANGED 0x01
 
byte bButtonStatus = 0;

void setup()
{
    int i;
    Serial.begin(57600);
    Serial.println("starting..");
    
    pinMode(LED11, OUTPUT);
    pinMode(LED12, OUTPUT);
    pinMode(LED13, OUTPUT);
    pinMode(ID0_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);
    
    //I used two arduino's on my test network.  One set to id 0 and the other to id 1 
    //using grounded or pulled high on pin 2.  
    i = digitalRead(ID0_PIN);
    vwmt_setup(i, RX_PIN, TX_PIN, 5000);

    Serial.print("my id:");
    Serial.println(i);
        
    //let's make sure the LED is working as advertised    
    for (i = 0; i < 5; i++)
    {
        digitalWrite(LED13, HIGH);
        delay(100);
        digitalWrite(LED13, LOW);
        delay(100);
    }
    
    bButtonStatus = digitalRead(BUTTON_PIN);
    Serial.print("Button is: ");
    Serial.println(bButtonStatus);
}

void loop() 
{
    vwmt_network_mx();
    
    if(vwmt_listen_for_message())
    {
        handle_message();

    }
    
    vwmt_send_my_next_message();

    //do something usefull
    generate_new_message();
    
}

////////////////////////
// EVENT HANDLERS / GENERATORS
///////////////////////
void generate_new_message()
{


    if (digitalRead(BUTTON_PIN) != bButtonStatus)
    {
        bButtonStatus = bButtonStatus ? 0 : 1;
        Serial.print("gnm:Observed button change to ");
        Serial.println(bButtonStatus);
        vwmt_queue_msg_all( MSG_BUTTON_CHANGED, &bButtonStatus, sizeof(bButtonStatus) ); 
    }

}

bool handle_message()
{
    uint8_t *pBuf;
    uint8_t bMsgType;
    //now handle the message
    //check for duplicate
    //handle the contents if not duplicate
    
    pBuf = vwmt_get_incoming_buffer();
    bMsgType = vwmt_get_incoming_message_type();
  
    switch (bMsgType)
    {
        case MSG_BUTTON_CHANGED:
            Serial.print("hm:Recieved button changed: ");
            Serial.println(pBuf[0], HEX);
            digitalWrite(LED13, pBuf[0]);
            break;
        default:
            Serial.print("hm:no handler for type: ");
            Serial.println(bMsgType, HEX);  
            break;
    }
    
    //this will be overwritten, but let's clean it up because that's what we do in C...when we're good, have time and remember.
    vwmt_clear_incoming_message();
}


