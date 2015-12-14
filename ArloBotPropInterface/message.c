#include <simpletools.h>
#include <fdserial.h>
#include "message.h"

//#define ECHO_SERIAL_MSG

typedef struct _Message_Buffer
{
    uint8_t is_full;
    uint8_t count;
    char    data[MAX_BUFFER_DATA];
} MESSAGE_BUFFER;

#define PEND_ON_BUFFER(buffer)  while(buffer.is_full)   \
                                {                       \
                                    ;                   \
                                }

                                
static volatile MESSAGE_BUFFER incoming_msg;
static volatile MESSAGE_BUFFER outgoing_msg;
#ifdef ECHO_SERIAL_MSG // Use this only for debugging problems with message handling
static MESSAGE_BUFFER echo_msg;
#endif
static fdserial *msg_serial;
static int msg_stack[128];
static int lock;


static void HandleSerial(void *par)
{
    int count;
    char next;
    
    while (1)
    {
        // Check for incoming message, read and store it, signal the message is ready
        if (!incoming_msg.is_full)
        { 
            if (fdserial_rxReady(msg_serial) != 0)
            {
                count = 0;
                
                while (count < sizeof(incoming_msg.data)) 
                {
                    next = readChar(msg_serial);
                    lockset(lock);
                    incoming_msg.data[count] = next;
#ifdef ECHO_SERIAL_MSG // Use this only for debugging problems with message handling
                    echo_msg.data[count] = next;
#endif                        
                    lockclr(lock);
                    
                    if (incoming_msg.data[count] == '\r' || incoming_msg.data[count] == '\n')
                    {
                        lockset(lock);
                        incoming_msg.count = count;
                        //incoming_msg[count + 1] = '\0';
                        incoming_msg.is_full = 1;
#ifdef ECHO_SERIAL_MSG // Use this only for debugging problems with message handling
                        echo_msg.count = count;
                        //echo_msg[count] = '\0';
                        echo_msg.is_full = 1;
#endif                        
                        lockclr(lock);
                        break;
                    }    
                    count++;
                }
            }
        }
        else
        {
            // This means we aren't processing incomming messages fast enough
            // Could implment an overflow counter that could be reported back as status
        }
        
#ifdef ECHO_SERIAL_MSG // Use this only for debugging problems with message handling
        lockset(lock);
        if (echo_msg.is_full)
        {
            dprint(msg_serial, "E:%s", echo_msg.data);
            echo_msg.is_full = 0;
        }            
        lockclr(lock);
#endif
        lockset(lock);
        // Check for outgoing message, send, and clear flag
        if (outgoing_msg.is_full)
        {
            dprint(msg_serial, "%s", outgoing_msg.data);
            outgoing_msg.is_full = 0;
        }
        lockclr(lock);
    }        
}

uint8_t AcquireMessage(char *buffer)
{
    if (incoming_msg.is_full)
    {
        lockset(lock);
        memcpy(buffer, (char *) incoming_msg.data, incoming_msg.count);
        //incoming_msg.data[incoming_msg.count + 1] = 0;
        incoming_msg.is_full = 0;
        lockclr(lock);
        return 1;
    }
    
    return 0;
}

void SendMessage(char* buffer)
{
    PEND_ON_BUFFER(outgoing_msg);
    lockset(lock);
    memcpy((char *) outgoing_msg.data, buffer, strlen(buffer));
    outgoing_msg.data[strlen(buffer)] = 0; // Null terminate the data
    outgoing_msg.is_full = 1;
    lockclr(lock);
}

void DEBUG(char *debug)
{
    SendMessage(debug);
}


void MessageStart()
{
    lock = locknew();
    lockclr(lock);
    simpleterm_close();
    msg_serial = fdserial_open(31, 30, 0, 115200);    
    cogstart(&HandleSerial, NULL, msg_stack, sizeof(msg_stack));
}