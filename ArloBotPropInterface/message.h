#ifndef MESSAGE_H
#define MESSAGE_H

#define MAX_BUFFER_DATA (120)

void DEBUG(char *debug);
void MessageStart();
uint8_t AcquireMessage(char *buffer);
void SendMessage(char* buffer);


#endif