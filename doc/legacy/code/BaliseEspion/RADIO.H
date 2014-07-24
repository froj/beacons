#define MAX_LGR_MESSAGE 11
extern unsigned char LgrMessage;
#ifdef EMISSION_RADIO
extern unsigned char * BrutMessageSending[MAX_LGR_MESSAGE + 2]; //Message à envoyer (Taille du message + CRC)
void SendMessage();
#endif
extern unsigned char BrutMessageReceive[MAX_LGR_MESSAGE];
//extern void (*OnMessageReceive)(void);
void InitializeRadio(void (*DoMessageReceive)(void));

