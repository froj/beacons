#define MAX_LGR_MESSAGE 15
extern unsigned char LgrMessage;
extern unsigned char * BrutMessageSending[MAX_LGR_MESSAGE + 2]; //Message à envoyer (Taille du message + CRC)
#ifdef RECEPTION_RADIO
extern unsigned char BrutMessageReceive[MAX_LGR_MESSAGE];
extern void (*OnMessageReceive)(void);
#endif
void InitializeRadio(void (*DoMessageReceive)(void));
void SendMessage();
