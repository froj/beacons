//SendUART.c
// Header files
#include "Define.h"
#include <uart.h>
#include <p30F6011a.h>

//TXBuffer = U1TXREG
#define BUFFER_UART1_TX_SIZE 50

unsigned char TXBuffer[BUFFER_UART1_TX_SIZE];
unsigned char * ptReadBuffer;
unsigned char * ptWriteBuffer;
unsigned char TransmitInProgress = 0;

void InitSendUART1()
{
	ptReadBuffer = TXBuffer;
	ptWriteBuffer = TXBuffer;
	U1STAbits.UTXISEL = 1;	
	U1STAbits.UTXEN = 1;
}

void CheckTransmitUART1()
{	
	if (ptReadBuffer != ptWriteBuffer)
	{
		U1TXREG = *ptReadBuffer;
		TransmitInProgress = 1;	
		ptReadBuffer++;
		if (ptReadBuffer >= TXBuffer + (BUFFER_UART1_TX_SIZE * sizeof(unsigned char)))
		{
			ptReadBuffer = TXBuffer;	
		}	
	}
	else
	{
		TransmitInProgress = 0;	
	}
}

void WriteUART1TXBuffer(unsigned char Value)
{
	*ptWriteBuffer = Value;
	
	ptWriteBuffer++;
	if (ptWriteBuffer >= TXBuffer + (BUFFER_UART1_TX_SIZE * sizeof(unsigned char)))
	{
		ptWriteBuffer = TXBuffer;	
	}
	
	if (TransmitInProgress == 0)
	{
		CheckTransmitUART1();	
	}		
}

void _ISR _U1TXInterrupt(void)
{
	CheckTransmitUART1();	
	IFS0bits.U1TXIF = 0;
}
