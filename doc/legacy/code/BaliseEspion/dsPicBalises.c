//---------------------------------------------------------------------------
// Header files
#include "Define.h"
#include "radio.h"
#include <p30F6011.h>

#include <uart.h>
#include "SendUART.h"
#include <math.h>
#include <incap.h>
#define PI 3.141592654

#define BAUDRATE 57600		// desired baud rate mode espion

#define Timer5Enable T5CONbits.TON
#define Timer5Int IEC1bits.T5IE
#define Timer3Enable T3CONbits.TON
#define Timer3Int IEC0bits.T3IE

//---------------------------------------------------------------------------
// Configuration bits
   _FOSC(CSW_FSCM_OFF & XT_PLL4);   //XT with 4xPLL oscillator, Failsafe clock off
   _FWDT(WDT_OFF);                  //Watchdog timer disabled
   _FBORPOR(PBOR_OFF & MCLR_EN);    //Brown-out reset disabled, MCLR reset enabled
   _FGS(CODE_PROT_OFF);             //Code protect disabled

//---------------------------------------------------------------------------
// Definitions

#define FCY 7372800                 //7.37MHz oscillator with 4xPLL -> 7.37MIPs
//++++++++++++++++++++++++++++++++
// Declaration des variables
//++++++++++++++++++++++++++++++++
short int Parametres;		  		//Paramètre pour transmission HF 
									//Bit 0: 0 Robot ami - 1 Robot ennemi
									//Bit 1: 0 Position standard - 1 Position space invader
//short int MsgRecuAmi;				//Message radio recu, temps ami
//short int MsgRecuEnnemi;			//Message radio recu, temps ennemi

unsigned int Temps[6];        	    //Valeur de temps entre interuption
//0: Valeur de temps entre interuption 3 et int 1
//1: Valeur de temps entre interuption 1 et int 2
//2: Valeur de temps entre interuption 2 et int 3
//3: Valeur de temps entre interuption 6 et int 4
//4: Valeur de temps entre interuption 4 et int 5
//5: Valeur de temps entre interuption 5 et int 6

//unsigned int TempsAmiAA; // Valeur temps moyen entre 2 passage consécutif sur A
//unsigned int TempsEnnemiAA; // Valeur temps moyen entre 2 passage consécutif sur A
unsigned int MemoTempoPulseCodeur;	
unsigned int TempoPulseCodeur;
unsigned int TempSurBaliseC;
unsigned int AngleAmi;
unsigned int PosAmiX;
unsigned int PosAmiY;
unsigned int PosEnnemiX;
unsigned int PosEnnemiY;
unsigned int GlobalTimer;
unsigned int StartCalculPos;
unsigned int TempTourCodeur;

//constante
float LongTerrainDiv2;
float LongTerrainDiv4;
float LargTerrainDiv2;
float LargTerrainDiv4;
float LongTerrain;
float LargTerrain;
float Constante1;
float ResultPosEnnemiXreel;//pour test
float ResultPosEnnemiYreel;//pour test
float ResultPosAmiXreel;//pour test
float ResultPosAmiYreel;//pour test
float AngleAmiReel;//pour test

unsigned int GlobalTimer = 0;

//++++++++++++++++++++++++++++++++
// Initialisation UART
//++++++++++++++++++++++++++++++++

void InitUART1(void)
{
	
	unsigned int baudvalue;
	unsigned int U1MODEvalue;
	unsigned int U1STAvalue;

	CloseUART1();
	
	ConfigIntUART1(UART_RX_INT_DIS & UART_RX_INT_PR2 & 
					UART_TX_INT_EN & UART_TX_INT_PR2);

	U1MODEvalue = 	UART_EN & UART_IDLE_CON &
						UART_DIS_WAKE & UART_DIS_LOOPBACK &
						UART_DIS_ABAUD & UART_NO_PAR_8BIT &
						UART_1STOPBIT;
	U1STAvalue =	UART_INT_TX &
						UART_TX_PIN_NORMAL &
						UART_TX_ENABLE & UART_INT_RX_CHAR &
						UART_ADR_DETECT_DIS &
						UART_RX_OVERRUN_CLEAR;
// The BAUDRATE = 19200,  FCY is already defined so use 
// it as "FCY".  Plug into the formula provided in  the slides 
	baudvalue = ((FCY/16)/BAUDRATE) - 1; // initialize the variable baudvalue
	OpenUART1(U1MODEvalue, U1STAvalue, baudvalue);
} 

//++++++++++++++++++++++++++++++++
// Reception HF
//++++++++++++++++++++++++++++++++

void DoMessageReceive(void)
{	
//test d'envoie de message ... envoie directement sur UART la reception HF
		/*unsigned char i;
		
		WriteUART1TXBuffer(LgrMessage);
		for(i = 0; i < LgrMessage; i++)
		{
			WriteUART1TXBuffer(BrutMessageReceive[i]);	
		}*/

// verifie que le calcul precedent est bien termine avant de redonner des nouvelles valeurs
	if (StartCalculPos == 0)
	{

// memorise la valeur du timer3 au moment de passer sur la balise B qui genere
// l'interrupt d'envoie HF/IR au moment ou la balise B capte le laser
// permettant de determiner l'angle du robot	
	TempSurBaliseC=TMR3; //TMR3 registre du timer 3 ReadTimer3();
	
    BrutMessageReceive[0]=BrutMessageReceive[0];
	//if ((BrutMessageReceive[0] & 0xF8) == 0x48)
	//{	

			Parametres = BrutMessageReceive[0]; //(BrutMessageReceive[0] & 0x03);		

			Temps[0] = (BrutMessageReceive[1] << 8);
			Temps[0] = (Temps[0] | BrutMessageReceive[2]);

			Temps[1] = (BrutMessageReceive[3] << 8);
			Temps[1] = (Temps[1] | BrutMessageReceive[4]);
			
			Temps[2] = 65535 - (Temps[1] + Temps[0]);

			Temps[3] = (BrutMessageReceive[5] << 8);
			Temps[3] = (Temps[3] | BrutMessageReceive[6]);
			
			Temps[4] = (BrutMessageReceive[7] << 8);
			Temps[4] = (Temps[4] | BrutMessageReceive[8]);

			Temps[5] = 65535 - (Temps[3] + Temps[4]);
			
			StartCalculPos =1;
          
     //}   
	}
}

//++++++++++++++++++++++++++++++++
// Initialisation CPU et ports
//++++++++++++++++++++++++++++++++

void InitBoard()
{
   //------------------------------------------------------------------------------
   //Intialisation des ports en entrées/sortie
   //Port B
   //00: Connecteur rj12 => ??
   //01: Connecteur rj12 => ??
   //02: VBAT => Entrée
   //03: Sortie carte extension => N/A => sortie
   //04: Sortie carte extension => N/A => sortie
   //05: Sortie carte extension => N/A => sortie
   //06: Sortie carte extension => N/A => sortie
   //07: Sortie carte extension => N/A => sortie
   //08: DipSwitch 1 => entrée
   //09: DipSwitch 2 => entrée
   //10: DipSwitch 3 => entrée
   //11: DipSwitch 4 => entrée
   //12: DipSwitch 5 => entrée
   //13: DipSwitch 6 => entrée
   //14: DipSwitch 7 => entrée
   //15: DipSwitch 8 => entrée
   TRISB = 0xFF04;   

   //PORT C
   //01: Contrôle 0 module hf => sortie
   //02: Controle 1 module hf => sortie
   //13: Réception module hf => entrée
   //14: Emission module hf => sortie
   TRISC = 0x2000;

   //PORT D
   //00: Emission us => sortie
   //01: Emission us => sortie
   //02: Emission ir => sortie
   //09: Réception ir gauche => entrée
   //10: Réception ir centre => entrée
   //11: Réception ir droite => entrée
   //Le reste carte d'extension non utilisé => sortie
   TRISD = 0x0E00;

   //PORT F
   //02: PIC RX => entrée
   //03: PIC TX => sortie
   TRISF = 0x0004;

   //PORT G
   //12: LED 1 => sortie
   //13: LED 2 => sortie
   //14: LED 3 => sortie
   //15: LED 4 => sortie
   TRISG = 0x0000;


   //émission radio au repos (Attention inversé)
   LATCbits.LATC14 = 1;

	//Mettre le port G à 0
	LATG = 0;

	//Eteindre toutes les LEDS sauf la 1 (Power LED)
	LATGbits.LATG12 = 0; //Power LED
	LATGbits.LATG13 = 1;
	LATGbits.LATG14 = 1;
	LATGbits.LATG15 = 1;	

	
}

//++++++++++++++++++++++++++++++++
// FONCTION CALCUL POSITION
//++++++++++++++++++++++++++++++++

void CalculPos(void)
{

// declaration des variables de la fonction CalculPos
	static float CC1x; //centre cercle 1 sur x
	static float CC1y; //centre cercle 1 sur y
	static float CC2x; //centre cercle 2 sur x
	static float CC2y; //centre cercle 2 sur y
	static float RC1; 	//rayon cercle 1 
	static float RC2; 	//rayon cercle 2 
	static float AngleABst; //angle AB en cas standard
	static float AngleBCst; //angle BC en cas standard
	static float AngleCAst; //angle CA en cas standard
	static float TempsTotSt; //Temps total en cas standard
	static float Calcul;
	static float ResultPosAmiX;
	static float ResultPosAmiY;
	static float ResultPosAmiXAA;
	static float ResultPosAmiYAA;
	static float Pente;
	static float DenomX;
	static float DenomY;
	static float NumX;
	static float NumY;
	static float ResultAngleAmi;

	static float CC1xEnnemi; //centre cercle 1 sur x
	static float CC1yEnnemi; //centre cercle 1 sur y
	static float CC2xEnnemi; //centre cercle 2 sur x
	static float CC2yEnnemi; //centre cercle 2 sur y
	static float RC1Ennemi; 	//rayon cercle 1 
	static float RC2Ennemi; 	//rayon cercle 2 
	static float AngleABEnnemist; //angle AB en cas standard
	static float AngleBCEnnemist; //angle BC en cas standard
	static float AngleCAEnnemist; //angle CA en cas standard
	static float TempsTotEnnemiSt; //Temps total en cas standard
	static float CalculEnnemi;
	static float ResultPosEnnemiX;
	static float ResultPosEnnemiY;
	static float ResultPosEnnemiXAA;
	static float ResultPosEnnemiYAA;
	static float PenteEnnemi;
	static float DenomEnnemiX;
	static float DenomEnnemiY;
	static float NumEnnemiX;
	static float NumEnnemiY;


	static unsigned int DeltaTBaliseCCodeur;
	static unsigned int AngleRobotSurBaliseC; 	//Angle par rapport a la position du robot et la balise C
	static unsigned int AngleCodeurBaliseC;	//Angle entre l'interrupt de la balise C et l'interrupt du codeur
	static unsigned int Offset;


//	if (Parametres==0)//Robot ami, Cas standard
//	{	
		// Calcul de la position du robot d'apres les Temps AB BC CA
		// Utiliser variable global Parametre pour séléction
		//calcul des angles se defini en radian
       
		AngleABst  = (float)(Temps[1])*Constante1;	
		AngleBCst  = (float)(Temps[2])*Constante1;
		AngleCAst  = (float)(Temps[0])*Constante1; 

		CC1x = (LongTerrainDiv2) - (LargTerrainDiv4/(tan(AngleABst)));
		CC1y = (LargTerrainDiv4) + (LongTerrainDiv2/(tan(AngleABst)));
		CC2x = (LargTerrainDiv2 / (tan(AngleBCst)));
		CC2y = LargTerrain/2.0; 

		Pente = (CC1y - CC2y)/(CC1x - CC2x);
		
		NumX = 2.0*((Pente*Pente*CC1x) - (Pente*CC1y));
		DenomX = ((Pente*Pente)+1.0);
		ResultPosAmiX =  NumX / DenomX;

		NumY = 2.0*((CC1y) - (Pente*CC1x));
		DenomY = ((Pente*Pente)+1.0);
		ResultPosAmiY = NumY / DenomY;

		ResultPosAmiXreel = (ResultPosAmiX/65535.0)*3044.0;
		ResultPosAmiYreel = (ResultPosAmiY/46933.74)*2180.0;

		PosAmiX = (int)ResultPosAmiX;
		PosAmiY = (int)ResultPosAmiY;

	
//	if (Parametres==2)//Robot ennemi, Cas standard
	{
		// Calcul de la position du robot ennemi d'apres les Temps AB BC CA
		// Utiliser variable global Parametre pour séléction
		// calcul des angles se defini en radian

		AngleABEnnemist  = (float)(Temps[4])*Constante1;	
		AngleBCEnnemist  = (float)(Temps[5])*Constante1;
		AngleCAEnnemist  = (float)(Temps[3])*Constante1; 

		CC1xEnnemi = (LongTerrainDiv2) - (LargTerrainDiv4/(tan(AngleABEnnemist)));
		CC1yEnnemi = (LargTerrainDiv4) + (LongTerrainDiv2/(tan(AngleABEnnemist)));
		CC2xEnnemi = (LargTerrainDiv2 / (tan(AngleBCEnnemist)));
		CC2yEnnemi = LargTerrain/2.0; 

		PenteEnnemi = (CC1yEnnemi - CC2yEnnemi)/(CC1xEnnemi - CC2xEnnemi);
		
		NumEnnemiX = 2.0*((PenteEnnemi*PenteEnnemi*CC1xEnnemi) - (PenteEnnemi*CC1yEnnemi));
		DenomEnnemiX = ((PenteEnnemi*PenteEnnemi)+1.0);
		ResultPosEnnemiX =  NumEnnemiX / DenomEnnemiX;

		NumEnnemiY = 2.0*((CC1yEnnemi) - (PenteEnnemi*CC1xEnnemi));
		DenomEnnemiY = ((PenteEnnemi*PenteEnnemi)+1.0);
		ResultPosEnnemiY = NumEnnemiY / DenomEnnemiY;

		ResultPosEnnemiXreel = (ResultPosEnnemiX/65535.0)*3044.0;
		ResultPosEnnemiYreel = (ResultPosEnnemiY/46933.74)*2180.0;

		PosEnnemiX = (int)ResultPosEnnemiX;
		PosEnnemiY = (int)ResultPosEnnemiY;
	}

	

// Calcul de l'angle du robot d'apres le pulse codeur et la balise C
// tous les calculs sont sur un modulo de 65535

	DeltaTBaliseCCodeur=TempSurBaliseC-TempoPulseCodeur;

	AngleCodeurBaliseC=(unsigned int)(((float)DeltaTBaliseCCodeur/(float)TempTourCodeur)*65535.0);
	
	AngleRobotSurBaliseC = (unsigned int)((65535.0/(2.0*PI))* (atan((LargTerrain-ResultPosAmiY)/ResultPosAmiX)));
	
	Offset = 0;//56223;
	
  	AngleAmi=AngleCodeurBaliseC+AngleRobotSurBaliseC-Offset;
	AngleAmiReel=((float)(AngleAmi)/65535.0)*360.0;


// Envoie des valeurs calculee sur UART au robot

		WriteUART1TXBuffer('A');
		WriteUART1TXBuffer('B');
		WriteUART1TXBuffer('C');
		WriteUART1TXBuffer(PosAmiX >> 8);	
		WriteUART1TXBuffer(PosAmiX);	
		WriteUART1TXBuffer(PosAmiY >> 8);	
		WriteUART1TXBuffer(PosAmiY);	
		WriteUART1TXBuffer(AngleAmi >> 8);	
		WriteUART1TXBuffer(AngleAmi);
		WriteUART1TXBuffer(PosEnnemiX >> 8);	
		WriteUART1TXBuffer(PosEnnemiX);	
		WriteUART1TXBuffer(PosEnnemiY >> 8);	
		WriteUART1TXBuffer(PosEnnemiY);	


//Debug pour Breakpoint... breakpoint actif apres 5sec 
//Pour recuperer un max de donnees valables 

	if (GlobalTimer > 20)
	{
		GlobalTimer = 0;
	}

//    }

}

//++++++++++++++++++++++++++++++++
// Debug interrupt CPU
//++++++++++++++++++++++++++++++++

//Rentre dans la boucle while si une erreur CPU intervient
void Error(char type)
{
	char toto;
	while (1)
	{
		toto = type;
	}
}


//_MathErr() is the Arithmetic Error Trap handler.
//The routine must have global scope in order to be an ISR.
//The trap handler name is chosen from the device linker script.
//The code execution reaches here based on Accumulator overflow errors
//or divide by zero errors
void __attribute__((__interrupt__)) _OscillatorFail(void)
{
	Error(1);
}

void __attribute__((__interrupt__)) _MathError(void)
{
	Error(12);
}

void __attribute__((__interrupt__)) _AddressError(void)
{
	Error(13);
}

void __attribute__((__interrupt__)) _StackError(void)
{
	Error(14);
}

//++++++++++++++++++++++++++++++++
// Interruption sur IC1 codeur
//++++++++++++++++++++++++++++++++

void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
	
	MemoTempoPulseCodeur=TempoPulseCodeur;
	TempoPulseCodeur= TMR3;

	TempTourCodeur=TempoPulseCodeur-MemoTempoPulseCodeur;	

    IFS0bits.IC1IF = 0; //voir DSPIC30F6011.pdf p.52
}

//++++++++++++++++++++++++++++++++
// Timer 5
//++++++++++++++++++++++++++++++++

void _ISR _T5Interrupt(void)
{	
	GlobalTimer++;
	if (LATGbits.LATG15 == 0)
	{
		LATGbits.LATG15 =1;
    }
    else
	{
		LATGbits.LATG15 =0;
    }  
		
	IFS1bits.T5IF = 0;
}


//---------------------------------------------------------------------------
// Main routine
//---------------------------------------------------------------------------

int main(void)
{		
	__asm__ (".global HEAPSIZE\n .equiv HEAPSIZE,0x100"); 


    InitBoard();
	SRbits.IPL = 1;	
	
	InitUART1();
	InitSendUART1();

	//Initialisation des variables	

 	LongTerrain = 65535.0;
 	LargTerrain = 65535.0 * (2180.0/3044.0);
	Constante1 = (2.0*PI)/65535.0; //constante calcul angle pour pos
	LongTerrainDiv2 = LongTerrain/2.0;
	LongTerrainDiv4 = LongTerrain/4.0;
	LargTerrainDiv2 = LargTerrain/2.0;
	LargTerrainDiv4 = LargTerrain/4.0;
	StartCalculPos = 0;
	GlobalTimer = 0;

	
	T5CON = 0x20; //prédiviseur 64x
	TMR5 = 0;
	PR5 = 11520; //100 ms
	Timer5Int = 1;
	IPC5bits.T5IP = 3;
	Timer5Enable = 1;

	//Activer timer 3 interupt et priorité 7
	ConfigIntCapture1(IC_INT_PRIOR_4 & IC_INT_ON);

	TMR3 = 0;
	Timer3Int = 0;
	T3CON = 0x8020; /* Timer 3 On Prédiviseur 64x*/ 

	// Configure the InputCapture in stop in idle mode , Timer 3 as source , interrupt on capture 1, I/C on every fall 	edge
	OpenCapture1(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_FALL_EDGE);

	InitializeRadio(DoMessageReceive); 

	while(1 == 1)
   {
	  if (StartCalculPos==1)
	  {
		 CalculPos(); 
		 StartCalculPos = 0;
	  }
   }
}



