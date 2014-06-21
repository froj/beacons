//++++++++++++++++++++++++++++++++
//Bibliotheque de base
//++++++++++++++++++++++++++++++++
#include <p30f6011A.h>
#include <incap.h>
#include "radio.h"

//++++++++++++++++++++++++++++++++
// Configuration bits
//++++++++++++++++++++++++++++++++
   _FOSC(CSW_FSCM_OFF & XT_PLL4);   //XT with 4xPLL oscillator, Failsafe clock off
   _FWDT(WDT_OFF);                  //Watchdog timer disabled
   _FBORPOR(PBOR_OFF & MCLR_EN);    //Brown-out reset disabled, MCLR reset enabled
   _FGS(CODE_PROT_OFF);             //Code protect disabled


//++++++++++++++++++++++++++++++++
// Definitions du microcontroleur
//++++++++++++++++++++++++++++++++
#define Fcy 7372800                 //7.37MHz oscillator with 4xPLL -> 7.37MIPs
#define Timer5Enable T5CONbits.TON  //Gestion du timer T5
#define Timer5Int IEC1bits.T5IE     //Gestion du timer T5
#define Timer3Enable T3CONbits.TON  //Gestion du timer T3
#define Timer3Int IEC0bits.T3IE     //Gestion du timer T3


//++++++++++++++++++++++++++++++++
// Declaration des variables
//++++++++++++++++++++++++++++++++
unsigned int Tempo;
short int DdeEnvoiMessage;         	//Demande envoie message
//short int MemoBaliseA;			 	//Memo si passage 2 fois devant A
short int Parametres;		  		//Paramètre pour transmission HF 
									//Bit 0: 0 Robot ami - 1 Robot ennemi
									//Bit 1: 0 Position standard - 1 Position space invader

unsigned int FrontMont[6];         	//Valeur tempo au front montant
//0: Valeur tempo au front montant sur interupte 1
//1: Valeur tempo au front montant sur interupte 2
//2: Valeur tempo au front montant sur interupte 3
//3: Valeur tempo au front montant sur interupte 4
//4: Valeur tempo au front montant sur interupte 5
//5: Valeur tempo au front montant sur interupte 6

unsigned int Temps[6];        	    //Valeur de temps entre interuption

//0: Valeur de temps entre interuption 3 et int 1
//1: Valeur de temps entre interuption 1 et int 2
//2: Valeur de temps entre interuption 2 et int 3
//3: Valeur de temps entre interuption 6 et int 4
//4: Valeur de temps entre interuption 4 et int 5
//5: Valeur de temps entre interuption 5 et int 6

unsigned int FrontMont2Old;  // Valeur tempsInt 2 tour précédent
unsigned int FrontMont5Old;  // Valeur tempsInt 2 tour précédent

unsigned int TimeIC1;		//Memo temps du timer 3 pour interruption 1
unsigned int TimeIC3;		//Memo temps du timer 3 pour interruption 3
unsigned int TimeIC5;		//Memo temps du timer 3 pour interruption 5

unsigned int TimeIC2;		//Memo temps du timer 3 pour interruption 2
unsigned int TimeIC4;		//Memo temps du timer 3 pour interruption 4
unsigned int TimeIC6;		//Memo temps du timer 3 pour interruption 6


unsigned int ErreurIterationBalise; //Permet de savoir si erreur
unsigned int DerniereBalise; //Dernière balise vue
unsigned int ErreurIterationBaliseEnnemi;
unsigned int DerniereBaliseEnnemi;

unsigned int GlobalTimer;

unsigned int logHistoryTab[500];  //Info pour debug
unsigned int logHistoryTabTimer[500];  //Info pour debug
unsigned int indexHistory;  //Info pour debug

struct TimeDebug
{
	unsigned int TD1;  //Info pour debug
	unsigned int TD2;   //Info pour debug
	unsigned int TD3;   //Info pour debug
	unsigned long TDTot;   //Info pour debug
} TimeDebugTab[200];

unsigned int indexTimeDebug;

//++++++++++++++++++++++++++++++++
// INITIALISATION DU CONTROLEUR
//++++++++++++++++++++++++++++++++
void InitBoard()
{
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
   // Port 4,5,8,9,10,11 en entrée
   //Le reste, carte d'extension non utilisé => sortie
   // 11 10  9  8 7 6 5 4   3  2 1 0
   //  0  1  0  1 0 1 0 1   0  1 0 1
   //  1  1  1  1 0 0 1 1   0  0 0 0 
   //TRISD = 0x0555;
   TRISD = 0x0F30;

   //PORT F
   //02: PIC RX  => entrée
   //03: PIC TX  => sortie
   //05: Data_IR => sortie
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

	//Eteindre toutes les LEDS 
	LATGbits.LATG12 = 1; 
	LATGbits.LATG13 = 1;
	LATGbits.LATG14 = 1;
	LATGbits.LATG15 = 1;	

}
//++++++++++++++++++++++++++++++++
// CALCUL DES TEMPS
//++++++++++++++++++++++++++++++++
//Index = 0,1,2 (balise A, B, C)
//TempoIC = TMR3
void CalculTemps(Index,TempoIC)
{	
	float 			TempsTotAmi;  //Somme des 3 temps robot ami
	float 			TempsTotEnnemi;  //Somme des 3 temps robot ennemi
	float 			VarTemp;      //Variable intermediaire pour calcul
  
	FrontMont[Index] = TempoIC;

	switch(Index) 	// Calcul Temps CA AB BC + éteindre LED en fonction des interruption balise A B ou C
	{
		case 0:	//12			
			LATGbits.LATG12 = 0;
 			LATGbits.LATG14 = 1;
			break;
		case 1:	//13					
			LATGbits.LATG13 = 0;
			LATGbits.LATG12 = 1;
			break;
		case 2:	//14					
			LATGbits.LATG14 = 0;
			LATGbits.LATG13 = 1;	

			TempsTotAmi = FrontMont[2]-FrontMont2Old; 

			VarTemp = (float)(FrontMont[0] - FrontMont2Old); //Calcul du temps
			Temps[0] = ( VarTemp / TempsTotAmi)*65535.0;   //Mise à l'echelle sur 65535
			
			VarTemp = (float)(FrontMont[1] - FrontMont[0]); //Calcul du temps
			Temps[1] = (VarTemp / TempsTotAmi)*65535.0;   //Mise à l'echelle sur 65535

			VarTemp = (float)(FrontMont[2] - FrontMont[1]); //Calcul du temps
			Temps[2] = (VarTemp / TempsTotAmi)*65535.0;   //Mise à l'echelle sur 65535                   //Ecriture du temps
			
			if (GlobalTimer > 20)
			{
				GlobalTimer = 0;
			}

			FrontMont2Old=FrontMont[2];			
			
			break;
		case 3:						
	 
			break;
		case 4:						

			break;
		case 5:						
			TempsTotEnnemi = FrontMont[5]-FrontMont5Old; 

			VarTemp = (float)(FrontMont[3] - FrontMont5Old); //Calcul du temps
			Temps[3] = ( VarTemp / TempsTotEnnemi)*65535.0;   //Mise à l'echelle sur 65535
			
			VarTemp = (float)(FrontMont[4] - FrontMont[3]); //Calcul du temps
			Temps[4] = (VarTemp / TempsTotEnnemi)*65535.0;   //Mise à l'echelle sur 65535

			VarTemp = (float)(FrontMont[5] - FrontMont[4]); //Calcul du temps
			Temps[5] = (VarTemp / TempsTotEnnemi)*65535.0;   //Mise à l'echelle sur 65535
			
			FrontMont5Old=FrontMont[5];				
	}
}

//++++++++++++++++++++++++++++++++
// LOGHISTORY ITERATION BALISE POUR DEBUG
//++++++++++++++++++++++++++++++++
void logHistory(unsigned int balise)
{
	logHistoryTab[indexHistory] = balise;  //Info pour debug
	switch (balise)
	{
		case 1 :
			logHistoryTabTimer[indexHistory] = TimeIC5;  //Info pour debug
			break;
		case 3 :
			logHistoryTabTimer[indexHistory] = TimeIC1;   //Info pour debug
			break;
		case 4 :
			logHistoryTabTimer[indexHistory] = TimeIC3;   //Info pour debug
			break;
	}
	//logHistoryTabTimer[indexHistory] = TMR3;
	indexHistory++;  //Info pour debug
	if (indexHistory == 500)
		indexHistory = 0;
}

//++++++++++++++++++++++++++++++++
// FONCTION POUR DEBUGAGE
//++++++++++++++++++++++++++++++++
void logTD(unsigned int T1, unsigned int T2, unsigned int T3)
{
	TimeDebugTab[indexTimeDebug].TD1 = T1; 
	TimeDebugTab[indexTimeDebug].TD2 = T2;
	TimeDebugTab[indexTimeDebug].TD3 = T3;
	TimeDebugTab[indexTimeDebug].TDTot = T1 + T2 + T3;
	indexTimeDebug++;
	if (indexTimeDebug == 200)
		indexTimeDebug = 0;
}
/*
//++++++++++++++++++++++++++++++++
// CALCUL DES TEMPS ENTRE 2 PASSAGE SUR BALISE A
//++++++++++++++++++++++++++++++++
void CalculTempsAA(Flanc, TempoIC)
{
	if (Flanc==1)		//Flanc montant
	{
		FrontMontAA = TempoIC;
	}
	else          		//Flant descendant
	{
		FrontDescAA = TempoIC;
		TempsAA=TempsInt[0]-((FrontDescAA/2) + (FrontMontAA/2));		//Valeurs temps entre 2 passage consécutif sur A
		TempsInt[0]=(TempsInt[0]/2)+(FrontDescAA/4) + (FrontMontAA/4);	//Temps moyen entre les 2 passages sur A (Temps1+Temps2)/2
	}
}
*/
//++++++++++++++++++++++++++++++++
// INTERUPTION 0 SUR IC5,RD4, Balise A Ami
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC5Interrupt(void)
{
	if(DerniereBalise!=1)
	{
		ErreurIterationBalise = ErreurIterationBalise || ((DerniereBalise != 1) && (DerniereBalise != 4));
		DerniereBalise = 1;	
		TimeIC5 = TMR3;
		logHistory(1);
		CalculTemps(0, TimeIC5);
	}

    IFS1bits.IC5IF = 0;
}

//++++++++++++++++++++++++++++++++
// INTERUPTION 1 SUR IC1,RD8, Balise B Ami
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
	if(DerniereBalise!=3)
	{
	    ErreurIterationBalise = ErreurIterationBalise || ((DerniereBalise != 1) && (DerniereBalise != 2));
	 	DerniereBalise =3;
		TimeIC1 = TMR3;
		logHistory(3);
		CalculTemps(1, TimeIC1);
		//MemoBaliseA=0;
	}
	
    IFS0bits.IC1IF = 0; //voir DSPIC30F6011.pdf p.52
}

//++++++++++++++++++++++++++++++++
// INTERUPTION 2 SUR IC3,RD10, Balise C Ami
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC3Interrupt(void)
{
	if(DerniereBalise!=4)
	{
	    ErreurIterationBalise = ErreurIterationBalise || (DerniereBalise != 3);
	 	DerniereBalise =4;

		//ReadCapture3(&TimeIC3);
		TimeIC3 = TMR3;
		logHistory(4);
		CalculTemps(2, TimeIC3);
		
   		if (ErreurIterationBalise == 0)
		{
			//DdeEnvoiMessage=1;
			
			//DEBUG
			logTD(Temps[0], Temps[1], Temps[2]);

			LgrMessage = 9;
			BrutMessageSending[0] = 0;
			BrutMessageSending[1] = (Temps[0] >> 8);
			BrutMessageSending[2] = Temps[0];
			BrutMessageSending[3] = (Temps[1] >> 8);
			BrutMessageSending[4] = Temps[1]; 

				if (ErreurIterationBaliseEnnemi == 0)
				{
					BrutMessageSending[5] = (Temps[3] >> 8);
					BrutMessageSending[6] = Temps[3];
					BrutMessageSending[7] = (Temps[4] >> 8);
					BrutMessageSending[8] = Temps[4]; 	
				}
				else
				{
					BrutMessageSending[5] = 0;
					BrutMessageSending[6] = 0;
					BrutMessageSending[7] = 0;
					BrutMessageSending[8] = 0; 	
					ErreurIterationBaliseEnnemi = 0;
				}
			SendMessage();
		}
		else
		{
			ErreurIterationBalise = 0;
			//ErreurIterationBaliseEnnemi = 0;
		}
	}
    IFS1bits.IC3IF = 0;
}

//++++++++++++++++++++++++++++++++
// INTERUPTION 3 SUR IC6,RD5, Balise A Ennemi
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC6Interrupt(void)
{
	if(DerniereBaliseEnnemi!=1)
	{
		ErreurIterationBaliseEnnemi = ErreurIterationBaliseEnnemi || ((DerniereBaliseEnnemi != 1) && (DerniereBaliseEnnemi != 4));
		DerniereBaliseEnnemi = 1;	
	    //ReadCapture5(&TimeIC5);
		TimeIC6 = TMR3;
		//logHistory(1);
		CalculTemps(3, TimeIC6);
	}

    IFS1bits.IC6IF = 0;
}

//++++++++++++++++++++++++++++++++
// INTERUPTION 4 SUR IC2,RD9, Balise B
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC2Interrupt(void)
{
	if(DerniereBaliseEnnemi!=3)
	{
	    ErreurIterationBaliseEnnemi = ErreurIterationBaliseEnnemi || ((DerniereBaliseEnnemi != 1) && (DerniereBaliseEnnemi != 2));
	 	DerniereBaliseEnnemi =3;
	   
		//ReadCapture1(&TimeIC1);
		TimeIC2 = TMR3;
		//logHistory(3);
		CalculTemps(4, TimeIC2);
	}
	
    IFS0bits.IC2IF = 0; //voir DSPIC30F6011.pdf p.52
}

//++++++++++++++++++++++++++++++++
// INTERUPTION 5 SUR IC4,RD11, Balise C
//++++++++++++++++++++++++++++++++
void __attribute__((__interrupt__)) _IC4Interrupt(void)
{
	if(DerniereBaliseEnnemi!=4)
	{
	    ErreurIterationBaliseEnnemi = ErreurIterationBaliseEnnemi || (DerniereBaliseEnnemi != 3);
	 	DerniereBaliseEnnemi =4;

		//ReadCapture3(&TimeIC3);
		TimeIC4 = TMR3;
		//logHistory(4);
		CalculTemps(5, TimeIC4);
		

	}
    IFS1bits.IC4IF = 0;
}

//++++++++++++++++++++++++++++++++
// RECEPTION MESSAGE ONDE RADIO
//++++++++++++++++++++++++++++++++
void DoMessageReceive(void)
{	
	//LgrMessage longueur message reception
	//BrutMessageReceive[0 a longueur message -1] reception
}

//++++++++++++++++++++++++++++++++
// INTERUPTION SUR TEMPO T5
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


//++++++++++++++++++++++++++++++++
// FONCTION PRINCIPAL
//++++++++++++++++++++++++++++++++
int main(void)
{
   	//Declaration de base
	__asm__ (".global HEAPSIZE\n .equiv HEAPSIZE,0x100"); 

    //Initialisation du controleur
    InitBoard();
	
	//Initialisation des variables	
	T5CON = 0x20;//0x10 prédiviseur 8x //// 0x20; //prédiviseur 64x
	TMR5 = 0;
	PR5 = 0x7530;
	Timer5Int = 1;
	IPC5bits.T5IP = 2;
	Timer5Enable = 1;
    ErreurIterationBalise =0;
	ErreurIterationBaliseEnnemi =0;
	DerniereBalise=0;
	DerniereBaliseEnnemi=0;
	indexHistory = 0;
	indexTimeDebug = 0;

	//Activer timer 3 interupt et priorité 1
	ConfigIntCapture1(IC_INT_PRIOR_6 & IC_INT_ON);
	ConfigIntCapture2(IC_INT_PRIOR_1 & IC_INT_ON);
	ConfigIntCapture3(IC_INT_PRIOR_6 & IC_INT_ON);
	ConfigIntCapture4(IC_INT_PRIOR_2 & IC_INT_ON);
	ConfigIntCapture5(IC_INT_PRIOR_6 & IC_INT_ON);
	ConfigIntCapture6(IC_INT_PRIOR_2 & IC_INT_ON);

	T3CON = 0x20;//0x10 prédiviseur 8x //// 0x20; //prédiviseur 64x
	TMR3 = 0;				//registre du timer 3
	PR3 = 0xFFFF;			//valeur de declenchement de l'interrupt
	Timer3Int = 0;			//activation ou non de l'interrupt sur timer 3	
	//IPC3bits.T5IP = 3;  	//priorite du timer 3
	Timer3Enable = 1;		//demarrage du timer 3
    

	/* Configure the InputCapture in stop in idle mode , Timer
	3 as source , interrupt on capture 1, I/C on every fall
	edge */
	OpenCapture1(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
	OpenCapture2(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
	OpenCapture3(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
	OpenCapture4(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
	OpenCapture5(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
	OpenCapture6(IC_IDLE_STOP & IC_TIMER3_SRC &	IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
    //Initialisation de la radio
	InitializeRadio(DoMessageReceive);

	//Programme principal
	while(1)
	{
	;
	}
}



