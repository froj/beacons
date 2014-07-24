//Define.h
//#define __dsPIC30F6011__
#include <p30F6011A.h>
#define FCY  7372800		// xtal = 7.3728Mhz; PLLx4

#define LISSING_FACTOR 0.5

#define TIME_OUT_DATA 6

#define NB_BALISES 2
#define BALISE_ID 1    //0=Notre robot 1=Adversaire


//#define BALISE_MASTER
//#define ROBOT_AMI
//#define ROBOT_ENNEMI
//#define BALISE_ESPION

//#ifdef BALISE_MASTER
	#define EMISSION_RADIO
	//#define READ_US		
//#endif

//#ifdef ROBOT_AMI
	//#define RECEPTION_RADIO
	//#define READ_IR
	//#define EMMIT_US
//#endif

//#ifdef ROBOT_ENNEMI
	//#define RECEPTION_RADIO
	//#define EMMIT_IR
	//#define EMMIT_US
//#endif

//#ifdef BALISE_ESPION
	//#define RECEPTION_RADIO
//#endif
