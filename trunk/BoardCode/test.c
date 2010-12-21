#include "msp430x22x2.h"
// #include <io.h>		    //Definitionen der Ein und Ausgaberegister des
				        //Prozessors

int main()				//main-Funktion
{
	unsigned int i=25000;		//Z?hlveriable
	WDTCTL = WDTPW + WDTHOLD;	//Watchdog Timer deaktivieren
	P1DIR=BIT0+BIT1;			//Den Pin 1 an Port 2 als ausgang schalten
					//gilt für das Textdisplay, beim Grafikdisplay
					//entsprechend an Pin 3, 4 oder 5 anpassen

	P1OUT |= BIT0+BIT1;		//Licht an
while(i--);
	while(1)
	{
		//Z?hler neu laden
		P1OUT&=~BIT0;		//Beleuchtung "Togglen"
	}
}
