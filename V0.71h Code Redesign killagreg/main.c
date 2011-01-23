// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) 04.2007 Holger Buss
// + Nur für den privaten Gebrauch
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Es gilt für das gesamte Projekt (Hardware, Software, Binärfiles, Sourcecode und Dokumentation),
// + dass eine Nutzung (auch auszugsweise) nur für den privaten und nicht-kommerziellen Gebrauch zulässig ist.
// + Sollten direkte oder indirekte kommerzielle Absichten verfolgt werden, ist mit uns (info@mikrokopter.de) Kontakt
// + bzgl. der Nutzungsbedingungen aufzunehmen.
// + Eine kommerzielle Nutzung ist z.B.Verkauf von MikroKoptern, Bestückung und Verkauf von Platinen oder Bausätzen,
// + Verkauf von Luftbildaufnahmen, usw.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Werden Teile des Quellcodes (mit oder ohne Modifikation) weiterverwendet oder veröffentlicht,
// + unterliegen sie auch diesen Nutzungsbedingungen und diese Nutzungsbedingungen incl. Copyright müssen dann beiliegen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Sollte die Software (auch auszugesweise) oder sonstige Informationen des MikroKopter-Projekts
// + auf anderen Webseiten oder Medien veröffentlicht werden, muss unsere Webseite "http://www.mikrokopter.de"
// + eindeutig als Ursprung verlinkt und genannt werden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder Funktion
// + Benutzung auf eigene Gefahr
// + Wir übernehmen keinerlei Haftung für direkte oder indirekte Personen- oder Sachschäden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Portierung der Software (oder Teile davon) auf andere Systeme (ausser der Hardware von www.mikrokopter.de) ist nur
// + mit unserer Zustimmung zulässig
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Funktion printf_P() unterliegt ihrer eigenen Lizenz und ist hiervon nicht betroffen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Redistributions of source code (with or without modifications) must retain the above copyright notice,
// + this list of conditions and the following disclaimer.
// +   * Neither the name of the copyright holders nor the names of contributors may be used to endorse or promote products derived
// +     from this software without specific prior written permission.
// +   * The use of this project (hardware, software, binary files, sources and documentation) is only permittet
// +     for non-commercial use (directly or indirectly)
// +     Commercial use (for excample: selling of MikroKopters, selling of PCBs, assembly, ...) is only permitted
// +     with our written permission
// +   * If sources or documentations are redistributet on other webpages, out webpage (http://www.MikroKopter.de) must be
// +     clearly linked as origin
// +   * porting to systems other than hardware from www.mikrokopter.de is not allowed
// +  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// +  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// +  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// +  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// +  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// +  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// +  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// +  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// +  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// +  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// +  POSSIBILITY OF SUCH DAMAGE.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <avr/boot.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "main.h"
#include "timer0.h"
#include "timer2.h"
#include "uart.h"
#if defined (__AVR_ATmega644P__)
#include "uart1.h"
#endif
#include "led.h"
#include "menu.h"
#include "fc.h"
#include "rc.h"
#include "analog.h"
#include "printf_P.h"
#ifdef USE_KILLAGREG
#include "mm3.h"
#endif
#ifdef USE_NAVICTRL
#include "spi.h"
#endif
#ifdef USE_MK3MAG
#include "mk3mag.h"
#endif
#include "twimaster.h"
#include "eeprom.h"
#include "_Settings.h"


uint8_t BoardRelease = 10;


//############################################################################
//Hauptprogramm
int main (void)
//############################################################################
{
	unsigned int timer;

	// disable interrupts global
	cli();

	// get board release
    DDRB  = 0x00;
    PORTB = 0x00;
    for(timer = 0; timer < 1000; timer++); // make some delay
    if(PINB & (1<<PINB0))
    {
		if(PINB & (1<<PINB1)) BoardRelease = 13;
		else BoardRelease = 11; // 12 is the same hardware
	}
    else
    {
		if(PINB & (1<<PINB1)) BoardRelease = 20; //
		else BoardRelease = 10;
	}

	// set LED ports as output
	DDRB |= (1<<DDB1)|(1<<DDB0);
	RED_ON;
	GRN_OFF;

	// disable watchdog
    MCUSR &=~(1<<WDRF);
    WDTCSR |= (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;

    BeepTime = 2000;

	PPM_in[CH_GAS] = 0;
	StickYaw = 0;
	StickRoll = 0;
	StickNick = 0;

    RED_OFF;

	// initalize modules
	//LED_Init(); Is done within ParamSet_Init() below
    TIMER0_Init();
    TIMER2_Init();
	USART0_Init();

	#if defined (__AVR_ATmega644P__)
	if (BoardRelease > 10) USART1_Init();
	#endif

    RC_Init();
   	ADC_Init();
	I2C_Init();


	#ifdef USE_NAVICTRL
	SPI_MasterInit();
	#endif
	#ifdef USE_KILLAGREG
	MM3_Init();
	#endif
	#ifdef USE_MK3MAG
	MK3MAG_Init();
	#endif

	// enable interrupts global
	sei();

	printf("\n\rFlightControl\n\rHardware:%d.%d\n\rSoftware:V%d.%d%c ",BoardRelease/10,BoardRelease%10, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH + 'a');
	printf("\n\r==============================");
	GRN_ON;

	// Parameter set handling
	ParamSet_Init();

    if(GetParamWord(PID_ACC_NICK) > 1023)
     {
       printf("\n\rACC not calibrated!");
     }

	//wait for a short time (otherwise the RC channel check won't work below)
	timer = SetDelay(500);
	while(!CheckDelay(timer));


	if(ParamSet.GlobalConfig & CFG_HEIGHT_CONTROL)
	 {
	   printf("\n\rCalibrating air pressure sensor..");
	   timer = SetDelay(1000);
       SearchAirPressureOffset();
   	   while (!CheckDelay(timer));
       printf("OK\n\r");
	}

	#ifdef USE_NAVICTRL
	printf("\n\rSupport for NaviCtrl");
	#endif

	#ifdef USE_KILLAGREG
	printf("\n\rSupport for MicroMag3 Compass");
	#endif

	#ifdef USE_MK3MAG
	printf("\n\rSupport for MK3MAG Compass");
	#endif



	#if (defined (USE_KILLAGREG) || defined (USE_MK3MAG))
	#if defined (__AVR_ATmega644P__)
	if(BoardRelease == 10)
	{
		printf("\n\rSupport for GPS at 1st UART");
	}
	else
	{
		printf("\n\rSupport for GPS at 2nd UART");
	}
	#else // (__AVR_ATmega644__)
	printf("\n\rSupport for GPS at 1st UART");
	#endif
	#endif



	SetNeutral();

	RED_OFF;

    BeepTime = 2000;
    ExternControl.Digital[0] = 0x55;


	printf("\n\rControl: ");
	if (ParamSet.GlobalConfig & CFG_HEADING_HOLD) printf("HeadingHold");
	else printf("Neutral");

	printf("\n\n\r");

    LCD_Clear();

    I2CTimeout = 5000;

	while (1)
	{
        if(UpdateMotor)      // control interval
        {
			UpdateMotor = 0; // reset Flag, is enabled every 2 ms by isr of timer0
			//PORTD |= (1<<PORTD4);
            MotorControl();
			//PORTD &= ~(1<<PORTD4);

			SendMotorData();

            RED_OFF;

            if(PcAccess) PcAccess--;
            else
            {
			   ExternControl.Config = 0;
               ExternStickNick= 0;
               ExternStickRoll = 0;
               ExternStickYaw = 0;
            }

            if(!I2CTimeout)
            {
				I2CTimeout = 5;
				I2C_Reset();
				if((BeepModulation == 0xFFFF) && (MKFlags & MKFLAG_MOTOR_RUN) )
				{
					BeepTime = 10000; // 1 second
					BeepModulation = 0x0080;
				}
			}
			else
			{
				I2CTimeout--;
				RED_OFF;
			}

			if(SIO_DEBUG && (!UpdateMotor || !(MKFlags & MKFLAG_MOTOR_RUN) ))
			{
				USART0_TransmitTxData();
				USART0_ProcessRxData();
			}
			else USART0_ProcessRxData();

			if(CheckDelay(timer))
			{
            	if(UBat < ParamSet.LowVoltageWarning)
                {
					BeepModulation = 0x0300;
					if(!BeepTime )
					{
						BeepTime = 6000; // 0.6 seconds
					}
                }
				#ifdef USE_NAVICTRL
				SPI_StartTransmitPacket();
				SendSPI = 4;
				#endif
				timer = SetDelay(20); // every 20 ms
            }

            LED_Update();
		}

		#ifdef USE_NAVICTRL
		if(!SendSPI)
		{	// SendSPI is decremented in timer0.c with a rate of 9.765 kHz.
			// within the SPI_TransmitByte() routine the value is set to 4.
			// I.e. the SPI_TransmitByte() is called at a rate of 9.765 kHz/4= 2441.25 Hz,
			// and therefore the time of transmission of a complete spi-packet (32 bytes) is 32*4/9.765 kHz = 13.1 ms.
			SPI_TransmitByte();
		}
		#endif
    }
 	return (1);
}

