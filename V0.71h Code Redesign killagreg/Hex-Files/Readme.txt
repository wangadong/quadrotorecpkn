V0.70d Ausgangsversion.

G.Stobrawa 02.08.2008:

 - Code st�rker modularisiert und restrukturiert
 - viele Kommentare zur Erkl�rug eingef�gt
 - konsequent englische Variablennamen 
 - PPM24 Support f�r bis zu 12 RC-Kan�le.
 - Support f�r Kompass CMPS01, MK3MAG oder MM3
 - 2. Uart wird nun unterst�tzt (MCU = atmega644p im Makefile)
 - Auswertung des UBX-Protocols an 1.  oder 2. Uart
 - einige kleinere Bugfixes
 - GPS-Hold-Funktion hinzugef�gt
 - GPS-Home-Funktion hinzugef�gt (wird beim kalibrieren gelernt)
 - Zus�tzliche Punkte im Men� des KopterTool zur Anzeige des GPS-Status und der MM3-Kalibierparameter


- Hardware Configuration:

   - Die PWM des MK3MAG/CMPS03 wird wie bisher standard �ber den Port PC4 (Pin5 an SV1 der FC) eingelesen.
 
   - Der MM3 wird wie folgt verbunden.

		FC 1.0/1.1/1.2 Level Shifter  MM3
		SCK (Pin1 SV5)    --->     SCK (Pin1)
		MISO (Pin3 SV5)   <---     MISO (Pin2)
		MOSI (Pin5 SV5)   --->     MOSI (Pin3)
		GND (Pin6 SV5)    --->     GND (Pin7 / Pin14)
		PC4 (Pin5 SV1)    --->     SS (Pin4)
		PC5 (Pin6 SV1)    --->     RESET (Pin6)

     Zus�tzlich ben�tigt der MM3 noch eine 3V oder 3V3 Versorgung an VDD (Pin12).

    - F�r das UBLOX-Modul muss noch GPS-GND mit FC-GND (Pin7 SV1) und
      die GPS-TXD Leitung mit FC-RXD (Pin1 SV1) verbunden werden,
      wenn man die FC 1.0 mit dem Atmega644 verwendet. 
      F�r die FC 1.1/1.2 mit Atmega644p-Best�ckung ben�tigt man FC-RXD1 (Pin3 SV1).
      Zus�tzlich ben�tigt das UBLOX-Modul noch eine 5V-Versorgung die ggf. von 
      der FC (an Pin2 SV1) abgegriffen werden kann.
      Wenn die FC g�ltige  Messages vom GPS empf�ngt, blinkt die rote LED mit 4 Hz.


- Konfiguration des MK
	- Die Firmware 0.70d ben�tigt mindestens das Mikrokpter Tool 1.53
 
	- Es sollte ein Haken bei GPS und Kompass gesetzt sein. Wenn nur GPS aktiviert ist, wird intern immer auch der
          Kompass aktiviert, da er f�r den GPS-Regler unbedingt notwendig ist.

	- Unter Sonstiges: Kompass-Wirkung etwa auf ca. 127.

	- User Parameters: (nur notwendig wenn man den MM3 verwendet)
	  Parameter 3 --> Calibration factor for transforming Gyro Integrals to angular degrees (~170) 
	  Parameter 4 --> Angle between the MM3 Board (Arrow) and the MK head (typical ~180)

	- NaviCtrl Paramter: (werden benutzt um die GPS-Regelung zu parametrisieren)
          - GPS Mode Control:
            Setzt man normalerweise auf ein Poti um w�rend des Fluges die GPS Funktion schalten zu k�nnen.
            <  50    : --> aid mode  (aktuelle Position wird gehalten, wenn keine Nick/Roll-Sticks in Mittenstellung)
            50 ..180 : --> free mode (GPS-Regelung inaktiv)
            > 180    : --> home mode (MK fliegt zur Home-Position, wenn diese nicht abgespeichert wurde -> aid mode)
            Ein Wechsel des Modes wird durch einen kurzen Piep quitiert.	
		
          - GPS Gain:
            Das ist der Verst�rkungsfaktor mit dem die GPS-Regelung in die Steuerung eingreift.
            Der typischer Wert ist 100. Man sollte diesen Wert anpassen, wenn nach einer einmal optimierten
            Kombination aus P,I,D aufgrund eines ver�nderten Fluggewichts oder Motorleistung das Regelverhalten
            zu stark (Wert verringern) oder zu schwach (Wert erh�hen) ausf�llt.
             
          - GPS Stick Threshold:
            Dieser Wert legt fest, ab welchem Nick/Roll-Stickausschlag ein manueller Eingriff des Piloten erkannt wird,
            und daher die GPS-Regelung abgeschatet wird. (Typicher Wert 10) Man sollte den Wert nicht zu kein w�hen,
            da man durch Trimmung an der Funke meist einige Counts in Mittelstellug des Nick/Roll-Sticks vorliegen.

          - Min. Sat.:
            Dies ist die Mindestanzahl der Sateliten, die bei einem 3D-Satfix empfangen werden m�ssen,
            damit die GPS-Regelung aktiv wird. F�r einen 3-D Satfix sind mindestens 4 Sats notwendig. Ein typischer Wert
            von 6 garantiert ausreichende GPS-Signalstabilit�t. 

          - GPS-P: ca. 90
          - GPS-I: ca. 5
          - GPS-D: ca 90
          - GPS Acc: unused

	  Der P-Parameter legt die St�rke der Regelung auf eine Positionsabweichung fest, d.h. unser virtueller
          Hilfspilot steuert st�rker zur Zielrichtung wenn dieser Faktor w�chst. Man kann sich das wie bei einem Pendel vorstellen.
          Je weiter es ausgelenkt ist (je weiter der MK von der Zielposition entfernt ist) desto st�rker ist die r�ckstellende Kraft.
          Das f�hrt nun gerade dazu das das Pendel in Richtung der Nulllage beschleunigt wird. Ist die Nulllage erreicht, wirkt in
          diesem Moment auch keine R�ckstellkraft. Jedoch hat das Pendel dort noch eine Geschwindigkeit, die dazu f�hrt dass es
          �ber die Nulllage hinweg ausschl�gt (der MK schie�t �ber das Ziel hinaus). Ohne jede "Reibung" oder D�mpfung w�rde
          das Spielchen immer so weiter gehen. Ja es kann sich sogar aufschaukeln. Je h�her dabei der P-Parameter dest st�rker
          schiebt man das Pendel an.

          Deshalb gibt es den D-Parameter, der daf�r sorgt, dass proportional gegen jede Geschwindigkeit �ber Grund gegensteuert wird.
          Das ist der Reibungsfaktor im Regelsystem. Daher wirkt es im Vergleich mit dem Pendel wie eine Schwerg�ngigkeit
          durch Reibung im System. Ist die Reibung sehr gro�, so w�rde ein ausgelenktes Pendel sehr langsam in die Nullage kriechen
          und dort stehenbleiben. Ist die Reibung klein schwingt das System noch eine ganze Weile nach.
          Nun gibt es aber genau ein Verh�ltnis von R�ckstellkraft (P) und Reibung (D) bei der ein ausgelenktes Pendel z�gig
          in die Nullage schwingt und dort stehenbleibt (nennt sich aperiodischer Genzfall). Dieses Setting gilt es zu finden.
          Das h�ngt nun aber von der Reaktion des MK auf die virtuellen GPS-Pilotenstickbewegungen ab. Diese unterligen nicht
          den P und D Werten der RC-Sticksettings, wohl aber den Gyrosettings.

          Obendrauf kommt noch die Tatsache, dass die GPS-Position und Geschwindigkeit �ber Gund auch bei einem unbewegten Kopter
          aufgrund der atmosph�rischen St�rungen und der Empfangsqualit�t schwanken. �bertragen auf das Bild mit dem Pendel bedeutet
          dies, dass der Punkt schwankt, an dem das Pendel aufgeh�ngt ist. Das f�hrt unweigerlich zum aufschaukeln,
          falls die Reibung (D) nicht ausreichend gro� ist.

          Ich hoffe diese Anschauung verhilft nun einigen hier die Parameter der GPS-Regelung besser zu verstehen
          und aus der Beobachtung des Flugverhaltens des MK auf die notwendige Parameter�nderung zu schlie�en.
	

- Zus�tzliche akustische Signale:

	Signale die eine Fehlfunktion anzeigen: 
	
	Dauerton:	Eine GPS-Funktion ist aktiviert und es werden keine oder
			unvollst�ndige GPS-Daten via UART empfangen.
			Hier ist zu ber�cksichtigen, dass die folgenden UBX Messages am UBLOX
			aktiviert wurden: NAV-POSLLH, NAV-SOL, NAV-VELNED.
			Fehlt eine dieser Messages wird breits der Dauerton gesetzt.
	
	2Hz Piepen:	Ist eine der GPS-Modi Aid oder Home aktiviert, zeigt dass den Empfang valider UBX-Daten,
			wenn noch kein 3D-Satfix vorliegt. Liegt ein 3D-SatFix vor und werden ausreichend Satelliten
                        enpfangen, so verstummt der Pieper.
			
	5Hz Piepen:     Ist der Comming Home Mode aktiv (GPS Mode Control>180) und wurde keine Home-Position 
                        gespeichert, so ert�nt ein 5Hz-Piepen.        
 	
	10Hz Piepen:	Die Kommunikation zum Kompassmodul ist gest�rt. (Funktioniert bei beiden Kompassmodulen)
			Der CompassValue in den Debugs vom Koptertool zeigt dann -1 an.

	Signale die eine Aktion best�tigen.
			
	1s Piepen:	Ert�nt nach dem Kalibrieren wenn die Home-Position erfolgreich gespeichert wurde.
        kurzer Piep:    Ert�nt bei einem Wechsel des GPS Control Modes.



- Inbetriebnahme:

	Nach dem Flashen der FC auf die Verison 0.70d sollte man zur Sicherheit den EEProm reseten und die Kalibrierung
        f�r den MK3MAG/MM3 wiederholen, da diese Daten an einer anderen Position im EEProm der FC abgelegt
        und wieder gelesen werden.

	Das Vorgehen erfolgt beim MM3 und MK3MAG exakt gleich wie hier beschrieben.
	http://www.mikrokopter.de/ucwiki/MK3Mag?highlight=%28mk3mag%29

	Zur Bestimmung des User Parameters 3 (Umrechnungsfaktor zwischen dem Gyrointegral und dem zugeh�rigen Neigungswinkel)
	hat sich folgendes Vorgehen bewehrt.
	
	- Man bestimmt den Wert des Roll- und Nick-Integrals f�r einen Neigungswinkel von 45�
          �ber die Ausgaben des Koptertools.
	- Aus diesem Wert kann man den ben�tigten UserParam3 berechnen.
		
		UserParam3 = (Nick-Integral(45�)+Roll-Integral(45�))/2*GyroACCFaktor/45�/8.     (typisch 170)
		
	- Nachdem dieser Wert �ber die Settings des Koptertools im MK abgepeichert ist, sollte sich der CompassValue bei
	  Verkippungen nur unwesentlich ver�ndern. 

	- Preflight GPS Test:
	  Ist der Kopter eingeschaltet, so kann man den GPS-Empfang �berpr�fen, in dem man GPS Mode Control < 50 setzt.
          Erh�lt man ein Dauerpiepen besteht keine Kommunikation zum GPS oder eine der ben�tigten UBX-Messseages fehlt.
          Blinkt die rote LED der FC, so werden grunds�tzlich valide Daten vom GPS empfangen. Abhilfe schafft dann die 
          Einstellung des GPS-Moduls via USB und u-Center sodass die UBX Messages NAV-POSLLH, NAV-SOL, NAV-VELNED
          mit 57600 baud auf zum Target 1 (RS232) gesendet werden. Es empfiehlt sich alle anderen Sendungen inkl. NMEA
          abzuschalten, da diese sonst durch den UBX-Parser auf der FC laufen und sinnlose Rechenzeit beanspruchen.
          Am einfachsten geht das mit dem Konfigurationsfile "Conrad LEA-5H Config.txt". Dazu verbindet man das
          UBLOX-Modul via USB mit dem PC und started das u-Center. Dan w�hlt man im Men�:
          "Tools->GPS Configuration" dieses File aus und klickt auf den Button "File >> GPS". 
          Hat man diese H�rde genommen (Dauerpiepsen ist Weg), wird wahrscheinlich ein 5Hz piepen zu h�ren sein.
          Dieses zeigt den korrekten Empfang aller UBX-Messages an. Je mehr Satelitten empfangen werden, desto l�nger werden
          die Pausen zwischen den Pieps, bis sie schlei�lich ganz verschwinden, was einen 3D-Satfix signalisiert.
          Die GPS-Funktionen k�nnen ggf. nun wieder abgeschaltet werden (50<GPS Mode Control<180). 
       
        - Aid Mode (Position Hold): 
          Ist 50 < GPS Mode Control < 180 so ist die dynamische Position Hold Funktion aktiv.
          Solange sich der Nick- & Roll-Stick in Zentralposiotion befinden (genauer Auschlag < GPS Stick Threshold)
          wird die laterale (XY)-Position durch die GPS-Regelung kontrolliert. Dabei wird versucht die Abweichung
          der aktuellen GPS-Position von der zum Zeitpunkt der letzen Nick/Roll-Stickbewegung gespeicherten Position zu                   minimieren. (Siehe dazu auch Gain-, D- & P-Parameter des GPS-Reglers).
          Dadurch kann man den MK manuell zu einer bestimmten Position steuern und beh�lt diese dann bei.  

          Es kann vorkommen, das bei extremer Timmerstellung an Nick und Roll der Funke der
          Stick Threshold bereits �berschritten wird. In diesem Fall wird das Position Hold 
          leider nicht aktiviert. Durch die automatische Abschaltung des GPS-Reglers f�r den Fall einer manuellen
          Bedienung kann man jederzeit in das Flugverhalten eingreifen.

        - Home Mode (Comming Home):
	  Man kann die Home Position setzen, indem man bei ausgeschateten Motoren den Gas/Gier-Stick nach oben rechts oder
          links dr�ckt. (Also w�hrend des Kalibrierens) Es kann sein, das zu diesem zeitpunkt noch kein SatFix vorliegt.
          In diesem Fall wird die Home-Position nicht gespeichert. Aktiviert man dann sp�ter w�hrend des Fluges                           den Home Mode (GPS Mode Control > 180) wird dieser Zustand mit einem 5Hz Piepen angezeigt. Der Kopter versucht dann
          wenigstens ein AID Mode.
          
          Das Comming Home ist analog dem Position Hold,
          Jedoch ist das Regelziel nicht die zuletzt gespeichwerte GPS-Position sondern die gespeicherte Home Position.
          Eine manualle Bedienung von Nick/Roll-Stick unterbricht auch hier sofort den GPS-Regler und die Steuerung erfolgt
          manuell.

 
  
