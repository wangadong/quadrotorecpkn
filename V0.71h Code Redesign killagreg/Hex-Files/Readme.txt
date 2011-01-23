V0.70d Ausgangsversion.

G.Stobrawa 02.08.2008:

 - Code stärker modularisiert und restrukturiert
 - viele Kommentare zur Erklärug eingefügt
 - konsequent englische Variablennamen 
 - PPM24 Support für bis zu 12 RC-Kanäle.
 - Support für Kompass CMPS01, MK3MAG oder MM3
 - 2. Uart wird nun unterstützt (MCU = atmega644p im Makefile)
 - Auswertung des UBX-Protocols an 1.  oder 2. Uart
 - einige kleinere Bugfixes
 - GPS-Hold-Funktion hinzugefügt
 - GPS-Home-Funktion hinzugefügt (wird beim kalibrieren gelernt)
 - Zusätzliche Punkte im Menü des KopterTool zur Anzeige des GPS-Status und der MM3-Kalibierparameter


- Hardware Configuration:

   - Die PWM des MK3MAG/CMPS03 wird wie bisher standard über den Port PC4 (Pin5 an SV1 der FC) eingelesen.
 
   - Der MM3 wird wie folgt verbunden.

		FC 1.0/1.1/1.2 Level Shifter  MM3
		SCK (Pin1 SV5)    --->     SCK (Pin1)
		MISO (Pin3 SV5)   <---     MISO (Pin2)
		MOSI (Pin5 SV5)   --->     MOSI (Pin3)
		GND (Pin6 SV5)    --->     GND (Pin7 / Pin14)
		PC4 (Pin5 SV1)    --->     SS (Pin4)
		PC5 (Pin6 SV1)    --->     RESET (Pin6)

     Zusätzlich benötigt der MM3 noch eine 3V oder 3V3 Versorgung an VDD (Pin12).

    - Für das UBLOX-Modul muss noch GPS-GND mit FC-GND (Pin7 SV1) und
      die GPS-TXD Leitung mit FC-RXD (Pin1 SV1) verbunden werden,
      wenn man die FC 1.0 mit dem Atmega644 verwendet. 
      Für die FC 1.1/1.2 mit Atmega644p-Bestückung benötigt man FC-RXD1 (Pin3 SV1).
      Zusätzlich benötigt das UBLOX-Modul noch eine 5V-Versorgung die ggf. von 
      der FC (an Pin2 SV1) abgegriffen werden kann.
      Wenn die FC gültige  Messages vom GPS empfängt, blinkt die rote LED mit 4 Hz.


- Konfiguration des MK
	- Die Firmware 0.70d benötigt mindestens das Mikrokpter Tool 1.53
 
	- Es sollte ein Haken bei GPS und Kompass gesetzt sein. Wenn nur GPS aktiviert ist, wird intern immer auch der
          Kompass aktiviert, da er für den GPS-Regler unbedingt notwendig ist.

	- Unter Sonstiges: Kompass-Wirkung etwa auf ca. 127.

	- User Parameters: (nur notwendig wenn man den MM3 verwendet)
	  Parameter 3 --> Calibration factor for transforming Gyro Integrals to angular degrees (~170) 
	  Parameter 4 --> Angle between the MM3 Board (Arrow) and the MK head (typical ~180)

	- NaviCtrl Paramter: (werden benutzt um die GPS-Regelung zu parametrisieren)
          - GPS Mode Control:
            Setzt man normalerweise auf ein Poti um wärend des Fluges die GPS Funktion schalten zu können.
            <  50    : --> aid mode  (aktuelle Position wird gehalten, wenn keine Nick/Roll-Sticks in Mittenstellung)
            50 ..180 : --> free mode (GPS-Regelung inaktiv)
            > 180    : --> home mode (MK fliegt zur Home-Position, wenn diese nicht abgespeichert wurde -> aid mode)
            Ein Wechsel des Modes wird durch einen kurzen Piep quitiert.	
		
          - GPS Gain:
            Das ist der Verstärkungsfaktor mit dem die GPS-Regelung in die Steuerung eingreift.
            Der typischer Wert ist 100. Man sollte diesen Wert anpassen, wenn nach einer einmal optimierten
            Kombination aus P,I,D aufgrund eines veränderten Fluggewichts oder Motorleistung das Regelverhalten
            zu stark (Wert verringern) oder zu schwach (Wert erhöhen) ausfällt.
             
          - GPS Stick Threshold:
            Dieser Wert legt fest, ab welchem Nick/Roll-Stickausschlag ein manueller Eingriff des Piloten erkannt wird,
            und daher die GPS-Regelung abgeschatet wird. (Typicher Wert 10) Man sollte den Wert nicht zu kein wähen,
            da man durch Trimmung an der Funke meist einige Counts in Mittelstellug des Nick/Roll-Sticks vorliegen.

          - Min. Sat.:
            Dies ist die Mindestanzahl der Sateliten, die bei einem 3D-Satfix empfangen werden müssen,
            damit die GPS-Regelung aktiv wird. Für einen 3-D Satfix sind mindestens 4 Sats notwendig. Ein typischer Wert
            von 6 garantiert ausreichende GPS-Signalstabilität. 

          - GPS-P: ca. 90
          - GPS-I: ca. 5
          - GPS-D: ca 90
          - GPS Acc: unused

	  Der P-Parameter legt die Stärke der Regelung auf eine Positionsabweichung fest, d.h. unser virtueller
          Hilfspilot steuert stärker zur Zielrichtung wenn dieser Faktor wächst. Man kann sich das wie bei einem Pendel vorstellen.
          Je weiter es ausgelenkt ist (je weiter der MK von der Zielposition entfernt ist) desto stärker ist die rückstellende Kraft.
          Das führt nun gerade dazu das das Pendel in Richtung der Nulllage beschleunigt wird. Ist die Nulllage erreicht, wirkt in
          diesem Moment auch keine Rückstellkraft. Jedoch hat das Pendel dort noch eine Geschwindigkeit, die dazu führt dass es
          über die Nulllage hinweg ausschlägt (der MK schießt über das Ziel hinaus). Ohne jede "Reibung" oder Dämpfung würde
          das Spielchen immer so weiter gehen. Ja es kann sich sogar aufschaukeln. Je höher dabei der P-Parameter dest stärker
          schiebt man das Pendel an.

          Deshalb gibt es den D-Parameter, der dafür sorgt, dass proportional gegen jede Geschwindigkeit über Grund gegensteuert wird.
          Das ist der Reibungsfaktor im Regelsystem. Daher wirkt es im Vergleich mit dem Pendel wie eine Schwergängigkeit
          durch Reibung im System. Ist die Reibung sehr groß, so würde ein ausgelenktes Pendel sehr langsam in die Nullage kriechen
          und dort stehenbleiben. Ist die Reibung klein schwingt das System noch eine ganze Weile nach.
          Nun gibt es aber genau ein Verhältnis von Rückstellkraft (P) und Reibung (D) bei der ein ausgelenktes Pendel zügig
          in die Nullage schwingt und dort stehenbleibt (nennt sich aperiodischer Genzfall). Dieses Setting gilt es zu finden.
          Das hängt nun aber von der Reaktion des MK auf die virtuellen GPS-Pilotenstickbewegungen ab. Diese unterligen nicht
          den P und D Werten der RC-Sticksettings, wohl aber den Gyrosettings.

          Obendrauf kommt noch die Tatsache, dass die GPS-Position und Geschwindigkeit über Gund auch bei einem unbewegten Kopter
          aufgrund der atmosphärischen Störungen und der Empfangsqualität schwanken. Übertragen auf das Bild mit dem Pendel bedeutet
          dies, dass der Punkt schwankt, an dem das Pendel aufgehängt ist. Das führt unweigerlich zum aufschaukeln,
          falls die Reibung (D) nicht ausreichend groß ist.

          Ich hoffe diese Anschauung verhilft nun einigen hier die Parameter der GPS-Regelung besser zu verstehen
          und aus der Beobachtung des Flugverhaltens des MK auf die notwendige Parameteränderung zu schließen.
	

- Zusätzliche akustische Signale:

	Signale die eine Fehlfunktion anzeigen: 
	
	Dauerton:	Eine GPS-Funktion ist aktiviert und es werden keine oder
			unvollständige GPS-Daten via UART empfangen.
			Hier ist zu berücksichtigen, dass die folgenden UBX Messages am UBLOX
			aktiviert wurden: NAV-POSLLH, NAV-SOL, NAV-VELNED.
			Fehlt eine dieser Messages wird breits der Dauerton gesetzt.
	
	2Hz Piepen:	Ist eine der GPS-Modi Aid oder Home aktiviert, zeigt dass den Empfang valider UBX-Daten,
			wenn noch kein 3D-Satfix vorliegt. Liegt ein 3D-SatFix vor und werden ausreichend Satelliten
                        enpfangen, so verstummt der Pieper.
			
	5Hz Piepen:     Ist der Comming Home Mode aktiv (GPS Mode Control>180) und wurde keine Home-Position 
                        gespeichert, so ertönt ein 5Hz-Piepen.        
 	
	10Hz Piepen:	Die Kommunikation zum Kompassmodul ist gestört. (Funktioniert bei beiden Kompassmodulen)
			Der CompassValue in den Debugs vom Koptertool zeigt dann -1 an.

	Signale die eine Aktion bestätigen.
			
	1s Piepen:	Ertönt nach dem Kalibrieren wenn die Home-Position erfolgreich gespeichert wurde.
        kurzer Piep:    Ertönt bei einem Wechsel des GPS Control Modes.



- Inbetriebnahme:

	Nach dem Flashen der FC auf die Verison 0.70d sollte man zur Sicherheit den EEProm reseten und die Kalibrierung
        für den MK3MAG/MM3 wiederholen, da diese Daten an einer anderen Position im EEProm der FC abgelegt
        und wieder gelesen werden.

	Das Vorgehen erfolgt beim MM3 und MK3MAG exakt gleich wie hier beschrieben.
	http://www.mikrokopter.de/ucwiki/MK3Mag?highlight=%28mk3mag%29

	Zur Bestimmung des User Parameters 3 (Umrechnungsfaktor zwischen dem Gyrointegral und dem zugehörigen Neigungswinkel)
	hat sich folgendes Vorgehen bewehrt.
	
	- Man bestimmt den Wert des Roll- und Nick-Integrals für einen Neigungswinkel von 45°
          über die Ausgaben des Koptertools.
	- Aus diesem Wert kann man den benötigten UserParam3 berechnen.
		
		UserParam3 = (Nick-Integral(45°)+Roll-Integral(45°))/2*GyroACCFaktor/45°/8.     (typisch 170)
		
	- Nachdem dieser Wert über die Settings des Koptertools im MK abgepeichert ist, sollte sich der CompassValue bei
	  Verkippungen nur unwesentlich verändern. 

	- Preflight GPS Test:
	  Ist der Kopter eingeschaltet, so kann man den GPS-Empfang überprüfen, in dem man GPS Mode Control < 50 setzt.
          Erhält man ein Dauerpiepen besteht keine Kommunikation zum GPS oder eine der benötigten UBX-Messseages fehlt.
          Blinkt die rote LED der FC, so werden grundsätzlich valide Daten vom GPS empfangen. Abhilfe schafft dann die 
          Einstellung des GPS-Moduls via USB und u-Center sodass die UBX Messages NAV-POSLLH, NAV-SOL, NAV-VELNED
          mit 57600 baud auf zum Target 1 (RS232) gesendet werden. Es empfiehlt sich alle anderen Sendungen inkl. NMEA
          abzuschalten, da diese sonst durch den UBX-Parser auf der FC laufen und sinnlose Rechenzeit beanspruchen.
          Am einfachsten geht das mit dem Konfigurationsfile "Conrad LEA-5H Config.txt". Dazu verbindet man das
          UBLOX-Modul via USB mit dem PC und started das u-Center. Dan wählt man im Menü:
          "Tools->GPS Configuration" dieses File aus und klickt auf den Button "File >> GPS". 
          Hat man diese Hürde genommen (Dauerpiepsen ist Weg), wird wahrscheinlich ein 5Hz piepen zu hören sein.
          Dieses zeigt den korrekten Empfang aller UBX-Messages an. Je mehr Satelitten empfangen werden, desto länger werden
          die Pausen zwischen den Pieps, bis sie schleißlich ganz verschwinden, was einen 3D-Satfix signalisiert.
          Die GPS-Funktionen können ggf. nun wieder abgeschaltet werden (50<GPS Mode Control<180). 
       
        - Aid Mode (Position Hold): 
          Ist 50 < GPS Mode Control < 180 so ist die dynamische Position Hold Funktion aktiv.
          Solange sich der Nick- & Roll-Stick in Zentralposiotion befinden (genauer Auschlag < GPS Stick Threshold)
          wird die laterale (XY)-Position durch die GPS-Regelung kontrolliert. Dabei wird versucht die Abweichung
          der aktuellen GPS-Position von der zum Zeitpunkt der letzen Nick/Roll-Stickbewegung gespeicherten Position zu                   minimieren. (Siehe dazu auch Gain-, D- & P-Parameter des GPS-Reglers).
          Dadurch kann man den MK manuell zu einer bestimmten Position steuern und behält diese dann bei.  

          Es kann vorkommen, das bei extremer Timmerstellung an Nick und Roll der Funke der
          Stick Threshold bereits überschritten wird. In diesem Fall wird das Position Hold 
          leider nicht aktiviert. Durch die automatische Abschaltung des GPS-Reglers für den Fall einer manuellen
          Bedienung kann man jederzeit in das Flugverhalten eingreifen.

        - Home Mode (Comming Home):
	  Man kann die Home Position setzen, indem man bei ausgeschateten Motoren den Gas/Gier-Stick nach oben rechts oder
          links drückt. (Also während des Kalibrierens) Es kann sein, das zu diesem zeitpunkt noch kein SatFix vorliegt.
          In diesem Fall wird die Home-Position nicht gespeichert. Aktiviert man dann später während des Fluges                           den Home Mode (GPS Mode Control > 180) wird dieser Zustand mit einem 5Hz Piepen angezeigt. Der Kopter versucht dann
          wenigstens ein AID Mode.
          
          Das Comming Home ist analog dem Position Hold,
          Jedoch ist das Regelziel nicht die zuletzt gespeichwerte GPS-Position sondern die gespeicherte Home Position.
          Eine manualle Bedienung von Nick/Roll-Stick unterbricht auch hier sofort den GPS-Regler und die Steuerung erfolgt
          manuell.

 
  
