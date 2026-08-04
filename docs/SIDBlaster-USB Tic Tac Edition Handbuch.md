![SIDBlaster-USB Tic Tac Edition](images/sidblaster.jpg)

# SIDBLASTER-USB — TIC TAC EDITION

*– Ein echter SID Chip auf PC oder Mac*

## HANDBUCH

von Andreas Schumm, Rev. 1.4

Copyright© 2024 [www.crazy-midi.de](http://www.crazy-midi.de/) – Lizenz: [CC BY-ND 4.0](https://creativecommons.org/licenses/by-nd/4.0/deed.de)

---

# 1. Einführung

Der SIDBlaster-USB Tic Tac ist eine hochwertige "Open-Source"-Hardware, die als kleine, an den USB-Port eines PCs oder Macs anschließbare Box die Nutzung eines echten "SID"-Soundchips zur C64-Emulation, Wiedergabe von SID-Tunes und Musikproduktion ermöglicht.

SIDBlaster-USB Tic Tac basiert auf "SIDBlaster-USB", und ist mit diesem zu 100% kompatibel.

## 1.1 Verbesserungen

- hochwertige Universalstromversorgung 9V oder 12V
- Original C64-Audioverkabelung
- Optionale Anschlussmöglichkeit für zwei Paddles
- Umschaltbare Filterkondensatoren
- Umschaltbare Kondensatoren für Paddle
- Passgenaue Platine zum Einbau in eine Tic Tac Schachtel
- Audio-Eingang
- Professionelle 6,3mm Mono-Audio-Klinkenbuchsen
- Gemütliche blaue Power-LED
- (Rev.1.2:) rote TX/RX-LED
- (Rev. 1.2 weißer Jumper zum Verbinden von GND und Shield (experimentell)
- vorbereitet für Lesezugriff (Firmware 1.1)

# 2. Allgemeine Informationen

- Plug and Play ist nicht möglich, der SIDBlaster muss vor der Verwendung in einer Anwendung an den Computer angeschlossen werden. Trennen Sie den SIDBlaster auch nicht vom Computer, bevor Sie die Anwendung beendet haben.
- Wenn Sie den SIDBlaster nicht benutzen, ist es empfehlenswert, die USB-Verbindung zu trennen, da dies die Lebensdauer des SID-Chips verlängert.
- Da ein SID-Chip Wärme erzeugt, verwenden Sie wenn möglich einen Kühlkörper auf dem Chip und sorgen Sie für ausreichende Belüftung (lassen Sie die Klappe der Tic Tac-Box offen).

# 3. Hardware-Installation

## 3.1 SID-chip-installation

![SID-Chip Installation](images/20220505_175739-cut-Groß.jpg)

*Ein SIDBlaster-USB mit korrekt installiertem 8580*

**Warnung!** Überprüfen Sie die Spannungseinstellungen an der SIDBlaster-Hardware. Wenn Sie sie falsch einstellen, z.B. wenn Sie 12V in einen 8580 SID-Chip schicken, der mit 9V läuft, werden Sie den Chip zerstören. Wenn Sie ein Multimeter haben, **prüfen Sie die Spannung an der SID-Chip-Buchse**, um sicherzugehen. Verwenden Sie immer ESD-Schutz (z.B. ein Antistatik-Armband), um den SID-Chip und andere elektrische Komponenten nicht zu beschädigen, wenn Sie ihn in den SIDBlaster einbauen.

**Um einen SID-Chip zu entfernen**, verwenden Sie z.B. einen Schraubendreher, hebeln Sie den Chip abwechselnd heraus. Achten Sie darauf, die Platine oder die Bauteile nicht zu beschädigen.

Vor dem Einsetzen eines SID-Chips biegen Sie die Pins mit einer Zange gerade, drücken Sie vorsichtig und überprüfen Sie, dass keiner der Pins umknickt.

## 3.2 Jumper-Einstellungen

![Jumper-Übersicht](images/SIDBlaster-USB Zeichnung.jpg)

### 3.2.1 Für den 6581 SID-chip:

- JP1: muss offen sein (12V).
- JP2-JP5: Setzen Sie alle Jumper auf die linke Seite (Stifte 1-2).

### 3.2.2 Für den 8580 SID-chip:

- JP1: muss geschlossen sein (9V).
- JP2-JP5: Setzen Sie alle Jumper auf die rechte Seite (Stifte 2-3).

## 3.3 Andere Jumper der Hardware

### 3.3.1 JP6 (Weiss)

Ab Rev. 1.2 und höher. Experimentell, verbindet die USB-Abschirmung mit Masse; Sie können es benutzen, um Störgeräuschen entgegenzuwirken.

### 3.3.2 SV2:Paddle-Anschluss (gelb)

Hier haben Sie Zugriff auf die beiden A/D-Wandler des SID-Chips. Sie können z.B. 2 Drehpotentiometer als Paddles anschließen. Dies ist vor allem für Programmierer interessant. Sie können die Funktion auch mit dem SID-Objekt für Max/MSP verwenden.

| Pin | Funktion |
|---|---|
| 1 | +5V |
| 2 | POTX |
| 3 | POTY |
| 4 | GND |

![Paddle-Anschluss Diagramm](images/Paddle_diagram.png)

### 3.3.3 SV1: ISP Connector

Verwenden Sie einen Picit 3 Programmer, um den Flash-Speicher des Mikrocontrollers zu programmieren. Als Software wird MPLAB IPE verwendet.

| Pin | Funktion |
|---|---|
| 1 | MCLR/VPP |
| 2 | +5V |
| 3 | GND |
| 4 | PGD (ICSPDAT) |
| 5 | PGC (ICSPCLK) |
| 6 | n.c. |

![SV1 ISP Connector Verkabelung](images/Picit3-SV1.jpg)

## 3.4 USB- und Audioanschlüsse

### 3.4.1 USB-Anschluss

Schließen Sie die SIDBlaster-Hardware mit einem USB-Kabel vom Typ A-B von guter Qualität an einen USB-Port an. Die Hardware kann auch mit einem USB-Hub guter Qualität funktionieren.

### 3.4.2 Audio-Ausgangsbuchse

Der Audioausgang ist als professionelle 6,3mm-Klinkenbuchse ausgeführt. Verbinden Sie den Audio-Ausgang des SIDBlaster mit Ihrem Mischpult oder Audio-Interface mit einem unsymmetrischen (Mono) Kabel.

### 3.4.3 Audio-Eingangsbuchse

Die zweite Audiobuchse am SIDBlaster ist ein Audioeingang und ist ebenfalls ein unsymmetrischer Anschluss. Wenn Sie sich nicht sicher sind, welcher Anschluss welcher ist, sind die Anschlüsse auf der Platine markiert. Seien Sie vorsichtig, was Sie an den Eingang des SID-Chips anschließen. Diese Chips sind alt und sehr empfindlich gegenüber Spannungsspitzen und zu hohen Spannungen.

## 3.5 exSIDBlaster (optionaler Firmware-Hack)

Wer möchte, kann seinen SIDBlaster mit einem kleinen Hardware-Eingriff und neuer Firmware in einen Teil eines exSID umwandeln – ein Projekt von Thibaut Varène mit besonders ausgereifter, zyklengenauer Firmware. Nach der Umwandlung steht effektiv ein exSID mit einem SID-Chip zur Verfügung, nutzbar z.B. mit JSIDPlay2 (dort „Fake Stereo" aktivieren).

Dies ist ein experimenteller Umbau auf eigene Verantwortung – weder Andreas Schumm noch Thibaut Varène übernehmen dafür Haftung. Details zur genauen Durchführung (PIC-Pin 7 durchtrennen, Brücke von PIC-Pin 15 zu SID-Sockel-Pin 5 löten, neue Firmware flashen) finden Sie im exSIDBlaster-Ordner des SIDBlaster-USB-Tic-Tac-Edition-Repositories.

# 4. Software

## 4.1 Der FTDI D2XX Treiber

### 4.1.1 Windows

Der SIDBlaster muss einen digitalen "Handshake" durchführen, wenn er zum ersten Mal über USB angeschlossen wird. Dies erfordert eine Internetverbindung. Der Handshake funktioniert nicht, wenn Ihre Internetverbindung in Windows auf "Getaktete Verbindung" eingestellt ist. Um dieses Problem zu lösen, deaktivieren Sie vorübergehend die "Getaktete Verbindung", warten Sie einen Moment, bis der SIDBlaster den Handshake durchführt, und aktivieren Sie die "Getaktete Verbindung" anschließend wieder.

Die neuesten Windows-Versionen stellen den FTDI-Treiber über die Update-Funktion zur Verfügung. Prüfen Sie also: Einstellungen / Updates / Optionale Updates.

Der SIDBlaster wird von Windows als "USB Serial Converter" erkannt. Er kann (im Gerätemanager) als COMx Gerät mit Ports erkannt werden. Edit: Ihr SIDBlaster muss in diesem Fall mit dem FTDI prog Tool und dem Template aus dem SIDBlaster Projekt korrekt geflasht werden. Die aktuelle hardsid.dll akzeptiert keine falsch geflashten SIDBlaster.

Bei älteren Windows-Versionen kann die Installation eines Treibers von FTDI notwendig sein, erhältlich bei: <http://www.ftdichip.com/Drivers/D2XX.htm>

### 4.1.2 Linux

Downloaden Sie den D2XX Treiber von: <https://ftdichip.com/drivers/d2xx-drivers/>

Bitte installieren Sie FTDI Treiber so, wie es im Kapitel 2 "Installing the D2XX driver" hier erklärt wird: <https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf>

Wenn das Gerät immer noch nicht verwendet werden kann, installieren Sie bitte die in Kapitel '1.1 Overview' beschriebene Abhilfe:

```
$ sudo vi /etc/udev/rules.d/91-sidblaster.rules

ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", RUN+="/bin/sh -c 'rmmod ftdi_sio && rmmod usbserial'"

$ sudo udevadm control --reload-rules && udevadm trigger
```

### 4.1.3 MacOS

Hinweis: Es kann notwendig sein, die Sicherheitsüberwachung in MacOS auszuschalten oder alle Entwickler zu autorisieren.

```
sudo spctl --master-disable
```

Könnte auch funktionieren:
```
sudo xattr -rd com.apple.quarantine / Your software path.app
```

Downloaden und installieren Sie den D2XX Treiber von: <https://ftdichip.com/drivers/d2xx-drivers/>

Folgen Sie den Instruktionen von: <https://ftdichip.com/wp-content/uploads/2020/08/AN_134_FTDI_Drivers_Installation_Guide_for_MAC_OSX-1.pdf>

Downloaden und installieren Sie D2XXHelper von derselben Website.

## 4.2 Die hardsid Library (hardsid.dll)

Ist sozusagen der "Treiber" bzw. die "Middleware" des SIDBlasters. Unter Windows besteht er aus einer umprogrammierten DLL des Hardsid, so dass Software, die für den Hardsid programmiert wurde, mit dem SIDBlaster kompatibel ist. Später wurde die DLL auch auf Linux und Macos portiert. Die DLL wird von Stein Pedersen erstellt und gepflegt. Die Linux/Mac Portierung wurde von Ken Händel erstellt.

### 4.2.1 Windows

- Download von: <https://crazy-midi.de/joomla/index.php/mydownloads>
- Empfehlung: Installieren Sie den Treiber über sein eigenes Installationsprogramm (installiert die DLL systemweit unter Windows\System32). Alternativ können Sie die passende hardsid.dll auch manuell in das Programmverzeichnis eines einzelnen Programms kopieren.
- Die 64-Bit-Version wird von der 64-Bit-Version von Vice sowie von den 64-Bit-Versionen von AIASS benötigt.

### 4.2.2 Linux

- Downloaden Sie von: <https://crazy-midi.de/joomla/index.php/mydownloads>
- Kopieren Sie libhardsid.so in /usr/local/lib/
- Wenden Sie chmod 0755 bei libhardsid.so an
- Kopieren Sie hardsid.hpp to /usr/local/include/

### 4.2.3 MacOS

- Downloaden Sie von: <https://crazy-midi.de/joomla/index.php/mydownloads>
- Kopieren Sie libhardsid.dylib in /usr/local/lib/
- Kopieren Sie hardsid.hpp nach /usr/local/include/

### 4.2.4 SIDBLASTERUSB_WRITEBUFFER

Abhängig von Ihrem System können Musikstücke mit hohen Datenraten (Multispeed-Tunes oder Digitunes) langsamer abgespielt werden, wenn die Latenzzeit des USB-Treibers zu hoch ist. Dies kann behoben werden, indem die Größe des Schreibpuffers des Treibers auf einen größeren Wert gesetzt wird, zum Beispiel. Auf schnellen Rechnern funktioniert das sogar bis auf 0.

#### 4.2.4.1 Windows
```
set SIDBLASTERUSB_WRITEBUFFER_SIZE=8
```

#### 4.2.4.2 Linux/MacOS
```
export SIDBLASTERUSB_WRITEBUFFER_SIZE=8
```

## 4.2.5 Das SIDBlasterTool

Mit dem SIDBlasterTool können Sie überprüfen, ob die Kommunikation zwischen Bibliothek und Gerät funktioniert.

Sie können auch den SID-Typ einstellen und die Seriennummer ändern. Der Typ wird als Teil der Gerätebeschreibung gespeichert und von Anwendungen wie JSIDPlay ausgewertet.

Das SIDBlasterTool ist Teil des Treiber-Pakets: <https://crazy-midi.de/joomla/index.php/mydownloads>

## 4.2.6 Hinweise für Entwickler

Für Entwickler, die Anwendungen unter Verwendung der hardsid-Bibliothek schreiben wollen, verweise ich auf hardsid.hpp sowie auf integrators_guide.md (liegt dem Treiber-Installationsprogramm bei). Der Quellcode des Treibers liegt hier: <https://github.com/gh0stless/SIDBlasterUSB_HardSID-emulation-driver>. Weitere Tipps finden Sie im AIASS-VST Repository im Ordner doc. Vergessen Sie nicht, die destructor-Methode beim Beenden unter MacOS und Linux manuell aufzurufen.

Eine ausführliche Anleitung zur Erstellung der hardsid-Bibliothek finden Sie hier (dank Ken Handel): <https://haendel.ddns.net/~ken/sidblaster.html>

## 4.3 Anwendungen

### 4.3.1 Vice64

Der berühmte C64-Emulator unterstützt bis zu 3 SIDBlaster. Im einfachsten Fall haben Sie nun einen C64 mit Originalsound. Sie können den SIDBlaster aber auch mit Vice64 als MIDI-Expander verwenden, indem Sie die MIDI-Emulation aktivieren und ein Synthesizer-Programm wie Station64 laden. (Nur für Windows).

### 4.3.2 ACID64 Player

Beste SIDBlaster-Unterstützung. Wenn Sie mehrere Geräte besitzen, können Sie sogar Stereo- und 3SID-Musik abspielen. Nur Windows. Wieder SIDBlaster-Unterstützung ab Version 4.3. Seit dieser Version spricht ACID64 den SIDBlaster direkt an – die hardsid-Bibliothek wird nicht mehr benötigt.

### 4.3.3 SidPlay2

Guter SID-Player, durch Wiedergabelisten auch als Jukebox geeignet. Nur für Windows.

### 4.3.4 GoatTracker

Tracker. Unterstützt einen oder zwei SIDBlaster. Nur für Windows.

### 4.3.5 JSIDPlay2

JSIDPlay2 ist ein fantastischer C64 Content Media Player. In der aktuellen Version bietet es die beste SIDBlaster-Unterstützung. Er ist vielleicht nicht ganz so handlich wie ACID64, aber wenn man sich einmal damit vertraut gemacht hat, ist es ein sehr leistungsfähiges und umfassendes Programm. Erhältlich für Windows, MacOS und Linux.

JSIDPlay2 unterstützt den SIDBlaster von Haus aus, lediglich Java ist erforderlich. Es ist somit nicht notwendig, D2XX-Treiber oder die hardsid-Bibliothek zu installieren.

#### 4.3.5.1 JSIDPlay2 Android app

Verwandeln Sie mit dieser App ihr Mobiltelefon in einen mobilen SIDPlayer. Verbinden Sie Ihren SIDBlaster mit Ihrem Mobiltelefon. Sie benötigen einen OTG-USB-Adapter und wahrscheinlich einen Mono-Stereo-Adapter für die Kopfhörer.

### 4.3.6 CloanTo's C64 Forever

Ebenfalls kompatibel. Kopieren Sie einfach die hardsid.dll in den Ordner von Vice (Help/About/VICE Plugin, Open File Location).

Dann geben Sie "-sidenginemodel hardsid" in der Option "custom parameters" (in Tools/Options/Emulation, in Plugins/VICE/Custom parameters...) ein.

### 4.3.7 AIASS

AIASS Is A SID Synthesizer

#### 4.3.7.1 sid-object for Max/MSP & Max4Live

Ist ein Max/MSP C-external für den SIDBlaster-USB.

#### 4.3.7.2 AIASS – Max4Live Gerät(e)

Der AIASS for MAX4LIVE ist ein Set von max4live Geräten, um einen oder mehrere SIDBlaster-USB als Synthesizer-Anwendung zu steuern. Dies macht es möglich, Musik mit einem originalen SID-Chip MOS 6581 oder MOS 8580 aus dem Commodore 64, in einer modernen DAW zu produzieren.

#### 4.3.7.3 AIASS – VST

AIASS als VST (auf allen Systemen)

### 4.3.8 ASID Protokoll Player

Kleines Programm mit dem man ASID MIDI Datenströme auf dem SIDBlaster wiedergeben kann. Unterstützt bis zu 3 SIDBlaster für 2SID und 3SID Tunes. Damit können sie zum Beispiel den Payer der Webseite DeepSID nutzen.

# 5. Betrieb mehrerer Geräte

Sie können mehrere SIDBlaster-USB gleichzeitig verwenden.

Dies wird von ACID Player, JSIDPlay2, Vice, Goattracker und AIASS unterstützt.

Der Vorteil liegt in der Wiedergabe von Stereo- oder 3Sid-Tunes, Fake-Stereo, automatischer Auswahl des richtigen SID-Typs mit JSIPlay2 (Typ mit dem SIDBlastertool einstellen), oder mehrerer Instrumente mit AIASS (noch nicht beim VST).

# 6. Links

- <http://crazy-midi.de/>
- <https://github.com/gh0stless/SIDBlaster-USB-Tic-Tac-Edition>
- <https://github.com/gh0stless/SIDBlasterTool>
- <https://github.com/gh0stless/SIDBlaster-ASID-Player>
- <https://github.com/gh0stless/AIASS-for-MAX4LIVE>
- <https://github.com/gh0stless/AIASS-Uno-VST>
- <https://github.com/gh0stless/SIDBlasterUSB_HardSID-emulation-driver>
- <https://vice-emu.sourceforge.io/>
- <https://www.acid64.com>
- <http://www.gsldata.se/c64/spw/> (SIDPlay2)
- <https://haendel.ddns.net/~ken/> (JSIDPlay2)
- <https://sourceforge.net/projects/goattracker2/>
- <https://www.facebook.com/groups/2305052182957954/>

# 7. Danksagungen

- Davey (The Phantom) für die Erstellung des ursprünglichen SIDBlasters.
- Stein Pedersen für die Unterstützung und die sidblaster.dll.
- Wilfred Bos für seinen ACID Player und Tipps und Hilfe.
- Ken Händel für den POSIX Port und seine Arbeit an der Bibliothek
- Karl-Werner Riedel für seine Hilfe bei der Entwicklung der Tic Tac Hardware.
- Magnus Hansson für das Schreiben einiger Originalteile dieses Handbuchs.
- Yvonne Hölzel für Revision und Übersetzungen.
- Ein spezieller Dank an Thibaut VARÈNE, für seinen exSID und seine tolle Firmware, die mit einem Hack auch mit dem SIDBlaster verwendet werden kann.
- Borjana Konstantinowa für ihre Geduld mit mir.

Coswig, Sachsen 04/08/26
