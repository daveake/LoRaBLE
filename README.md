This is generic firmware to make various devices into HAB LoRa receivers.  Currently hash settings for:

- TTGO T-Beam
- TTGO LoRa OLED V1
- TTGO LoRa OLED V2
- Uputronics LoRaGo

The receiver can be connected to a phone or tablet or PC, using Bluetooth or BLE (Bluetooth Low Power) or USB Serial so in order to view the received telemetry.

I have written several programs suitable for decoding this telemetry, uploading to Habhub/HAB.Link, and providing a UI with mapping etc., e.g.:

- HAB Base
- HAB Explora
- HAB PADD
- HAB LCARS

See my github account and blog for details about this host software for Windows, Mac, Raspberry Pi, iPad, iPhone, Android phone or tablet.


Libraries
=========

To build LoraBluetooth you may need various libraries into your Arduino IDE.  For boards with an OLED:

	- Adafruit GFX Library
	- Adafruit SSD1306 Library

These Libraries can be added via the Ardunio IDE menu: Sketch -> Include Library -> Manage Libraries…

For boards that use the AXP202X power chip, ie the TTGO V1+ T-Beam, the AXP202X Library is required:

https://github.com/lewisxhe/AXP202X_Library

This library can be added via the Ardunio IDE menu: Sketch -> Include Library -> Add .zip Library…


Serial Protocol
===============

The same protocol is used over USB, Bluetooth and BLE.  It accepts commands of the form

something=value<CR><LF>

The somethings are, currently:

	- CurrentRSSI=<RSSI>
	- Message=<telemetry>
	- Hex=<hex packet e.g. SSDV>
	- FreqErr=<error in kHz>
	- PacketRSSI=<RSSI>
	- PacketSNR=<SNR>

The serial interface accepts a few commands, each of the form

~<command><value><CR>

(a trailing <LF> can be sent but is ignored).  Accepted commands are responded to with an OK (* <CR> <LF>) and rejected commands (unknown command, or invalid command value) with a WTF (? <CR> <LF>)

The current commands are:

	- F<frequency in MHz>
	- M<mode>
	- B<bandwidth>
	- E<error coding from 5 to 8>
	- S<spreading factor from 6 to 11>
	- I<1=implicit mode, 0=explicit mode>
	- L(low data rate optimisation: 1=enable, 0=disable)
	
The supported modes are:

0 = (normal for telemetry)  Explicit mode, Error coding 4:8, Bandwidth 20.8kHz, SF 11, Low data rate optimize on
1 = (normal for SSDV)       Implicit mode, Error coding 4:5, Bandwidth 20.8kHz,  SF 6, Low data rate optimize off
2 = (normal for repeater)   Explicit mode, Error coding 4:8, Bandwidth 62.5kHz,  SF 8, Low data rate optimize off	

Bandwidth value strings can be 7K8, 10K4, 15K6, 20K8, 31K25, 41K7, 62K5, 125K, 250K, 500K

History
=======

23/09/2016	V1.1	- Added Hex=... message for any packet that is not ASCII telemetry
					- Added LoRa modes 3-7K8
					- Increased baud rate to 57,600 (from 9,600) so handle high LoRa data rates esp. with SSDV
					
30/06/2016	V1.0	- First version
