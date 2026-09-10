/*  Lud-WS  Router  ;  Pi Pico 2  rp2350
 */



#include <MIDI.h>
#define ID_DISPLAY  'D' 
#define ID_SYNTH_A1 'a'
#define ID_SYNTH_A2 'b'
#define ID_SYNTH_A3 'c'
#define ID_SYNTH_B  'B'
#define ID_ROUTER   'R' // this one
#define ID_CTRL     'C'
#define ID_MOD      'M'
#define ID_TEENSY   'T'
#define ID_POWER    'P'

// ========================== COMMAND CODES ==========================
#define CMD_PING 'p'   // Ping request p (minuscolo)
#define CMD_PONG 'P'   // Ping response P (maiuscolo)

// UART0: pin GP0 (TX) e GP1 (RX)  MIDI ext
// UART1: pin GP8 (TX) e GP9 (RX) Display
// Nota: Serial1 e Serial2 sono già definite dal core

// --- 6 Porte SerialPIO (Software) ---
// Ogni porta richiede un pin TX e uno RX
SerialPIO SerialSynthA(2, 3);   //  Lud-WS-SynthA_M
SerialPIO SerialSynthB(4, 5);   // Lud-WS-SynthB
SerialPIO SerialCtrl(6, 7);   // Lud-WS-Ctrl
SerialPIO SerialMod(10, 11); // Lud-WS-Mod
SerialPIO SerialTeensy(12, 13); // Lud-WS-Teensy
SerialPIO SerialPower(14, 15); // Lud-WS-Power

  MIDI_CREATE_INSTANCE(HardwareSerial, Serial1,  MIDI);
 byte midi_SynthA_CH = 1;
  byte midi_SynthB_CH = 2;
  void sendCC_A(byte number, byte value) {
	SerialSynthA.write('m'); //
   SerialSynthA.write('c'); //
 SerialSynthA.write(number);
  SerialSynthA.write(value);
  	 SerialSynthA.write('&');
	 SerialSynthA.write('!');
}
void sendCC_B(byte number, byte value) {
	SerialSynthB.write('m'); //
   SerialSynthB.write('c'); //
 SerialSynthB.write(number);
  SerialSynthB.write(value);
  	 SerialSynthB.write('&');
	 SerialSynthB.write('!');
}
   void sendBenderA( int bend){ // 
   SerialSynthA.write('m'); //
   SerialSynthA.write('b'); //
 SerialSynthA.write((uint8_t*)&bend, 4);
  	 SerialSynthA.write('&');
	 SerialSynthA.write('!');
  }
   void sendBenderB( int bend){ // 
   SerialSynthB.write('m'); //
   SerialSynthB.write('b'); //
 SerialSynthB.write((uint8_t*)&bend, 4);
  	 SerialSynthB.write('&');
	 SerialSynthB.write('!');
  }
  void sendNoteOnOffA(byte status, byte pitch, byte velocity){ // 
   SerialSynthA.write('m'); 
   SerialSynthA.write(status); //0 = noteOff, 1 = noteOn
    SerialSynthA.write(pitch);
   SerialSynthA.write(pitch);
  SerialSynthA.write(velocity);
  	 SerialSynthA.write('&');
	 SerialSynthA.write('!');
  }
  void sendNoteOnOffB(byte status, byte pitch, byte velocity){ // 
   SerialSynthB.write('m'); 
   SerialSynthB.write(status); //0 = noteOff, 1 = noteOn
    SerialSynthB.write(pitch);
   SerialSynthB.write(pitch);
  SerialSynthB.write(velocity);
  	 SerialSynthB.write('&');
	 SerialSynthB.write('!');
  }
   void handleNoteOn(byte channel, byte pitch, byte velocity)
{
	if(channel == midi_SynthA_CH){
		sendNoteOnOffA(1,pitch,velocity);
	}
	else if(channel == midi_SynthB_CH){
		sendNoteOnOffB(1,pitch,velocity);
	}
}
 void handleNoteOff(byte channel, byte pitch, byte velocity)
{
	if(channel == midi_SynthA_CH){
		sendNoteOnOffA(0,pitch,velocity);
	}
	else if(channel == midi_SynthB_CH){
		sendNoteOnOffB(0,pitch,velocity);
	}
}
void handlePitchBend(byte channel, int bend) {
		if(channel == midi_SynthA_CH){
		sendBenderA(bend);
	}
	else if(channel == midi_SynthB_CH){
		sendBenderB(bend);
	}
}
void handleControlChange(byte channel, byte number, byte value) {
		if(channel == midi_SynthA_CH){
		sendCC_A(number, value);
	}
	else if(channel == midi_SynthB_CH){
			sendCC_B(number, value);
	}
}
void setup() {	
	 Serial2.begin(115200); //Display
    MIDI.begin(MIDI_CHANNEL_OMNI);
	  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.setHandlePitchBend(handlePitchBend);
	SerialSynthA.begin(115200); 
	SerialSynthB.begin(115200); 
	SerialCtrl.begin(115200); 
	SerialMod.begin(115200);
	SerialTeensy.begin(115200); 
	SerialPower.begin(115200); 
}

void loop() {
	MIDI.read();
if(Serial2.available()){ //display in 
	char p = Serial2.read();	
	if(p == CMD_PING){ // ping 
		char m = Serial2.read();	
		switch(m){
			case ID_POWER:
			SerialPower.write(CMD_PING);
			SerialPower.write('!');
			SerialPower.write('&');
			break;
			case ID_SYNTH_A1:
			SerialSynthA.write(CMD_PING);
			SerialSynthA.write('a');
			SerialSynthA.write('!');
			SerialSynthA.write('&');
			break;
				case ID_SYNTH_A2:
			SerialSynthA.write(CMD_PING);
			SerialSynthA.write('b');
			SerialSynthA.write('!');
			SerialSynthA.write('&');
			break;
				case ID_SYNTH_A3:
			SerialSynthA.write(CMD_PING);
			SerialSynthA.write('c');
			SerialSynthA.write('!');
			SerialSynthA.write('&');
			break;
			case ID_SYNTH_B:
			SerialSynthB.write(CMD_PING);
			SerialSynthB.write('!');
			SerialSynthB.write('&');
			break;
			case ID_ROUTER:
			Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_ROUTER);
			Serial2.write('!');
			Serial2.write('&');
			break;
			case ID_TEENSY:
			SerialTeensy.write(CMD_PING);
			SerialTeensy.write('!');
			SerialTeensy.write('&');
			break;
			case ID_MOD:
			SerialMod.write(CMD_PING);
			SerialMod.write('!');
			SerialMod.write('&');
			break;
			case ID_CTRL:
			SerialCtrl.write(CMD_PING);
			SerialCtrl.write('!');
			SerialCtrl.write('&');
			break;
		}
	}
	else if(p == ID_CTRL){ // 
	SerialCtrl.write(Serial2.read());
	}
	else if(p == ID_MOD){ // 
	SerialMod.write(Serial2.read());
	}
	else if(p == ID_TEENSY){ // 
	SerialTeensy.write(Serial2.read());
	}
	else if(p == ID_SYNTH_A1){ // 
	SerialSynthA.write(Serial2.read());
	}
	else if(p == ID_SYNTH_B){ // 
	SerialSynthB.write(Serial2.read());
	}
	else if(p == ID_POWER){ // 
	SerialPower.write(Serial2.read());
	}
	}	

if(SerialSynthA.available()){
Serial.println("receve SerialSynthA ");
	char p = SerialSynthA.read();	
	if(p == CMD_PING){
		 p = SerialSynthA.read();	
		 if(p == 'a'){
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_SYNTH_A1);
			Serial2.write('!');
			Serial2.write('&');
		 }
		 else if(p == 'b'){
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_SYNTH_A2);
			Serial2.write('!');
			Serial2.write('&');
		 }
		  else if(p == 'c'){
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_SYNTH_A3);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
}
if(SerialSynthB.available()){
Serial.println("receve SerialSynthB");
	char p = SerialSynthB.read();	
	if(p == CMD_PING){
		 p = SerialSynthB.read();	
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_SYNTH_B);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
 if(SerialCtrl.available()){
Serial.println("receve SerialCtrl");
	char p = SerialCtrl.read();	
	if(p == CMD_PING){
		 p = SerialCtrl.read();	
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_CTRL);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
 if(SerialMod.available()){
Serial.println("receve SerialMod");
	char p = SerialMod.read();	
	if(p == CMD_PING){
		 p = SerialMod.read();	
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_MOD);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
	if(SerialTeensy.available()){
Serial.println("receve SerialTeensy");
	char p = SerialTeensy.read();	
	if(p == CMD_PING){
		 p = SerialTeensy.read();	
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_TEENSY);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
		if(SerialPower.available()){
Serial.println("receve SerialPower");
	char p = SerialPower.read();	
	if(p == CMD_PING){
		 p = SerialPower.read();	
		Serial2.write(ID_DISPLAY);
			Serial2.write(CMD_PONG);
			Serial2.write(ID_POWER);
			Serial2.write('!');
			Serial2.write('&');
		 }
	}
}
