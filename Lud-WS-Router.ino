/*  Lud-WS Router ; Pi Pico 2 rp2350
 *  Dual mode:
 *   - Legacy protocol (storico)
 *   - LWSv1 [SENDER][CMD][LEN][PAYLOAD]&!
 */

#include <Arduino.h>
#include <MIDI.h>
#include "serial_protocol.h"   // NUOVO: condiviso con Display

#define ID_DISPLAY  'D'
#define ID_SYNTH_A1 'a'
#define ID_SYNTH_A2 'b'
#define ID_SYNTH_A3 'c'
#define ID_SYNTH_B  'B'
#define ID_ROUTER   'R'
#define ID_CTRL     'C'
#define ID_MOD      'M'
#define ID_TEENSY   'T'
#define ID_POWER    'P'

// ========================== COMMAND CODES ==========================
#define CMD_PING 'p'
#define CMD_PONG 'P'

// ========================== FEATURE FLAGS ==========================
#define ENABLE_LEGACY_PROTO 1
#define ENABLE_LWS_V1       1

// UART mapping nel tuo progetto:
// Serial1 -> MIDI (MIDI lib)
// Serial2 -> Display
#define DISPLAY_PORT Serial2

// --- 6 Porte SerialPIO ---
SerialPIO SerialSynthA(2, 3);     // Lud-WS-SynthA_M
SerialPIO SerialSynthB(4, 5);     // Lud-WS-SynthB
SerialPIO SerialCtrl(6, 7);       // Lud-WS-Ctrl
SerialPIO SerialMod(10, 11);      // Lud-WS-Mod
SerialPIO SerialTeensy(12, 13);   // Lud-WS-Teensy
SerialPIO SerialPower(14, 15);    // Lud-WS-Power

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

byte midi_SynthA_CH = 1;
byte midi_SynthB_CH = 2;

// LWS state
#if ENABLE_LWS_V1
static LwsFrame rxDisplayLws;
#endif

// ========================== HELPERS ==========================
inline void writeEnd(Stream& s) {
  s.write('&');
  s.write('!');
}

// -------- Legacy TX to Display --------
void sendPongToDisplayLegacy(char who) {
  DISPLAY_PORT.write(ID_DISPLAY);
  DISPLAY_PORT.write(CMD_PONG);
  DISPLAY_PORT.write(who);
  writeEnd(DISPLAY_PORT);
}

// -------- LWS TX to Display --------
#if ENABLE_LWS_V1
void sendPongToDisplayLws(char who) {
  uint8_t payload[1] = {(uint8_t)who}; // opzionale: chi ha risposto
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, CMD_PONG, payload, 1);
}
void sendPingToDisplayLws() {
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, CMD_PING, nullptr, 0);
}
void sendStatusToDisplayLws(const char* txt) {
  uint8_t len = (uint8_t)min((int)strlen(txt), 250);
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, 's', (const uint8_t*)txt, len);
}
void sendErrorToDisplayLws(const char* txt) {
  uint8_t len = (uint8_t)min((int)strlen(txt), 250);
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, 'e', (const uint8_t*)txt, len);
}
void sendCCToDisplayLws(uint8_t cc, uint8_t val) {
  uint8_t p[2] = {cc, val};
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, 'c', p, 2);
}
void sendNoteToDisplayLws(uint8_t onoff, uint8_t pitch, uint8_t vel) {
  uint8_t p[3] = {onoff, pitch, vel};
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, 'n', p, 3);
}
void sendBendToDisplayLws(int32_t bend) {
  uint8_t p[4];
  lws_pack_i32_le(bend, p);
  lws_send_frame(DISPLAY_PORT, ID_ROUTER, 'b', p, 4);
}
#endif

// ========================== MIDI -> SYNTH (legacy invariato) ==========================
void sendCC_A(byte number, byte value) {
  SerialSynthA.write('m');
  SerialSynthA.write('c');
  SerialSynthA.write(number);
  SerialSynthA.write(value);
  writeEnd(SerialSynthA);

#if ENABLE_LWS_V1
  sendCCToDisplayLws(number, value);
#endif
}

void sendCC_B(byte number, byte value) {
  SerialSynthB.write('m');
  SerialSynthB.write('c');
  SerialSynthB.write(number);
  SerialSynthB.write(value);
  writeEnd(SerialSynthB);

#if ENABLE_LWS_V1
  sendCCToDisplayLws(number, value);
#endif
}

void sendBenderA(int bend) {
  SerialSynthA.write('m');
  SerialSynthA.write('b');
  SerialSynthA.write((uint8_t*)&bend, 4);
  writeEnd(SerialSynthA);

#if ENABLE_LWS_V1
  sendBendToDisplayLws((int32_t)bend);
#endif
}

void sendBenderB(int bend) {
  SerialSynthB.write('m');
  SerialSynthB.write('b');
  SerialSynthB.write((uint8_t*)&bend, 4);
  writeEnd(SerialSynthB);

#if ENABLE_LWS_V1
  sendBendToDisplayLws((int32_t)bend);
#endif
}

void sendNoteOnOffA(byte status, byte pitch, byte velocity) {
  SerialSynthA.write('m');
  SerialSynthA.write(status);   // 0 noteOff, 1 noteOn
  SerialSynthA.write(pitch);
  SerialSynthA.write(velocity); // FIX bug
  writeEnd(SerialSynthA);

#if ENABLE_LWS_V1
  sendNoteToDisplayLws(status, pitch, velocity);
#endif
}

void sendNoteOnOffB(byte status, byte pitch, byte velocity) {
  SerialSynthB.write('m');
  SerialSynthB.write(status);   // 0 noteOff, 1 noteOn
  SerialSynthB.write(pitch);
  SerialSynthB.write(velocity); // FIX bug
  writeEnd(SerialSynthB);

#if ENABLE_LWS_V1
  sendNoteToDisplayLws(status, pitch, velocity);
#endif
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH) sendNoteOnOffA(1, pitch, velocity);
  else if (channel == midi_SynthB_CH) sendNoteOnOffB(1, pitch, velocity);
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH) sendNoteOnOffA(0, pitch, velocity);
  else if (channel == midi_SynthB_CH) sendNoteOnOffB(0, pitch, velocity);
}
void handlePitchBend(byte channel, int bend) {
  if (channel == midi_SynthA_CH) sendBenderA(bend);
  else if (channel == midi_SynthB_CH) sendBenderB(bend);
}
void handleControlChange(byte channel, byte number, byte value) {
  if (channel == midi_SynthA_CH) sendCC_A(number, value);
  else if (channel == midi_SynthB_CH) sendCC_B(number, value);
}

// ========================== Legacy routing ==========================
void forwardPingToNodeLegacy(char nodeId) {
  switch (nodeId) {
    case ID_POWER:
      SerialPower.write(CMD_PING); SerialPower.write(ID_POWER); writeEnd(SerialPower); break;
    case ID_SYNTH_A1:
      SerialSynthA.write(CMD_PING); SerialSynthA.write('a'); writeEnd(SerialSynthA); break;
    case ID_SYNTH_A2:
      SerialSynthA.write(CMD_PING); SerialSynthA.write('b'); writeEnd(SerialSynthA); break;
    case ID_SYNTH_A3:
      SerialSynthA.write(CMD_PING); SerialSynthA.write('c'); writeEnd(SerialSynthA); break;
    case ID_SYNTH_B:
      SerialSynthB.write(CMD_PING); SerialSynthB.write(ID_SYNTH_B); writeEnd(SerialSynthB); break;
    case ID_ROUTER:
      sendPongToDisplayLegacy(ID_ROUTER);
#if ENABLE_LWS_V1
      sendPongToDisplayLws(ID_ROUTER);
#endif
      break;
    case ID_TEENSY:
      SerialTeensy.write(CMD_PING); SerialTeensy.write(ID_TEENSY); writeEnd(SerialTeensy); break;
    case ID_MOD:
      SerialMod.write(CMD_PING); SerialMod.write(ID_MOD); writeEnd(SerialMod); break;
    case ID_CTRL:
      SerialCtrl.write(CMD_PING); SerialCtrl.write(ID_CTRL); writeEnd(SerialCtrl); break;
    default: break;
  }
}

void handleDisplayInputLegacyByte(uint8_t b) {
#if ENABLE_LEGACY_PROTO
  char p = (char)b;
  if (p == CMD_PING) {
    if (DISPLAY_PORT.available() > 0) {
      char m = (char)DISPLAY_PORT.read();
      forwardPingToNodeLegacy(m);
    }
  } else if (p == ID_CTRL) {
    if (DISPLAY_PORT.available() > 0) SerialCtrl.write(DISPLAY_PORT.read());
  } else if (p == ID_MOD) {
    if (DISPLAY_PORT.available() > 0) SerialMod.write(DISPLAY_PORT.read());
  } else if (p == ID_TEENSY) {
    if (DISPLAY_PORT.available() > 0) SerialTeensy.write(DISPLAY_PORT.read());
  } else if (p == ID_SYNTH_A1) {
    if (DISPLAY_PORT.available() > 0) SerialSynthA.write(DISPLAY_PORT.read());
  } else if (p == ID_SYNTH_B) {
    if (DISPLAY_PORT.available() > 0) SerialSynthB.write(DISPLAY_PORT.read());
  } else if (p == ID_POWER) {
    if (DISPLAY_PORT.available() > 0) SerialPower.write(DISPLAY_PORT.read());
  }
#endif
}

// ========================== LWS RX from Display ==========================
#if ENABLE_LWS_V1
void handleDisplayFrameLws(const LwsFrame& f) {
  const char cmd = (char)f.cmd;

  switch (cmd) {
    case CMD_PING: {
      // rispondi pong (con who=R)
      sendPongToDisplayLws(ID_ROUTER);
#if ENABLE_LEGACY_PROTO
      sendPongToDisplayLegacy(ID_ROUTER);
#endif
      break;
    }
    case 'c': { // opzionale: comando verso router
      // payload [cc,val]
      if (f.len >= 2) {
        // se vuoi applicare qualcosa nel router, fallo qui
      }
      break;
    }
    case 'n':
    case 'b':
    case 's':
    case 'e':
    case 'a':
    case CMD_PONG:
    default:
      break;
  }
}
#endif

void pollDisplayPort() {
  while (DISPLAY_PORT.available() > 0) {
    uint8_t b = (uint8_t)DISPLAY_PORT.read();

#if ENABLE_LWS_V1
    if (lws_parse_byte(b, rxDisplayLws)) {
      handleDisplayFrameLws(rxDisplayLws);
      continue;
    }
#endif

    // fallback legacy sul singolo byte
    handleDisplayInputLegacyByte(b);
  }
}

// ========================== NODES -> Display pong ==========================
void handleNodePong(Stream& node, char nodeId, const char* dbgName, bool synthAHasSubId = false) {
  if (!node.available()) return;

  char p = (char)node.read();
  if (p != CMD_PING) return;

  char who = nodeId;
  if (synthAHasSubId) {
    if (node.available() > 0) {
      char sub = (char)node.read();
      if (sub == 'a' || sub == 'b' || sub == 'c') who = sub;
    }
  } else {
    if (node.available() > 0) (void)node.read(); // consuma eventuale id
  }

#if ENABLE_LEGACY_PROTO
  sendPongToDisplayLegacy(who);
#endif
#if ENABLE_LWS_V1
  sendPongToDisplayLws(who);
#endif
}

// ========================== SETUP / LOOP ==========================
void setup() {
  Serial.begin(115200);       // debug USB
  DISPLAY_PORT.begin(115200); // Display UART

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

#if ENABLE_LWS_V1
  sendStatusToDisplayLws("Router boot");
#endif
}

void loop() {
  MIDI.read();

  // input da Display (LWS + legacy)
  pollDisplayPort();

  // input dai nodi
  handleNodePong(SerialSynthA, ID_SYNTH_A1, "SerialSynthA", true);
  handleNodePong(SerialSynthB, ID_SYNTH_B,  "SerialSynthB");
  handleNodePong(SerialCtrl,   ID_CTRL,     "SerialCtrl");
  handleNodePong(SerialMod,    ID_MOD,      "SerialMod");
  handleNodePong(SerialTeensy, ID_TEENSY,   "SerialTeensy");
  handleNodePong(SerialPower,  ID_POWER,    "SerialPower");

#if ENABLE_LWS_V1
  // heartbeat opzionale router->display
  static uint32_t tPing = 0;
  if (millis() - tPing >= 1000) {
    tPing = millis();
    sendPingToDisplayLws();
  }
#endif
}