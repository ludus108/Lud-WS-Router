/*  Lud-WS Router ; Pi Pico 2 rp2350
 *  Bridge tra Display (LWSv1.1) e nodi (protocollo legacy).
 *
 *  LWSv1.1 frame format (lato Display):
 *      [SENDER][SEQ][CMD][LEN][PAYLOAD...][CRC8][&][!]
 *
 *  Il Router:
 *    - Riceve PING dal Display e li instrada ai nodi (legacy)
 *    - Traduce le risposte dei nodi in PONG LWSv1.1 verso il Display
 *    - Risponde ai PING indirizzati a se stesso ('R')
 *    - Fa da ponte MIDI -> eventi LWS verso il Display
 *
 *  Il protocollo verso i nodi resta legacy (byte-stream con '&!').
 */

#include <Arduino.h>
#include <MIDI.h>
#include "serial_protocol.h"   // condiviso con Display, versione LWSv1.1

// ========================== MCU IDS ==========================
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
#define CMD_PING       'p'
#define CMD_PONG       'P'
#define CMD_PARAM      'S'
#define CMD_PARAM_REL  'R'
#define CMD_PARAM_ACK  'A'

// ========================== FEATURE FLAGS ==========================
#define ENABLE_LEGACY_PROTO 1
#define ENABLE_LWS_V1       1

// UART mapping:
//   Serial1 -> MIDI
//   Serial2 -> Display
#define DISPLAY_PORT Serial2

// --- 6 porte SerialPIO verso i nodi ---
SerialPIO SerialSynthA(2, 3);
SerialPIO SerialSynthB(4, 5);
SerialPIO SerialCtrl(6, 7);
SerialPIO SerialMod(10, 11);
SerialPIO SerialTeensy(12, 13);
SerialPIO SerialPower(14, 15);

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

byte midi_SynthA_CH = 1;
byte midi_SynthB_CH = 2;

// ========================== LWS STATE ==========================
#if ENABLE_LWS_V1
static LwsParser lwsParserDisplay;     // parser verso il Display
static uint8_t   lwsTxSeq = 0;         // SEQ rolling per i frame TX

static inline uint8_t nextSeq() { return lwsTxSeq++; }

// Wrapper: invia un frame LWS al Display con il prossimo SEQ.
// 'sender' puo' essere ID_ROUTER per i messaggi del Router stesso,
// oppure l'ID del nodo quando si spoofano le risposte (bridge trasparente).
static inline void txLws(char sender, uint8_t cmd,
                         const uint8_t* p = nullptr, uint8_t len = 0) {
  lws_send_frame(DISPLAY_PORT, (uint8_t)sender, nextSeq(), cmd, p, len);
}
#endif

// ========================== HELPERS LEGACY ==========================
inline void writeEnd(Stream& s) {
  s.write('&');
  s.write('!');
}

void sendPongToDisplayLegacy(char who) {
  DISPLAY_PORT.write(ID_DISPLAY);
  DISPLAY_PORT.write(CMD_PONG);
  DISPLAY_PORT.write(who);
  writeEnd(DISPLAY_PORT);
}

// ========================== LWS TX VERSO IL DISPLAY ==========================
#if ENABLE_LWS_V1
void sendPongToDisplayLws(char who) {
  // 'who' = ID del nodo che ha risposto (spoofing del SENDER)
  txLws(who, CMD_PONG, nullptr, 0);
}

void sendPingToDisplayLws() {
  txLws(ID_ROUTER, CMD_PING, nullptr, 0);
}

void sendStatusToDisplayLws(const char* txt) {
  uint8_t len = (uint8_t)min((int)strlen(txt), 250);
  txLws(ID_ROUTER, 's', (const uint8_t*)txt, len);
}

void sendErrorToDisplayLws(const char* txt) {
  uint8_t len = (uint8_t)min((int)strlen(txt), 250);
  txLws(ID_ROUTER, 'e', (const uint8_t*)txt, len);
}

void sendCCToDisplayLws(uint8_t cc, uint8_t val) {
  uint8_t p[2] = {cc, val};
  txLws(ID_ROUTER, 'c', p, 2);
}

void sendNoteToDisplayLws(uint8_t onoff, uint8_t pitch, uint8_t vel) {
  uint8_t p[3] = {onoff, pitch, vel};
  txLws(ID_ROUTER, 'n', p, 3);
}

void sendBendToDisplayLws(int32_t bend) {
  uint8_t p[4];
  lws_pack_i32_le(bend, p);
  txLws(ID_ROUTER, 'b', p, 4);
}

// ACK per un CMD_PARAM_REL ricevuto dal Display
void sendParamAckToDisplayLws(uint8_t ackedSeq, uint8_t ackedCmd) {
  uint8_t p[2] = { ackedSeq, ackedCmd };
  txLws(ID_ROUTER, CMD_PARAM_ACK, p, 2);
}
#endif

// ========================== MIDI -> SYNTH (legacy + telemetria LWS) ==========================
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
  SerialSynthA.write(status);
  SerialSynthA.write(pitch);
  SerialSynthA.write(velocity);
  writeEnd(SerialSynthA);
#if ENABLE_LWS_V1
  sendNoteToDisplayLws(status, pitch, velocity);
#endif
}

void sendNoteOnOffB(byte status, byte pitch, byte velocity) {
  SerialSynthB.write('m');
  SerialSynthB.write(status);
  SerialSynthB.write(pitch);
  SerialSynthB.write(velocity);
  writeEnd(SerialSynthB);
#if ENABLE_LWS_V1
  sendNoteToDisplayLws(status, pitch, velocity);
#endif
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH)      sendNoteOnOffA(1, pitch, velocity);
  else if (channel == midi_SynthB_CH) sendNoteOnOffB(1, pitch, velocity);
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH)      sendNoteOnOffA(0, pitch, velocity);
  else if (channel == midi_SynthB_CH) sendNoteOnOffB(0, pitch, velocity);
}
void handlePitchBend(byte channel, int bend) {
  if (channel == midi_SynthA_CH)      sendBenderA(bend);
  else if (channel == midi_SynthB_CH) sendBenderB(bend);
}
void handleControlChange(byte channel, byte number, byte value) {
  if (channel == midi_SynthA_CH)      sendCC_A(number, value);
  else if (channel == midi_SynthB_CH) sendCC_B(number, value);
}

// ========================== LEGACY ROUTING (verso i nodi) ==========================
void forwardPingToNodeLegacy(char nodeId) {
  switch (nodeId) {
    case ID_POWER:
      SerialPower.write(CMD_PING);    SerialPower.write(ID_POWER);    writeEnd(SerialPower);    break;
    case ID_SYNTH_A1:
      SerialSynthA.write(CMD_PING);   SerialSynthA.write('a');        writeEnd(SerialSynthA);   break;
    case ID_SYNTH_A2:
      SerialSynthA.write(CMD_PING);   SerialSynthA.write('b');        writeEnd(SerialSynthA);   break;
    case ID_SYNTH_A3:
      SerialSynthA.write(CMD_PING);   SerialSynthA.write('c');        writeEnd(SerialSynthA);   break;
    case ID_SYNTH_B:
      SerialSynthB.write(CMD_PING);   SerialSynthB.write(ID_SYNTH_B); writeEnd(SerialSynthB);   break;
    case ID_ROUTER:
      // Risposta diretta: e' il Router stesso
      sendPongToDisplayLegacy(ID_ROUTER);
#if ENABLE_LWS_V1
      sendPongToDisplayLws(ID_ROUTER);
#endif
      break;
    case ID_TEENSY:
      SerialTeensy.write(CMD_PING);   SerialTeensy.write(ID_TEENSY);  writeEnd(SerialTeensy);   break;
    case ID_MOD:
      SerialMod.write(CMD_PING);      SerialMod.write(ID_MOD);        writeEnd(SerialMod);      break;
    case ID_CTRL:
      SerialCtrl.write(CMD_PING);     SerialCtrl.write(ID_CTRL);      writeEnd(SerialCtrl);     break;
    default: break;
  }
}

// ========================== LEGACY RX from Display ==========================
// NOTA: usato solo come fallback se il parser LWS non ha sincronizzato.
// Con LWSv1.1 attivo, tutti i frame dovrebbero passare dal parser.
#if ENABLE_LEGACY_PROTO
void handleDisplayInputLegacyByte(uint8_t b) {
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
}
#endif

// ========================== LWS RX FROM DISPLAY ==========================
#if ENABLE_LWS_V1
void handleDisplayFrameLws(const LwsFrame& f) {
  const char cmd = (char)f.cmd;

  switch (cmd) {
    case CMD_PING: {
      // Il Display include nel payload il nodo da interrogare.
      char target = f.len > 0 ? (char)f.data[0] : ID_ROUTER;
      if (target == ID_ROUTER) {
        sendPongToDisplayLws(ID_ROUTER);
      } else {
        forwardPingToNodeLegacy(target);
      }
      break;
    }

    case CMD_PARAM: {
      // Fire-and-forget: il Router non ha parametri propri, ma se il
      // Display indirizza un param verso un nodo, lo inoltriamo via legacy.
      // (Formato legacy per i param verso i nodi non e' ancora definito,
      //  quindi per ora logghiamo soltanto.)
      Serial.printf("[LWS] PARAM target=%c key=%c val=%u (dropped)\n",
                    f.len >= 1 ? (char)f.data[0] : '?',
                    f.len >= 2 ? (char)f.data[1] : '?',
                    f.len >= 3 ? f.data[2] : 0);
      break;
    }

    case CMD_PARAM_REL: {
      // Reliable: il Router conferma con ACK (anche se non applica nulla).
      Serial.printf("[LWS] PARAM_REL seq=%u target=%c key=%c val=%u\n",
                    f.seq,
                    f.len >= 1 ? (char)f.data[0] : '?',
                    f.len >= 2 ? (char)f.data[1] : '?',
                    f.len >= 3 ? f.data[2] : 0);
      sendParamAckToDisplayLws(f.seq, f.cmd);
      break;
    }

    case 'c':
    case 'n':
    case 'b':
    case 's':
    case 'e':
    case 'a':
    case CMD_PONG:
    default:
      // Comandi non indirizzati al Router: ignora.
      break;
  }
}
#endif

void pollDisplayPort() {
  while (DISPLAY_PORT.available() > 0) {
    uint8_t b = (uint8_t)DISPLAY_PORT.read();

#if ENABLE_LWS_V1
    LwsFrame f;
    if (lwsParserDisplay.feed(b, f)) {
      handleDisplayFrameLws(f);
      continue;
    }
    // NOTA: in LWSv1.1 il parser consuma il byte ad ogni chiamata e ritorna
    // true solo a frame completo. Non c'e' modo di "ripescare" il byte per
    // il legacy handler. Se hai bisogno del dual-mode reale, usa due porte
    // separate o un flag di modalita' negoziato all'avvio.
#endif
  }
}

// ========================== NODES -> DISPLAY PONG ==========================
// I nodi rispondono con protocollo legacy: 'p' + [sub_id?]
// Il Router traduce in LWS PONG verso il Display, spoofando il SENDER
// con l'ID del nodo che ha effettivamente risposto.
void handleNodePong(Stream& node, char nodeId, const char* dbgName,
                    bool synthAHasSubId = false) {
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
    if (node.available() > 0) (void)node.read();
  }

#if ENABLE_LWS_V1
  sendPongToDisplayLws(who);
  (void)dbgName;
#else
  sendPongToDisplayLegacy(who);
  (void)dbgName;
#endif
}

// ========================== SETUP / LOOP ==========================
void setup() {
  Serial.begin(115200);        // debug USB
  DISPLAY_PORT.begin(115200);  // Display UART

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
  // Reset esplicito del parser e del contatore SEQ
  lwsParserDisplay.reset();
  lwsTxSeq = 0;

  sendStatusToDisplayLws("Router boot (LWSv1.1)");
#endif

  Serial.println("[Router] LWSv1.1 ready");
}

void loop() {
  MIDI.read();

  // Input dal Display (LWS parser prioritario)
  pollDisplayPort();

  // Input dai nodi (legacy -> LWS PONG)
  handleNodePong(SerialSynthA, ID_SYNTH_A1, "SerialSynthA", true);
  handleNodePong(SerialSynthB, ID_SYNTH_B,  "SerialSynthB");
  handleNodePong(SerialCtrl,   ID_CTRL,     "SerialCtrl");
  handleNodePong(SerialMod,    ID_MOD,      "SerialMod");
  handleNodePong(SerialTeensy, ID_TEENSY,   "SerialTeensy");
  handleNodePong(SerialPower,  ID_POWER,    "SerialPower");

#if ENABLE_LWS_V1
  // Heartbeat Router -> Display (1 Hz)
  static uint32_t tPing = 0;
  if (millis() - tPing >= 1000) {
    tPing = millis();
    sendPingToDisplayLws();
  }
#endif
}