/*  Lud-WS Router ; Pi Pico 2 rp2350

    V 0.0.3

 *  Bridge tra Display (LWSv1.1) e nodi.
 *
 *  Stato attuale:
 *    - Teensy      : LWSv1.1 nativo  -> bridged as-is verso il Display
 *    - Altri nodi  : legacy '&!'     -> tradotti in PONG LWS dal Router
 *    - Router      : risponde ai PING indirizzati a se stesso ('R')
 *    - MIDI        : genera telemetria LWS verso il Display
 *
 *  LWSv1.1 frame format:
 *      [SENDER][SEQ][CMD][LEN][PAYLOAD...][CRC8][&][!]
 *
 *  Il Router e' un bridge trasparente per i nodi LWS: i frame passano
 *  invariati (SENDER, SEQ, CMD, PAYLOAD originali). Solo i frame dei nodi
 *  legacy vengono tradotti.
 */

#include <Arduino.h>
#include <MIDI.h>
#include "serial_protocol.h"   // condiviso con Display, versione LWSv1.1

// Lud-WS-Router.ino
#define MCU_ID 'R'
#include "comunicazioni_mcu.h"
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
byte midi_Drum_CH = 3;

// ========================== LWS STATE ==========================
#if ENABLE_LWS_V1
static LwsParser lwsParserDisplay;     // parser verso il Display
static uint8_t   lwsTxSeq = 0;         // SEQ rolling per i frame TX del Router

static inline uint8_t nextSeq() { return lwsTxSeq++; }

// Wrapper: invia un frame LWS al Display con il prossimo SEQ del Router.
// Usato solo per frame GENERATI dal Router (PONG, status, telemetria MIDI).
// Per i frame INOLTRATI da nodi LWS si usa lws_send_frame() diretto,
// per preservare SENDER/SEQ originali.
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

// ACK per un CMD_PARAM_REL ricevuto dal Display e diretto al Router
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

// ========================== NODE ROUTING TABLE ==========================
// Identifica se un nodo parla LWS nativo (true) o legacy (false).
// Man mano che si migrano i nodi, aggiungere qui il loro ID.
static bool isLwsNode(char id) {
#if ENABLE_LWS_V1
  return (id == ID_TEENSY);
#else
  (void)id;
  return false;
#endif
}

// Restituisce la porta fisica associata a un ID nodo.
static Stream* portForNode(char id) {
  switch (id) {
    case ID_SYNTH_A1:
    case ID_SYNTH_A2:
    case ID_SYNTH_A3: return &SerialSynthA;
    case ID_SYNTH_B:  return &SerialSynthB;
    case ID_CTRL:     return &SerialCtrl;
    case ID_MOD:      return &SerialMod;
    case ID_TEENSY:   return &SerialTeensy;
    case ID_POWER:    return &SerialPower;
    default:          return nullptr;
  }
}

// ========================== LEGACY PING VERSO NODI ==========================
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

// ========================== PING DISPATCHER ==========================
// Decide se mandare un PING LWS o legacy a seconda del tipo di nodo.
#if ENABLE_LWS_V1
void forwardPingToNode(char target) {
  if (target == ID_ROUTER) {
    sendPongToDisplayLws(ID_ROUTER);
    return;
  }
  Stream *p = portForNode(target);
  if (!p) return;

  if (isLwsNode(target)) {
    // LWS PING: [target_id] come payload
    uint8_t payload[1] = { (uint8_t)target };
    lws_send_frame(*p, ID_ROUTER, nextSeq(), CMD_PING, payload, 1);
  } else {
    forwardPingToNodeLegacy(target);
  }
}

// ========================== PARAM DISPATCHER ==========================
// Inoltra un CMD_PARAM / CMD_PARAM_REL dal Display verso il nodo target.
// Per nodi LWS: frame inoltrato invariato (stesso SENDER/SEQ del Display).
// Per nodi legacy: non ancora implementato (solo log).
void forwardParamToNode(const LwsFrame &f) {
  if (f.len < 1) return;
  char target = (char)f.data[0];

  if (target == ID_ROUTER) {
    // Il Router non ha parametri propri, ma conferma se REL
    if ((char)f.cmd == CMD_PARAM_REL) {
      sendParamAckToDisplayLws(f.seq, f.cmd);
    }
    return;
  }

  Stream *p = portForNode(target);
  if (!p) {
    Serial.printf("[LWS] param: unknown target %c\n", target);
    return;
  }

  if (isLwsNode(target)) {
    // Forward invariato: SENDER='D' (Display), SEQ=Display's seq.
    // Il Teensy applichera' e rispondera' con ACK usando lo STESSO seq.
    lws_send_frame(*p, f.sender, f.seq, f.cmd, f.data, f.len);
  } else {
    // Legacy: formato param verso nodi non ancora definito
    Serial.printf("[LWS] param to legacy node %c dropped (not impl.)\n", target);
    if ((char)f.cmd == CMD_PARAM_REL) {
      // Best-effort: ACK per non far scadere il pending sul Display
      sendParamAckToDisplayLws(f.seq, f.cmd);
    }
  }
}
#endif

// ========================== LWS RX FROM DISPLAY ==========================
#if ENABLE_LWS_V1
void handleDisplayFrameLws(const LwsFrame& f) {
  const char cmd = (char)f.cmd;

  switch (cmd) {
    case CMD_PING: {
      char target = f.len > 0 ? (char)f.data[0] : ID_ROUTER;
      forwardPingToNode(target);
      break;
    }

    case CMD_PARAM:
    case CMD_PARAM_REL: {
      forwardParamToNode(f);
      break;
    }

    case 'c':
    case 'n':
    case 'b':
    case 's':
    case 'e':
    case 'a':
    case CMD_PONG:
    case CMD_PARAM_ACK:
    default:
      // Comandi non indirizzati al Router: ignora silenziosamente.
      break;
  }
}
#endif

// ========================== LEGACY RX from Display (fallback) ==========================
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

void pollDisplayPort() {
  while (DISPLAY_PORT.available() > 0) {
    uint8_t b = (uint8_t)DISPLAY_PORT.read();

#if ENABLE_LWS_V1
    LwsFrame f;
    if (lwsParserDisplay.feed(b, f)) {
      handleDisplayFrameLws(f);
      continue;
    }
    // NOTA: in LWSv1.1 il parser consuma il byte ad ogni chiamata.
    // Il path legacy sotto e' di fatto irraggiungibile quando il Display
    // parla LWS. Tenuto per sicurezza durante la fase di migrazione.
#endif

#if ENABLE_LEGACY_PROTO
    handleDisplayInputLegacyByte(b);
#endif
  }
}

// ========================== NODES -> DISPLAY (LWS BRIDGE) ==========================
// Struttura di stato per ogni porta che parla LWS nativo.
// Le porte ancora legacy NON vanno qui: usano handleNodePong().
#if ENABLE_LWS_V1
struct NodePort {
  Stream*   s;
  char      id;
  LwsParser parser;
};

static NodePort nodePorts[] = {
  // Migrati a LWS nativo:
  { &SerialTeensy, ID_TEENSY, {} },
  // Aggiungere qui Ctrl, Mod, Power, Synth... quando migrati.
};
static const size_t NODE_PORT_COUNT = sizeof(nodePorts) / sizeof(nodePorts[0]);

// Legge le porte LWS, ricompone i frame e li inoltra al Display invariati.
// I frame PONG, ACK, TIMELINE ecc. generati dai nodi arrivano al Display
// come se fossero diretti, con SENDER = ID del nodo.
static void pollNodePorts() {
  for (size_t i = 0; i < NODE_PORT_COUNT; i++) {
    NodePort &np = nodePorts[i];
    while (np.s->available() > 0) {
      uint8_t b = (uint8_t)np.s->read();
      LwsFrame f;
      if (np.parser.feed(b, f)) {
        lws_send_frame(DISPLAY_PORT, f.sender, f.seq, f.cmd, f.data, f.len);
        Serial.printf("[LWS] bridge %c -> D: cmd=%c len=%u\n",
                      (char)f.sender, (char)f.cmd, f.len);
      }
    }
  }
}
#endif

// ========================== NODES LEGACY -> DISPLAY PONG ==========================
// I nodi legacy rispondono con 'p' + [sub_id?] + '&!'.
// Il Router li traduce in PONG LWS verso il Display.
// NON va chiamata per le porte presenti in nodePorts[].
#if ENABLE_LEGACY_PROTO
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
#endif

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
  lwsParserDisplay.reset();
  lwsTxSeq = 0;
  for (size_t i = 0; i < NODE_PORT_COUNT; i++) nodePorts[i].parser.reset();

  sendStatusToDisplayLws("Router boot (LWSv1.1)");
#endif

  Serial.println("[Router] LWSv1.1 ready");
}

void loop() {
  MIDI.read();

  // --- Display -> Router/Nodi ---
  pollDisplayPort();

  // --- Nodi LWS -> Display (bridge trasparente) ---
#if ENABLE_LWS_V1
  pollNodePorts();
#endif

  // --- Nodi legacy -> Display (PONG tradotto) ---
#if ENABLE_LEGACY_PROTO
  handleNodePong(SerialSynthA, ID_SYNTH_A1, "SerialSynthA", true);
  handleNodePong(SerialSynthB, ID_SYNTH_B,  "SerialSynthB");
  handleNodePong(SerialCtrl,   ID_CTRL,     "SerialCtrl");
  handleNodePong(SerialMod,    ID_MOD,      "SerialMod");
  // SerialTeensy NON passa da qui: e' LWS nativo (in nodePorts[])
  handleNodePong(SerialPower,  ID_POWER,    "SerialPower");
#endif

#if ENABLE_LWS_V1
  // Heartbeat Router -> Display (1 Hz)
  static uint32_t tPing = 0;
  if (millis() - tPing >= 1000) {
    tPing = millis();
    sendPingToDisplayLws();
  }
#endif
}