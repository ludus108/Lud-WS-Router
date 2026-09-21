/*  Lud-WS Router ; Pi Pico 2 rp2350
 *
 *  V 0.0.5
 *
 *  Bridge tra Display (LWSv1.1) e nodi.
 *
 *  Aggiornamenti v0.0.5:
 *    - SynthA (a, b, c) riconosciuti come nodi LWS nativi
 *    - Forwarding trasparente dei CMD estesi SynthA
 *    - MIDI hardware convertito in CMD_MIDI_*_V verso SynthA 'a'
 *    - Rimosso il path legacy per SynthA (resta solo per SynthB)
 *
 *  Topologia:
 *    Display <-- 1Mbps --> Router
 *                            ├─ SerialSynthA (SerialPIO GP2/3, 115200) → [a]→[b]→[c]
 *                            ├─ SerialTeensy (Serial1 GP0/1, 1Mbps)     → [T]
 *                            ├─ SerialSynthB (SerialPIO GP8/9, 115200) → [B] legacy
 *                            ├─ SerialCtrl   (SerialPIO GP6/7, 115200) → [C]
 *                            ├─ SerialMod    (SerialPIO GP10/11, 115200) → [M]
 *                            ├─ SerialPower  (SerialPIO GP14/15, 115200) → [P]
 *                            └─ SerialMidi   (SerialPIO GP12/13, 31250)  MIDI IN
 */

#include <Arduino.h>
#include <MIDI.h>

// ---------------------------------------------------------------------
// Config di sistema
// ---------------------------------------------------------------------
#define LWS_BAUD        1000000UL   // UART hardware (Teensy + Display)
#define NODE_BAUD       115200UL    // SerialPIO verso nodi
#define SYNTHA_BAUD     115200UL    // SerialPIO verso [a] (cascata A)
#define SYNTHB_BAUD     115200UL    // SerialPIO verso [B] legacy

#include "serial_protocol.h"

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

// ========================== COMMAND CODES (legacy fallback) ==========================
#ifndef CMD_PING
  #define CMD_PING       'p'
  #define CMD_PONG       'P'
  #define CMD_PARAM      'S'
  #define CMD_PARAM_REL  'R'
  #define CMD_PARAM_ACK  'A'
#endif

// ========================== FEATURE FLAGS ==========================
#define ENABLE_LEGACY_PROTO 1
#define ENABLE_LWS_V1       1

// ========================== UART MAPPING ==========================
// UART hardware:
//   Serial1 (UART0) -> Teensy
//   Serial2 (UART1) -> Display
// SerialPIO:
//   SerialSynthA -> GP2 (TX) / GP3 (RX)  [a] cascata
//   SerialCtrl   -> GP6 / GP7
//   SerialSynthB -> GP8 / GP9            legacy
//   SerialMod    -> GP10 / GP11
//   SerialMidi   -> GP12 / GP13          MIDI IN
//   SerialPower  -> GP14 / GP15
// ==========================
#define DISPLAY_PORT Serial2
#define SerialTeensy Serial1

SerialPIO SerialSynthA(2, 3);
SerialPIO SerialCtrl  (6, 7);
SerialPIO SerialSynthB(8, 9);
SerialPIO SerialMod   (10, 11);
SerialPIO SerialMidi  (12, 13);
SerialPIO SerialPower (14, 15);

// MIDI IN
MIDI_CREATE_INSTANCE(SerialPIO, SerialMidi, MIDI);

// Canali MIDI (numerazione 1..16)
byte midi_SynthA_CH = 1;
byte midi_SynthB_CH = 2;
byte midi_Drum_CH   = 3;

// ========================== LWS STATE ==========================
#if ENABLE_LWS_V1
static LwsParser lwsParserDisplay;
static uint8_t   lwsTxSeq = 0;

static inline uint8_t nextSeq() { return lwsTxSeq++; }

static inline void txLws(char sender, uint8_t cmd,
                         const uint8_t* p = nullptr, uint8_t len = 0) {
  lws_send_frame(DISPLAY_PORT, (uint8_t)sender, nextSeq(), cmd, p, len);
}
#endif

// ========================== HELPERS LEGACY ==========================
inline void writeEnd(Stream& s) { s.write('&'); s.write('!'); }

// ========================== LWS TX VERSO IL DISPLAY ==========================
#if ENABLE_LWS_V1
void sendPongToDisplayLws(char who) {
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
  txLws(ID_ROUTER, CMD_ERROR, (const uint8_t*)txt, len);
}

// Telemetria MIDI verso il Display
void sendCCToDisplayLws(uint8_t cc, uint8_t val) {
  uint8_t p[2] = {cc, val};
  txLws(ID_ROUTER, CMD_MIDI_CC, p, 2);
}
void sendNoteToDisplayLws(uint8_t onoff, uint8_t pitch, uint8_t vel) {
  uint8_t p[3] = {onoff, pitch, vel};
  txLws(ID_ROUTER, CMD_MIDI_NOTE, p, 3);
}
void sendBendToDisplayLws(int32_t bend) {
  uint8_t p[4];
  lws_pack_i32_le(bend, p);
  txLws(ID_ROUTER, CMD_MIDI_BEND, p, 4);
}
#endif

// ========================== ROUTING TABLE ==========================
// Nodi LWS nativi: il Router forwarda i loro frame al Display invariati.
static bool isLwsNode(char id) {
#if ENABLE_LWS_V1
  // Nodi nativi (frame LWS v1.1):
  return (id == ID_TEENSY ||
          id == ID_SYNTH_A1 || id == ID_SYNTH_A2 || id == ID_SYNTH_A3);
#else
  (void)id;
  return false;
#endif
}

// Porta fisica associata a un ID nodo
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

// Ritorna il target di un frame voice-specific SynthA in base alla voice.
// voice 0 → a ; voice 1,2 → b ; voice 3,4 → c
static char targetForVoice(uint8_t voice) {
  if (voice == 0)                return ID_SYNTH_A1;
  if (voice == 1 || voice == 2)  return ID_SYNTH_A2;
  if (voice == 3 || voice == 4)  return ID_SYNTH_A3;
  return ID_SYNTH_A1;
}

// ========================== LEGACY PING VERSO NODI ==========================
// Usato solo per i nodi ancora legacy (SynthB, Ctrl, Mod, Power).
void forwardPingToNodeLegacy(char nodeId) {
  switch (nodeId) {
    case ID_POWER:
      SerialPower.write(CMD_PING);   SerialPower.write(ID_POWER);   writeEnd(SerialPower);   break;
    case ID_SYNTH_B:
      SerialSynthB.write(CMD_PING);  SerialSynthB.write(ID_SYNTH_B);writeEnd(SerialSynthB);  break;
    case ID_ROUTER:
      sendPongToDisplayLws(ID_ROUTER);
      break;
    case ID_MOD:
      SerialMod.write(CMD_PING);     SerialMod.write(ID_MOD);       writeEnd(SerialMod);     break;
    case ID_CTRL:
      SerialCtrl.write(CMD_PING);    SerialCtrl.write(ID_CTRL);     writeEnd(SerialCtrl);    break;
    default: break;
  }
}

// ========================== PING DISPATCHER ==========================
void forwardPingToNode(char target) {
  if (target == ID_ROUTER) {
    sendPongToDisplayLws(ID_ROUTER);
    return;
  }
  Stream *p = portForNode(target);
  if (!p) return;

  if (isLwsNode(target)) {
    uint8_t payload[1] = { (uint8_t)target };
    lws_send_frame(*p, ID_ROUTER, nextSeq(), CMD_PING, payload, 1);
  } else {
    forwardPingToNodeLegacy(target);
  }
}

// ========================== PARAM DISPATCHER ==========================
// Inoltra un CMD dal Display verso il nodo target.
// Supporta i CMD estesi SynthA.
void forwardParamToNode(const LwsFrame &f) {
  if (f.len < 1) return;
  char target = (char)f.data[0];

  if (target == ID_ROUTER) {
    if ((char)f.cmd == CMD_PARAM_REL) {
      // ACK al Display
      uint8_t p[2] = { f.seq, f.cmd };
      txLws(ID_ROUTER, CMD_PARAM_ACK, p, 2);
    }
    return;
  }

  Stream *p = portForNode(target);
  if (!p) {
    Serial.printf("[LWS] param: target sconosciuto %c\n", target);
    return;
  }

  if (isLwsNode(target)) {
    // Forward invariato: SENDER='D' (Display), SEQ=Display's seq.
    lws_send_frame(*p, f.sender, f.seq, f.cmd, f.data, f.len);
  } else {
    // Legacy: non implementato per i parametri complessi
    Serial.printf("[LWS] param to legacy %c dropped\n", target);
    if ((char)f.cmd == CMD_PARAM_REL) {
      uint8_t p2[2] = { f.seq, f.cmd };
      txLws(ID_ROUTER, CMD_PARAM_ACK, p2, 2);
    }
  }
}

// ========================== MIDI DISPATCHER (SynthA) ==========================
// Riceve un CMD_MIDI_*_V dal Display e lo forwarda al target corretto
// in base alla voice. Il target è già nel payload, ma lo ricalcoliamo
// per sicurezza (se il Display sbaglia, lo correggiamo).
static void forwardMidiVToSynthA(const LwsFrame& f) {
  if (f.len < 2) return;
  uint8_t voice = f.data[1];  // data[0]=target, data[1]=voice
  char target = (char)f.data[0];

  // Se il target è già valido e corrisponde a un LWS node SynthA, usa quello
  if (target != ID_SYNTH_A1 && target != ID_SYNTH_A2 && target != ID_SYNTH_A3) {
    target = targetForVoice(voice);
    // aggiorna il byte di target nel frame (copia locale)
  }

  Stream *p = portForNode(target);
  if (!p) return;

  if (isLwsNode(target)) {
    // Forward invariato al nodo LWS (che sia 'a', 'b' o 'c')
    // Il frame attraverserà la catena: se target != 'a', il Master
    // farà relay verso i successivi.
    lws_send_frame(*p, f.sender, f.seq, f.cmd, f.data, f.len);
  }
}

// ========================== LWS RX FROM DISPLAY ==========================
void handleDisplayFrameLws(const LwsFrame& f) {
  const char cmd = (char)f.cmd;

  switch (cmd) {

    // ---------- PING ----------
    case CMD_PING: {
      char target = f.len > 0 ? (char)f.data[0] : ID_ROUTER;
      forwardPingToNode(target);
      break;
	      // ---------- PRESET TRANSFER (SynthA + SynthB) ----------
    case CMD_PRESET_BEGIN:
    case CMD_PRESET_CHUNK:
    case CMD_PRESET_END:
    case CMD_PRESET_READ:
        forwardParamToNode(f);   // forward per target
        break;

    // ---------- PRESET ACK/DUMP (dai nodi al Display) ----------
    case CMD_PRESET_ACK:
    case CMD_PRESET_DUMP_BEGIN:
    case CMD_PRESET_DUMP_CHUNK:
    case CMD_PRESET_DUMP_END:
        // Questi sono destinati al Display: il Router li forwarda nel
        // bridge nodi→Display automaticamente (pollNodePorts).
        // Non serve gestirli qui nel dispatch del Display.
        break;
    }

    // ---------- PARAM standard + estesi SynthA ----------
    case CMD_PARAM:
    case CMD_PARAM_REL:
    case CMD_PARAM_VOCE:
    case CMD_PARAM_I32:
    case CMD_PARAM_I32_V:
      forwardParamToNode(f);
      break;

    // ---------- MIDI estesi con voice (SynthA) ----------
    case CMD_MIDI_NOTE_V:
    case CMD_MIDI_BEND_V:
    case CMD_MIDI_CC_V:
      forwardMidiVToSynthA(f);
      break;

    // ---------- CMD non diretti al Router ----------
    case CMD_PONG:
    case CMD_PARAM_ACK:
    case CMD_MIDI_CC:
    case CMD_MIDI_NOTE:
    case CMD_MIDI_BEND:
    case CMD_ERROR:
    default:
      break;
  }
}

// ========================== LEGACY RX FROM DISPLAY ==========================
#if ENABLE_LEGACY_PROTO
void handleDisplayInputLegacyByte(uint8_t b) {
  char p = (char)b;
  if (p == CMD_PING) {
    if (DISPLAY_PORT.available() > 0) {
      char m = (char)DISPLAY_PORT.read();
      forwardPingToNode(m);
    }
  } else if (p == ID_CTRL) {
    if (DISPLAY_PORT.available() > 0) SerialCtrl.write(DISPLAY_PORT.read());
  } else if (p == ID_MOD) {
    if (DISPLAY_PORT.available() > 0) SerialMod.write(DISPLAY_PORT.read());
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
#endif

#if ENABLE_LEGACY_PROTO
    handleDisplayInputLegacyByte(b);
#endif
  }
}

// ========================== NODES -> DISPLAY (LWS BRIDGE) ==========================
#if ENABLE_LWS_V1
struct NodePort {
  Stream*   s;
  char      id;
  LwsParser parser;
};

static NodePort nodePorts[] = {
  // Nodi LWS nativi, con parser dedicato
  { &SerialTeensy, ID_TEENSY,   {} },
  { &SerialSynthA, ID_SYNTH_A1, {} },  // gestisce anche b e c (cascata)
};
static const size_t NODE_PORT_COUNT = sizeof(nodePorts) / sizeof(nodePorts[0]);

// Legge le porte LWS e forwarda i frame al Display invariati.
// Il SENDER del frame determina chi ha parlato ('T', 'a', 'b', 'c').
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

// ========================== MIDI HANDLERS ==========================
// Il Router riceve il MIDI hardware e lo converte in LWS.
// SynthA è ora LWS nativo → CMD_MIDI_*_V con target e voice.
// SynthB è ancora legacy → protocollo '&!' su SerialSynthB.

// --- Helper LWS verso SynthA ---
#if ENABLE_LWS_V1
static void sendMidiNoteV_SynthA(uint8_t voice, uint8_t onoff,
                                 uint8_t pitch, uint8_t vel) {
  char target = targetForVoice(voice);
  uint8_t p[5] = { (uint8_t)target, voice, onoff, pitch, vel };
  Stream *sp = portForNode(target);
  if (sp) lws_send_frame(*sp, ID_ROUTER, nextSeq(), CMD_MIDI_NOTE_V, p, 5);
}

static void sendMidiBendV_SynthA(uint8_t voice, int32_t bend) {
  char target = targetForVoice(voice);
  uint8_t p[6];
  p[0] = (uint8_t)target;
  p[1] = voice;
  lws_pack_i32_le(bend, &p[2]);
  Stream *sp = portForNode(target);
  if (sp) lws_send_frame(*sp, ID_ROUTER, nextSeq(), CMD_MIDI_BEND_V, p, 6);
}

static void sendMidiCCV_SynthA(uint8_t voice, uint8_t cc, uint8_t value) {
  char target = targetForVoice(voice);
  uint8_t p[4] = { (uint8_t)target, voice, cc, value };
  Stream *sp = portForNode(target);
  if (sp) lws_send_frame(*sp, ID_ROUTER, nextSeq(), CMD_MIDI_CC_V, p, 4);
}
#endif

// --- Helper legacy verso SynthB ---
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

void sendBenderB(int bend) {
  SerialSynthB.write('m');
  SerialSynthB.write('b');
  SerialSynthB.write((uint8_t*)&bend, 4);
  writeEnd(SerialSynthB);
#if ENABLE_LWS_V1
  sendBendToDisplayLws((int32_t)bend);
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

// --- MIDI callbacks ---
void handleNoteOn(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH) {
#if ENABLE_LWS_V1
    // Il Router manda sempre voice=0 al SynthA.
    // Il Master in Poly broadcasta a tutte le voci.
    // In MultiMono, il Display è responsabile di mandare le altre voice.
    sendMidiNoteV_SynthA(0, 1, pitch, velocity);
    sendNoteToDisplayLws(1, pitch, velocity);
#endif
  } else if (channel == midi_SynthB_CH) {
    sendNoteOnOffB(1, pitch, velocity);
  }
}

void handleNoteOff(byte channel, byte pitch, byte velocity) {
  if (channel == midi_SynthA_CH) {
#if ENABLE_LWS_V1
    sendMidiNoteV_SynthA(0, 0, pitch, velocity);
    sendNoteToDisplayLws(0, pitch, velocity);
#endif
  } else if (channel == midi_SynthB_CH) {
    sendNoteOnOffB(0, pitch, velocity);
  }
}

void handlePitchBend(byte channel, int bend) {
  if (channel == midi_SynthA_CH) {
#if ENABLE_LWS_V1
    sendMidiBendV_SynthA(0, (int32_t)bend);
    sendBendToDisplayLws((int32_t)bend);
#endif
  } else if (channel == midi_SynthB_CH) {
    sendBenderB(bend);
  }
}

void handleControlChange(byte channel, byte number, byte value) {
  if (channel == midi_SynthA_CH) {
#if ENABLE_LWS_V1
    sendMidiCCV_SynthA(0, number, value);
    sendCCToDisplayLws(number, value);
#endif
  } else if (channel == midi_SynthB_CH) {
    sendCC_B(number, value);
  }
}

// ========================== NODES LEGACY -> DISPLAY PONG ==========================
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
#endif
}
#endif

// ========================== SETUP / LOOP ==========================
void setup() {
  Serial.begin(115200);                  // USB debug
  DISPLAY_PORT.begin(LWS_BAUD);          // UART1 → Display @ 1 Mbps
  SerialTeensy.begin(LWS_BAUD);          // UART0 → Teensy  @ 1 Mbps

  // MIDI
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.setHandlePitchBend(handlePitchBend);

  // SerialPIO verso nodi
  SerialSynthA.begin(SYNTHA_BAUD);       // [a] cascata SynthA @ 115200
  SerialSynthB.begin(SYNTHB_BAUD);       // [B] legacy
  SerialCtrl.begin(NODE_BAUD);
  SerialMod.begin(NODE_BAUD);
  SerialPower.begin(NODE_BAUD);

#if ENABLE_LWS_V1
  lwsParserDisplay.reset();
  lwsTxSeq = 0;
  for (size_t i = 0; i < NODE_PORT_COUNT; i++) nodePorts[i].parser.reset();
  sendStatusToDisplayLws("Router boot (v0.0.5, SynthA LWS)");
#endif

  Serial.println("[Router] LWSv1.1 ready (SynthA as LWS nodes)");
}

void loop() {
  MIDI.read();

  // Display → Router → Nodi
  pollDisplayPort();

  // Nodi LWS → Display
#if ENABLE_LWS_V1
  pollNodePorts();
#endif

  // Nodi legacy → Display (solo quelli che non sono LWS nativi)
#if ENABLE_LEGACY_PROTO
  handleNodePong(SerialSynthB, ID_SYNTH_B, "SerialSynthB");
  handleNodePong(SerialCtrl,   ID_CTRL,     "SerialCtrl");
  handleNodePong(SerialMod,    ID_MOD,      "SerialMod");
  handleNodePong(SerialPower,  ID_POWER,    "SerialPower");
  // SerialSynthA NON passa da qui: è LWS nativo (in nodePorts[])
#endif

  // Heartbeat Router → Display (1 Hz)
#if ENABLE_LWS_V1
  static uint32_t tPing = 0;
  if (millis() - tPing >= 1000) {
    tPing = millis();
    sendPingToDisplayLws();
  }
#endif
}