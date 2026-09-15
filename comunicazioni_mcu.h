#ifndef COMUNICAZIONI_MCU_H
#define COMUNICAZIONI_MCU_H

/*
 * ============================================================================
 *  comunicazioni_mcu.h — LWSv1.1 — LATO MCU
 * ============================================================================
 *
 *  Gemello di comunicazioni.h lato Display. Va incluso in ogni nodo
 *  (Router, Synth A1/A2/A3, Synth B, Ctrl, Mod, Teensy, Power) e usa lo
 *  STESSO serial_protocol.h condiviso.
 *
 *  CHANGELOG
 *  ---------------------------------------------------------------------------
 *  [10.1] FIX : ID_POWER aggiunto alla mappa MCU (lato Display).
 *  [10.2] ADD : CRC-8/ATM nel frame.
 *  [10.3] ADD : SEQ + protocollo ibrido di ACK.
 *  [10.4] ADD : 3 comandi MIDI (CMD_MIDI_CC/NOTE/BEND) + callbacks.
 *  [10.4] FIX : accetta frame sia da ID_DISPLAY che da ID_ROUTER.
 *               Il Router e' l'unico che puo' inviare comandi operativi
 *               (PING, MIDI) perche' e' il bridge verso i nodi.
 *
 *  Formato frame LWSv1.1:
 *      [SENDER][SEQ][CMD][LEN][PAYLOAD...][CRC8][&][!]
 *
 *  CONFIGURAZIONE (prima dell'include):
 *      MCU_ID      : ID char del nodo (es. 'a', 'B', 'C', ...)
 *      LWS_SERIAL  : Stream LWS (default: Serial1)
 *      LWS_DEBUG   : Stream debug (default: Serial)
 *
 *  USO TIPICO:
 *
 *      void on_param(char t, char k, uint8_t v) { ... }
 *      void on_err(const char *m) { ... }
 *      void on_cc(uint8_t cc, uint8_t val) { ... }
 *      void on_note(uint8_t on, uint8_t pitch, uint8_t vel) { ... }
 *      void on_bend(int32_t b) { ... }
 *
 *      void setup() {
 *          LWS_SERIAL.begin(115200);
 *          lws_set_callbacks(on_param, on_err);
 *          lws_set_midi_callbacks(on_cc, on_note, on_bend);
 *      }
 *
 *      void loop() {
 *          lws_mcu_poll();
 *      }
 * ============================================================================
 */

#include <Arduino.h>
#include "serial_protocol.h"

// ========================== CONFIGURAZIONE NODO ==========================
#ifndef MCU_ID
#define MCU_ID 'x'
#endif

#ifndef LWS_SERIAL
#define LWS_SERIAL Serial1
#endif

#ifndef LWS_DEBUG
#define LWS_DEBUG Serial
#endif

// ========================== IDENTIFICATORI ==========================
#define ID_DISPLAY      'D'
#define ID_ROUTER       'R'

// ========================== SEQ + PENDING ACK ==========================
static uint8_t g_tx_seq = 0;
static inline uint8_t lws_next_seq() { return g_tx_seq++; }

#define LWS_PENDING_MAX     8
#define LWS_ACK_TIMEOUT_MS  300
#define LWS_ACK_RETRIES_MAX 3

struct LwsPendingAck {
    bool     used;
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  data[8];
    uint8_t  retries;
    uint32_t t_sent;
};
static LwsPendingAck g_pending[LWS_PENDING_MAX] = {};

static LwsPendingAck* lws_pending_find(uint8_t seq) {
    for (int i = 0; i < LWS_PENDING_MAX; i++)
        if (g_pending[i].used && g_pending[i].seq == seq) return &g_pending[i];
    return nullptr;
}
static LwsPendingAck* lws_pending_alloc() {
    for (int i = 0; i < LWS_PENDING_MAX; i++)
        if (!g_pending[i].used) return &g_pending[i];
    return nullptr;
}

// ========================== CALLBACKS UTENTE ==========================
typedef void (*lws_param_cb_t)(char target, char key, uint8_t value);
typedef void (*lws_error_cb_t)(const char *msg);
typedef void (*lws_midi_cc_cb_t)(uint8_t cc, uint8_t value);
typedef void (*lws_midi_note_cb_t)(uint8_t onoff, uint8_t pitch, uint8_t vel);
typedef void (*lws_midi_bend_cb_t)(int32_t bend);

static lws_param_cb_t     g_param_cb     = nullptr;
static lws_error_cb_t     g_error_cb     = nullptr;
static lws_midi_cc_cb_t   g_midi_cc_cb   = nullptr;
static lws_midi_note_cb_t g_midi_note_cb = nullptr;
static lws_midi_bend_cb_t g_midi_bend_cb = nullptr;

static inline void lws_set_callbacks(lws_param_cb_t on_param,
                                     lws_error_cb_t on_error) {
    g_param_cb = on_param;
    g_error_cb = on_error;
}

static inline void lws_set_midi_callbacks(lws_midi_cc_cb_t   on_cc,
                                          lws_midi_note_cb_t on_note,
                                          lws_midi_bend_cb_t on_bend) {
    g_midi_cc_cb   = on_cc;
    g_midi_note_cb = on_note;
    g_midi_bend_cb = on_bend;
}

// ========================== TX ==========================
static void lws_send_pong() {
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PONG, nullptr, 0);
}

static void lws_send_param(char target, char key, uint8_t value) {
    uint8_t p[3] = { (uint8_t)target, (uint8_t)key, value };
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PARAM, p, 3);
}

static void lws_send_param_reliable(char target, char key, uint8_t value) {
    uint8_t p[3] = { (uint8_t)target, (uint8_t)key, value };
    uint8_t seq  = lws_next_seq();

    LwsPendingAck *pa = lws_pending_alloc();
    if (pa) {
        pa->used    = true;
        pa->seq     = seq;
        pa->cmd     = CMD_PARAM_REL;
        pa->len     = 3;
        memcpy(pa->data, p, 3);
        pa->retries = 0;
        pa->t_sent  = millis();
    } else {
        LWS_DEBUG.println("[LWS] coda ACK piena");
    }

    lws_send_frame(LWS_SERIAL, MCU_ID, seq, CMD_PARAM_REL, p, 3);
}

static void lws_send_error(const char *msg) {
    uint8_t p[64];
    size_t l = strlen(msg);
    if (l > 62) l = 62;
    p[0] = (uint8_t)ID_DISPLAY;
    memcpy(&p[1], msg, l);
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_ERROR, p, 1 + (uint8_t)l);
}

static void lws_send_ack(uint8_t acked_seq, uint8_t acked_cmd) {
    uint8_t p[2] = { acked_seq, acked_cmd };
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PARAM_ACK, p, 2);
}

// ========================== RX ==========================
static LwsParser lwsParser;

static void lws_process_frame(const LwsFrame &f) {
    // Accettiamo frame dal Display (forwarding) e dal Router (PING, MIDI).
    char s = (char)f.sender;
    if (s != ID_DISPLAY && s != ID_ROUTER) return;

    switch ((char)f.cmd) {
        case CMD_PING: {
            if (f.len >= 1 && (char)f.data[0] == MCU_ID) {
                lws_send_pong();
            }
            break;
        }

        case CMD_PARAM: {
            if (f.len >= 3 && g_param_cb) {
                g_param_cb((char)f.data[0], (char)f.data[1], f.data[2]);
            }
            break;
        }

        case CMD_PARAM_REL: {
            if (f.len >= 3) {
                if (g_param_cb) {
                    g_param_cb((char)f.data[0], (char)f.data[1], f.data[2]);
                }
                lws_send_ack(f.seq, f.cmd);
            }
            break;
        }

        case CMD_PARAM_ACK: {
            if (f.len >= 2) {
                uint8_t acked_seq = f.data[0];
                uint8_t acked_cmd = f.data[1];
                LwsPendingAck *pa = lws_pending_find(acked_seq);
                if (pa && pa->cmd == acked_cmd) {
                    pa->used = false;
                } else {
                    LWS_DEBUG.printf("[LWS] ACK orfano seq=%u\n", acked_seq);
                }
            }
            break;
        }

        case CMD_MIDI_CC: {
            if (f.len >= 2 && g_midi_cc_cb) {
                g_midi_cc_cb(f.data[0], f.data[1]);
            }
            break;
        }

        case CMD_MIDI_NOTE: {
            if (f.len >= 3 && g_midi_note_cb) {
                g_midi_note_cb(f.data[0], f.data[1], f.data[2]);
            }
            break;
        }

        case CMD_MIDI_BEND: {
            if (f.len >= 4 && g_midi_bend_cb) {
                g_midi_bend_cb(lws_unpack_i32_le(&f.data[0]));
            }
            break;
        }

        case CMD_ERROR: {
            char msg[64] = {0};
            if (f.len >= 2) {
                size_t l = f.len - 1;
                if (l > 62) l = 62;
                memcpy(msg, &f.data[1], l);
                msg[l] = '\0';
            }
            if (g_error_cb) g_error_cb(msg);
            else LWS_DEBUG.printf("[LWS] ERROR: %s\n", msg);
            break;
        }

        default:
            break;
    }
}

static inline void lws_mcu_read() {
    while (LWS_SERIAL.available() > 0) {
        uint8_t b = (uint8_t)LWS_SERIAL.read();
        LwsFrame f;
        if (lwsParser.feed(b, f)) lws_process_frame(f);
    }
}

// ========================== RETRY PENDING ==========================
static inline void lws_mcu_poll() {
    lws_mcu_read();

    uint32_t now = millis();
    for (int i = 0; i < LWS_PENDING_MAX; i++) {
        LwsPendingAck *pa = &g_pending[i];
        if (!pa->used) continue;
        if (now - pa->t_sent < LWS_ACK_TIMEOUT_MS) continue;

        if (pa->retries >= LWS_ACK_RETRIES_MAX) {
            LWS_DEBUG.printf("[LWS] ACK timeout seq=%u\n", pa->seq);
            pa->used = false;
            continue;
        }

        pa->retries++;
        pa->t_sent = now;
        lws_send_frame(LWS_SERIAL, MCU_ID, pa->seq, pa->cmd, pa->data, pa->len);
        LWS_DEBUG.printf("[LWS] retry seq=%u tent=%u\n", pa->seq, pa->retries);
    }
}

static inline void lws_reset() {
    memset(g_pending, 0, sizeof(g_pending));
    lwsParser.reset();
    g_tx_seq = 0;
}

#endif  // COMUNICAZIONI_MCU_H