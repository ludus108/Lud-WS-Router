#ifndef COMUNICAZIONI_H
#define COMUNICAZIONI_H

/*
 * ============================================================================
 *  comunicazioni.h — LWSv1.1 (LUD-WS Serial Protocol v1.1) — LATO MCU
 * ============================================================================
 *
 *  Questo file è il gemello di comunicazioni.h lato Display. Va incluso nel
 *  progetto di ogni nodo (Router, Synth A1/A2/A3, Synth B, Ctrl, Mod, Teensy,
 *  Power) e usa lo STESSO serial_protocol.h condiviso.
 *
 *  CHANGELOG rispetto a LWSv1:
 *  ---------------------------------------------------------------------------
 *  [10.1] FIX  : ID_POWER aggiunto alla mappa MCU (lato Display). Lato MCU non
 *                cambia nulla, ma il nodo Power ora può essere riconosciuto.
 *
 *  [10.2] ADD  : CRC-8/ATM (poly 0x07, init 0x00) nel frame.
 *                Frame con CRC errato vengono scartati silenziosamente.
 *
 *  [10.3] ADD  : SEQ byte + protocollo ibrido di ACK:
 *                  - CMD_PING      -> risposta PONG (affidabile)
 *                  - CMD_PARAM     -> fire-and-forget (potenziometri, alta freq)
 *                  - CMD_PARAM_REL -> ACK obbligatorio (config, comandi critici)
 *                Coda pending da 8 slot, timeout 300 ms, 3 retry.
 *
 *  Formato frame LWSv1.1 (condiviso con il Display):
 *      [SENDER][SEQ][CMD][LEN][PAYLOAD...][CRC8][&][!]
 *
 *  CONFIGURAZIONE (definisci queste macro PRIMA di includere il file,
 *  oppure modificale qui sotto nel blocco "Configurazione nodo"):
 *      MCU_ID             : ID char del nodo (es. 'a', 'B', 'R', 'M', ...)
 *      LWS_SERIAL         : Stream usato per LWS (default: Serial1)
 *      LWS_DEBUG          : Stream usato per log di debug (default: Serial)
 *
 *  USO TIPICO:
 *
 *      // In setup():
 *      lws_set_callbacks(on_param_set, on_error_received);
 *      LWS_SERIAL.begin(115200);
 *
 *      // Nel loop():
 *      lws_mcu_poll();            // gestisce RX e retry ACK
 *      // ...
 *      if (pot_changed()) {
 *          lws_send_param('A', 'g', cutOff_value);          // fire-and-forget
 *      }
 *      if (save_requested()) {
 *          lws_send_param_reliable('A', 'a', wave_mode);    // con ACK
 *      }
 *
 *  NOTE:
 *      - I byte '&' (0x26) e '!' (0x21) non devono comparire nel payload.
 *      - lws_mcu_poll() DEVE essere chiamata ad ogni iterazione di loop().
 * ============================================================================
 */

#include <Arduino.h>
#include "serial_protocol.h"

// ========================== CONFIGURAZIONE NODO ==========================
// Sovrascrivibili dal progetto che include questo header.

#ifndef MCU_ID
#define MCU_ID 'x'          // <-- OGNI NODO DEVE DEFINIRLO (es. 'a', 'B', 'R')
#endif

#ifndef LWS_SERIAL
#define LWS_SERIAL Serial1
#endif

#ifndef LWS_DEBUG
#define LWS_DEBUG Serial
#endif

// ========================== COMANDI (devono combaciare col Display) ==========================
#define CMD_PING        'p'
#define CMD_PONG        'P'
#define CMD_PARAM       'S'
#define CMD_PARAM_REL   'R'
#define CMD_PARAM_ACK   'A'
#define CMD_GET_PARAM   'G'
#define CMD_ERROR       'E'
#define CMD_STATUS      'Z'

// ID del Display (unico interlocutore del nodo)
#define ID_DISPLAY      'D'

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
// Implementa queste funzioni nel tuo sketch .ino e passale a lws_set_callbacks().
//   on_param : chiamata quando arriva CMD_PARAM o CMD_PARAM_REL dal Display.
//              'target' e' 'A' o 'B', 'key' e' la lettera del parametro,
//              'value' e' il nuovo valore (0..255).
//   on_error : chiamata quando arriva CMD_ERROR (stringa terminata da '\0').

typedef void (*lws_param_cb_t)(char target, char key, uint8_t value);
typedef void (*lws_error_cb_t)(const char *msg);

static lws_param_cb_t g_param_cb = nullptr;
static lws_error_cb_t g_error_cb = nullptr;

static inline void lws_set_callbacks(lws_param_cb_t on_param,
                                     lws_error_cb_t on_error) {
    g_param_cb = on_param;
    g_error_cb = on_error;
}

// ========================== TX ==========================
// Risposta al PING
static void lws_send_pong() {
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PONG, nullptr, 0);
}

// Fire-and-forget: usa per potenziometri, slider, alta frequenza.
// Se un frame si perde, il successivo (fra 33 ms) lo sovrascrive.
//   target : 'A' o 'B'  (banco del synth di appartenenza del nodo)
//   key    : lettera del parametro (vedi mappe in globals.h del Display)
//   value  : 0..255
static void lws_send_param(char target, char key, uint8_t value) {
    uint8_t p[3] = { (uint8_t)target, (uint8_t)key, value };
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PARAM, p, 3);
}

// Reliable: usa per comandi critici (config, load preset, cambio wave mode).
// Entra in coda pending, retry fino a 3 volte, timeout 300 ms.
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

// Errore verso il Display: stringa breve (max 62 char)
static void lws_send_error(const char *msg) {
    uint8_t p[64];
    size_t l = strlen(msg);
    if (l > 62) l = 62;
    p[0] = (uint8_t)ID_DISPLAY;          // target del messaggio
    memcpy(&p[1], msg, l);
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_ERROR, p, 1 + (uint8_t)l);
}

// ACK per un frame CMD_PARAM_REL ricevuto
static void lws_send_ack(uint8_t acked_seq, uint8_t acked_cmd) {
    uint8_t p[2] = { acked_seq, acked_cmd };
    lws_send_frame(LWS_SERIAL, MCU_ID, lws_next_seq(), CMD_PARAM_ACK, p, 2);
}

// ========================== RX ==========================
static LwsParser lwsParser;

static void lws_process_frame(const LwsFrame &f) {
    // Accettiamo frame solo dal Display (il nodo non parla con altri nodi)
    if ((char)f.sender != ID_DISPLAY) return;

    switch ((char)f.cmd) {
        case CMD_PING: {
            // Il payload contiene l'ID del target: rispondi solo se sei tu
            if (f.len >= 1 && (char)f.data[0] == MCU_ID) {
                lws_send_pong();
            }
            break;
        }

        case CMD_PARAM: {
            // Fire-and-forget: applica e basta
            if (f.len >= 3 && g_param_cb) {
                g_param_cb((char)f.data[0], (char)f.data[1], f.data[2]);
            }
            break;
        }

        case CMD_PARAM_REL: {
            // Reliable: applica e invia ACK
            if (f.len >= 3) {
                if (g_param_cb) {
                    g_param_cb((char)f.data[0], (char)f.data[1], f.data[2]);
                }
                lws_send_ack(f.seq, f.cmd);
            }
            break;
        }

        case CMD_PARAM_ACK: {
            // ACK di un nostro CMD_PARAM_REL: chiudi il pending
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

        case CMD_ERROR: {
            // Errore dal Display
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

// Da chiamare nel loop() ogni volta che ci sono byte disponibili
static inline void lws_mcu_read() {
    while (LWS_SERIAL.available() > 0) {
        uint8_t b = (uint8_t)LWS_SERIAL.read();
        LwsFrame f;
        if (lwsParser.feed(b, f)) lws_process_frame(f);
    }
}

// ========================== RETRY PENDING ==========================
// Da chiamare nel loop() ad ogni iterazione (dopo lws_mcu_read()).
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

// ========================== UTILITY ==========================
static inline void lws_reset() {
    memset(g_pending, 0, sizeof(g_pending));
    lwsParser.reset();
    g_tx_seq = 0;
}

#endif