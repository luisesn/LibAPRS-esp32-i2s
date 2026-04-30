// #include "FakeArduino.h"
#include <stdint.h>
#include <stdbool.h>

#include "FIFO.h"
#include "CRC-CCIT.h"
#include "HDLC.h"
#include "AFSK.h"
#include "AX25.h"


#ifdef __cplusplus
extern "C"
{
#endif

void APRS_init(int reference, bool open_squelch);
void APRS_poll(void);

// Register frame callbacks after APRS_init().
// Classic APRS mode : APRS_set_msg_hook(aprs_msg_callback)
// KISS TNC mode     : APRS_set_raw_hook(my_raw_callback)
void APRS_set_msg_hook(ax25_callback_t hook);
void APRS_set_raw_hook(ax25_raw_callback_t hook);

// Transmit a pre-built raw AX.25 frame over the air (KISS TNC mode).
void APRS_send_raw_frame(const uint8_t *buf, size_t len);

void APRS_setCallsign(char *call, int ssid);
void APRS_setDestination(char *call, int ssid);
void APRS_setMessageDestination(char *call, int ssid);
void APRS_setPath1(char *call, int ssid);
void APRS_setPath2(char *call, int ssid);

void APRS_setPreamble(unsigned long pre);
void APRS_setTail(unsigned long tail);
void APRS_useAlternateSymbolTable(bool use);
void APRS_setSymbol(char sym);

void APRS_setLat(char *lat);
void APRS_setLon(char *lon);
void APRS_setPower(int s);
void APRS_setHeight(int s);
void APRS_setGain(int s);
void APRS_setDirectivity(int s);

void APRS_sendPkt(void *_buffer, size_t length);
void APRS_sendLoc(void *_buffer, size_t length);
void APRS_sendMsg(void *_buffer, size_t length);
void APRS_msgRetry();
// Encode and queue an APRS message frame for TX without blocking the calling
// task. Safe to call from any task (e.g. HTTP handler). Uses afsk_queue_tx_frame
// internally so it works correctly in KISS TNC mode.
// to_ssid: 0-15, or -1 to omit the SSID suffix from the destination field.
void APRS_queue_msg(const char *to_call, int to_ssid, const char *text);
// Queue an APRS ACK for a received message. msg_id is the numeric ID string
// from the incoming {NNN} field (e.g. "042"). Builds ":DEST     :ackNNN".
void APRS_queue_ack(const char *to_call, int to_ssid, const char *msg_id);
// Copy the active callsign and SSID into caller-supplied buffers.
// buf_out must be at least 7 bytes; either pointer may be NULL.
void APRS_getCallsign(char *buf_out, int *ssid_out);
// Queue a raw APRS beacon (position/status/weather). info must be the
// complete APRS info field starting with '!', '=', '>', etc.
// Sends as a UI frame to the configured DST/PATH (broadcast). Safe from any task.
void APRS_queue_beacon(const char *info);
void APRS_printSettings();


#ifdef __cplusplus
}
#endif