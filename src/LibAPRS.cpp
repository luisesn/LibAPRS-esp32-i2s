#include "FakeArduino.h"
#include "AFSK.h"
#include "AX25.h"
#include "LibAPRS.h"

// Afsk modem;
AX25Ctx AX25;
static Afsk s_modem;
#define countof(a) sizeof(a)/sizeof(a[0])

int LibAPRS_vref = REF_3V3;
bool LibAPRS_open_squelch = false;

unsigned long custom_preamble = 350UL;
unsigned long custom_tail = 50UL;

AX25Call src;
AX25Call dst;
AX25Call path1;
AX25Call path2;

char CALL[7] = "NOCALL";
int CALL_SSID = 0;
char DST[7] = "APZMDM";
int DST_SSID = 0;
char PATH1[7] = "WIDE1";
int PATH1_SSID = 1;
char PATH2[7] = "WIDE2";
int PATH2_SSID = 2;

AX25Call path[4];

// Location packet assembly fields
char latitude[9];
char longtitude[10];
char symbolTable = '/';
char symbol = 'n';

uint8_t power = 10;
uint8_t height = 10;
uint8_t gain = 10;
uint8_t directivity = 10;
/////////////////////////

// Message packet assembly fields
char message_recip[7];
int message_recip_ssid = -1;

int message_seq = 0;
char lastMessage[67];
size_t lastMessageLen;
bool message_autoAck = false;
/////////////////////////

void APRS_init(int reference, bool open_squelch) {
    LibAPRS_vref = reference;
    LibAPRS_open_squelch = open_squelch;

    AFSK_init(&s_modem);
    ax25_init(&AX25, NULL);  // registrar hook con APRS_set_msg_hook o APRS_set_raw_hook
}

void APRS_poll(void) {
    ax25_poll(&AX25);
}

void APRS_set_msg_hook(ax25_callback_t hook) {
    AX25.hook = hook;
}

void APRS_set_raw_hook(ax25_raw_callback_t hook) {
    AX25.raw_hook = hook;
}

void APRS_send_raw_frame(const uint8_t *buf, size_t len) {
    ax25_sendRaw(&AX25, (void *)buf, len);
    finish_transmission();
}

void APRS_setCallsign(char *call, int ssid) {
    memset(CALL, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        CALL[i] = call[i];
        i++;
    }
    CALL_SSID = ssid;
}

void APRS_setDestination(char *call, int ssid) {
    memset(DST, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        DST[i] = call[i];
        i++;
    }
    DST_SSID = ssid;
}

void APRS_setPath1(char *call, int ssid) {
    memset(PATH1, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        PATH1[i] = call[i];
        i++;
    }
    PATH1_SSID = ssid;
}

void APRS_setPath2(char *call, int ssid) {
    memset(PATH2, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        PATH2[i] = call[i];
        i++;
    }
    PATH2_SSID = ssid;
}

void APRS_setMessageDestination(char *call, int ssid) {
    memset(message_recip, 0, 7);
    int i = 0;
    while (i < 6 && call[i] != 0) {
        message_recip[i] = call[i];
        i++;
    }
    message_recip_ssid = ssid;
}

void APRS_setPreamble(unsigned long pre) {
    custom_preamble = pre;
}

void APRS_setTail(unsigned long tail) {
    custom_tail = tail;
}

void APRS_useAlternateSymbolTable(bool use) {
    if (use) {
        symbolTable = '\\';
    } else {
        symbolTable = '/';
    }
}

void APRS_setSymbol(char sym) {
    symbol = sym;
}

void APRS_setLat(char *lat) {
    memset(latitude, 0, 9);
    int i = 0;
    while (i < 8 && lat[i] != 0) {
        latitude[i] = lat[i];
        i++;
    }
}

void APRS_setLon(char *lon) {
    memset(longtitude, 0, 10);
    int i = 0;
    while (i < 9 && lon[i] != 0) {
        longtitude[i] = lon[i];
        i++;
    }
}

void APRS_setPower(int s) {
    if (s >= 0 && s < 10) {
        power = s;
    }
}

void APRS_setHeight(int s) {
    if (s >= 0 && s < 10) {
        height = s;
    }
}

void APRS_setGain(int s) {
    if (s >= 0 && s < 10) {
        gain = s;
    }
}

void APRS_setDirectivity(int s) {
    if (s >= 0 && s < 10) {
        directivity = s;
    }
}

void APRS_printSettings() {
    Serial.println(F("LibAPRS Settings:"));
    Serial.print(F("Callsign:     ")); Serial.print(CALL); Serial.print(F("-")); Serial.println(CALL_SSID);
    Serial.print(F("Destination:  ")); Serial.print(DST); Serial.print(F("-")); Serial.println(DST_SSID);
    Serial.print(F("Path1:        ")); Serial.print(PATH1); Serial.print(F("-")); Serial.println(PATH1_SSID);
    Serial.print(F("Path2:        ")); Serial.print(PATH2); Serial.print(F("-")); Serial.println(PATH2_SSID);
    Serial.print(F("Message dst:  ")); if (message_recip[0] == 0) { Serial.println(F("N/A")); } else { Serial.print(message_recip); Serial.print(F("-")); Serial.println(message_recip_ssid); }
    Serial.print(F("TX Preamble:  ")); Serial.println(custom_preamble);
    Serial.print(F("TX Tail:      ")); Serial.println(custom_tail);
    Serial.print(F("Symbol table: ")); if (symbolTable == '/') { Serial.println(F("Normal")); } else { Serial.println(F("Alternate")); }
    Serial.print(F("Symbol:       ")); Serial.println(symbol);
    Serial.print(F("Power:        ")); if (power < 10) { Serial.println(power); } else { Serial.println(F("N/A")); }
    Serial.print(F("Height:       ")); if (height < 10) { Serial.println(height); } else { Serial.println(F("N/A")); }
    Serial.print(F("Gain:         ")); if (gain < 10) { Serial.println(gain); } else { Serial.println(F("N/A")); }
    Serial.print(F("Directivity:  ")); if (directivity < 10) { Serial.println(directivity); } else { Serial.println(F("N/A")); }
    Serial.print(F("Latitude:     ")); if (latitude[0] != 0) { Serial.println(latitude); } else { Serial.println(F("N/A")); }
    Serial.print(F("Longtitude:   ")); if (longtitude[0] != 0) { Serial.println(longtitude); } else { Serial.println(F("N/A")); }
}

void APRS_sendPkt(void *_buffer, size_t length) {

    uint8_t *buffer = (uint8_t *)_buffer;

    memcpy(dst.call, DST, 6);
    dst.ssid = DST_SSID;

    memcpy(src.call, CALL, 6);
    src.ssid = CALL_SSID;

    memcpy(path1.call, PATH1, 6);
    path1.ssid = PATH1_SSID;

    memcpy(path2.call, PATH2, 6);
    path2.ssid = PATH2_SSID;

    path[0] = dst;
    path[1] = src;
    path[2] = path1;
    path[3] = path2;

    ax25_sendVia(&AX25, path, countof(path), buffer, length);
}

// Dynamic RAM usage of this function is 30 bytes
void APRS_sendLoc(void *_buffer, size_t length) {
    size_t payloadLength = 20+length;
    bool usePHG = false;
    if (power < 10 && height < 10 && gain < 10 && directivity < 9) {
        usePHG = true;
        payloadLength += 7;
    }
    uint8_t *packet = (uint8_t*)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = '=';
    packet[9] = symbolTable;
    packet[19] = symbol;
    ptr++;
    memcpy(ptr, latitude, 8);
    ptr += 9;
    memcpy(ptr, longtitude, 9);
    ptr += 10;
    if (usePHG) {
        packet[20] = 'P';
        packet[21] = 'H';
        packet[22] = 'G';
        packet[23] = power+48;
        packet[24] = height+48;
        packet[25] = gain+48;
        packet[26] = directivity+48;
        ptr+=7;
    }
    if (length > 0) {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
    }

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

// Dynamic RAM usage of this function is 18 bytes
void APRS_sendMsg(void *_buffer, size_t length) {
    if (length > 67) length = 67;
    size_t payloadLength = 11+length+4;

    uint8_t *packet = (uint8_t*)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = ':';
    int callSize = 6;
    int count = 0;
    while (callSize--) {
        if (message_recip[count] != 0) {
            packet[1+count] = message_recip[count];
            count++;
        }
    }
    if (message_recip_ssid != -1) {
        packet[1+count] = '-'; count++;
        if (message_recip_ssid < 10) {
            packet[1+count] = message_recip_ssid+48; count++;
        } else {
            packet[1+count] = 49; count++;
            packet[1+count] = message_recip_ssid-10+48; count++;
        }
    }
    while (count < 9) {
        packet[1+count] = ' '; count++;
    }
    packet[1+count] = ':';
    ptr += 11;
    if (length > 0) {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
        memcpy(lastMessage, buffer, length);
        lastMessageLen = length;
    }

    message_seq++;
    if (message_seq > 999) message_seq = 0;

    packet[11+length] = '{';
    int n = message_seq % 10;
    int d = ((message_seq % 100) - n)/10;
    int h = (message_seq - d - n) / 100;

    packet[12+length] = h+48;
    packet[13+length] = d+48;
    packet[14+length] = n+48;

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}

void APRS_msgRetry() {
    message_seq--;
    APRS_sendMsg(lastMessage, lastMessageLen);
}

// ─── APRS_queue_msg ───────────────────────────────────────────────────────────
// Builds a complete AX.25 UI frame for an APRS message packet and enqueues it
// for TX via afsk_queue_tx_frame(). This avoids calling AFSK_transmit directly,
// which would violate the adc_continuous task-ownership constraint in KISS mode.

static void encode_call_field(uint8_t *buf, const char *call, uint8_t ssid, bool last)
{
    for (int i = 0; i < 6; i++) {
        buf[i] = (uint8_t)((call[i] ? (uint8_t)call[i] : (uint8_t)' ') << 1);
    }
    buf[6] = (uint8_t)(0x60u | ((ssid & 0x0Fu) << 1) | (last ? 0x01u : 0x00u));
}

// Shared helper: build ":DEST     :info_text" APRS info field, wrap in an
// AX.25 UI frame using the current CALL/PATH settings and queue it for TX.
static void queue_aprs_info(const char *to_call, int to_ssid,
                            const char *info_text, size_t info_len)
{
    if (info_len > 71) info_len = 71;
    // payload: ':' + 9-char addr + ':' + info_text  → max 1+9+1+71 = 82
    uint8_t payload[82];
    size_t pay_len = 0;
    payload[pay_len++] = ':';
    int count = 0;
    int call_len = (int)strlen(to_call);
    if (call_len > 6) call_len = 6;
    for (int i = 0; i < call_len; i++) { payload[pay_len++] = (uint8_t)to_call[i]; count++; }
    if (to_ssid >= 0 && to_ssid <= 15) {
        payload[pay_len++] = '-'; count++;
        if (to_ssid < 10) {
            payload[pay_len++] = (uint8_t)('0' + to_ssid); count++;
        } else {
            payload[pay_len++] = '1'; count++;
            payload[pay_len++] = (uint8_t)('0' + (to_ssid - 10)); count++;
        }
    }
    while (count < 9) { payload[pay_len++] = ' '; count++; }
    payload[pay_len++] = ':';
    memcpy(payload + pay_len, info_text, info_len);
    pay_len += info_len;

    uint8_t frame[28 + 2 + 82];
    encode_call_field(frame +  0, DST,   (uint8_t)DST_SSID,   false);
    encode_call_field(frame +  7, CALL,  (uint8_t)CALL_SSID,  false);
    encode_call_field(frame + 14, PATH1, (uint8_t)PATH1_SSID, false);
    encode_call_field(frame + 21, PATH2, (uint8_t)PATH2_SSID, true);
    frame[28] = AX25_CTRL_UI;
    frame[29] = AX25_PID_NOLAYER3;
    memcpy(frame + 30, payload, pay_len);
    afsk_queue_tx_frame(frame, 30 + pay_len);
}

void APRS_getCallsign(char *buf_out, int *ssid_out)
{
    if (buf_out) { strncpy(buf_out, CALL, 6); buf_out[6] = '\0'; }
    if (ssid_out) *ssid_out = CALL_SSID;
}

void APRS_queue_msg(const char *to_call, int to_ssid, const char *text)
{
    if (!to_call || !text) return;
    size_t text_len = strlen(text);
    if (text_len > 67) text_len = 67;

    // Append {NNN} message sequence number after text
    char info[67 + 4 + 1];
    memcpy(info, text, text_len);
    message_seq++;
    if (message_seq > 999) message_seq = 0;
    info[text_len]     = '{';
    info[text_len + 1] = (char)('0' + message_seq / 100);
    info[text_len + 2] = (char)('0' + (message_seq % 100) / 10);
    info[text_len + 3] = (char)('0' + message_seq % 10);
    queue_aprs_info(to_call, to_ssid, info, text_len + 4);
}

void APRS_queue_ack(const char *to_call, int to_ssid, const char *msg_id)
{
    if (!to_call || !msg_id) return;
    char info[10]; // "ack" + up to 5 digits
    int n = snprintf(info, sizeof(info), "ack%s", msg_id);
    if (n > 0) queue_aprs_info(to_call, to_ssid, info, (size_t)n);
}

void APRS_queue_beacon(const char *info)
{
    if (!info) return;
    size_t info_len = strlen(info);
    if (info_len > 200) info_len = 200;

    // AX.25 UI frame: [DST:7][SRC:7][PATH1:7][PATH2:7][CTRL:1][PID:1][INFO...]
    uint8_t frame[30 + 200];
    encode_call_field(frame +  0, DST,   (uint8_t)DST_SSID,   false);
    encode_call_field(frame +  7, CALL,  (uint8_t)CALL_SSID,  false);
    encode_call_field(frame + 14, PATH1, (uint8_t)PATH1_SSID, false);
    encode_call_field(frame + 21, PATH2, (uint8_t)PATH2_SSID, true);
    frame[28] = AX25_CTRL_UI;
    frame[29] = AX25_PID_NOLAYER3;
    memcpy(frame + 30, info, info_len);
    afsk_queue_tx_frame(frame, 30 + info_len);
}
