#ifndef AFSK_H
#define AFSK_H

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
// #include <avr/pgmspace.h>
#include "FIFO.h"
#include "HDLC.h"
#include "driver/gpio.h"   // required by the LED_TX/RX macros below

#define TRUE_SIN_LEN 512
#define SIN_LEN (TRUE_SIN_LEN * OVERSAMPLING)
// sin_table[] stores one quarter-period (0°–90°) of an 8-bit unsigned sine wave,
// 128 entries (midpoint = 128, peak = 255). sinSample() reconstructs the full
// 512-sample half-period by mirror symmetry, then applies negation for the second
// half. With OVERSAMPLING=5 the effective table spans 2560 points, providing
// adequate spectral purity for Bell 202 AFSK (mark 1200 Hz / space 2200 Hz).
static const uint8_t sin_table[] =
{
    128, 129, 131, 132, 134, 135, 137, 138, 140, 142, 143, 145, 146, 148, 149, 151,
    152, 154, 155, 157, 158, 160, 162, 163, 165, 166, 167, 169, 170, 172, 173, 175,
    176, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 196, 197,
    198, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215, 217,
    218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233,
    234, 234, 235, 236, 237, 238, 238, 239, 240, 241, 241, 242, 243, 243, 244, 245,
    245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 250, 251, 251, 252, 252, 252,
    253, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255,
};

inline static uint8_t sinSample(uint16_t i) {
    i /= OVERSAMPLING;
    uint16_t newI = i % (TRUE_SIN_LEN/2);
    newI = (newI >= (TRUE_SIN_LEN/4)) ? (TRUE_SIN_LEN/2 - newI -1) : newI;
    uint8_t sine = sin_table[newI];
    return (i >= (TRUE_SIN_LEN/2)) ? (255 - sine) : sine;
}


#define SWITCH_TONE(inc)  (((inc) == MARK_INC) ? SPACE_INC : MARK_INC)
#define BITS_DIFFER(bits1, bits2) (((bits1)^(bits2)) & 0x01)
#define DUAL_XOR(bits1, bits2) ((((bits1)^(bits2)) & 0x03) == 0x03)
#define SIGNAL_TRANSITIONED(bits) DUAL_XOR((bits), (bits) >> 2)
#define TRANSITION_FOUND(bits) BITS_DIFFER((bits), (bits) >> 1)

#define CONFIG_AFSK_RX_BUFLEN 8192
#define CONFIG_AFSK_TX_BUFLEN 8192
#define CONFIG_AFSK_RXTIMEOUT 0
#define CONFIG_AFSK_PREAMBLE_LEN 150UL
#define CONFIG_AFSK_TRAILER_LEN 50UL
#define SAMPLERATE 9600
#define BITRATE    1200
#define SAMPLESPERBIT (SAMPLERATE / BITRATE)
#define SAMPLESPERBIT_TX (CONFIG_AFSK_DAC_SAMPLERATE / BITRATE)
#define BIT_STUFF_LEN 5
#define MARK_FREQ  1200
#define SPACE_FREQ 2200
#define PHASE_BITS   8                              // How much to increment phase counter each sample
#define PHASE_INC    1                              // Nudge by an eigth of a sample each adjustment
#define PHASE_MAX    (SAMPLESPERBIT * PHASE_BITS)   // Resolution of our phase counter = 64
#define PHASE_THRESHOLD  (PHASE_MAX / 2)            // Target transition point of our phase window


typedef struct Hdlc
{
    uint8_t demodulatedBits;
    uint8_t bitIndex;
    uint8_t currentByte;
    bool receiving;
} Hdlc;

typedef struct Afsk
{
    // Stream access to modem
    FILE fd;

    // General values
    Hdlc hdlc;                              // We need a link control structure
    uint16_t preambleLength;                // Length of sync preamble
    uint16_t tailLength;                    // Length of transmission tail

    // Modulation values
    uint8_t sampleIndex;                    // Current sample index for outgoing bit
    uint8_t currentOutputByte;              // Current byte to be modulated
    uint8_t txBit;                          // Mask of current modulated bit
    bool bitStuff;                          // Whether bitstuffing is allowed

    uint8_t bitstuffCount;                  // Counter for bit-stuffing

    uint16_t phaseAcc;                      // Phase accumulator
    uint16_t phaseInc;                      // Phase increment per sample

    FIFOBuffer txFifo;                      // FIFO for transmit data
    uint8_t txBuf[CONFIG_AFSK_TX_BUFLEN];   // Actial data storage for said FIFO

    volatile bool sending;                  // Set when modem is sending

    // Demodulation values
    FIFOBuffer delayFifo;                   // Delayed FIFO for frequency discrimination
    int8_t delayBuf[SAMPLESPERBIT / 2 + 1]; // Actual data storage for said FIFO

    FIFOBuffer rxFifo;                      // FIFO for received data
    uint8_t rxBuf[CONFIG_AFSK_RX_BUFLEN];   // Actual data storage for said FIFO

    int16_t iirX[2];                        // IIR Filter X cells
    int16_t iirY[2];                        // IIR Filter Y cells

    uint8_t sampledBits;                    // Bits sampled by the demodulator (at ADC speed)
    int8_t currentPhase;                    // Current phase of the demodulator
    uint8_t actualBits;                     // Actual found bits at correct bitrate

    volatile int status;                    // Status of the modem, 0 means OK

} Afsk;

#define DIV_ROUND(dividend, divisor)  (((dividend) + (divisor) / 2) / (divisor))
#define MARK_INC   (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)MARK_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))
#define SPACE_INC  (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)SPACE_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))

#define AFSK_DAC_IRQ_START()   do { extern bool hw_afsk_dac_isr; hw_afsk_dac_isr = true; } while (0)
#define AFSK_DAC_IRQ_STOP()    do { extern bool hw_afsk_dac_isr; hw_afsk_dac_isr = false; } while (0)
// AVR legacy: configured Port D bits [7:3] as DAC output pins. On ESP32 the
// DAC is managed entirely by the dac_continuous driver. This macro is defined
// for compatibility but is never called.
#define AFSK_DAC_INIT()        do { /* no-op on ESP32 */ } while (0)

// Optional TX/RX LED indicators driven by configurable GPIO pins.
// Call AFSK_set_leds() after APRS_init() to enable; pass -1 to disable either.
// Defaults: both disabled (-1), preserving previous no-op behaviour.
extern int s_led_tx_gpio;
extern int s_led_rx_gpio;

// LED_INIT macros are no-ops: GPIO configuration is done in AFSK_set_leds().
#define LED_TX_INIT() do { } while (0)
#define LED_TX_ON()   do { if (s_led_tx_gpio >= 0) gpio_set_level((gpio_num_t)s_led_tx_gpio, 1); } while (0)
#define LED_TX_OFF()  do { if (s_led_tx_gpio >= 0) gpio_set_level((gpio_num_t)s_led_tx_gpio, 0); } while (0)

#define LED_RX_INIT() do { } while (0)
#define LED_RX_ON()   do { if (s_led_rx_gpio >= 0) gpio_set_level((gpio_num_t)s_led_rx_gpio, 1); } while (0)
#define LED_RX_OFF()  do { if (s_led_rx_gpio >= 0) gpio_set_level((gpio_num_t)s_led_rx_gpio, 0); } while (0)

#ifdef __cplusplus
extern "C"
{
#endif

void AFSK_init(Afsk *afsk);
void AFSK_transmit(char *buffer, size_t size);
void AFSK_poll(Afsk *afsk);
void AFSK_adc_isr(Afsk *afsk, int8_t currentSample);

void finish_transmission();

void afsk_putchar(char c);
int afsk_getchar(void);

// TX dispatch: set the function that receive_audio_task will call to send a
// frame. Must be called before APRS_set_raw_hook() to avoid cross-task ADC
// mutex conflicts (server_task enqueues; receive_audio_task dispatches).
void afsk_set_tx_fn(void (*fn)(const uint8_t *, size_t));
void afsk_queue_tx_frame(const uint8_t *data, size_t len);

// Configure optional TX and RX LED indicator GPIOs.
// Pass a valid GPIO number (>= 0) to enable; pass -1 to disable.
// Must be called after APRS_init(). Defaults to -1 (both LEDs disabled).
void AFSK_set_leds(int gpio_tx, int gpio_rx);

// RX audio hook: called with each decimated logical sample (int8_t, 9600 Hz).
// Allows attaching external tasks (e.g. audio streaming) in a non-blocking way.
void afsk_set_audio_hook(void (*fn)(int8_t));

extern Afsk *AFSK_modem;
extern volatile int8_t audio_peak;

#ifdef __cplusplus
} // extern "C"
#endif

#endif
