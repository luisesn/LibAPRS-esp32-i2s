#include <string.h>
#include "AFSK.h"
#include "AX25.h"
#include "LibAPRS.h"
#include "FakeArduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/dac_continuous.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include "morse.h"
#include "rx_stats.h"
#include "aux_config.h"
#include "cJSON.h"

extern unsigned long custom_preamble;
extern unsigned long custom_tail;
extern int LibAPRS_vref;


Afsk *AFSK_modem;
Afsk *AFSK_modem_v2 = NULL;
static Afsk afsk_v2_instance;

// Runtime config for dual modem (read from config.json "rx" section)
static bool s_dual_modem_enabled = false;
static int  s_squelch_threshold  = 64;
static bool s_deemphasis_enabled = false;

// Stats routing pointer: set to &rx_stats_v1 or &rx_stats_v2 before each
// hdlcParse call so the common function can instrument the right counter set.
static RxStats *s_cur_stats = NULL;

// FIR decimation for v2 (Hann window, sum=256, replaces simple average)
static const int16_t fir5[OVERSAMPLING] = { 6, 58, 128, 58, 6 };
static uint16_t v2_fir_buf[OVERSAMPLING];

// V2 bandpass biquad state (1700 Hz centre, Q=2, Q15 coefficients)
static int32_t bp_x1 = 0, bp_x2 = 0, bp_y1 = 0, bp_y2 = 0;

// V2 post-discriminator 2nd-order Butterworth LP state (extra samples v1 doesn't need)
static int32_t s_v2_disc_x0 = 0, s_v2_disc_x1 = 0;
static int32_t s_v2_disc_y0 = 0, s_v2_disc_y1 = 0;
static int32_t s_v2_lp_x2   = 0, s_v2_lp_y2   = 0;

// V2 de-emphasis and AGC state
static int32_t s_v2_deemph_state = 0;
static int32_t dc_offset_v2_q10  = 2048 << 10;
static int32_t agc_env_q6        = 32 << 6;

// V2 squelch power estimator
static int32_t s_v2_squelch_power = 0;

// V2 fast clock-sync state
static int  v2_hdlc_flags_seen = 0;
static bool v2_was_receiving   = false;

// Exported for /api/rx/stats
volatile int32_t afsk_v2_agc_peak     = 0;
volatile bool    afsk_v2_squelch_open = false;

portMUX_TYPE g_fifo_mux = portMUX_INITIALIZER_UNLOCKED;

// Required by the AFSK_DAC_IRQ_START/STOP macros defined in AFSK.h.
// On the original AVR port this flag enabled the DAC ISR in hardware.
// On ESP32 it only tracks whether the sample generator is active; it is
// not currently read by any code path but must exist for the linker.
bool hw_afsk_dac_isr = false;

static dac_continuous_handle_t dac_handle;
static adc_continuous_handle_t adc_handle;
static bool dac_enabled = false;

// TX frame queue design rationale:
// The ESP-IDF adc_continuous driver internally binds a mutex to the FreeRTOS
// task that called adc_continuous_start(). Only that same task can later call
// adc_continuous_stop() without deadlocking. server_task must therefore never
// call APRS_send_raw_frame() directly (which internally calls adc_continuous_stop).
// Instead it enqueues outgoing frames here via afsk_queue_tx_frame(); the
// receive_audio_task dequeues and dispatches them at the top of each loop
// iteration, keeping all ADC stop/start calls in the same task context.
typedef struct { uint8_t data[AX25_MAX_FRAME_LEN]; size_t len; } afsk_tx_frame_t;
static QueueHandle_t             s_tx_queue = NULL;
static void (*s_tx_fn)(const uint8_t *, size_t) = NULL;
static TaskHandle_t              s_aprs_poll_task = NULL;

void afsk_set_tx_fn(void (*fn)(const uint8_t *, size_t)) { s_tx_fn = fn; }

static void (*s_audio_hook)(int8_t) = NULL;
void afsk_set_audio_hook(void (*fn)(int8_t)) { s_audio_hook = fn; }

void afsk_queue_tx_frame(const uint8_t *data, size_t len) {
    if (!s_tx_queue || !data || len == 0) return;
    afsk_tx_frame_t f;
    if (len > AX25_MAX_FRAME_LEN) len = AX25_MAX_FRAME_LEN;
    memcpy(f.data, data, len);
    f.len = len;
    xQueueSendToBack(s_tx_queue, &f, 0);  // non-blocking: drops frame if queue is full
}

extern void APRS_poll(void);

// Runs APRS_poll() outside the RX sample ingestion loop so user callbacks
// cannot directly stall receive_audio_task.
static void aprs_poll_task(void *arg) {
    (void)arg;
    for (;;) {
        APRS_poll();
        vTaskDelay(1);
    }
}

// true while DAC is active (TX); receive_audio_task pauses during this time.
static volatile bool tx_mode = false;

// Absolute peak of the last decimated sample; read and reset by audio_level_task.
volatile int8_t audio_peak = 0;

// DMA read chunk size per iteration. At 48 kHz, 1024 samples ≈ 21 ms:
// small enough for low latency, large enough to amortise the FreeRTOS
// overhead of each adc_continuous_read() call.
#define ADC_FRAME_SIZE        1024
#define ADC_FRAMES_IN_POOL    4
#define ADC_READ_BUF_BYTES    (ADC_FRAME_SIZE * SOC_ADC_DIGI_RESULT_BYTES)


// Forward declarations
int afsk_getchar(void);
void afsk_putchar(char c);
void receive_audio_task(void *arg);

// Optional LED indicator GPIOs (-1 = disabled). Set via AFSK_set_leds().
int s_led_tx_gpio = -1;
int s_led_rx_gpio = -1;

void AFSK_set_leds(int gpio_tx, int gpio_rx) {
    s_led_tx_gpio = gpio_tx;
    s_led_rx_gpio = gpio_rx;
    gpio_config_t led_cfg = {
        .pin_bit_mask = 0,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_tx >= 0) {
        led_cfg.pin_bit_mask = 1ULL << gpio_tx;
        gpio_config(&led_cfg);
        gpio_set_level((gpio_num_t)gpio_tx, 0);
    }
    if (gpio_rx >= 0) {
        led_cfg.pin_bit_mask = 1ULL << gpio_rx;
        gpio_config(&led_cfg);
        gpio_set_level((gpio_num_t)gpio_rx, 0);
    }
}

// On classic ESP32, dac_continuous and adc_continuous both use the I2S0
// peripheral internally for DMA. They cannot be active simultaneously.
// Half-duplex switching is implemented here: the ADC runs during RX and the
// DAC during TX. switch_to_tx() stops and deinits the ADC before creating the
// DAC; switch_to_rx() deletes the DAC before re-creating the ADC.

static void adc_peripheral_start(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = ADC_READ_BUF_BYTES * ADC_FRAMES_IN_POOL,
        .conv_frame_size    = ADC_READ_BUF_BYTES,
        .flags              = {},
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t adc_pattern = {
        .atten     = AUDIO_ADC_ATTEN,
        .channel   = AUDIO_ADC_CHANNEL,
        .unit      = AUDIO_ADC_UNIT,
        .bit_width = AUDIO_ADC_BITWIDTH,
    };
    adc_continuous_config_t dig_cfg = {
        .pattern_num    = 1,
        .adc_pattern    = &adc_pattern,
        .sample_freq_hz = TNC_I2S_SAMPLE_RATE,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

// TX DMA chunk size — must equal dac_continuous buf_size (2048 bytes).
// Each dac_continuous_write() fills exactly one DMA descriptor (≈42 ms at 48 kHz).
// With desc_num=8, the pool is ≈336 ms — enough margin to survive WiFi task
// preemptions (priority 23 vs. receive_audio_task priority 10). Writing less
// than buf_size per call drains a descriptor in <1 ms, starving subsequent
// writes and causing dac_continuous_write() to time out.
#define TX_SAMPLE_BUFLEN 2048

// Releases I2S0 from the ADC and hands it to the DAC for transmission.
// Always called from receive_audio_task (the same task that started the ADC),
// so adc_continuous_stop() respects the ESP-IDF internal per-handle mutex.
static void switch_to_tx(void) {
    tx_mode = true;
    ESP_LOGI("AFSK", "switch_to_tx: stopping ADC");
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    adc_handle = NULL;

    // Brief delay to let I2S0 fully release before the DAC driver claims it.
    vTaskDelay(pdMS_TO_TICKS(20));

    dac_continuous_config_t dac_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = 8,
        .buf_size  = 2048,
        .freq_hz   = CONFIG_AFSK_DAC_SAMPLERATE,
        .offset    = 0,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_cfg, &dac_handle));
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
    dac_enabled = true;

    // Prime the DMA with one full descriptor of silence so the I2S oscillator
    // is running before the first real audio write arrives.
    {
        uint8_t silence[TX_SAMPLE_BUFLEN];
        memset(silence, 128, sizeof(silence));
        size_t bw = 0;
        dac_continuous_write(dac_handle, silence, sizeof(silence), &bw, 500);
    }

    // DAC_CHANNEL_MODE_SIMUL drives I2S in stereo: L-channel → DAC1 (GPIO26),
    // R-channel → DAC0 (GPIO25). GPIO26 doubles as the PTT output. After
    // dac_continuous_new_channels() the pin is reconfigured to analog mode;
    // gpio_set_direction() reclaims it as a digital output so PTT level
    // control works correctly.
    gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT);
    ESP_LOGI("AFSK", "switch_to_tx: DAC ready");
}

// Returns I2S0 to the ADC after transmission completes.
static void switch_to_rx(void) {
    if (dac_enabled) {
        dac_continuous_disable(dac_handle);
        dac_enabled = false;
    }
    ESP_ERROR_CHECK(dac_continuous_del_channels(dac_handle));
    dac_handle = NULL;

    adc_peripheral_start();
    tx_mode = false;
    printf("Switched back to RX mode.\n");
}

void AFSK_hw_init(void) {
    // PTT pin: active-high output (1 = transmitting, 0 = idle). Start idle.
    gpio_config_t ptt_cfg = {
        .pin_bit_mask = 1ULL << GPIO_PTT_OUT,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ptt_cfg));
    gpio_set_level(GPIO_PTT_OUT, 0);  // idle: active-high → 0 = no TX

    // Always enable v1 stats tracking
    s_cur_stats = &rx_stats_v1;

    // Read rx config
    cJSON *cfg = config_get();
    if (cfg) {
        cJSON *rx = cJSON_GetObjectItem(cfg, "rx");
        if (rx) {
            cJSON *it;
            it = cJSON_GetObjectItem(rx, "dual_modem_enabled");
            if (cJSON_IsBool(it)) s_dual_modem_enabled = cJSON_IsTrue(it);
            it = cJSON_GetObjectItem(rx, "squelch_threshold");
            if (cJSON_IsNumber(it)) s_squelch_threshold = (int)it->valuedouble;
            it = cJSON_GetObjectItem(rx, "deemphasis_enabled");
            if (cJSON_IsBool(it)) s_deemphasis_enabled = cJSON_IsTrue(it);
        }
    }

    if (s_dual_modem_enabled) {
        memset(&afsk_v2_instance, 0, sizeof(afsk_v2_instance));
        AFSK_modem_v2 = &afsk_v2_instance;
        AFSK_modem_v2->phaseInc = MARK_INC;
        fifo_init(&AFSK_modem_v2->delayFifo,
                  (uint8_t *)AFSK_modem_v2->delayBuf,
                  sizeof(AFSK_modem_v2->delayBuf));
        fifo_init(&AFSK_modem_v2->rxFifo,
                  AFSK_modem_v2->rxBuf,
                  sizeof(AFSK_modem_v2->rxBuf));
        for (int i = 0; i < SAMPLESPERBIT / 2; i++)
            fifo_push(&AFSK_modem_v2->delayFifo, 0);
        ESP_LOGI("AFSK", "V2 dual modem enabled (squelch=%d deemph=%d)",
                 s_squelch_threshold, (int)s_deemphasis_enabled);
    }

    // TX queue created here; receive_audio_task starts the ADC and dispatches TX.
    s_tx_queue = xQueueCreate(4, sizeof(afsk_tx_frame_t));

    // Keep APRS parsing/callbacks out of the RX hot path.
    xTaskCreate(aprs_poll_task, "aprs_poll_task", 3072, NULL, 9, &s_aprs_poll_task);

    // Continuous-streaming receive task.
    // Stack 8192: the task also handles TX dispatch (ax25_sendRaw + finish_transmission).
    xTaskCreate(receive_audio_task, "receive_audio_task", 8192, NULL, 10, NULL);
}

void AFSK_init(Afsk *afsk) {
    // Allocate modem struct memory
    memset(afsk, 0, sizeof(*afsk));
    AFSK_modem = afsk;
    // Set phase increment
    afsk->phaseInc = MARK_INC;
    // Initialise FIFO buffers
    fifo_init(&afsk->delayFifo, (uint8_t *)afsk->delayBuf, sizeof(afsk->delayBuf));
    fifo_init(&afsk->rxFifo, afsk->rxBuf, sizeof(afsk->rxBuf));
    fifo_init(&afsk->txFifo, afsk->txBuf, sizeof(afsk->txBuf));

    // Fill delay FIFO with zeroes
    for (int i = 0; i<SAMPLESPERBIT / 2; i++) {
        fifo_push(&afsk->delayFifo, 0);
    }

    AFSK_hw_init();

}

static void AFSK_txStart(Afsk *afsk) {
    if (!afsk->sending) {
        afsk->phaseInc = MARK_INC;
        afsk->phaseAcc = 0;
        afsk->bitstuffCount = 0;
        afsk->sending = true;
        LED_TX_ON();
        afsk->preambleLength = DIV_ROUND(custom_preamble * BITRATE, 8000);
        AFSK_DAC_IRQ_START();
    }
    /*ATOMIC_BLOCK(ATOMIC_RESTORESTATE)*/ {
      afsk->tailLength = DIV_ROUND(custom_tail * BITRATE, 8000);
    }

    // printf("AFSK_txStart\n");
}

static uint8_t tx_sample_buf[TX_SAMPLE_BUFLEN];
uint8_t AFSK_dac_isr(Afsk *afsk);

// Generates one block of modem samples and writes it to the continuous DAC.
// Always writes TX_SAMPLE_BUFLEN bytes: pads with silence (0x80) if transmission
// ends before the buffer is full. This keeps the DMA pipeline alive until
// finish_transmission() writes its own silence tail.
void transmit_audio_i2s(Afsk *afsk) {
    if (!tx_mode) {
        switch_to_tx();
    }
    // DAC already enabled by switch_to_tx(); do not re-enable here.
    gpio_set_level(GPIO_PTT_OUT, 1);  // assert PTT: active-high → 1 = TX

    int i = 0;
    for (i = 0; afsk->sending && i < TX_SAMPLE_BUFLEN; i++) {
        tx_sample_buf[i] = AFSK_dac_isr(afsk);
    }
    // Pad the remainder of the descriptor with silence so the DMA does not
    // starve between this write and the silence writes in finish_transmission().
    if (i < TX_SAMPLE_BUFLEN) {
        memset(tx_sample_buf + i, 128, TX_SAMPLE_BUFLEN - i);
    }

    size_t bytes_written = 0;
    esp_err_t err = dac_continuous_write(
        dac_handle, tx_sample_buf, TX_SAMPLE_BUFLEN, &bytes_written, 2000);
    if (err != ESP_OK || bytes_written == 0) {
        ESP_LOGE("AFSK", "dac_continuous_write error %s (written=%u/%d)",
                 esp_err_to_name(err), (unsigned)bytes_written, TX_SAMPLE_BUFLEN);
        afsk->sending = false;
    }
}

void afsk_putchar(char c) {
    AFSK_txStart(AFSK_modem);
    while(fifo_isfull_locked(&AFSK_modem->txFifo)) {
        transmit_audio_i2s(AFSK_modem);
    }
    fifo_push_locked(&AFSK_modem->txFifo, c);
    // transmit_audio_i2s(AFSK_modem);
}

int afsk_getchar(void) {
    if (fifo_isempty_locked(&AFSK_modem->rxFifo)) {
        return EOF;
    } else {
        return fifo_pop_locked(&AFSK_modem->rxFifo);
    }
}

void AFSK_transmit(char *buffer, size_t size) {
    fifo_flush(&AFSK_modem->txFifo);
    for (size_t i = 0; i < size; i++) {
        if (fifo_isfull_locked(&AFSK_modem->txFifo)) {
            transmit_audio_i2s(AFSK_modem);
        }
        afsk_putchar(buffer[i]);
    }
    finish_transmission();
}

void finish_transmission() {
    ESP_LOGI("AFSK", "finish_transmission");
    while (AFSK_modem->sending) {
        transmit_audio_i2s(AFSK_modem);
    }

    // Tail silence: one full descriptor (≈42 ms) gives the remote demodulator
    // time to close the frame. The last sample block already contained silence
    // padding; this write queues one more descriptor into the DMA.
    {
        uint8_t silence[TX_SAMPLE_BUFLEN];
        memset(silence, 128, sizeof(silence));
        size_t bw = 0;
        dac_continuous_write(dac_handle, silence, sizeof(silence), &bw, 2000);
    }

    // Wait for the DMA to drain all queued descriptors before stopping the DAC.
    // dac_continuous_disable() inside switch_to_rx() halts the DAC immediately;
    // without this delay the tail silence is cut short → CRC failure at receiver.
    // Worst case: desc_num(8) × buf_size(2048) / 48000 Hz ≈ 341 ms.
    vTaskDelay(pdMS_TO_TICKS(350));

    gpio_set_level(GPIO_PTT_OUT, 0);  // release PTT: active-high → 0 = idle

    // Return I2S0 to the ADC and resume reception.
    switch_to_rx();
    ESP_LOGI("AFSK", "custom_preamble=%lu custom_tail=%lu", custom_preamble, custom_tail);
}

// ---------------------------------------------------------------------------
// Morse beacon DAC primitives (declared in AFSK.h, called from morse.c)
// Must be invoked from receive_audio_task to respect the I2S0 mutex.
// ---------------------------------------------------------------------------
void afsk_morse_tx_begin(void) {
    switch_to_tx();
    gpio_set_level(GPIO_PTT_OUT, 1);  // PTT active-high
}

void afsk_morse_tx_frame(const uint8_t *buf) {
    size_t bw = 0;
    esp_err_t err = dac_continuous_write(dac_handle, (uint8_t *)buf,
                                         TX_SAMPLE_BUFLEN, &bw, 2000);
    if (err != ESP_OK) {
        ESP_LOGE("AFSK", "morse dac_write: %s", esp_err_to_name(err));
    }
}

void afsk_morse_tx_end(void) {
    // One full descriptor of tail silence so the remote demodulator
    // can settle before PTT drops.
    uint8_t silence[TX_SAMPLE_BUFLEN];
    memset(silence, 128, sizeof(silence));
    size_t bw = 0;
    dac_continuous_write(dac_handle, silence, sizeof(silence), &bw, 2000);
    vTaskDelay(pdMS_TO_TICKS(350));
    gpio_set_level(GPIO_PTT_OUT, 0);  // PTT idle
    switch_to_rx();
}

uint8_t AFSK_dac_isr(Afsk *afsk) {
    if (afsk->sampleIndex == 0) {
        if (afsk->txBit == 0) {
            if (fifo_isempty(&afsk->txFifo) && afsk->tailLength == 0) {
                AFSK_DAC_IRQ_STOP();
                afsk->sending = false;
                LED_TX_OFF();
                return 0;
            } else {
                if (!afsk->bitStuff) afsk->bitstuffCount = 0;
                afsk->bitStuff = true;
                if (afsk->preambleLength == 0) {
                    if (fifo_isempty(&afsk->txFifo)) {
                        afsk->tailLength--;
                        afsk->currentOutputByte = HDLC_FLAG;
                    } else {
                        afsk->currentOutputByte = fifo_pop(&afsk->txFifo);
                    }
                } else {
                    afsk->preambleLength--;
                    afsk->currentOutputByte = HDLC_FLAG;
                }
                if (afsk->currentOutputByte == AX25_ESC) {
                    if (fifo_isempty(&afsk->txFifo)) {
                        AFSK_DAC_IRQ_STOP();
                        afsk->sending = false;
                        LED_TX_OFF();
                        return 0;
                    } else {
                        afsk->currentOutputByte = fifo_pop(&afsk->txFifo);
                    }
                } else if (afsk->currentOutputByte == HDLC_FLAG || afsk->currentOutputByte == HDLC_RESET) {
                    afsk->bitStuff = false;
                }
            }
            afsk->txBit = 0x01;
        }

        if (afsk->bitStuff && afsk->bitstuffCount >= BIT_STUFF_LEN) {
            afsk->bitstuffCount = 0;
            afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
        } else {
            if (afsk->currentOutputByte & afsk->txBit) {
                afsk->bitstuffCount++;
            } else {
                afsk->bitstuffCount = 0;
                afsk->phaseInc = SWITCH_TONE(afsk->phaseInc);
            }
            afsk->txBit <<= 1;
        }

        afsk->sampleIndex = SAMPLESPERBIT_TX;
    }

    afsk->phaseAcc += afsk->phaseInc;
    afsk->phaseAcc %= SIN_LEN;
    afsk->sampleIndex--;

    return sinSample(afsk->phaseAcc);
}

static bool hdlcParse(Hdlc *hdlc, bool bit, FIFOBuffer *fifo) {
    // Initialise a return value. We start with the
    // assumption that all is going to end well :)
    bool ret = true;

    // Bitshift our byte of demodulated bits to
    // the left by one bit, to make room for the
    // next incoming bit
    hdlc->demodulatedBits <<= 1;
    // And then put the newest bit from the
    // demodulator into the byte.
    hdlc->demodulatedBits |= bit ? 1 : 0;

    // Now we'll look at the last 8 received bits, and
    // check if we have received a HDLC flag (01111110)
    if (hdlc->demodulatedBits == HDLC_FLAG) {
        if (s_cur_stats) s_cur_stats->hdlc_flags += 1;
        // If we have, check that our output buffer is
        // not full.
        if (!fifo_isfull(fifo)) {
            // If it isn't, we'll push the HDLC_FLAG into
            // the buffer and indicate that we are now
            // receiving data. For bling we also turn
            // on the RX LED.
            fifo_push(fifo, HDLC_FLAG);
            hdlc->receiving = true;
            LED_RX_ON();
        } else {
            // If the buffer is full, we have a problem
            // and abort by setting the return value to
            // false and stopping the here.
            if (s_cur_stats) s_cur_stats->fifo_overflows += 1;
            ret = false;
            hdlc->receiving = false;
            LED_RX_OFF();
        }

        // Everytime we receive a HDLC_FLAG, we reset the
        // storage for our current incoming byte and bit
        // position in that byte. This effectively
        // synchronises our parsing to  the start and end
        // of the received bytes.
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;
        return ret;
    }

    // Check if we have received a RESET flag (01111111)
    // In this comparison we also detect when no transmission
    // (or silence) is taking place, and the demodulator
    // returns an endless stream of zeroes. Due to the NRZ
    // coding, the actual bits send to this function will
    // be an endless stream of ones, which this AND operation
    // will also detect.
    if ((hdlc->demodulatedBits & HDLC_RESET) == HDLC_RESET) {
        // If we have, something probably went wrong at the
        // transmitting end, and we abort the reception.
        hdlc->receiving = false;
        LED_RX_OFF();
        return ret;
    }

    // If we have not yet seen a HDLC_FLAG indicating that
    // a transmission is actually taking place, don't bother
    // with anything.
    if (!hdlc->receiving)
        return ret;

    // First check if what we are seeing is a stuffed bit.
    // Since the different HDLC control characters like
    // HDLC_FLAG, HDLC_RESET and such could also occur in
    // a normal data stream, we employ a method known as
    // "bit stuffing". All control characters have more than
    // 5 ones in a row, so if the transmitting party detects
    // this sequence in the _data_ to be transmitted, it inserts
    // a zero to avoid the receiving party interpreting it as
    // a control character. Therefore, if we detect such a
    // "stuffed bit", we simply ignore it and wait for the
    // next bit to come in.
    //
    // We do the detection by applying an AND bit-mask to the
    // stream of demodulated bits. This mask is 00111111 (0x3f)
    // if the result of the operation is 00111110 (0x3e), we
    // have detected a stuffed bit.
    if ((hdlc->demodulatedBits & 0x3f) == 0x3e)
        return ret;

    // If we have an actual 1 bit, push this to the current byte
    // If it's a zero, we don't need to do anything, since the
    // bit is initialized to zero when we bitshifted earlier.
    if (hdlc->demodulatedBits & 0x01)
        hdlc->currentByte |= 0x80;

    // Increment the bitIndex and check if we have a complete byte
    if (++hdlc->bitIndex >= 8) {
        // If we have a HDLC control character, put a AX.25 escape
        // in the received data. We know we need to do this,
        // because at this point we must have already seen a HDLC
        // flag, meaning that this control character is the result
        // of a bitstuffed byte that is equal to said control
        // character, but is actually part of the data stream.
        // By inserting the escape character, we tell the protocol
        // layer that this is not an actual control character, but
        // data.
        if ((hdlc->currentByte == HDLC_FLAG ||
             hdlc->currentByte == HDLC_RESET ||
             hdlc->currentByte == AX25_ESC)) {
            // We also need to check that our received data buffer
            // is not full before putting more data in
            if (!fifo_isfull(fifo)) {
                fifo_push(fifo, AX25_ESC);
            } else {
                // If it is, abort and return false
                if (s_cur_stats) s_cur_stats->fifo_overflows += 1;
                hdlc->receiving = false;
                LED_RX_OFF();
                ret = false;
            }
        }

        // Push the actual byte to the received data FIFO,
        // if it isn't full.
        if (!fifo_isfull(fifo)) {
            fifo_push(fifo, hdlc->currentByte);
        } else {
            // If it is, well, you know by now!
            if (s_cur_stats) s_cur_stats->fifo_overflows += 1;
            hdlc->receiving = false;
            LED_RX_OFF();
            ret = false;
        }

        // Wipe received byte and reset bit index to 0
        hdlc->currentByte = 0;
        hdlc->bitIndex = 0;

    } else {
        // We don't have a full byte yet, bitshift the byte
        // to make room for the next bit
        hdlc->currentByte >>= 1;
    }

    //digitalWrite(13, LOW);
    return ret;
}


void AFSK_adc_isr(Afsk *afsk, int8_t currentSample) {
        s_cur_stats = &rx_stats_v1;
        // Frequency discriminator using an autocorrelation technique:
        // multiply the current sample by a sample delayed by SAMPLESPERBIT/2 = 4
        // logical samples. At the 9600 Hz logical rate that is 4/9600 ≈ 416 µs.
        // A positive product indicates a mark tone (1200 Hz); a negative product
        // indicates a space tone (2200 Hz). The result is low-pass filtered with
        // a Chebyshev IIR to smooth out noise between transitions.
        //
        // Logical sample rate : 9600 Hz → sample period ≈ 104 µs
        // Delay               : SAMPLESPERBIT/2 = 4 samples → 4/9600 ≈ 416 µs
        // Samples per bit     : SAMPLERATE / BITRATE = 9600 / 1200 = 8; half = 4
        //
        // >>2 divides by 4 (scales the product to prevent IIR overflow).

    afsk->iirX[0] = afsk->iirX[1];
    afsk->iirX[1] = ((int8_t)fifo_pop(&afsk->delayFifo) * currentSample) >> 2;

    afsk->iirY[0] = afsk->iirY[1];

    afsk->iirY[1] = afsk->iirX[0] + afsk->iirX[1] + (afsk->iirY[0] >> 1); // Chebyshev filter


    // We put the sampled bit in a delay-line:
    // First we bitshift everything 1 left
    afsk->sampledBits <<= 1;
    // And then add the sampled bit to our delay line
    afsk->sampledBits |= (afsk->iirY[1] > 0) ? 1 : 0;

    // Put the current raw sample in the delay FIFO
    fifo_push(&afsk->delayFifo, currentSample);

    // We need to check whether there is a signal transition.
    // If there is, we can recalibrate the phase of our
    // sampler to stay in sync with the transmitter. A bit of
    // explanation is required to understand how this works.
    // Since we have PHASE_MAX/PHASE_BITS = 8 samples per bit,
    // we employ a phase counter (currentPhase), that increments
    // by PHASE_BITS everytime a sample is captured. When this
    // counter reaches PHASE_MAX, it wraps around by modulus
    // PHASE_MAX. We then look at the last three samples we
    // captured and determine if the bit was a one or a zero.
    //
    // This gives us a "window" looking into the stream of
    // samples coming from the ADC. Sort of like this:
    //
    //   Past                                      Future
    //       0000000011111111000000001111111100000000
    //                   |________|
    //                       ||
    //                     Window
    //
    // Every time we detect a signal transition, we adjust
    // where this window is positioned little. How much we
    // adjust it is defined by PHASE_INC. If our current phase
    // phase counter value is less than half of PHASE_MAX (ie,
    // the window size) when a signal transition is detected,
    // add PHASE_INC to our phase counter, effectively moving
    // the window a little bit backward (to the left in the
    // illustration), inversely, if the phase counter is greater
    // than half of PHASE_MAX, we move it forward a little.
    // This way, our "window" is constantly seeking to position
    // it's center at the bit transitions. Thus, we synchronise
    // our timing to the transmitter, even if it's timing is
    // a little off compared to our own.
    if (SIGNAL_TRANSITIONED(afsk->sampledBits)) {
        if (afsk->currentPhase < PHASE_THRESHOLD) {
            afsk->currentPhase += PHASE_INC;
        } else {
            afsk->currentPhase -= PHASE_INC;
        }
    }

    // We increment our phase counter
    afsk->currentPhase += PHASE_BITS;

    // Check if we have reached the end of
    // our sampling window.
    if (afsk->currentPhase >= PHASE_MAX) {
        // If we have, wrap around our phase
        // counter by modulus
        afsk->currentPhase %= PHASE_MAX;

        // Bitshift to make room for the next
        // bit in our stream of demodulated bits
        afsk->actualBits <<= 1;

        // We determine the actual bit value by reading
        // the last 3 sampled bits. If there is two or
        // more 1's, we will assume that the transmitter
        // sent us a one, otherwise we assume a zero
        uint8_t bits = afsk->sampledBits & 0x07;
        if (bits == 0x07 || // 111
            bits == 0x06 || // 110
            bits == 0x05 || // 101
            bits == 0x03    // 011
            ) {
            afsk->actualBits |= 1;
        }

         //// Alternative using five bits ////////////////
         // uint8_t bits = afsk->sampledBits & 0x0f;
         // uint8_t c = 0;
         // c += bits & BV(1);
         // c += bits & BV(2);
         // c += bits & BV(3);
         // c += bits & BV(4);
         // c += bits & BV(5);
         // if (c >= 3) afsk->actualBits |= 1;
        /////////////////////////////////////////////////

        // Now we can pass the actual bit to the HDLC parser.
        // We are using NRZ coding, so if 2 consecutive bits
        // have the same value, we have a 1, otherwise a 0.
        // We use the TRANSITION_FOUND function to determine this.
        //
        // This is smart in combination with bit stuffing,
        // since it ensures a transmitter will never send more
        // than five consecutive 1's. When sending consecutive
        // ones, the signal stays at the same level, and if
        // this happens for longer periods of time, we would
        // not be able to synchronize our phase to the transmitter
        // and would start experiencing "bit slip".
        //
        // By combining bit-stuffing with NRZ coding, we ensure
        // that the signal will regularly make transitions
        // that we can use to synchronize our phase.
        //
        // We also check the return of the Link Control parser
        // to check if an error occured.

        if (!hdlcParse(&afsk->hdlc, !TRANSITION_FOUND(afsk->actualBits), &afsk->rxFifo)) {
            afsk->status |= 1;
            if (fifo_isfull(&afsk->rxFifo)) {
                fifo_flush(&afsk->rxFifo);
                afsk->status = 0;
            }
        }
    }

}


// Estimated DC offset, tracked with an EMA (Exponential Moving Average) with a
// time constant of ~1024 logical samples (at 9600 Hz ≈ 107 ms). The initial
// value is the mid-scale point of the 12-bit ADC (0..4095 → midpoint 2048).
// Stored in Q10.10 fixed-point to avoid floating-point arithmetic on the hot path.
static int32_t dc_offset_q10 = 2048 << 10;

// Converts a raw 12-bit ADC sample (0..4095) to a centred, scaled int8_t,
// removing the DC component incrementally using the EMA above.
// After DC removal the range is roughly ±2048; right-shifting by 4 maps it
// to ±127, fitting the int8_t expected by AFSK_adc_isr.
static inline int8_t adc_to_s8(uint16_t raw12) {
    dc_offset_q10 += (raw12 - (dc_offset_q10 >> 10));
    int32_t centered = (int32_t)raw12 - (dc_offset_q10 >> 10);  // ~ -2048..2047
    int32_t s = centered >> 4;                                   // ~ -128..127
    if (s < -128) s = -128;
    else if (s > 127) s = 127;
    return (int8_t)s;
}

// ─── V2 modem: AGC + filters + improvements ──────────────────────────────────

// AGC-enhanced sample converter for v2. DC removal uses the same EMA approach
// as adc_to_s8; an additional envelope follower normalises the output to ~75 %
// of full scale (±96) regardless of the input signal level.
static inline int8_t adc_to_s8_v2(uint16_t raw12) {
    dc_offset_v2_q10 += (raw12 - (dc_offset_v2_q10 >> 10));
    int32_t centered = (int32_t)raw12 - (dc_offset_v2_q10 >> 10);
    int32_t abs_c = centered < 0 ? -centered : centered;
    // Asymmetric EMA: fast attack (~8 samples), slow decay (~512 samples)
    if ((abs_c << 6) > agc_env_q6)
        agc_env_q6 += ((abs_c << 6) - agc_env_q6) >> 3;
    else
        agc_env_q6 += ((abs_c << 6) - agc_env_q6) >> 9;
    int32_t peak = agc_env_q6 >> 6;
    if (peak < 4)   peak = 4;    // max gain ~24×
    if (peak > 512) peak = 512;  // min gain (ADC full range)
    afsk_v2_agc_peak = peak;
    int32_t s = (centered * 96) / peak;
    if (s < -127) s = -127;
    if (s >  127) s =  127;
    return (int8_t)s;
}

// Bandpass biquad: centre 1700 Hz, Q=2, @ 9600 Hz (Q15)
// Passes the Bell-202 window (1200–2200 Hz), rejects out-of-band noise.
static const int32_t BP_B0 =  14871;
static const int32_t BP_B2 = -14871;
static const int32_t BP_A1 = -12058;
static const int32_t BP_A2 =  17897;

static inline int8_t afsk_bandpass_v2(int8_t in) {
    int32_t x0 = in;
    int32_t y0 = (BP_B0 * x0 + BP_B2 * bp_x2
                  - BP_A1 * bp_y1 - BP_A2 * bp_y2) >> 15;
    bp_x2 = bp_x1; bp_x1 = x0;
    bp_y2 = bp_y1; bp_y1 = y0;
    if (y0 < -128) y0 = -128;
    if (y0 >  127) y0 =  127;
    return (int8_t)y0;
}

// Pre-emphasis filter (1st order high-shelf) that compensates for the
// de-emphasis applied by FM receivers before the discriminator output.
// α ≈ 64/256 (τ ≈ 75 µs at 9600 Hz).  Only active when s_deemphasis_enabled.
static inline int8_t afsk_deemph_v2(int8_t in) {
    s_v2_deemph_state = ((256 - 64) * (int32_t)in + 64 * s_v2_deemph_state) >> 8;
    int32_t out = (int32_t)in - s_v2_deemph_state;
    if (out < -128) out = -128;
    if (out >  127) out =  127;
    return (int8_t)out;
}

// 2nd-order Butterworth LP post-discriminator, cutoff 600 Hz @ 9600 Hz (Q15).
// Replaces the 1st-order Chebyshev used in v1.
static const int32_t LP_B0 =  1048;
static const int32_t LP_B1 =  2097;
static const int32_t LP_B2 =  1048;
static const int32_t LP_A1 = -23148;
static const int32_t LP_A2 =  7339;

// V2 demodulator: same structure as AFSK_adc_isr with the following changes:
//   Phase 1a — 5-bit majority voter (vs 3-bit in v1)
//   Phase 1b — aggressive phase-lock during first 4 HDLC flags
//   Phase 3b — bandpass IIR before the correlator
//   Phase 3c — 2nd-order Butterworth LP after the correlator
//   Phase 5  — optional pre-emphasis
static void afsk_v2_process(Afsk *afsk, int8_t currentSample) {
    s_cur_stats = &rx_stats_v2;

    if (s_deemphasis_enabled)
        currentSample = afsk_deemph_v2(currentSample);

    // Phase 3b: bandpass before correlator
    currentSample = afsk_bandpass_v2(currentSample);

    // Correlator (same as v1)
    s_v2_disc_x0 = s_v2_disc_x1;
    s_v2_disc_x1 = ((int8_t)fifo_pop(&afsk->delayFifo) * currentSample) >> 2;

    // Phase 3c: 2nd-order Butterworth LP (replaces v1 Chebyshev)
    s_v2_disc_y0 = s_v2_disc_y1;
    s_v2_disc_y1 = (LP_B0 * s_v2_disc_x1 + LP_B1 * s_v2_disc_x0 + LP_B2 * s_v2_lp_x2
                    - LP_A1 * s_v2_disc_y0 - LP_A2 * s_v2_lp_y2) >> 15;
    s_v2_lp_x2 = s_v2_disc_x0;
    s_v2_lp_y2 = s_v2_disc_y0;

    afsk->sampledBits <<= 1;
    afsk->sampledBits |= (s_v2_disc_y1 > 0) ? 1 : 0;

    fifo_push(&afsk->delayFifo, currentSample);

    // Phase 1b: faster phase acquisition during preamble (first 4 HDLC flags)
    int phase_inc_v2 = (v2_hdlc_flags_seen < 4) ? 2 : 1;
    if (SIGNAL_TRANSITIONED(afsk->sampledBits)) {
        if (afsk->currentPhase < PHASE_THRESHOLD)
            afsk->currentPhase += phase_inc_v2;
        else
            afsk->currentPhase -= phase_inc_v2;
    }

    afsk->currentPhase += PHASE_BITS;

    if (afsk->currentPhase >= PHASE_MAX) {
        afsk->currentPhase %= PHASE_MAX;
        afsk->actualBits <<= 1;

        // Phase 1a: 5-bit majority voter
        uint8_t bits5 = afsk->sampledBits & 0x1f;
        if (__builtin_popcount(bits5) >= 3)
            afsk->actualBits |= 1;

        bool was_receiving = afsk->hdlc.receiving;
        if (!hdlcParse(&afsk->hdlc, !TRANSITION_FOUND(afsk->actualBits), &afsk->rxFifo)) {
            afsk->status |= 1;
            if (fifo_isfull(&afsk->rxFifo)) {
                fifo_flush(&afsk->rxFifo);
                afsk->status = 0;
            }
        }
        // Reset fast-sync counter when a frame ends
        if (was_receiving && !afsk->hdlc.receiving)
            v2_hdlc_flags_seen = 0;
        // Track flag count for Phase 1b
        v2_was_receiving = afsk->hdlc.receiving;
    }
    (void)v2_was_receiving;
}

int afsk_getchar_v2(void) {
    if (!AFSK_modem_v2 || fifo_isempty_locked(&AFSK_modem_v2->rxFifo))
        return EOF;
    return fifo_pop_locked(&AFSK_modem_v2->rxFifo);
}

// Continuous-streaming receive task: every decimated logical sample is fed
// immediately to the AFSK demodulator without buffering or batching.
// Real-time per-sample processing is required by the HDLC synchroniser to
// detect 0x7E flags and maintain phase lock across the entire frame.
void receive_audio_task(void *arg) {
    int ov_count = 0;
    int32_t ov_sum = 0;
    static uint8_t read_buf[ADC_READ_BUF_BYTES];

    // Start the ADC from this task so that the ESP-IDF internal mutex is bound
    // to it. switch_to_tx/rx always stops/starts the ADC from within this task.
    adc_peripheral_start();

    for (;;) {
        // Dispatch any pending TX frame (queued by server_task via afsk_queue_tx_frame).
        // Running from this task, adc_continuous_stop/start honour the ESP-IDF mutex.
        if (!tx_mode && s_tx_queue && s_tx_fn) {
            afsk_tx_frame_t f;
            if (xQueueReceive(s_tx_queue, &f, 0) == pdTRUE) {
                s_tx_fn(f.data, f.len);
            }
        }

        // Morse beacon dispatch: must run from this task (I2S0 mutex).
        if (!tx_mode) {
            morse_check_and_dispatch();
        }

        if (tx_mode) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 20 ms timeout instead of portMAX_DELAY: allows switch_to_tx() to call
        // adc_continuous_stop() and unblock this read when a TX frame is dispatched.
        // With portMAX_DELAY the task would block indefinitely once the ADC stops.
        uint32_t bytes_read = 0;
        esp_err_t err = adc_continuous_read(adc_handle, read_buf, sizeof(read_buf),
                                            &bytes_read, pdMS_TO_TICKS(20));
        if (err != ESP_OK || bytes_read == 0) {
            continue;
        }

        for (uint32_t i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *d = (adc_digi_output_data_t *)&read_buf[i];
            // Filter by channel: in TYPE1 format (ESP32) d->type1.channel is the ADC channel.
            if (d->type1.channel != AUDIO_ADC_CHANNEL) continue;

            // Accumulate OVERSAMPLING raw samples (48 kHz physical) then average
            // to produce one logical sample at 9600 Hz for the AFSK demodulator.
            // Also fill FIR buffer for v2 (same window, weighted instead of flat).
            if (AFSK_modem_v2)
                v2_fir_buf[ov_count] = d->type1.data;

            ov_sum += d->type1.data;
            if (++ov_count >= OVERSAMPLING) {
                uint16_t avg12 = (uint16_t)(ov_sum / OVERSAMPLING);
                ov_sum = 0;
                ov_count = 0;

                // V1 path (unchanged)
                int8_t sample = adc_to_s8(avg12);
                int8_t a = sample < 0 ? -sample : sample;
                if (a > audio_peak) audio_peak = a;
                AFSK_adc_isr(AFSK_modem, sample);
                if (s_audio_hook) s_audio_hook(sample);

                // V2 path: FIR decimation + AGC + squelch
                if (AFSK_modem_v2) {
                    int32_t acc = 0;
                    for (int k = 0; k < OVERSAMPLING; k++)
                        acc += (int32_t)fir5[k] * v2_fir_buf[k];
                    uint16_t avg12_v2 = (uint16_t)(acc >> 8);

                    int8_t s8_v2 = adc_to_s8_v2(avg12_v2);

                    // Squelch: power EMA on AGC output
                    s_v2_squelch_power +=
                        ((int32_t)s8_v2 * s8_v2 - s_v2_squelch_power) >> 8;
                    afsk_v2_squelch_open =
                        (s_v2_squelch_power > s_squelch_threshold);

                    if (afsk_v2_squelch_open)
                        afsk_v2_process(AFSK_modem_v2, s8_v2);
                }
            }
        }
        // Yield at least 1 tick after each DMA frame. Without this,
        // receive_audio_task (priority 10) would spin continuously — the ADC
        // ring buffer is always non-empty — starving the IDLE task and
        // triggering the 5-second task watchdog.
        vTaskDelay(1);
    }
}

