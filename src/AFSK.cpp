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

extern unsigned long custom_preamble;
extern unsigned long custom_tail;
extern int LibAPRS_vref;


Afsk *AFSK_modem;

// Definición requerida por las macros AFSK_DAC_IRQ_START/STOP en AFSK.h.
// En el puerto AVR original habilitaba el ISR de DAC; aquí solo indica
// si el generador de muestras está activo (no se consulta actualmente,
// pero la variable debe existir para que linkee).
bool hw_afsk_dac_isr = false;

static dac_continuous_handle_t dac_handle;
static adc_continuous_handle_t adc_handle;
static bool dac_enabled = false;

// Cola de tramas TX pendientes: server_task encola, receive_audio_task despacha.
// Así el mutex del ADC siempre lo adquiere y libera la misma tarea.
typedef struct { uint8_t data[AX25_MAX_FRAME_LEN]; size_t len; } afsk_tx_frame_t;
static QueueHandle_t             s_tx_queue = NULL;
static void (*s_tx_fn)(const uint8_t *, size_t) = NULL;

void afsk_set_tx_fn(void (*fn)(const uint8_t *, size_t)) { s_tx_fn = fn; }

void afsk_queue_tx_frame(const uint8_t *data, size_t len) {
    if (!s_tx_queue || !data || len == 0) return;
    afsk_tx_frame_t f;
    if (len > AX25_MAX_FRAME_LEN) len = AX25_MAX_FRAME_LEN;
    memcpy(f.data, data, len);
    f.len = len;
    xQueueSendToBack(s_tx_queue, &f, 0);  // no bloqueante: descarta si llena
}
// true while DAC is active (TX); receive_audio_task pauses during this time.
static volatile bool tx_mode = false;

// Pico absoluto de la última muestra decimada; lo lee y resetea audio_level_task.
volatile int8_t audio_peak = 0;

// Tamaño del chunk DMA leído por iteración. A 48 kHz, 1024 muestras = ~21 ms,
// suficientemente pequeño para latencia baja y suficientemente grande para amortizar
// la sobrecarga de FreeRTOS por `adc_continuous_read`.
#define ADC_FRAME_SIZE        1024
#define ADC_FRAMES_IN_POOL    4
#define ADC_READ_BUF_BYTES    (ADC_FRAME_SIZE * SOC_ADC_DIGI_RESULT_BYTES)


// Forward declerations
int afsk_getchar(void);
void afsk_putchar(char c);
void receive_audio_task(void *arg);

void AFSK_hw_refDetect(void) {
    // TODO quitar
}

// En ESP32 clásico, dac_continuous y adc_continuous comparten el periférico I2S0
// internamente, por lo que no pueden estar activos al mismo tiempo.
// Usamos conmutación half-duplex: el ADC corre durante RX y el DAC durante TX.

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

// Cede I2S0 del ADC al DAC para poder transmitir.
// Llamado siempre desde receive_audio_task (la misma tarea que inició el ADC),
// por lo que adc_continuous_stop respeta el mutex interno de ESP-IDF.
static void switch_to_tx(void) {
    tx_mode = true;
    ESP_LOGI("AFSK", "switch_to_tx: stopping ADC");
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    adc_handle = NULL;

    // Pequeña pausa para que I2S0 se libere completamente antes de que el DAC lo tome.
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

    // Escribir silencio inicial para que los descriptores DMA tengan datos y
    // el oscilador I2S arranque correctamente antes de enviar audio real.
    uint8_t silence[256];
    memset(silence, 128, sizeof(silence));
    size_t bw = 0;
    dac_continuous_write(dac_handle, silence, sizeof(silence), &bw, 200);

    // SIMUL mode configura I2S en estéreo (L→DAC2=GPIO26, R→DAC1=GPIO25).
    // Reafirmar GPIO26 como salida digital para que PTT siga siendo controlable.
    gpio_set_direction(GPIO_PTT_OUT, GPIO_MODE_OUTPUT);
    ESP_LOGI("AFSK", "switch_to_tx: DAC ready");
}

// Devuelve I2S0 al ADC tras finalizar la transmisión.
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
    // PTT como salida (activo alto: 1 = TX, 0 = reposo). Inicialmente en reposo.
    gpio_config_t ptt_cfg = {
        .pin_bit_mask = 1ULL << GPIO_PTT_OUT,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ptt_cfg));
    gpio_set_level(GPIO_PTT_OUT, 0);  // reposo: activo alto → 0 = sin TX

    // La cola de TX se crea aquí; receive_audio_task arranca el ADC y despacha TX.
    s_tx_queue = xQueueCreate(4, sizeof(afsk_tx_frame_t));

    // Tarea de recepción en streaming continuo.
    // Stack 8192: la tarea también despacha TX (ax25_sendRaw + finish_transmission).
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

// Un descriptor DAC = buf_size bytes = 2048/48000 s ≈ 42 ms de audio.
// Con 8 descriptores el DMA tiene ~336 ms de margen frente a preempciones WiFi
// (prio 23 > prio 10 de receive_audio_task). Si TX_SAMPLE_BUFLEN < buf_size el
// descriptor dura menos y el DMA puede quedarse sin datos entre escrituras.
#define TX_SAMPLE_BUFLEN 2048
static uint8_t tx_sample_buf[TX_SAMPLE_BUFLEN];
uint8_t AFSK_dac_isr(Afsk *afsk);

// Genera un bloque de muestras del modem y lo escribe al DAC continuo.
void transmit_audio_i2s(Afsk *afsk) {
    if (!tx_mode) {
        switch_to_tx();
    }
    // DAC ya habilitado en switch_to_tx(); no re-habilitar aquí.
    gpio_set_level(GPIO_PTT_OUT, 1);  // PTT pulsado: activo alto → 1 = TX

    int i = 0;
    for (i = 0; afsk->sending && i < TX_SAMPLE_BUFLEN; i++) {
        tx_sample_buf[i] = AFSK_dac_isr(afsk);
    }
    if (i == 0) return;

    size_t bytes_written = 0;
    // Timeout de 2 s: si el DMA no consume en ese tiempo algo falló en el driver.
    esp_err_t err = dac_continuous_write(
        dac_handle, tx_sample_buf, i, &bytes_written, 2000);
    if (err != ESP_OK || bytes_written == 0) {
        ESP_LOGE("AFSK", "dac_continuous_write error %s (written=%u/%d)",
                 esp_err_to_name(err), (unsigned)bytes_written, i);
        afsk->sending = false;  // abortar TX antes de colgar indefinidamente
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

    // Cola de silencio (valor medio 128 = 0 V tras el acople capacitivo)
    // para permitir que el demodulador externo cierre la trama con limpieza.
    uint8_t silence[256];
    memset(silence, 128, sizeof(silence));
    for (int i = 0; i < 20; i++) {
        size_t bw = 0;
        dac_continuous_write(dac_handle, silence, sizeof(silence), &bw, 500);
    }

    gpio_set_level(GPIO_PTT_OUT, 0);  // PTT liberado: activo alto → 0 = sin TX

    // Libera I2S0 del DAC y reactiva el ADC para volver al modo recepción.
    switch_to_rx();
    ESP_LOGI("AFSK", "custom_preamble=%lu custom_tail=%lu", custom_preamble, custom_tail);
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
        // Aquí se usa un autocorrelador para detectar los tonos
    // usando un retraso de 446 uS
    // 1 segundo son 9600 muestras
    // 1000000/9600 = 104.16666666666667 uS por muestra
    // deberemos aplicar un retraso de 446/104.16666666666667 = 4.28 muestras
    // El retraso se fija en la configuración de la estructura afsk

    // samples per bit = 9600/1200 = 8 -> 8/2 = 4 pero no sé porque lo justifica así

    // >>1 divide por 2, >>2 divide por 4

    // To determine the received frequency, and thereby
    // the bit of the sample, we multiply the sample by
    // a sample delayed by (samples per bit / 2).
    // We then lowpass-filter the samples with a
    // Chebyshev filter. The lowpass filtering serves
    // to "smooth out" the variations in the samples.

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


extern void APRS_poll(void);
// uint8_t poll_timer = 0;
// ISR(ADC_vect) {
//     TIFR1 = _BV(ICF1);
//     AFSK_adc_isr(AFSK_modem, ((int16_t)((ADC) >> 2) - 128));
//     if (hw_afsk_dac_isr) {
//         DAC_PORT = (AFSK_dac_isr(AFSK_modem) & 0xF0) | _BV(3);
//     } else {
//         DAC_PORT = 128;
//     }

//     poll_timer++;
//     if (poll_timer > 3) {
//         poll_timer = 0;
//         APRS_poll();
//     }
// }

// Offset DC estimado con EMA (constante de tiempo ~1024 muestras a 48 kHz ≈ 21 ms).
// Valor inicial: punto medio del ADC de 12 bits = 2048.
// Se mantiene en Q10.10 fixed-point para suavizar sin perder precisión.
static int32_t dc_offset_q10 = 2048 << 10;

// Convierte una muestra cruda del ADC (0..4095) a int8_t centrado y escalado,
// aplicando eliminación DC incremental.
static inline int8_t adc_to_s8(uint16_t raw12) {
    dc_offset_q10 += (raw12 - (dc_offset_q10 >> 10));
    int32_t centered = (int32_t)raw12 - (dc_offset_q10 >> 10);  // ~ -2048..2047
    int32_t s = centered >> 4;                                   // ~ -128..127
    if (s < -128) s = -128;
    else if (s > 127) s = 127;
    return (int8_t)s;
}

// Tarea de recepción en modo streaming continuo: cada muestra decimada pasa
// inmediatamente por el demodulador AFSK, sin acumulaciones ni pausas.
// Esto es lo que requiere el sincronizador HDLC para detectar flags 0x7E
// y mantener la fase a lo largo de toda la trama.
void receive_audio_task(void *arg) {
    uint8_t poll_timer = 0;
    int ov_count = 0;
    int32_t ov_sum = 0;
    static uint8_t read_buf[ADC_READ_BUF_BYTES];

    // El ADC se arranca desde esta tarea para que el mutex interno de ESP-IDF
    // quede asociado a ella. switch_to_tx/rx lo para/arranca siempre desde aquí.
    adc_peripheral_start();

    for (;;) {
        // Despachar trama TX pendiente (encolada por server_task vía afsk_queue_tx_frame).
        // Al ejecutar desde esta tarea, adc_continuous_stop/start respetan el mutex.
        if (!tx_mode && s_tx_queue && s_tx_fn) {
            afsk_tx_frame_t f;
            if (xQueueReceive(s_tx_queue, &f, 0) == pdTRUE) {
                s_tx_fn(f.data, f.len);
            }
        }

        if (tx_mode) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint32_t bytes_read = 0;
        esp_err_t err = adc_continuous_read(adc_handle, read_buf, sizeof(read_buf),
                                            &bytes_read, pdMS_TO_TICKS(20));
        if (err != ESP_OK || bytes_read == 0) {
            continue;
        }

        for (uint32_t i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *d = (adc_digi_output_data_t *)&read_buf[i];
            // Filtrar por canal: en modo TYPE1 (ESP32) d->type1.channel indica el canal.
            if (d->type1.channel != AUDIO_ADC_CHANNEL) continue;

            // Acumular para decimar x OVERSAMPLING (48 kHz → 9600 Hz lógicos).
            ov_sum += d->type1.data;
            if (++ov_count >= OVERSAMPLING) {
                uint16_t avg12 = (uint16_t)(ov_sum / OVERSAMPLING);
                ov_sum = 0;
                ov_count = 0;

                int8_t sample = adc_to_s8(avg12);
                int8_t a = sample < 0 ? -sample : sample;
                if (a > audio_peak) audio_peak = a;
                AFSK_adc_isr(AFSK_modem, sample);

                // APRS_poll procesa el FIFO rxFifo y dispara el callback si llega trama.
                // Se llama cada 4 muestras (~2,4 ms a 9600 Hz) para no saturar el loop.
                if (++poll_timer > 3) {
                    poll_timer = 0;
                    APRS_poll();
                }
            }
        }
        // Cede el CPU al menos 1 tick tras procesar cada trama. Sin esto, la tarea
        // corre en bucle cerrado (el ring buffer del ADC siempre tiene datos) y
        // IDLE0 nunca puede ejecutar su reset del watchdog.
        vTaskDelay(1);
    }
}

