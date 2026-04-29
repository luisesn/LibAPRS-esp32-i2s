#include "constants.h"

#ifndef DEVICE_CONFIGURATION
#define DEVICE_CONFIGURATION

// CPU settings
#ifndef F_CPU
    #define F_CPU 16000000
#endif

#ifndef FREQUENCY_CORRECTION
    #define FREQUENCY_CORRECTION 0
#endif


#define GPIO_AUDIO_TRIGGER GPIO_NUM_37
// Audio input pin is controlled by I2S_ADC_CHANNEL below.
// Audio output pin is hardcoded on hardware as GPIO 25
#define GPIO_PTT_OUT GPIO_NUM_26
#define ESP_INTR_FLAG_DEFAULT 0


#define DESIRED_SAMPLE_RATE   (9600)
#define OVERSAMPLING          (5)
// Physical DAC/ADC sample rate = DESIRED_SAMPLE_RATE x OVERSAMPLING = 48 000 Hz.
// receive_audio_task decimates by OVERSAMPLING (averages 5 raw samples) to produce
// one logical sample at 9600 Hz for the AFSK demodulator.
#define TNC_I2S_SAMPLE_RATE        (DESIRED_SAMPLE_RATE * OVERSAMPLING)
#define CONFIG_AFSK_DAC_SAMPLERATE (TNC_I2S_SAMPLE_RATE)

// Audio input: GPIO 35 (ADC1_CHANNEL_7); audio output: GPIO 25 (DAC0).
// ADC1 is used (not ADC2) because ADC2 conflicts with Wi-Fi when Wi-Fi is active.
#define AUDIO_ADC_UNIT            ADC_UNIT_1
#define AUDIO_ADC_CHANNEL         ADC_CHANNEL_7
#define AUDIO_ADC_ATTEN           ADC_ATTEN_DB_12   // range ~0..3.1 V
#define AUDIO_ADC_BITWIDTH        ADC_BITWIDTH_12   // 12-bit: 0..4095

#define KEEP_RECORDING_THRESH  (5)
#endif // DEVICE_CONFIGURATION