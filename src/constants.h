// AVR CPU target identifiers — not used on ESP32. Retained so that inherited
// #ifdef guards in device.h compile without modification.
#define m328p  0x01
#define m1284p 0x02
#define m644p  0x03

// Voltage reference selectors passed to APRS_init() as the 'reference' argument.
// Values match the original LibAPRS API; the actual ADC reference is configured
// in device.h via AUDIO_ADC_ATTEN, not via these constants.
#define REF_3V3 0x01
#define REF_5V  0x02
