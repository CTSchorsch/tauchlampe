/*
 * Tauchlampe FIX
 * für Buck-Regler MAX16820
 * 
 */
#include <avr/io.h>

#define SW_VERSION_MAJOR    5
#define SW_VERSION_MINOR    1

#define F_CPU   3300000UL // 20Mhz clock speed / 6 prescaler
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define ADC_SHIFT_DIV64 (6)

#define ADC_CHAN_TEMP_INTERNAL  0
#define ADC_CHAN_VCC            1
#define ADC_CHAN_TEMP           2

#define NTC_VALUE_PCB           10000.0
#define NTC_SERIES_R            82500.0
#define NTC_BETA                3950.0
#define NTC_TN                  (273.15 + 25.0)
#define NTC_VOLTAGE(adc_val)    (float)((adc_val*1.1)/1024)
#define NTC_VALUE(adc_val)      (uint16_t)((NTC_SERIES_R*(NTC_VOLTAGE(adc_val)/5.0)) / (1 - (NTC_VOLTAGE(adc_val)/5.0)))
#define NTC_TEMP                (float)((1.0 /  ( (1.0 / NTC_TN) + (1.0 / NTC_BETA) * logf(NTC_VALUE(ADC_VAL[ADC_CHAN_TEMP]) / NTC_VALUE_PCB) ) )-273.15)

#define R_MESS_1 82500.0
#define R_MESS_2 6810.0
#define U_ADC_REF 1.1
#define U_VCC 		            (float)    ( (((ADC_VAL[ADC_CHAN_VCC]) * U_ADC_REF) / 1024) * ((R_MESS_1 + R_MESS_2)/ R_MESS_2) )

enum { LED_AUS = 0, LED_AN, LED_LANGSAM, LED_SCHNELL };
enum { BAT_OK = 0, BAT_HALF, BAT_LOW, BAT_EMPTY };

// Max, Min und Hysterese Werte
#define PWM_MAX 100       // Dimm-Level	MAX
#define PWM_70 70
#define PWM_OVERTEMP 30  //			    bei �berTemperatur
#define PWM_30 30
#define PWM_MIN 20        //				bei Unterspannung
#define PWM_AUS 0  //				bei starkter Unterspannung

// Ladeschlußspannung 4,2V, Entladeschlussspannung 2,75V, 3 in Serie
#define V_VOLL 12.0  // Akku voll Spannung in volt
#define V_HALB 10.5
#define V_LEER 9.5  // Akku fast leer Spannung in Volt, ab hier Dimmen
#define V_AUS 8.0   // Akku leer Spannung in Volt

#define OVERTEMP_HIGH   70
#define OVERTEMP_LOW    60

#define DIMSTATE_ADDR 0


// Helper
#define sbi(x, b) x.OUTSET = (1 << (b))
#define cbi(x, b) x.OUTCLR = (1 << (b))

// Hardware Pins
#define LED_PORT PORTA
#define LED_PIN 2
#define LED_PIN_bm  PIN2_bm
#define PWM_PORT PORTA
#define PWM_PIN 5
#define MODE_PORT PORTA
#define MODE_PIN_bm PIN4_bm

#define VMESS_PORT PORTB
#define VMESS_PIN 4
#define VMESS_ADC_CHAN 9
#define TMESS_PIN PORTB
#define TMESS_PIN 5
#define TMESS_ADC_CHAN 8

#define UART_BAUD 9600
#define UART_RX_PORT PORTB
#define UART_RX_PIN 1
#define UART_TX_PORT PORTB
#define UART_TX_PIN 2