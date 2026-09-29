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

// Spannungsteiler VMESS (VCC -> R3 -> VMESS -> R4 -> GND), HW v6
// ACHTUNG: Quellimpedanz = R1||R2 = 62,9k. Das ist weit ueber dem, was der
// ADC bei Default-Timing treiben kann -> ADC0 laeuft deshalb langsam
// (PRESC_DIV32) mit verlaengerter Sample-Zeit (SAMPLEN=31, SAMPCAP=1).
#define R_MESS_1 825000.0   // R3
#define R_MESS_2 68100.0    // R4 (68k1 laut BOM)
#define U_ADC_REF 1.1
// Korrekturfaktor fuer die Toleranz der internen 1,1V Referenz (+-3%).
// Ermittlung: U_gemessen(Multimeter) / U_ausgegeben(UART) -> hier eintragen.
// 2026-09-29 auf HW v6 gegen Multimeter geprueft: die UART-Ausgabe stimmt
// auf wenige mV, es ist also keine Korrektur noetig -> bleibt bewusst 1.0.
#define U_CAL 1.0

// Rohwert (auf 10 Bit normiert) -> Batteriespannung in Volt
#define ADC_TO_VOLT(adc)    ( (float)(adc) * (float)(U_ADC_REF / 1024.0) \
                              * (float)((R_MESS_1 + R_MESS_2) / R_MESS_2) \
                              * (float)U_CAL )

enum { BAT_OK = 0, BAT_HALF, BAT_LOW, BAT_EMPTY };

// Max, Min und Hysterese Werte
#define PWM_MAX 100       // Dimm-Level	MAX
#define PWM_70 70
#define PWM_OVERTEMP 30  //			    bei �berTemperatur
#define PWM_30 30
#define PWM_MIN 20        //				bei Unterspannung
#define PWM_AUS 0  //				bei starkter Unterspannung

// Ladeschlußspannung 4,2V, Entladeschlussspannung 2,75V, 3 in Serie
#define V_HALB 10.5
#define V_LEER 9.2  // Akku fast leer Spannung in Volt, ab hier Dimmen
#define V_AUS 8.5   // Akku leer Spannung in Volt

#define OVERTEMP_HIGH   70
#define OVERTEMP_LOW    60

#define DIMSTATE_ADDR 0

#define WAIT_TIME 300   //entprellzeit in ms

// Hardware Pins
#define LED_PORT PORTA
#define LED_PIN 2
#define LED_PIN_bm  PIN2_bm
#define PWM_PORT PORTA
#define PWM_PIN 5
#define MODE_PORT PORTA
#define MODE_PIN_bm PIN4_bm

// HW v6: VMESS haengt an PB5 = AIN8. PB4 ist unbeschaltet.
// Der NTC-Zweig entfaellt, es wird der CPU-interne Sensor benutzt.
#define VMESS_PORT PORTB
#define VMESS_PIN 5
#define VMESS_ADC_CHAN 8

// 1 = einmal pro Messung Rohwert/Spannung/Temperatur auf UART ausgeben
#define DEBUG_UART 1

#define UART_BAUD 9600
#define UART_RX_PORT PORTB
#define UART_RX_PIN 1
#define UART_TX_PORT PORTB
#define UART_TX_PIN 2