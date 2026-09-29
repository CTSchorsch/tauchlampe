/*
 * Tauchlampe FIX
 * für Buck-Regler MAX16820
 *
 * CPU:
 * ATTINY1616
 *
 * Mikrocontroller Code:
 * Version 5.1
 *
 *
 * History:
 * 2025-10-24   HW v6.0 ready
 * 2022-09-30   CPU Wechsel für V5 auf ATTINY1616
 * 2022-10-27   NTC Zweig abgeklemmt. Nutze CPU T Sensor
 *              IDLE Stromaufnahme bei 230u
 * 
 */
#include "led_buck_3a.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "util/delay.h"
#include "util/atomic.h"
#include "string.h"
#include "stdbool.h"


/*
    ADC_VAL     Wert
    0           Internal T-Sensor
    1           Eingangsspannung
    2           PCB Temperatur
*/
volatile uint16_t ADC_VAL[2];
// wird vom ADC ISR gesetzt sobald ein kompletter Durchlauf (Temp + VCC)
// vorliegt. Nur dann darf CheckConditions() die Werte auswerten.
volatile bool adc_newData = false;

volatile uint8_t batteryStatus = BAT_OK;
volatile uint8_t newLevel = PWM_AUS;
volatile uint8_t pwmLevel = PWM_AUS;
volatile bool isOvertemp = false;
volatile uint32_t ms_ticks = 0;
volatile uint32_t but_ticks = 0;
// nur von CheckConditions() angefasst, deshalb nicht volatile.
// Reset laeuft ueber das Flag voltminReset (float ist nicht atomar).
static float voltmin = 14.0;
volatile bool voltminReset = false;
volatile bool pressed = false;

//1ms tick
ISR(TCB1_INT_vect)
{
    static uint32_t msec = 0;

    msec++;
    ms_ticks++;
    but_ticks++;
    //starte alle 1000ms ADC Lauf
    if ( (msec%1000) == 0){
        ADC0.COMMAND = ADC_STCONV_bm;
    }

    switch (batteryStatus) {
            case BAT_EMPTY:
                if ((msec % 100) == 0) {
                    LED_PORT.OUTTGL = LED_PIN_bm;
                }
                break;
            case BAT_LOW:
                if ((msec % 800) == 0) {
                    LED_PORT.OUTTGL = LED_PIN_bm;
                }
                break;
            case BAT_HALF:
                LED_PORT.OUTCLR = LED_PIN_bm;
                break;
            case BAT_OK:
                LED_PORT.OUTSET = LED_PIN_bm;
                break;
        }


    //interrupt flag löschen
    TCB1.INTFLAGS = TCB_CAPT_bm;
}

ISR (ADC0_RESRDY_vect) 
{
    static uint8_t chan_sel = 0;
    uint16_t val;
    
    val = ADC0.RES >> ADC_SHIFT_DIV64;
    switch (chan_sel) {
        case ADC_CHAN_TEMP_INTERNAL: 
            ADC_VAL[0] = val;
            ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc;
            chan_sel = ADC_CHAN_VCC;
            ADC0.COMMAND = ADC_STCONV_bm;
            break;
        
        case ADC_CHAN_VCC:
            ADC_VAL[1] = val;
            ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
            chan_sel = ADC_CHAN_TEMP_INTERNAL;
            adc_newData = true;
            break;
    }
}

ISR (PORTA_PORT_vect)
{
    //flags zurücksetzen
    uint8_t flags = PORTA.INTFLAGS;
    static unsigned wait;


    if (! (MODE_PORT.IN & MODE_PIN_bm) ) {
        PORTA.INTFLAGS = flags;   
        return; 
    }
    //fallende Flanke
    if (flags & MODE_PIN_bm) {
        if (pwmLevel == PWM_AUS) {
            if (!pressed) {
                pressed = true;
                but_ticks = 0;
                PORTA.INTFLAGS = flags;
                return;
            } else {
                if (but_ticks < WAIT_TIME) {
                    PORTA.INTFLAGS = flags;
                    return;
                }
                wait = ms_ticks;                
                pressed=false;
              //weiter machen
            }
        } else {
            // letzter klick mehr als 300ms her
            if (ms_ticks - wait > WAIT_TIME) {
                wait = ms_ticks;
            // sonst warten
            } else {
                PORTA.INTFLAGS = flags;
                return;
            }
        }

        switch (pwmLevel) {
            case PWM_AUS:
				//reset minmum voltage
				voltminReset = true;
                newLevel = PWM_MAX;
                break;
            case PWM_MAX:
                newLevel = PWM_70;
                break;
            case PWM_70:
                newLevel = PWM_30;
                break;
            case PWM_30:
                newLevel = PWM_AUS; 
                break;
            default:
                newLevel = PWM_AUS;
        }
    }
    //lösche interrupt flag
    PORTA.INTFLAGS = flags;
}

void USART0_sendChar(char c) 
{
    while (!(USART0.STATUS & USART_DREIF_bm));

    USART0.TXDATAL = c;
}

void USART0_sendString(const char *str)
{
    for(size_t i = 0; i < strlen(str); i++) {
        USART0_sendChar(str[i]);
    }
}

#if DEBUG_UART
// vorzeichenbehaftete Dezimalausgabe ohne printf (spart Flash und Stack)
static void USART0_sendInt(int32_t v)
{
    char buf[12];
    uint8_t i = 0;

    if (v < 0) {
        USART0_sendChar('-');
        v = -v;
    }
    do {
        buf[i++] = '0' + (char)(v % 10);
        v /= 10;
    } while (v);
    while (i) USART0_sendChar(buf[--i]);
}

// Ausgabe z.B.:  ADC=766 U=10802mV Umin=10750mV T=41C
void USART0_sendMeasurement(uint16_t raw, float u, float umin, int16_t temp)
{
    USART0_sendString("ADC=");
    USART0_sendInt(raw);
    USART0_sendString(" U=");
    USART0_sendInt((int32_t)(u * 1000.0 + 0.5));
    USART0_sendString("mV Umin=");
    USART0_sendInt((int32_t)(umin * 1000.0 + 0.5));
    USART0_sendString("mV T=");
    USART0_sendInt(temp);
    USART0_sendString("C\r\n");
}
#endif

void setPWM(uint8_t level)  //level in Prozent
{
    uint8_t val = (uint8_t)((255 * level)/100);

    if (level == 0) {
        TCB0.CTRLB &= ~TCB_CCMPEN_bm;
        TCB0.CTRLA &= ~TCB_ENABLE_bm;
        PORTA_OUT &= ~PIN5_bm;
    } else if (level == 100) {
        TCB0.CTRLB &= ~TCB_CCMPEN_bm;
        TCB0.CTRLA &= ~TCB_ENABLE_bm;
        PORTA_OUT |= PIN5_bm;
    } else {
        TCB0.CTRLB |= TCB_CCMPEN_bm;
        TCB0.CCMP = (val << 8) | 0xff;
        TCB0.CTRLA |= TCB_ENABLE_bm;
    }
}

void startup(uint8_t val) 
{
    //disable Portchange interrupt during startup
    PORTA.PIN4CTRL &= ~PORT_ISC_BOTHEDGES_gc;
    for (uint8_t i = 0; i < val; i++) {
        if (newLevel == PWM_AUS) {
            pwmLevel = PWM_AUS;
            setPWM(pwmLevel);
            //reactivate Interrupt
            PORTA.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc;
            return;
        }
        setPWM(i);
        _delay_ms(10);
    }
    PORTA.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc;
}

// Rohwert wird uebergeben, damit der Wert nicht waehrend der Rechnung
// vom ADC ISR ueberschrieben werden kann.
int16_t getOnChipTemperature(uint16_t adc_raw)
{
    int8_t  sigrow_offset = SIGROW.TEMPSENSE1;
    uint8_t sigrow_gain = SIGROW.TEMPSENSE0;

    uint32_t temp = adc_raw - sigrow_offset;
    temp *= sigrow_gain;
    temp += 0x80;
    temp >>= 8;                 // temp ist jetzt Kelvin

    // ohne Clamp wuerde ein Wert < 273K (z.B. vor der ersten Messung)
    // unterlaufen und faelschlich Uebertemperatur ausloesen
    if (temp < 273) return -273;
    return (int16_t)(temp - 273);
}

uint8_t CheckConditions(void)
{
    // Default MAX: solange keine gueltige Messung vorliegt darf nicht
    // gedimmt werden (vorher stand hier PWM_AUS)
    static uint8_t dimmlevel = PWM_MAX;
    // zuletzt zurueckgegebenes Ergebnis inkl. Uebertemperatur-Begrenzung
    static uint8_t result = PWM_MAX;
    static uint8_t cnt = 0;
    uint16_t adc_vcc, adc_temp;
    int16_t temperature;
    float u_vcc;

    if (voltminReset) {
        voltminReset = false;
        voltmin = 14.0;
        cnt = 0;
    }

    // CheckConditions() wird in der Hauptschleife sehr oft pro Sekunde
    // aufgerufen, der ADC liefert aber nur 1x/s. Ohne diese Abfrage lief
    // der Zaehler unten in Mikrosekunden durch und ein einziger zu
    // niedriger Messwert (Lastspitze, Einschaltstrom) wurde sofort und
    // dauerhaft als voltmin uebernommen -> Lampe dimmt und kommt nicht
    // wieder hoch.
    if (!adc_newData) {
        return result;
    }

    // 16 Bit Werte atomar uebernehmen, sonst kann der ADC ISR zwischen
    // Low- und High-Byte zuschlagen und einen Muellwert liefern
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        adc_temp = ADC_VAL[ADC_CHAN_TEMP_INTERNAL];
        adc_vcc  = ADC_VAL[ADC_CHAN_VCC];
        adc_newData = false;
    }

    u_vcc = ADC_TO_VOLT(adc_vcc);
    temperature = getOnChipTemperature(adc_temp);

#if DEBUG_UART
    USART0_sendMeasurement(adc_vcc, u_vcc, voltmin, temperature);
#endif

    // immer die Min Voltage nach einschalten nehmen
    // und 10 Messungen ~ 10 Sekunden warten eh Wert uebernommen wird
    if (u_vcc < voltmin) {
       if (cnt++ > 10) voltmin = u_vcc;
    } else
        cnt = 0;

    //Overtemp geht vor Voltage
    if (isOvertemp ) {
        if (temperature < OVERTEMP_LOW) {
            isOvertemp = false;
        }
    } else {
        if (temperature > OVERTEMP_HIGH) {
            isOvertemp = true;
        } 
    }
    //Batterieladung prüfen
    if (voltmin < V_AUS) {
        batteryStatus = BAT_EMPTY;
        dimmlevel = PWM_AUS;
    } else if (voltmin < V_LEER) {
        batteryStatus = BAT_LOW;
        dimmlevel = PWM_MIN;
    } else if (voltmin < V_HALB) {
        batteryStatus = BAT_HALF;
        dimmlevel = PWM_MAX;
    } else {
        batteryStatus = BAT_OK;
        dimmlevel = PWM_MAX;
    }
    if (isOvertemp && (dimmlevel > PWM_OVERTEMP)) {
        result = PWM_OVERTEMP;
    } else {
        result = dimmlevel;
    }

    return result;

}

void gotoSleep(void) {

    
    pressed = false;
    PORTA.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc;
    LED_PORT.OUTSET = LED_PIN_bm;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}


void port_init() 
{
    //PORT A Pins
    PORTA.DIR = 0xFF; //all out
    //PWM Pin low
    PORTA_OUT &= ~PIN5_bm;
    PORTA.DIR &= ~PIN4_bm; //Pin 4 Input -> Mode
    
    //UART Config
    PORTB.DIR = 0x1F;  // Pin 5 input

    //Analog in    
    PORTB.PIN5CTRL &= ~PORT_ISC_gm;
    PORTB.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN5CTRL &= ~PORT_PULLUPEN_bm;

    //Port C
    PORTC.DIR = 0xF; //all out
}

void init() 
{
    //set clock to 3.3 MHz (20MHz / 6 Prescaler)
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm);

    //ADC config
    // Der Spannungsteiler hat mit 825k/68k1 eine Quellimpedanz von ~63k.
    // Der ADC laedt bei jedem Sample seinen S&H Kondensator aus diesem
    // Knoten nach. Bei schnellem Takt ergibt das einen mittleren
    // Eingangsstrom, der ueber 63k einen deutlichen Spannungsabfall
    // erzeugt -> der ADC misst systematisch zu wenig und die Lampe dimmt
    // zu frueh. Deshalb: kleiner S&H Kondensator, langsamer ADC Takt und
    // lange Sample-Zeit.
    VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
    // SAMPCAP: laut Datenblatt bei Referenz > 1,0V zu setzen (halbiert C_S&H)
    // PRESC_DIV32: CLK_ADC = 3,3MHz/32 = 103kHz (zulaessig 50k..1,5M)
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_PRESC_DIV32_gc | ADC_REFSEL_INTREF_gc;
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
    ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
    // Einschwingzeit der Referenz und nach Kanalwechsel abwarten
    ADC0.CTRLD = ADC_INITDLY_DLY64_gc;
    // SAMPLEN = 31 -> Sample-Fenster 33 CLK_ADC = 320us.
    // Deckt auch die vom Temperatursensor geforderten min. 32us ab.
    ADC0.SAMPCTRL = 0x1F;
    ADC0.INTCTRL = ADC_RESRDY_bm;  //interrupt activieren
    ADC0.COMMAND = ADC_STCONV_bm;

    //TMR B1 for 1ms ticks
    TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   
    TCB1.INTCTRL = TCB_CAPT_bm;
    TCB1.CCMP = 3370;
    //TMR B0 for PWM
    TCB0.CCMP = 0x80FF;
    TCB0.CTRLA |= TCB_CLKSEL_CLKDIV1_gc;
    TCB0.CTRLB |= TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;


    //UART Config
    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);
    USART0.CTRLB |= USART_TXEN_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_ENABLE_bm;
    
    ADC0.COMMAND = ADC_STCONV_bm;
    _delay_ms(10);
    //Interrupt enable
    sei();    
 
}

void main ()
{
    bool tank_start = false;
    
    port_init();
    init();
  
    //check mode pin
    //HIGH beim starten -> Akkufach
    if (MODE_PORT.IN & MODE_PIN_bm) {
        pwmLevel = PWM_AUS;
        setPWM(pwmLevel);
        //Akkufach hat Taster, interrupt aktivieren
        PORTA.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc;
        gotoSleep();
        

    //LOW beim starten -> Akkutank
    } else {
        tank_start = true;
        newLevel = eeprom_read_byte(DIMSTATE_ADDR);
        if (newLevel == 0) newLevel = PWM_MAX; //first time 
        switch (newLevel) {
            case PWM_MAX:
                eeprom_update_byte(DIMSTATE_ADDR, PWM_70);
                break;
            case PWM_70:
                eeprom_update_byte(DIMSTATE_ADDR, PWM_30);
                break;
            case PWM_30:
                eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX);
                break;
            default:
                eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX);
                break;
        }
        //new Level ist gewünschtes Level, pwmLEvel maximal mögliches
        pwmLevel = CheckConditions();
        if (pwmLevel >= newLevel)
            pwmLevel = newLevel;
        startup(newLevel);
    }
    
    ms_ticks = 0;
    while (1) {
        if ((ms_ticks > 10000) && tank_start) {
            tank_start = false;
            eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX);
        }
        //Lampe aus
        if (pwmLevel == PWM_AUS) {
            if (ms_ticks < WAIT_TIME*2) {
                pressed = 0;
                continue; // entprellen des letzten klicks vor aus, damit doppelclick für an geht
            } 
            if (but_ticks > 1100)  //Zweiter Klick kam nicht
            gotoSleep();
          
            if (newLevel > PWM_AUS) {
                pwmLevel = CheckConditions();
                if (pwmLevel >= newLevel)
                    pwmLevel = newLevel;  // CheckConditions erlaubt höheren dimmwert
                startup(pwmLevel);
            }
        // Lampe an
        } else {
            if (newLevel > PWM_AUS) {
                pwmLevel = CheckConditions(); 
                if (pwmLevel >= newLevel)
                    pwmLevel = newLevel;  // CheckConditions erlaubt höheren
                                        // dimmwert
            } else {
                pwmLevel = newLevel;
                ms_ticks = 0;
            }
            setPWM(pwmLevel);
        }
     }

}