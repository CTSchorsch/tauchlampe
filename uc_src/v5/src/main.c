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
#include "string.h"
#include "stdbool.h"


/*
    ADC_VAL     Wert
    0           Internal T-Sensor
    1           Eingangsspannung
    2           PCB Temperatur
*/
uint16_t ADC_VAL[2];

uint8_t batteryStatus = BAT_OK;
uint8_t newLevel = PWM_AUS;
uint8_t pwmLevel = PWM_AUS;
bool isOvertemp = false;
uint32_t ms_ticks = 0;
uint32_t but_ticks = 0;
float voltmin = 14.0;
bool pressed = false;

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
				voltmin = 14;
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

void USART0_sendString(char *str)
{
    for(size_t i = 0; i < strlen(str); i++) {
        USART0_sendChar(str[i]);
    }
}

void setPWM(uint8_t level)  //level in Prozent
{
    char buff[100];
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

uint16_t getOnChipTemperature() 
{
    int8_t  sigrow_offset = SIGROW.TEMPSENSE1;
    uint8_t sigrow_gain = SIGROW.TEMPSENSE0;
    
    uint32_t temp = ADC_VAL[ADC_CHAN_TEMP_INTERNAL] - sigrow_offset;
    temp *= sigrow_gain;
    temp += 0x80;
    temp >>= 8;
    return (uint16_t) temp-273;
}

uint8_t CheckConditions(void) 
{
    static uint8_t dimmlevel = PWM_AUS;
    static uint8_t cnt = 0;
    
    // immer die Min Voltage nach einschalten nehmen
    // und 10 Messungen ~ 5 Sekunden warten eh Wert �bernommen wird
    if (U_VCC < voltmin) {
       if (cnt++ > 10) voltmin = U_VCC;
    } else
        cnt = 0;

    //Overtemp geht vor Voltage
    if (isOvertemp ) {
        if (getOnChipTemperature() < OVERTEMP_LOW) {
            isOvertemp = false;
        }
    } else {
        if (getOnChipTemperature() > OVERTEMP_HIGH) {
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
        return PWM_OVERTEMP;
    }

    return dimmlevel;

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
    VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
    ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc;
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
    ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
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
    char buff[128];
    uint8_t a,b;
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
    
    //sprintf(buff,"G-LAMP - SW Version: %d.%d\n",SW_VERSION_MAJOR,SW_VERSION_MINOR);
    //USART0_sendString(buff);

    ms_ticks = 0;
    while (1) {
        if ( ms_ticks % 1000 == 0) {
            a = (uint8_t) U_VCC;
            b = (uint8_t) ((U_VCC - a)*100);
            //sprintf(buff, "Vbat %d.%dV, T-CPU: %d\n",a,b, getOnChipTemperature());
            //USART0_sendString(buff);
        }
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