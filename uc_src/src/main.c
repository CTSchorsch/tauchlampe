/*
 * Tauchlampe FIX
 * f�r Buck-Regler MAX16820, abgeleitet von SST-90 Special Markus Edition
 *
 * Mikrocontroller Code:
 * Version 5.0
 *
 * History:
 * 10.4.2015	Initial (GS)
 * 30.10.2015	Enable LED dimming: Switch off/on during 2 seconds
 * 				steps through dimlevel PWM_MAX, PWM_OVERTEMP, PWM_MIN
 * 27.11.2015	Dimmfunktion f�r normalen Betrieb nicht gewollt, entfernt
 * 21.12.2016	Aufger�umt und Bugs behoben, Pull-Up am ADC eingang AUS !!!
 * 14.08.2019 Temperaturmessung raus, dafür MODE Umschaltung für Batterie oder Tankbetrieb
 * 27.08.2019 Dimmbar
 */

#define F_CPU	8e6

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>
#include <avr/sleep.h>

// Helper
#define sbi(x,b) x |= (1 << (b))
#define cbi(x,b) x &= ~(1 << (b))
#define DIMSTATE_ADDR 0

// Max, Min und Hysterese Werte
#define PWM_MAX			255		// Dimm-Level	MAX
#define PWM_OVERTEMP	128		//			    bei �berTemperatur
#define PWM_MIN			50		//				bei Unterspannung
#define PWM_70      179
#define PWM_30      77
#define PWM_AUS			0	//				bei starkter Unterspannung
// Ladeschlußspannung 4,2V, Entladeschlussspannung 2,75V, 3 in Serie
#define	V_VOLL			12.0	// Akku voll Spannung in volt
#define V_HALB			10.5
#define	V_LEER			9.5	// Akku fast leer Spannung in Volt, ab hier Dimmen
#define	V_AUS			  8.5		// Akku leer Spannung in Volt Volt

// Umrechnung der Spannung:
// Spannung an Spannungsteiler / 1,1 V * 2^10 = Spannung in AD Einheiten
// Spannungsteiler ist 6k80/(82k5+6k81) = 0,07625

#define R_MESS_1 82500.0
#define R_MESS_2 6810.0
#define U_ADC_REF 1.1
#define SPANNUNG_ZU_ADC(Spannung)	(uint16_t)( ((Spannung * (R_MESS_2/(R_MESS_1+R_MESS_2))) * 1024) / U_ADC_REF )
#define ADC_ZU_SPANNUNG(Adc)		(float)((U_ADC_REF * Adc)/(1024 * (R_MESS_2/(R_MESS_1+R_MESS_2))))

enum { LED_AUS = 0, LED_AN, LED_LANGSAM, LED_SCHNELL};
enum { BAT_OK = 0, BAT_HALF, BAT_LOW, BAT_EMPTY };

// Globale Variabeln
volatile unsigned char batteryStatus=BAT_OK;
volatile unsigned int ms_ticks=0;
volatile unsigned int int_ticks=0;
volatile unsigned char newLevel= PWM_AUS;
volatile unsigned char pwmLevel = PWM_AUS;
volatile unsigned char pressed = 0;
uint32_t eeVersion EEMEM = 0x31;

/**************** ISR Routinen ***************************************/

ISR ( TIM1_COMPA_vect )
{
  ms_ticks++;
  int_ticks++;

  if (int_ticks > 300) PCMSK0 = (1 << PCINT2); //enable for PINA2
  if (int_ticks > 1000) pressed = 0;

  switch (batteryStatus) {
		case BAT_EMPTY:
			if ((ms_ticks % 100) == 0) {
			  PORTB ^= ( 1 << PINB0 ); // LED schnell
			}
			break;
		case BAT_LOW:
			if ((ms_ticks % 1000) == 0) {
			  PORTB ^= ( 1 << PINB0 ); // LED langsam
			}
			break;
		case BAT_HALF:
			cbi(PORTB,PINB0); //LED an
			break;
		case BAT_OK:
			sbi(PORTB,PINB0);//LED aus
			break;
	}
}

ISR ( PCINT0_vect)
{

  if ( ! (PINA & 0x04) ) { //fallende Flanke
    PCMSK0 = 0;
    int_ticks=0;
    if ( (pwmLevel == PWM_AUS) && (pressed == 0)) {
      pressed = 1;
      return;
    }

    switch (pwmLevel) {
      case PWM_AUS: newLevel = PWM_MAX; break;
      case PWM_MAX: newLevel = PWM_70; break;
      case PWM_70: newLevel = PWM_30; break;
      case PWM_30: newLevel = PWM_AUS; break;
      default: newLevel = PWM_AUS;
    }
  }

}

/********** ADC *******************************************************/
/* ADC initialisieren */
void ADC_Init(void)
{
  uint16_t result;

  ADMUX = (1<<REFS1); // interne Referenzspannung (1,1 V) als Refernz f�r den ADC
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);     // Frequenzvorteiler;
  ADCSRA |= (1<<ADEN);                  // ADC aktivieren, single Conversion
  ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung, Dummy Readout
  while (ADCSRA & (1<<ADSC) );        // auf Abschluss der Konvertierung warten
  result = ADCL ;					// Register muss gelesen werden
  result |= (ADCH << 8);
}

/* ADC Einzelmessung*/
uint16_t ADC_Read( uint8_t channel )
{
	uint16_t result;

  ADMUX = (ADMUX & ~(0x3F)) | (channel & 0x3F);
  ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
  while (ADCSRA & (1<<ADSC) );   // auf Abschluss der Konvertierung warten
  result = ADCL ;
  result |= (ADCH << 8);
  return result ;                    // ADC auslesen und zur�ckgeben
}

/* ADC Mehrfachmessung mit Mittelwertbbildung */
/* beachte: Wertebereich der Summenvariablen */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t nsamples )
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < nsamples; ++i ) {
    sum += ADC_Read( channel );
  }
  return (uint16_t)( sum / nsamples );
}

/********** PWM *******************************************************/
/*
 * Inits PWM on Timer 0 and Output Compare Register B
 */
void pwm_init( void )
{
	TCCR0B = 0;  // no clock select -> off
	OCR0B = 0;
	TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // non inverting, fast pwm
}
  nop;
/*
 * Set OCR0B for compare
 */
void pwm_set( uint8_t val )
{
	if ( val > 0 ) {
		OCR0B = val;
	  //TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // non inverting, fast pwm
    TCCR0B = (1 << CS02) ; // clock divider 256: 8 Mhz / 256 / 256 = 122 Hz PWM frequency

	} else {
    OCR0B = 0;
    while (PINA & 0x80); // auf low warten
		TCCR0B = 0; // no clock select -> off
    //TCCR0A = (1 << WGM01) | (1 << WGM00);

	}
}

/********** TICKS ******************************************************
 * Setup timer/counter 1 for 1 ms ticks
 */

 void ticks_init( void )
 {
     TCCR1A = 0;
     //clock divider 64: 8 Mhz / 64 = 8 uS pro timer => 125 f�r 1ms
     //Clear Timer on Compare with OCR1A
     TCCR1B = (1 << WGM12) |(1 << CS11) | (1 << CS10);
     OCR1AH = 0;
	 OCR1AL = 125;
	 TIMSK1 = (1 << OCIE1A); // Enable Capture Interrupt
 }
 /********* LAMPE ******************************************************/

void startup ( uint8_t val )
{
	for (uint8_t i= 0; i < val; i++) {
    if ( newLevel == PWM_AUS ) {
      pwmLevel = PWM_AUS;
      pwm_set(pwmLevel);
      return;
    }
		pwm_set( i );
		_delay_ms(10);
	}
}

uint8_t CheckConditions(void )
{
	uint16_t volt;
	static uint16_t voltmin=1100;
	static uint8_t dimmlevel = PWM_AUS;
	static uint8_t cnt=0;

	volt=ADC_Read_Avg(0,16); // CHannel 0, 16 samples

	//immer die Min Voltage nach einschalten nehmen
	//und 10 Messungen ~ 5 Sekunden warten eh Wert �bernommen wird
    if (volt < voltmin) {
		if ( cnt++ > 10 )
			voltmin = volt;
	} else cnt=0;

	if ( voltmin < SPANNUNG_ZU_ADC(V_AUS) ) {
		batteryStatus = BAT_EMPTY;
		dimmlevel=PWM_AUS;
	} else if ( voltmin < SPANNUNG_ZU_ADC(V_LEER) ) {
		batteryStatus=BAT_LOW;
		dimmlevel=PWM_30;
	} else if ( voltmin < SPANNUNG_ZU_ADC(V_HALB) ) {
		batteryStatus=BAT_HALF;
		dimmlevel=PWM_MAX;
	}  else {
		batteryStatus=BAT_OK;
		dimmlevel=PWM_MAX;
	}
	return dimmlevel;
}

/********** MAIN ******************************************************/

void gotoSleep ( void )
{
    PCMSK0 = (1 << PCINT2); //enable for PINA2
    cbi(ADCSRA,ADEN);
    TCCR1B = 0;
    sbi(PORTB,PINB0);//LED aus
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//    power_all_disable();
    sleep_enable();
    sleep_mode();
    sleep_disable();
  //  power_all_enable();
    sbi(ADCSRA,ADEN);
    TCCR1B = (1 << WGM12) |(1 << CS11) | (1 << CS10);

}

int main (void)
{
  uint8_t startupflag = 0;

	// Set MasterClock to 8MHz
	clock_prescale_set(clock_div_1);
	DDRA = 0x82;		// Port A:  PWM (7) out, VMESS (1) out, rest input
	DDRB = 0xFB; 		//alles Ausgang, bis auf B2 (INT0)
	PORTA = 0x02;  //Externer Pullup an
  pwm_init();
	ADC_Init();
	ticks_init();
  sei();

  if ( PINA & 0x04 ) { //Low bei Tank, High bei Battery
    //battery
    pwmLevel = PWM_AUS;
    pwm_set(pwmLevel);
    GIMSK |= (1<< PCIE0);  //enable Pin Change Interrupt on PORTA
    PCMSK0 = (1 << PCINT2); //enable for PINA2
    gotoSleep();
  } else {
    //tank
    PORTA = 0x00; //externer Pullup aus
    startupflag = 1;
    newLevel = eeprom_read_byte(DIMSTATE_ADDR);
    pwmLevel = newLevel;
    switch ( newLevel ) {
      case PWM_MAX: eeprom_update_byte(DIMSTATE_ADDR, PWM_70); break;
      case PWM_70: eeprom_update_byte(DIMSTATE_ADDR, PWM_30); break;
      case PWM_30: eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX); break;
      default: eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX); break;
    }
    startup( newLevel );
  }
  ms_ticks = 0;
  while(1) {
    //rest dimlevel to maximum
    if ( (ms_ticks > 10000) && (startupflag==1) )  {
      startupflag = 0;
      eeprom_update_byte(DIMSTATE_ADDR, PWM_MAX);
    }

    //Lampe aus
    if (pwmLevel == PWM_AUS) {
      pwm_set(pwmLevel);
      if ( newLevel > PWM_AUS ) {
        pwmLevel = CheckConditions();
        if (pwmLevel > newLevel) pwmLevel = newLevel;  //CheckConditions erlaubt höheren dimmwert
        startup ( pwmLevel );
      }
    //Lampe an
    } else {
      pwmLevel = CheckConditions();
      if (pwmLevel > newLevel) pwmLevel = newLevel;  //CheckConditions erlaubt höheren dimmwert
      pwm_set ( pwmLevel );
      if ( pwmLevel == PWM_AUS ) {
        gotoSleep();
      }
    }
  }
	return 0;
}
