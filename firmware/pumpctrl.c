/*
 * pumpctrl.c
 *
 * Created: 2015/5/16 16:30:00
 *  Author: whowe
 */ 

#define F_CPU 8000000

#include <avr/signature.h>
const char fusedata[] __attribute__ ((section (".fuse"))) =
{0xE2, 0xDF, 0xFF};
const char lockbits[] __attribute__ ((section (".lockbits"))) =
{0xFC};

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

typedef unsigned char byte;

#define LED_OUT_PA 0
#define LED_OUT_PB 1

typedef struct tagLED
{
    byte port;
    byte bit;
} LED;

const LED L1 = { LED_OUT_PB, PORTB0};
const LED L2 = { LED_OUT_PB, PORTB1};
const LED L3 = { LED_OUT_PA, PORTA1};
const LED L4 = { LED_OUT_PA, PORTA2};

#define THROTTLE  (0 << REFS0) | (7 << MUX0)    /* Vcc as Reference, Single-end PA7 */
#define VBAT  (0 << REFS0) | (6 << MUX0)    /* Vcc as Reference, Single-end PA7 */

void init();
void init_port();
void init_adc();
void init_pwm();

void output_pwm(unsigned short pow);

unsigned short read_adc(byte source);

void diagnose();

    
void led_init(LED led);
void led_on(LED led);
void led_off(LED led);


void init()
{
    init_port();
    init_pwm();
    init_adc();
    wdt_enable(WDTO_2S);
}

void init_port()
{
    /* Clear Output */
    PORTA = 0x00;
    PORTB = 0x00;
    
    /* PA[7..0] Input, PB2 as PWM Out, LEDs out */
    DDRA = 0;
    DDRB = 1 << DDB2;
    
    led_init(L1);
    led_init(L2);
    led_init(L3);
    led_init(L4);
}

void init_pwm()
{
    /* Fast PWM */
    TCCR0A = 0 << COM0A0 | 3 << WGM00; /* Not to turn on right now */
    /* Fast PWM, Prescaler div1 */
    TCCR0B = 0 << WGM02 | 1 << CS00;
}

void init_adc()
{
    /* ADC Enable, Prescaler factor 64 */ 
    ADCSRA = (1 << ADEN) | (7 << ADPS0);
    /* Left Adjusted */
    ADCSRB = (1 << ADLAR);
    /* Digital Buffer All Cleared */
    DIDR0 = 0xff;
    /* Initial Input THROTTLE*/
    ADMUX = THROTTLE;
}

unsigned short read_adc(byte source)
{
    ADMUX = source;
    
    /* Give up the first conversion */
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    
    /* Do another conversion */
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    
    return ADCL | (ADCH << 8);
}

void output_pwm(unsigned short pow)
{
    if (pow == 0)
    {
        TCCR0A &= ~(3 << COM0A0); /* Turn off pwm */
        OCR0A = 0;
    }
    else
    {
        TCCR0A |= 2 << COM0A0; /* Turn on pwm */
        OCR0A = pow - 1;
    }
}

void led_init(LED led)
{
    if (led.port == LED_OUT_PA)
        DDRA |= (1 << led.bit);
    else if (led.port == LED_OUT_PB)
        DDRB |= (1 << led.bit);
}

void led_on(LED led)
{
    if (led.port == LED_OUT_PA)
        PORTA |= (1 << led.bit);
    else if (led.port == LED_OUT_PB)
        PORTB |= (1 << led.bit);
}

void led_off(LED led)
{
    if (led.port == LED_OUT_PA)
        PORTA &= ~(1 << led.bit);
    else if (led.port == LED_OUT_PB)
        PORTB &= ~(1 << led.bit);
}


int main(void)
{
    init();

    #ifdef DIAGNOSE
    diagnose();
    #endif

    while (1)
    {
    }
}

#ifdef DIAGNOSE
void diagnose()
{
    output_pwm(0);
    while (1)
    {
        led_on(L1); led_off(L4); _delay_ms(80);
        output_pwm(read_adc(THROTTLE) >> 8);
        
        led_on(L2); led_off(L1); _delay_ms(80);
        output_pwm(read_adc(THROTTLE) >> 8);
        
        led_on(L3); led_off(L2); _delay_ms(80);
        output_pwm(read_adc(THROTTLE) >> 8);
        
        led_on(L4); led_off(L3); _delay_ms(80);
        output_pwm(read_adc(THROTTLE) >> 8);
        
        wdt_reset();
    }
}
#endif
