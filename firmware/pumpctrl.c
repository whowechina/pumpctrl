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

const LED L1 = { LED_OUT_PA, PORTA4};
const LED L2 = { LED_OUT_PA, PORTA5};
const LED L3 = { LED_OUT_PA, PORTA6};
const LED L4 = { LED_OUT_PA, PORTA7};

#define THROTTLE  (0 << REFS0) | (1 << MUX0)    /* Vcc as Reference, Single-end ADC1 */
#define VBAT  (0 << REFS0) | (0 << MUX0)    /* Vcc as Reference, Single-end ADC0 */

#define BATT_LEVEL_BOOT 630
#define BATT_LEVEL_1 600
#define BATT_LEVEL_2 700
#define BATT_LEVEL_3 790
#define BATT_LEVEL_4 870


void init();
void init_port();
void init_adc();
void init_pwm();

void output_pwm(unsigned short pow);

unsigned short read_adc(byte source);

void diagnose();
void low_batt();

    
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
    
    read_adc(VBAT);
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
    unsigned short vbat;
    unsigned short thr;
    
    init();

    _delay_ms(50);
    
    #ifdef DIAGNOSE
    diagnose();
    #endif

    while (1)
    {
        thr = read_adc(THROTTLE);
        vbat = read_adc(VBAT) >> 6;
        vbat += read_adc(VBAT) >> 6;
        vbat += read_adc(VBAT) >> 6;
        vbat += read_adc(VBAT) >> 6;
        vbat >>= 2;
        
        if (vbat >= BATT_LEVEL_BOOT)
        {
            output_pwm(thr >> 8);
            _delay_ms(100);
        }
        else
        {
            low_batt();
        }
        
        if (vbat >= BATT_LEVEL_4)
            led_on(L4);
        else
            led_off(L4);
        
        if (vbat >= BATT_LEVEL_3)
            led_on(L3);
        else
            led_off(L3);

        if (vbat >= BATT_LEVEL_2)
            led_on(L2);
        else
            led_off(L2);

        if (vbat >= BATT_LEVEL_1)
        {
            led_on(L1);
            output_pwm(thr >> 8);
            _delay_ms(50);
        }            
        else
        {
            led_off(L1);
            low_batt();
        }            

        wdt_reset();
    }
}

void low_batt()
{
    led_off(L1);
    led_off(L2);
    led_off(L3);
    led_off(L4);
    while (1)
    {
        output_pwm(0);
        
        led_on(L1);
        _delay_ms(300);
        
        led_off(L1);
        _delay_ms(300);
        
        wdt_reset();
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
