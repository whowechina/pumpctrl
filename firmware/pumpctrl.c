/*
 * pumpctrl.c
 *
 * Created: 2015/5/16 16:30:00
 *  Author: whowe
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/signature.h>
#include <avr/eeprom.h>

const char fusedata[] __attribute__ ((section (".fuse"))) =
{0xD2, 0xDF, 0xFF};
const char lockbits[] __attribute__ ((section (".lockbits"))) =
{0xFC};
unsigned char eeprom[] __attribute__ ((section (".eeprom"))) =
{0x00, 0x00, 0x00, 0x00};

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
#define HVOUT  (PORTB1)
#define SW_HV  (PINA3) /* Switch for HV */
#define SW_TRIGGER (PINA2) /* Switch for CAL */

#define EEPROM_ADDR_HV 2
#define EEPROM_ADDR_MODE 3

unsigned char hvout;
unsigned char mode;

#define BATT_LEVEL_BOOT 740
#define BATT_LEVEL_1 720
#define BATT_LEVEL_2 840
#define BATT_LEVEL_3 879
#define BATT_LEVEL_4 922


void init();
void init_port();
void init_adc();
void init_pwm();

void restore_hv();
void backup_hv();
void output_hv();
void disable_hv();
void check_hv();

void output_pwm(unsigned short pow);

unsigned short read_adc(byte source);

void low_batt();
    
void led_init(LED led);
void led_on(LED led);
void led_off(LED led);

#define TRIGGER_ON 0
#define TRIGGER_OFF 1
#define TRIGGER_FIRE 2
#define TRIGGER_RELEASE 3

byte trigger();

void all_off();

void restore_mode();
void backup_mode();
void display_mode();

void run();
void idle();

void init()
{
    init_port();
    init_pwm();
    init_adc();
    restore_hv();
    restore_mode();
    wdt_enable(WDTO_2S);
}

void init_port()
{
    /* Clear Output */
    PORTA = 0x00;
    PORTB = 0x00;
    
    /* PA[7..0] Input, PB2 as PWM Out, PB1 as HV Out, LEDs all out */
    DDRA = 0;
    DDRB = (1 << DDB1) | (1 << DDB2);
    
    led_init(L1);
    led_init(L2);
    led_init(L3);
    led_init(L4);
    
    PORTA = (1 << SW_HV) | (1 << SW_TRIGGER); /* Pull-up HV Switch and CAL Switch Pin */
}

void init_pwm()
{
    /* Fast PWM */
    TCCR0A = 0 << COM0A0 | 3 << WGM00; /* Not to turn on right now */
    /* Fast PWM, Prescaler div 1 */
    TCCR0B = 0 << WGM02 | 1 << CS00;
}

void init_adc()
{
    /* ADC Enable, Prescaler factor 64 */ 
    ADCSRA = (1 << ADEN) | (7 << ADPS0);
    /* Left Adjusted */
    ADCSRB = (1 << ADLAR);
    /* Digital Buffer All Cleared */
    DIDR0 = 1 << ADC0D;
    /* Initial Input THROTTLE*/
    ADMUX = THROTTLE;
    
    read_adc(VBAT);
}

void restore_hv()
{
    hvout = eeprom_read_byte(& eeprom[EEPROM_ADDR_HV]);
}

void toggle_hv()
{
    if (hvout)
        hvout = 0;
    else
        hvout = 0xff;

    output_hv();
    
    eeprom_write_byte(& eeprom[EEPROM_ADDR_HV], hvout);
}

void output_hv()
{
    if (hvout)
        PORTB |= (1 << HVOUT);
    else
        PORTB &= ~(1 << HVOUT);
}

void disable_hv()
{
	PORTB &= ~(1 << HVOUT);
}

void all_off()
{
	disable_hv();
	led_off(L1);
	led_off(L2);
	led_off(L3);
	led_off(L4);
	output_pwm(0);
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

/* 0: min (off), 255: max */
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


void check_hv()
{
    static byte old = 0;
    byte i, hv;

    for (i = 0; i < 10; i ++)
    {
        hv = bit_is_clear(PINA, SW_HV);
        if (hv == old)
            return;
        _delay_ms(1);
    }
    
    old = hv;
    if (old)
    {
        toggle_hv();
    }
}

byte trigger()
{
    static byte old = 0;
    byte i, trg;

    for (i = 0; i < 10; i ++)
    {
	    trg = bit_is_clear(PINA, SW_TRIGGER);
	    if (trg == old)
            return (trg) ? TRIGGER_ON : TRIGGER_OFF;

	    _delay_ms(1);
    }
    old = trg;
    
    return (trg) ? TRIGGER_FIRE : TRIGGER_RELEASE;
}

void restore_mode()
{
    mode = eeprom_read_byte(& eeprom[EEPROM_ADDR_MODE]);
}

void backup_mode()
{
    eeprom_write_byte(& eeprom[EEPROM_ADDR_MODE], mode);
}

void display_mode()
{
    if (mode)
    {
        led_on(L1);
        _delay_ms(100);

        led_off(L1);
        led_on(L2);
        _delay_ms(100);

        led_off(L2);        
        led_on(L3);
        _delay_ms(100);

        led_off(L3);
        led_on(L4);
        _delay_ms(100);

        led_off(L4);
    }
    else
    {
        led_on(L4);
        _delay_ms(100);

        led_off(L4);
        led_on(L3);
        _delay_ms(100);

        led_off(L3);
        led_on(L2);
        _delay_ms(100);

        led_off(L2);
        led_on(L1);
        _delay_ms(100);

        led_off(L1);
    }
}

void idle()
{
    unsigned short toggle_count = 0;

	all_off();
    
	while (1)
	{
		if (trigger() == TRIGGER_FIRE)
			break;

        if (bit_is_clear(PINA, SW_HV))
            toggle_count ++;
        else
            toggle_count = 0;
        
        if (toggle_count > 150) /* about 3 seconds */
        {
            toggle_count = 0;
            mode = ! mode;
            backup_mode();
            display_mode();
        }
        
		_delay_ms(20);
	    wdt_reset();
	}
}

void run()
{
    unsigned short vbat;
    unsigned short thr;
	byte bootcount = 0;

	read_adc(THROTTLE); /* dummy read */
	read_adc(VBAT); /* dummy read */
	
    while (1)
    {
	    check_hv();
	    thr = read_adc(THROTTLE);
	    vbat = read_adc(VBAT) >> 6;
	    vbat += read_adc(VBAT) >> 6;
	    vbat += read_adc(VBAT) >> 6;
	    vbat += read_adc(VBAT) >> 6;
	    vbat >>= 2;
	    
		if (bootcount == 0)
		{
			/* ready to start ? */
			if (vbat >= BATT_LEVEL_BOOT)
			{
				output_pwm(255 - (thr >> 9));
				bootcount ++;
				_delay_ms(50);
			}
			else
			{
				low_batt();
				return;
			}
		}
		else if (bootcount < 10)
		{
			/* starting */
			bootcount ++;
			_delay_ms(50);
		}
		else
		{
			/* started */
			if (vbat >= BATT_LEVEL_BOOT)
			{
				output_hv();
			}
			else
			{
				low_batt();
				return;
			}
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
		    output_pwm(255 - (thr >> 9));
		    _delay_ms(50);
	    }
	    else
	    {
		    led_off(L1);
		    low_batt();
			return;
	    }
	
        if (trigger() == ((mode) ? TRIGGER_FIRE : TRIGGER_OFF))
            break;

	    wdt_reset();
    }
}

int main(void)
{
	while (1)
	{
        init();
		idle();
		run();
	}
}

void low_batt()
{
	int i, j;
	
    led_off(L1);
    led_off(L2);
    led_off(L3);
    led_off(L4);
	output_pwm(0);
	disable_hv();
    
	for (i = 0; i < 20; i ++)
    {
        ((i & 0x01) == 0) ? led_on(L1) : led_off(L1);
		
        for (j = 0; j < 30; j ++)
		{
			_delay_ms(10);
            
            if (trigger() == ((mode) ? TRIGGER_FIRE : TRIGGER_OFF))
                return;
		}
        
        wdt_reset();
    }        
}
