#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

volatile uint16_t frequency = 440;
volatile uint8_t print_flag = 0;

volatile int16_t manual_offset = 0;

void UART_init(uint16_t ubrr) {
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void UART_send_string(const char *s) {
    while (*s) UART_send_char(*s++);
}

ISR(USART_RX_vect) {
    char c = UDR0;

    if (c == '+') {
        manual_offset += 10;
        UART_send_string("\r\n");
    }
    else if (c == '-') {
        manual_offset -= 10;
        UART_send_string("\r\n");
    }
}

void ADC_init() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_read() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void Timer1_init() {
    DDRB |= (1 << PB1);
    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << WGM12);
}

void Timer1_set_frequency(uint16_t f) {
    if (f < 50) f = 50;
    if (f > 1000) f = 1000;

    uint16_t ocr = (F_CPU / (2UL * 8UL * f)) - 1;
    OCR1A = ocr;
    
    if (!(TCCR1B & (1 << CS11))) {
        TCCR1B |= (1 << CS11);
    }
}

void Timer1_stop() {
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

void Timer2_init() {
    DDRD |= (1 << PD3);
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    OCR2B = 0;
}

void Timer2_stop() {
    OCR2B = 0;
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

void Timer2_start() {
    TCCR2B |= (1 << CS22);
}

void Timer0_init(void) {
    TCCR0A = (1 << WGM01);
    OCR0A  = 249;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    static uint16_t ms = 0;
    ms++;
    if (ms >= 2000) {   
        print_flag = 1;
        ms = 0;
    }

    static uint8_t fade_cnt = 0;
    static int8_t fade_dir = 1;
    static uint8_t brightness = 0;

    fade_cnt++;
    if (fade_cnt >= 10) { 
        fade_cnt = 0;
        
        brightness += fade_dir;
        OCR2B = brightness; 

        if (brightness == 255) fade_dir = -1;
        else if (brightness == 0) fade_dir = 1;
    }
}

int main(void) {
    UART_init(103);
    ADC_init();
    Timer1_init();
    Timer0_init();
    Timer2_init();

    sei();

    DDRD &= ~(1 << PD7);
    PORTD |= (1 << PD7);

    while (1) {
        if (!(PIND & (1 << PD7))) {
            
            Timer1_stop();
            Timer2_stop();
            
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sleep_cpu();
            sleep_disable();
            
            Timer2_start();
            continue;
        }

        uint16_t adc_val = ADC_read();
        uint16_t base_frequency = 50 + (adc_val * 950UL / 1023UL);

        frequency = base_frequency + manual_offset;

        if (frequency < 50) frequency = 50;
        if (frequency > 1000) frequency = 1000;

        Timer1_set_frequency(frequency);

        if (print_flag) {
            print_flag = 0;
            char msg[40];
            sprintf(msg, "frequency is %u Hz\r\n", frequency);
            UART_send_string(msg);
        }
    }
}
