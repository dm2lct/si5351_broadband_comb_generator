/*****************************************************************/
/*             Comb generator with Si5153 and ATMega328PB        */
/*  ************************************************************ */
/*  Mikrocontroller:  ATMEL AVR ATmega328, 8 MHz                 */
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Author:           Luzia Christiane Tesar                     */
/*  Last Change:      2025-08-22                                */
/*****************************************************************/

// This is a minimal Firmware for the SI5351 Comb Generator
// Developed 2025 at the chair of elektromagnetic theory and 
// compatibility at TU Dresden
// it enables basic interpretion of some SCPI-Style commands and a manual control of the generator using a rotary switch
// The code snippet for the communication with the si5351 is a fork from DK7IH:
// https://dk7ih.de/a-simple-software-to-control-the-si5351a-generator-chip/

#define F_CPU 8000000UL
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "uart.h"

#define BAUDRATE 9600
#define TRUE 1
#define FALSE 0

#define CHAR_NEWLINE '\n'
#define CHAR_RETURN '\r'
#define RETURN_NEWLINE "\r\n"
#define CR "\r\n"
volatile unsigned char data_in[32];
unsigned char command_in[32];

volatile unsigned char data_count;
volatile unsigned char command_ready;
volatile unsigned char remote;

volatile char cmd[32], value[32], unit[32];
unsigned long t1, freq;

/////////////////////
//Defines for Si5351
/////////////////////
#define SI5351_ADDRESS 0xC0 // 0b11000000 for my module. Others may vary! The 0x60 did NOT work with my module!

//Set of Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define PLLX_SRC		15
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define CLK3_CONTROL            19
#define CLK4_CONTROL            20
#define CLK5_CONTROL            21
#define CLK6_CONTROL            22
#define CLK7_CONTROL            23
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

//The unavoidable functional stuff
int main(void);
void wait_ms(int);

//  TWI Declarations
void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t u8data);
uint8_t twi_get_status(void);

//  SI5351 Declarations
void si5351_write(int, int);
void si5351_start(void);
void si5351_set_freq(int, unsigned long);

/////////////////////
//
//   TWI-Functions
//
/////////////////////
void twi_init(void)
{
    //set SCL to 400kHz
    TWSR0 = 0x00;
    TWBR0 = 0x0C;
	
    //enable TWI
    TWCR0 = (1<<TWEN);
}

//Send start signal
void twi_start(void)
{
    TWCR0 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR0 & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR0 = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR0 = u8data;
    TWCR0 = (1<<TWINT)|(1<<TWEN);
    while ((TWCR0 & (1<<TWINT)) == 0);
}

////////////////////////////////
//
// Si5351A commands
//
///////////////////////////////
void si5351_write(int reg_addr, int reg_value)
{
   twi_start();
   twi_write(SI5351_ADDRESS);
   twi_write(reg_addr);
   twi_write(reg_value);
   twi_stop();
} 

// Set PLLs (VCOs) to internal clock rate of 900 MHz 
// Equation fVCO = fXTAL * (a+b/c) (=> AN619 p. 3 
void si5351_start(void) 
{ 
  unsigned long a, b, c; 
  unsigned long p1, p2, p3; 
   
  // Init clock chip 
  si5351_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default),  
                                          // for bits 5:0 see also AN619 p. 60 
  si5351_write(CLK_ENABLE_CONTROL, 0x7E); // Enable CLK0 and CLK7 
  si5351_write(CLK0_CONTROL, 0x0F);       // Set PLLA to CLK0, 8 mA output 
  si5351_write(CLK1_CONTROL, 0x80);       // CLK1 output OFF 
  si5351_write(CLK2_CONTROL, 0x80);       // CLK2 output OFF 
  si5351_write(CLK3_CONTROL, 0x80);       // CLK3 output OFF 
  si5351_write(CLK4_CONTROL, 0x80);       // CLK4 output OFF 
  si5351_write(CLK5_CONTROL, 0x80);       // CLK5 output OFF 
  si5351_write(CLK6_CONTROL, 0x80);       // CLK6 output OFF 
  si5351_write(CLK7_CONTROL, 0x80);       // CLK7 output OFF
  si5351_write(PLL_RESET, 0xA0);          // Reset PLLA and PLLB 
 
  // Set VCOs of PLLA and PLLB to 900 MHz 
  a = 36;           // Division factor 900/25 MHz 
  b = 0;            // Numerator, sets b/c=0 
  c = 1048575;      //Max. resolution, but irrelevant in this case (b=0) 
 
  //Formula for splitting up the numbers to register data, see AN619 
  p1 = 128 * a + (unsigned long) (128 * b / c) - 512; 
  p2 = 128 * b - c * (unsigned long) (128 * b / c); 
  p3  = c; 
 
  //Write data to registers PLLA and PLLB so that both VCOs are set to 900MHz intermal freq 
  si5351_write(SYNTH_PLL_A, 0xFF); 
  si5351_write(SYNTH_PLL_A + 1, 0xFF); 
  si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16); 
  si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8); 
  si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF)); 
  si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16)); 
  si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8); 
  si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF)); 
 
  si5351_write(SYNTH_PLL_B, 0xFF); 
  si5351_write(SYNTH_PLL_B + 1, 0xFF); 
  si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16); 
  si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8); 
  si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF)); 
  si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16)); 
  si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8); 
  si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF)); 
 
} 
 
void si5351_set_freq(int synth, unsigned long freq) 
{ 
   
  unsigned long  a, b, c = 1048575; 
  unsigned long f_xtal = 25000000; 
  double fdiv = (double) (f_xtal * 36) / freq; //division factor fvco/freq (will be integer part of a+b/c) 
  double rm; //remainder 
  unsigned long p1, p2, p3; 
   
  a = (unsigned long) fdiv; 
  rm = fdiv - a;  //(equiv. b/c) 
  b = rm * c; 
  p1  = 128 * a + (unsigned long) (128 * b / c) - 512; 
  p2 = 128 * b - c * (unsigned long) (128 * b / c); 
  p3 = c; 
   
  //Write data to multisynth registers of synth n 
  si5351_write(synth, 0xFF);      //1048757 MSB 
  si5351_write(synth + 1, 0xFF);  //1048757 LSB 
  si5351_write(synth + 2, (p1 & 0x00030000) >> 16); 
  si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8); 
  si5351_write(synth + 4, (p1 & 0x000000FF)); 
  si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16)); 
  si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8); 
  si5351_write(synth + 7, (p2 & 0x000000FF)); 
} 
/////////////////////////////////////////////
//              M  I  S  C  
/////////////////////////////////////////////
//Substitute defective _delay_ms() function in delay.h
void wait_ms(int ms)
{
    int t1, t2;
    int dtime = (int) 137 * 8;

    for(t1 = 0; t1 < ms; t1++)
    {
        for(t2 = 0; t2 < dtime; t2++)
        {
            asm volatile ("nop" ::);
        }
    }        
}

/////////////////////////////////////////////
//              U  A  R  T  
/////////////////////////////////////////////

void uart_init (void)
{
    //uint8_t sreg = SREG;
    uint16_t ubrr = (uint16_t) ((uint32_t) F_CPU/(16UL*BAUDRATE) - 1);

    UBRR0H = (uint8_t) (ubrr>>8);
    UBRR0L = (uint8_t) (ubrr);
    // Interrupts kurz deaktivieren 
    cli();
    // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren 
    // Data mode 8N1, asynchron 
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (0 << 4) | (0 << 3) | (3 << 1);
    // Flush Receive-Buffer (entfernen evtl. vorhandener ungültiger Werte) 
    do
    {
        // UDR auslesen (Wert wird nicht verwendet) 
        UDR0;
    }
    while (UCSR0A & (1 << RXC0));

    // Rücksetzen von Receive und Transmit Complete-Flags 
    UCSR0A = (1 << RXC0) | (1 << TXC0);

}


int uart_putc (const uint8_t c)
{
	    // Warten, bis UDR bereit ist für einen neuen Wert
    while (!(UCSR0A & (1 << UDRE0)))
        ;

    // UDR Schreiben startet die Übertragung
    UDR0 = c;

    return 1;
}

void uart_puts (const char *s)
{
    do
    {
        uart_putc (*s);
    }
    while (*s++);
}


void usart_ok()
{
    uart_puts("OK\r\n");
}

void copy_command ()
{
    // The USART might interrupt this - don't let that happen!
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // Copy the contents of data_in into command_in
        memcpy(command_in, data_in, 32);

        // Now clear data_in, the USART can reuse it now
        memset(data_in, 0, 32);
    }
}

void split_command(char str[]){
	str[strlen(str)-1] = '\0';
	strcpy(cmd, strtok(str , " "));
    	strcpy(value, strtok(NULL, " "));
    	strcpy(unit , strtok(NULL, " "));
}

void process_command()
{
	if (strcmp(cmd, "*IDN?")==0){
		remote = TRUE;
		uart_puts("TUD TET_EMV Kammgenerator V1.0\n");
	}
	else if (strcmp(cmd, "*RST")==0){
		si5351_write(CLK_ENABLE_CONTROL, 0xFF);  
		remote = FALSE;
	}
	else if (strcmp(cmd, "FREQ?")==0){                	
		remote = TRUE;
  		char f[32];
  		uart_puts(ltoa(freq,f,10));
		uart_puts(" Hz\n");
        }
	else if (strcmp(cmd, "FREQ")==0){
		remote = TRUE;
  		if (strcmp(unit, "Hz")==0){
			freq=atol(value);	
		}
		if (strcmp(unit, "kHz")==0){
			freq=atol(value)*1000;	
		}
		if (strcmp(unit, "MHz")==0){
			freq=atol(value)*1000000;	
		}
		if (freq >= 1000000 && freq <= 150000000){
			si5351_set_freq(SYNTH_MS_0, freq);
		}
		else {
			uart_puts("-100\n");
		}
	}
	else if (strcmp(cmd, "OUTP")==0){ 
		remote = TRUE;
		if (strcmp(value, "OFF")==0){
			si5351_write(CLK_ENABLE_CONTROL, 0xFF);  
		}
		else if (strcmp(value, "ON")==0){
			si5351_write(CLK_ENABLE_CONTROL, 0x7E); 
		}
		else {
			uart_puts("-100\n");
		}

	}
	else {
		uart_puts("-100\n");
	}


        
}

int main(void)
{
	int SwitchPos=0;
	int lastSwitchPos;	
	DDRD |= (1<< DDD2);
	DDRD  &= ~(1<<PD3)|~(1<<PD4)|~(1<<PD5)|~(1<<PD6)|~(1<<PD7);
	uart_init();
	sei();
	PORTC = 0x30;//I²C-Bus lines: PC4=SDA, PC5=SCL 
				
	twi_init();
	wait_ms(100);
	si5351_start();
	wait_ms(100);
    while (1)
    {
	if (command_ready == TRUE) {
		copy_command();
		split_command(command_in);
		process_command();
        	command_ready = FALSE;
        }
	PORTD |= (1 << PD2);  //PB0 im PORTB setzen
        wait_ms(100);
	if(PIND & (1 << PD3)) {
		//OFF/LOKAL
		SwitchPos=0;
	}
	if(PIND & (1 << PD4)) {
		//100k
		SwitchPos=1;
	}
	if(PIND & (1 << PD5)) {
		//1M
		SwitchPos=2;
	}
	if(PIND & (1 << PD6)) {
		//10M
		SwitchPos=3;
	}
	if(PIND & (1 << PD7)) {
		//100M
		SwitchPos=4;
	}
	
	wait_ms(100);
	PORTD &= ~(1 << PD2); //PD2 im PORTD löschen
	
	if(SwitchPos!=lastSwitchPos){
		switch(SwitchPos){
			case 0:
				remote=FALSE;
				si5351_write(CLK_ENABLE_CONTROL, 0xFF);  // Output off
			break;

			case 1:
				freq=1000000;
			break;

			case 2:
				freq=10000000;
			break;

			case 3:
				freq=50000000;
			break;

			case 4:
				freq=100000000;
			break;

		}	
		if(remote==FALSE && SwitchPos!=0){
			si5351_write(CLK_ENABLE_CONTROL, 0x7E); 
			si5351_set_freq(SYNTH_MS_0, freq);
		}
		lastSwitchPos=SwitchPos;
	}
    }

    return 0;
}

ISR(USART0_RX_vect)
{
	// Get data from the USART in register
    	data_in[data_count] = UDR0;

	// End of line!
    	if (data_in[data_count] == '\n') {
        	command_ready = TRUE;
		// Reset to 0, ready to go again
        	data_count = 0;
	} else {
        	data_count++;
    }
}


