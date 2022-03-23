/*
 * W3_microcontrollers _isp.c
 *
 * Created: 9-3-2022 15:45:52
 * Author : nianl
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define SS 2        // pin 10 arduino uno
#define MOSI 3      // pin 11 arduino uno
#define MISO 4      // pin 12 arduino uno
#define SCK 5       // pin 13 arduino uno
#define TXD 1

#define  FOSC 16000000
#define  BAUD 9600
#define  MYUBBR FOSC/16/BAUD -1

#define IODIR 0
#define OUTP_LATCH 10

volatile uint8_t spi_rx_data = 0;
volatile uint8_t spi_txrx_done = 0;

uint8_t serialReadPos = 0;
uint8_t serialWritePos = 0;
#define UART_BUFF_SIZE 10
char serialbufffer [UART_BUFF_SIZE];


void UARD_int(void){
	//*************************This is the setup of the USART**********************************************************
	UBRR0H = (MYUBBR >> 8); // This is the USART BAUD generator register. Write the last bits of the value of 'BRC' to this 'UBRR0H' register. In this case 'BRC' is the value required for the BAUD-generator.
	UBRR0L = MYUBBR; // This is also the USART BAUD generator register. Write the value of the 'BRC' to this 'UBRR0L' register.
	
	UCSR0B = (1 << TXEN0) |  (1 << TXCIE0) | (1 << RXEN0) | (1 << RXCIE0); // Here, you enable the TX and the RX for the USART communication. Also, you enable the interrupt generated by a complete conversion of your RX. Also enable TX with her specified interrupt.
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Here, you are using 8-bits data (the 'UCSZ00' bit), and in total 2 stop bits (the 'USBS0' bit).
	
}
 void appendSerial(char c){//read out string one char at the time
	 serialbufffer[serialWritePos] = c;
	 serialWritePos++;
	 
	 if(serialWritePos >= UART_BUFF_SIZE){
		 serialWritePos = 0;
	 }
 }
void serialWrite(char c[]){ //get string in to buffer
	for(uint8_t i = 0; i < strlen(c); i++){
		appendSerial(c[i]);
	}
	if(UCSR0A & (1 << UDRE0)){
		UDR0 = 0;
	}
}

ISR(SPI_STC_vect){
	if(SPSR & 0x40){
		spi_rx_data = SPDR;
		spi_rx_data = 0;
		spi_txrx_done = 1;
	}else{
		spi_rx_data = SPDR;
		spi_txrx_done = 1; 
	
       }
}
void spi_int(void){
	DDRB |= (1 << DDB5) | (1 << DDB3) | (1 << DDB2);
	PORTB |= 1 << PORTB2; //pull up resistor enbaled 
	
	SPCR = (1 << SPIE) | (1 << SPE) | (1 << CPOL) << (0 << CPHA) | (0b11 << SPR0);
}

void spi_sent(uint8_t *tx, uint8_t *rx){
	spi_txrx_done = 0;
	SPDR = tx;
	while(spi_txrx_done != 0);
	*rx = spi_rx_data;	
}

void  spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len ){
	PORTB &= ~(1 << PORTB2); 
	//_delay_ms(100);
	for(uint16_t i =0; i < len; i++){
		spi_sent(&tx[i], &rx[i]);
	}
	PORTB |= (1 << PORTB2);
	//_delay_ms(100);
}
int main(void)
{
	spi_int();
	uint8_t tx_data = 0;
	uint8_t rx_data = 0; 
	
	sei();

    while (1) 
    {
		tx_data++;
		spi_transfer(&tx_data,&rx_data, 1);
		if(tx_data >= 255){
			tx_data =0; 
		}
	memset(serialbufffer,0,sizeof(serialbufffer));
	sprintf(serialbufffer, "%c %d", rx_data, tx_data);
	serialWrite(serialbufffer);
	
	}
}
