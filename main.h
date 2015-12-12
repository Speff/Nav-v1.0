/*
 * main.h
 *
 * Created: 12/11/2015 1:33:09 AM
 *  Author: Sahil
 */ 

//#define F_CPU 8000000UL

#define N_LEDS 5

// from AVR035: Efficient C Coding for AVR
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))
#define FLIPBITMASK(x,y) (x ^= (y))
#define CHECKBITMASK(x,y) (x & (y))

#define PORT_SPI_SS			PORTB
#define PORT_SPI_SCK		PORTB
#define PORT_SPI_MOSI		PORTB
#define PORT_SPI_MISO		PORTB
#define PORT_LED_CLOCK		PORTB
#define PORT_LED_DATA		PORTB
#define PORT_UART_RX		PORTD
#define PORT_UART_TX		PORTD
#define PORT_EN_GPS			PORTD
#define PORT_INT1			PORTD
#define PORT_INT2			PORTD
#define PORT_ADD_CURRENT	PORTC
#define PORT_BOOST_EN		PORTC

#define PIN_SPI_SS			PORTB0
#define PIN_SPI_SCK			PORTB1
#define PIN_SPI_MOSI		PORTB2
#define PIN_SPI_MISO		PORTB3
#define PIN_LED_CLOCK		PORTB6
#define PIN_LED_DATA		PORTB5
#define PIN_UART_RX			PORTD2
#define PIN_UART_TX			PORTD3
#define PIN_EN_GPS			PORTD4
#define PIN_INT1			PORTD6
#define PIN_INT2			PORTD7
#define PIN_ADD_CURRENT		PORTC6
#define PIN_BOOST_EN		PORTC7


void configurePorts(void);
void enableLEDs();
void disableLEDs();
