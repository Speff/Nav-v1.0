/*
 * main.h
 *
 * Created: 12/11/2015 1:33:09 AM
 *  Author: Sahil
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <string.h>
#include <math.h>

#include "LSM303D.h"
#include "Light_apa102/apa102_config.h"
#include "Light_apa102/light_apa102.h"
#include "LUFA-Interface/GenericHID.h"

#define N_LEDS	5
#define MAG_RES	6.1036e-5
#define ACL_RES	0.000122072

// from AVR035: Efficient C Coding for AVR
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))
#define FLIPBITMASK(x,y) (x ^= (y))
#define CHECKBITMASK(x,y) (x & (y))

struct XYZLimits { int16_t xMin; int16_t xMax; int16_t yMin; int16_t yMax; int16_t zMin; int16_t zMax;};
struct XYZData { int16_t x; int16_t y; int16_t z;};
struct IMUReadings { float pitch; float roll; float heading;};

void configurePorts(void);
void enableLEDs();
void disableLEDs();
void setLEDColor(struct cRGB *, uint8_t);
void int16ToRGB(int16_t, int16_t, int16_t, struct cRGB *);
void doubleToRGB(double, double, double, struct cRGB *);
void floatTo360RGB(float, struct cRGB *);
void printLED8bCode(uint8_t);
void setUpLSM(struct IMUReadings *);
void processCompass(struct IMUReadings *);

void fillCompletedString();
void setUpUART();
void enableGPS();
void disableGPS();

uint8_t spiRxTX(uint8_t, uint8_t, uint8_t);
int16_t spiRxTX16b(uint8_t, uint8_t, int16_t);

// Port Usages
// Port B
// -B0 | Output	| Accelerometer SPI !SS
// -B1 | Output	| Accelerometer SPI CLK & ISP Header
// -B2 | Output | Accelerometer SPI MOSI & ISP Header
// -B3 | Input	| Accelerometer SPI MISO & ISP Header
// -B5 | Output	| Serial LED Data
// -B6 | Output	| Serial LED Clock
// Port C
// -C6 | Output	| Toggle USB max current flow (Active high - need thermistor installed)
// -C7 | Output | Activate/Deactivate 5v boost converter
// Port D
// -D2 | Input	| GPS UART RX
// -D3 | Output | GPS UART TX
// -D4 | Output	| Enable GPS
// -D6 | Input	| Accelerometer Interrupt #1
// -D7 | Input	| Accelerometer Interrupt #2

#define SPI_READ			0x80
#define SPI_WRITE			0x00
#define SPI_MULTIPLEACCESS	0x40

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

#define LED_COLOR_WHITE		0
#define LED_COLOR_BLUE		1
#define LED_COLOR_RED		2
#define LED_COLOR_GREEN		3
#define LED_COLOR_YELLOW	4
#define LED_COLOR_BROWN		5
#define LED_COLOR_ORANGE	6
#define LED_COLOR_PINK		7
#define LED_COLOR_OFF		8
#define LED_COLOR_TURQUOISE 9
