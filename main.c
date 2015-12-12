/*
 * Nav-v1.0 Firmware
 *
 * Created: 12/10/2015 9:00:32 PM
 * Author : Sahil Patel
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "main.h"

#include "Light_apa102/apa102_config.h"
#include "Light_apa102/light_apa102.h"

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

ISR(TIMER0_OVF_vect) {
	// Timer code
}

int main(void){
	struct cRGB LEDArray[N_LEDS];	// LED Color Array
	
	configurePorts();				// Set up I/O ports
	
	//TCCR0B = (1<<CS00);			// 001 = CLK; 010 = CLK/8; 011 = CLK/64
									// 100 = CLK/256; 101 = CLK/1024
	//TIMSK0 = (1<<TOIE0);			// TOIE0: Timer/Counter0 Overflow Interrupt Enable
									// OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
									// OCIE0B: Timer/Counter0 Output Compare Match B Interrupt Enable
	//TCNT0 = 0x00;					// Timer/Counter Register - Set to 0 at start
	
	enableLEDs();
	
	//sei();
	
   // Main loop
    while(1){
		LEDArray[0].r=0xFF;	LEDArray[0].g=0xFF;	LEDArray[0].b=0xFF;	LEDArray[0].fade=0x01;
		LEDArray[1].r=0xFF;	LEDArray[1].g=0xFF;	LEDArray[1].b=0xFF;	LEDArray[1].fade=0x01;
		LEDArray[2].r=0xFF;	LEDArray[2].g=0xFF;	LEDArray[2].b=0xFF;	LEDArray[2].fade=0x01;
		LEDArray[3].r=0x00;	LEDArray[3].g=0xFF;	LEDArray[3].b=0xFF;	LEDArray[3].fade=0x01;
		LEDArray[4].r=0x00;	LEDArray[4].g=0xFF;	LEDArray[4].b=0xFF;	LEDArray[4].fade=0x01;
		apa102_setleds(LEDArray,5);
		_delay_ms(500);  
    }
}

void enableLEDs(){
	PORT_BOOST_EN |= (1<<PIN_BOOST_EN);
}

void disableLEDs(){
	PORT_BOOST_EN &= ~(1<<PIN_BOOST_EN);
}

void configurePorts(){
	// Configure PortC I/O directionality
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB5) | (1<<DDB6);
	DDRB &= ~(1<<DDB3);
	
	// Configure PortC I/O directionality
	DDRC |= (1<<DDC6) | (1<<DDC7);
	
	// Configure PortD I/O directionality
	DDRD |= (1<<DDD3) | (1<<DDD4);
	DDRD &= ~((1<<DDD2) | (1<<DDD6) | (1<<DDD7));
}

